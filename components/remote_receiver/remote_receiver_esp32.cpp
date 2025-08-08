#include "remote_receiver.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace esphome {
namespace remote_receiver {

static const char *const TAG = "remote_receiver.esp32";

// RX-done ISR callback: push event into our queue
static bool IRAM_ATTR rmt_rx_callback(rmt_channel_handle_t,
                                      const rmt_rx_done_event_data_t *edata,
                                      void *user_data) {
  auto queue = static_cast<QueueHandle_t>(user_data);
  BaseType_t high_wakeup = pdFALSE;
  xQueueSendFromISR(queue, edata, &high_wakeup);
  return high_wakeup == pdTRUE;
}

void RemoteReceiverComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Remote Receiver (ESP-IDF 5 API)...");
  this->pin_->setup();

  // Create RX channel (1 MHz -> duration fields are microseconds)
  rmt_rx_channel_config_t rx_cfg = {
      .gpio_num = (gpio_num_t) this->pin_->get_pin(),
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 1'000'000,
      .mem_block_symbols = 256,   // tune if needed
      .flags = {}
  };

  this->error_code_ = rmt_new_rx_channel(&rx_cfg, &this->rx_channel_);
  if (this->error_code_ != ESP_OK) {
    ESP_LOGE(TAG, "rmt_new_rx_channel failed: %s", esp_err_to_name(this->error_code_));
    this->mark_failed();
    return;
  }

  // Queue for RX events
  this->rx_queue_ = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));

  // Register callback
  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = rmt_rx_callback;
  this->error_code_ = rmt_rx_register_event_callbacks(this->rx_channel_, &cbs, this->rx_queue_);
  if (this->error_code_ != ESP_OK) {
    ESP_LOGE(TAG, "rmt_rx_register_event_callbacks failed: %s", esp_err_to_name(this->error_code_));
    this->mark_failed();
    return;
  }

  // Enable channel
  this->error_code_ = rmt_enable(this->rx_channel_);
  if (this->error_code_ != ESP_OK) {
    ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(this->error_code_));
    this->mark_failed();
    return;
  }

  // Arm receive (idle_us becomes max range; filter_us becomes min)
  rmt_receive_config_t rx_conf = {
      .signal_range_min_ns = (uint32_t) (this->filter_us_ * 1000ULL),
      .signal_range_max_ns = (uint32_t) (this->idle_us_   * 1000ULL),
  };
  this->error_code_ = rmt_receive(this->rx_channel_, nullptr, 0, &rx_conf);
  if (this->error_code_ != ESP_OK) {
    ESP_LOGE(TAG, "rmt_receive start failed: %s", esp_err_to_name(this->error_code_));
    this->mark_failed();
    return;
  }
}

void RemoteReceiverComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Remote Receiver:");
  LOG_PIN("  Pin: ", this->pin_);
  if (this->pin_->digital_read()) {
    ESP_LOGW(TAG, "Signal starts HIGH. If decoding is inverted, use 'inverted: true' on the pin.");
  }
  ESP_LOGCONFIG(TAG, "  Buffer Size: %u", this->buffer_size_);
  ESP_LOGCONFIG(TAG, "  Tolerance: %u%%", this->tolerance_);
  ESP_LOGCONFIG(TAG, "  Filter (min pulse): %u us", this->filter_us_);
  ESP_LOGCONFIG(TAG, "  Idle after: %u us", this->idle_us_);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "RMT init failed: %s", esp_err_to_name(this->error_code_));
  }
}

void RemoteReceiverComponent::loop() {
  if (!this->rx_channel_ || !this->rx_queue_) return;

  rmt_rx_done_event_data_t rx_data;
  if (xQueueReceive(this->rx_queue_, &rx_data, 0) == pdTRUE) {
    // Convert symbols to the pulse list used by RemoteReceiverBase
    this->decode_rmt_(rx_data.received_symbols, rx_data.num_symbols);

    if (!this->temp_.empty()) {
      this->temp_.push_back(-this->idle_us_);
      this->call_listeners_dumpers_();
    }

    // Re-arm receive
    rmt_receive_config_t rx_conf = {
        .signal_range_min_ns = (uint32_t) (this->filter_us_ * 1000ULL),
        .signal_range_max_ns = (uint32_t) (this->idle_us_   * 1000ULL),
    };
    (void) rmt_receive(this->rx_channel_, nullptr, 0, &rx_conf);
  }
}

void RemoteReceiverComponent::decode_rmt_(const rmt_symbol_word_t *item, size_t len) {
  bool prev_level = false;
  uint32_t prev_length = 0;
  this->temp_.clear();
  int32_t multiplier = this->pin_->is_inverted() ? -1 : 1;

  ESP_LOGVV(TAG, "START:");
  for (size_t i = 0; i < len; i++) {
    ESP_LOGVV(TAG, "%u A: %s %uus", i, item[i].level0 ? "ON" : "OFF", item[i].duration0);
    ESP_LOGVV(TAG, "%u B: %s %uus", i, item[i].level1 ? "ON" : "OFF", item[i].duration1);
  }

  this->temp_.reserve(len * 2);  // each symbol has two halves

  for (size_t i = 0; i < len; i++) {
    // Half A
    if (item[i].duration0 != 0u) {
      if (bool(item[i].level0) == prev_level) {
        prev_length += item[i].duration0;
      } else {
        if (prev_length > 0) {
          this->temp_.push_back((prev_level ? 1 : -1) * int32_t(prev_length) * multiplier);
        }
        prev_level = bool(item[i].level0);
        prev_length = item[i].duration0;
      }
    }
    // Half B
    if (item[i].duration1 != 0u) {
      if (bool(item[i].level1) == prev_level) {
        prev_length += item[i].duration1;
      } else {
        if (prev_length > 0) {
          this->temp_.push_back((prev_level ? 1 : -1) * int32_t(prev_length) * multiplier);
        }
        prev_level = bool(item[i].level1);
        prev_length = item[i].duration1;
      }
    }
  }

  if (prev_length > 0) {
    this->temp_.push_back((prev_level ? 1 : -1) * int32_t(prev_length) * multiplier);
  }
}

}  // namespace remote_receiver
}  // namespace esphome

#endif
