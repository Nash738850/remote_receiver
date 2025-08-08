#include "remote_receiver.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace esphome {
namespace remote_receiver {

static const char *const TAG = "remote_receiver.esp32";

// ISR callback: push RX-done event into our queue
static bool IRAM_ATTR rmt_rx_callback(rmt_channel_handle_t,
                                      const rmt_rx_done_event_data_t *edata,
                                      void *user_data) {
  auto q = static_cast<QueueHandle_t>(user_data);
  BaseType_t hp_wakeup = pdFALSE;
  xQueueSendFromISR(q, edata, &hp_wakeup);
  return hp_wakeup == pdTRUE;
}

void RemoteReceiverComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Remote Receiver (ESP-IDF 5 RMT v2)...");
  this->pin_->setup();

  // Configure RX channel (explicit field assignment to appease 5.3.x)
  rmt_rx_channel_config_t rx_cfg{};
  rx_cfg.gpio_num = (gpio_num_t) this->pin_->get_pin();
  rx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  rx_cfg.resolution_hz = 1000000;    // 1 tick = 1 µs
  rx_cfg.mem_block_symbols = 256;    // tune as needed
  // rx_cfg.flags: leave defaults; no invert_in in 5.3.x (we handle inversion in software)

  esp_err_t err = rmt_new_rx_channel(&rx_cfg, &this->rx_channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "rmt_new_rx_channel failed: %s", esp_err_to_name(err));
    this->error_code_ = err;
    this->mark_failed();
    return;
  }

  // Create queue & register callback
  this->rx_queue_ = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
  rmt_rx_event_callbacks_t cbs{};
  cbs.on_recv_done = rmt_rx_callback;
  err = rmt_rx_register_event_callbacks(this->rx_channel_, &cbs, this->rx_queue_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "rmt_rx_register_event_callbacks failed: %s", esp_err_to_name(err));
    this->error_code_ = err;
    this->mark_failed();
    return;
  }

  // Enable channel
  err = rmt_enable(this->rx_channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(err));
    this->error_code_ = err;
    this->mark_failed();
    return;
  }

  // Arm reception (min/max in nanoseconds)
  rmt_receive_config_t rx_conf{};
  rx_conf.signal_range_min_ns = (uint32_t) (this->filter_us_ * 1000ULL);
  rx_conf.signal_range_max_ns = (uint32_t) (this->idle_us_   * 1000ULL);

  err = rmt_receive(this->rx_channel_, nullptr, 0, &rx_conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "rmt_receive start failed: %s", esp_err_to_name(err));
    this->error_code_ = err;
    this->mark_failed();
    return;
  }
}

void RemoteReceiverComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Remote Receiver:");
  LOG_PIN("  Pin: ", this->pin_);
  if (this->pin_->digital_read()) {
    ESP_LOGW(TAG, "Signal starts HIGH. If decoding seems inverted, use 'inverted: true' on the pin.");
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

  rmt_rx_done_event_data_t evt;
  if (xQueueReceive(this->rx_queue_, &evt, 0) == pdTRUE) {
    // Convert symbols to pulse list used by RemoteReceiverBase
    this->decode_rmt_(evt.received_symbols, evt.num_symbols);

    if (!this->temp_.empty()) {
      this->temp_.push_back(-this->idle_us_);
      this->call_listeners_dumpers_();
    }

    // Re-arm reception
    rmt_receive_config_t rx_conf{};
    rx_conf.signal_range_min_ns = (uint32_t) (this->filter_us_ * 1000ULL);
    rx_conf.signal_range_max_ns = (uint32_t) (this->idle_us_   * 1000ULL);
    (void) rmt_receive(this->rx_channel_, nullptr, 0, &rx_conf);
  }
}

void RemoteReceiverComponent::decode_rmt_(const rmt_symbol_word_t *sym, size_t len) {
  bool prev_level = false;
  uint32_t prev_len = 0;
  this->temp_.clear();
  const int32_t mult = this->pin_->is_inverted() ? -1 : 1;

  this->temp_.reserve(len * 2);  // two halves per symbol

  for (size_t i = 0; i < len; i++) {
    // Half A
    if (sym[i].duration0 != 0u) {
      if ((bool) sym[i].level0 == prev_level) {
        prev_len += sym[i].duration0;
      } else {
        if (prev_len > 0) {
          this->temp_.push_back((prev_level ? 1 : -1) * (int32_t) prev_len * mult);
        }
        prev_level = (bool) sym[i].level0;
        prev_len = sym[i].duration0;
      }
    }
    // Half B
    if (sym[i].duration1 != 0u) {
      if ((bool) sym[i].level1 == prev_level) {
        prev_len += sym[i].duration1;
      } else {
        if (prev_len > 0) {
          this->temp_.push_back((prev_level ? 1 : -1) * (int32_t) prev_len * mult);
        }
        prev_level = (bool) sym[i].level1;
        prev_len = sym[i].duration1;
      }
    }
  }

  if (prev_len > 0) {
    this->temp_.push_back((prev_level ? 1 : -1) * (int32_t) prev_len * mult);
  }
}

}  // namespace remote_receiver
}  // namespace esphome

#endif  // USE_ESP32
