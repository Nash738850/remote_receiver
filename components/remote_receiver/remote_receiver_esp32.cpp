#include "remote_receiver.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

// New RMT v2 (IDF 5.x)
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Legacy (deprecated) RMT v1, used only as a fallback if v2 returns ESP_ERR_NOT_FOUND
#include "driver/rmt.h"

namespace esphome {
namespace remote_receiver {

static const char *const TAG = "remote_receiver.esp32";

// ---------- v2 (IDF5) path ----------
static bool IRAM_ATTR rmt_rx_callback(rmt_channel_handle_t,
                                      const rmt_rx_done_event_data_t *edata,
                                      void *user_data) {
  auto q = static_cast<QueueHandle_t>(user_data);
  BaseType_t hp_wakeup = pdFALSE;
  xQueueSendFromISR(q, edata, &hp_wakeup);
  return hp_wakeup == pdTRUE;
}

static esp_err_t start_rmt_v2(RemoteReceiverComponent *self) {
  // Config
  rmt_rx_channel_config_t rx_cfg{};
  rx_cfg.gpio_num = (gpio_num_t) self->pin_->get_pin();
  rx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  rx_cfg.resolution_hz = 1000000;     // 1 tick = 1 µs
  rx_cfg.mem_block_symbols = 256;     // safe default

  esp_err_t err = rmt_new_rx_channel(&rx_cfg, &self->rx_channel_);
  if (err != ESP_OK) return err;

  self->rx_queue_ = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
  rmt_rx_event_callbacks_t cbs{};
  cbs.on_recv_done = rmt_rx_callback;
  ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(self->rx_channel_, &cbs, self->rx_queue_), TAG, "cb fail");
  ESP_RETURN_ON_ERROR(rmt_enable(self->rx_channel_), TAG, "enable fail");

  rmt_receive_config_t rx_conf{};
  rx_conf.signal_range_min_ns = (uint32_t) (self->filter_us_ * 1000ULL);
  rx_conf.signal_range_max_ns = (uint32_t) (self->idle_us_   * 1000ULL);
  ESP_RETURN_ON_ERROR(rmt_receive(self->rx_channel_, nullptr, 0, &rx_conf), TAG, "receive start fail");

  ESP_LOGI(TAG, "RMT v2 RX started (GPIO%d)", self->pin_->get_pin());
  return ESP_OK;
}

// ---------- v1 (legacy) fallback ----------
static esp_err_t start_rmt_v1(RemoteReceiverComponent *self) {
  ESP_LOGW(TAG, "Falling back to legacy RMT driver (deprecated)...");
  // Configure legacy RMT with 1 MHz tick
  rmt_config_t cfg{};
  cfg.rmt_mode = RMT_MODE_RX;
  cfg.gpio_num = (gpio_num_t) self->pin_->get_pin();
  cfg.clk_div = 80; // 80MHz APB / 80 = 1 MHz tick
  cfg.mem_block_num = 1;
  cfg.channel = RMT_CHANNEL_0;  // try channel 0 first for RX; IDF maps as needed
  cfg.rx_config.filter_en = (self->filter_us_ > 0);
  cfg.rx_config.filter_ticks_thresh = self->filter_us_;
  cfg.rx_config.idle_threshold = self->idle_us_;

  esp_err_t err = rmt_config(&cfg);
  if (err != ESP_OK) return err;
  ESP_RETURN_ON_ERROR(rmt_driver_install(cfg.channel, self->buffer_size_, 0), TAG, "driver install fail");

  // Ringbuffer handle
  RingbufHandle_t rb = nullptr;
  ESP_RETURN_ON_ERROR(rmt_get_ringbuf_handle(cfg.channel, &rb), TAG, "get ringbuf fail");
  self->ringbuf_ = rb;

  ESP_RETURN_ON_ERROR(rmt_rx_start(cfg.channel, true), TAG, "rx start fail");
  ESP_LOGI(TAG, "Legacy RMT RX started (GPIO%d, ch%u)", self->pin_->get_pin(), (unsigned) cfg.channel);
  return ESP_OK;
}

void RemoteReceiverComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Remote Receiver...");
  this->pin_->setup();

  // Try modern driver first
  esp_err_t err = start_rmt_v2(this);
  if (err == ESP_ERR_NOT_FOUND) {
    // No free v2 channels (common on ESP32-C3). Try legacy.
    esp_err_t err_fallback = start_rmt_v1(this);
    if (err_fallback != ESP_OK) {
      ESP_LOGE(TAG, "Legacy RMT fallback failed: %s", esp_err_to_name(err_fallback));
      this->error_code_ = err_fallback;
      this->mark_failed();
      return;
    }
    return;
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "RMT v2 init failed: %s", esp_err_to_name(err));
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
  // v2 path: queue-based
  if (this->rx_channel_ && this->rx_queue_) {
    rmt_rx_done_event_data_t evt;
    if (xQueueReceive(this->rx_queue_, &evt, 0) == pdTRUE) {
      this->decode_rmt_(evt.received_symbols, evt.num_symbols);
      if (!this->temp_.empty()) {
        this->temp_.push_back(-this->idle_us_);
        this->call_listeners_dumpers_();
      }
      // re-arm
      rmt_receive_config_t rx_conf{};
      rx_conf.signal_range_min_ns = (uint32_t) (this->filter_us_ * 1000ULL);
      rx_conf.signal_range_max_ns = (uint32_t) (this->idle_us_   * 1000ULL);
      (void) rmt_receive(this->rx_channel_, nullptr, 0, &rx_conf);
    }
    return;
  }

  // legacy path: ringbuffer-based
  if (this->ringbuf_) {
    size_t size = 0;
    auto *items = (rmt_item32_t *) xRingbufferReceive(this->ringbuf_, &size, 0);
    if (items != nullptr) {
      size_t count = size / sizeof(rmt_item32_t);
      // Reuse decoder (expects v2 symbols), convert legacy items into the same shape
      // Create a small stack buffer of v2 symbols and feed in chunks
      // For performance, decode v1 directly:
      bool prev_level = false;
      uint32_t prev_len = 0;
      this->temp_.clear();
      const int32_t mult = this->pin_->is_inverted() ? -1 : 1;

      for (size_t i = 0; i < count; i++) {
        if (items[i].duration0) {
          if ((bool) items[i].level0 == prev_level) prev_len += items[i].duration0;
          else {
            if (prev_len) this->temp_.push_back((prev_level ? 1 : -1) * (int32_t) prev_len * mult);
            prev_level = (bool) items[i].level0;
            prev_len = items[i].duration0;
          }
        }
        if (items[i].duration1) {
          if ((bool) items[i].level1 == prev_level) prev_len += items[i].duration1;
          else {
            if (prev_len) this->temp_.push_back((prev_level ? 1 : -1) * (int32_t) prev_len * mult);
            prev_level = (bool) items[i].level1;
            prev_len = items[i].duration1;
          }
        }
      }
      if (prev_len) this->temp_.push_back((prev_level ? 1 : -1) * (int32_t) prev_len * mult);

      vRingbufferReturnItem(this->ringbuf_, items);

      if (!this->temp_.empty()) {
        this->temp_.push_back(-this->idle_us_);
        this->call_listeners_dumpers_();
      }
    }
  }
}

void RemoteReceiverComponent::decode_rmt_(const rmt_symbol_word_t *sym, size_t len) {
  bool prev_level = false;
  uint32_t prev_len = 0;
  this->temp_.clear();
  const int32_t mult = this->pin_->is_inverted() ? -1 : 1;

  this->temp_.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    if (sym[i].duration0) {
      if ((bool) sym[i].level0 == prev_level) prev_len += sym[i].duration0;
      else {
        if (prev_len) this->temp_.push_back((prev_level ? 1 : -1) * (int32_t) prev_len * mult);
        prev_level = (bool) sym[i].level0;
        prev_len = sym[i].duration0;
      }
    }
    if (sym[i].duration1) {
      if ((bool) sym[i].level1 == prev_level) prev_len += sym[i].duration1;
      else {
        if (prev_len) this->temp_.push_back((prev_level ? 1 : -1) * (int32_t) prev_len * mult);
        prev_level = (bool) sym[i].level1;
        prev_len = sym[i].duration1;
      }
    }
  }
  if (prev_len) this->temp_.push_back((prev_level ? 1 : -1) * (int32_t) prev_len * mult);
}

}  // namespace remote_receiver
}  // namespace esphome

#endif  // USE_ESP32
