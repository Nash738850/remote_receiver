#include "remote_receiver.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include "driver/rmt_rx.h"
#include "esp_idf_version.h"

namespace esphome {
namespace remote_receiver {

static const char *const TAG = "remote_receiver.esp32";

void RemoteReceiverComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Remote Receiver...");

  rmt_rx_channel_config_t rx_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 1000000,  // 1us per tick
      .mem_block_symbols = 64,
      .gpio_num = (gpio_num_t) this->pin_->get_pin(),
      .flags = {
          .with_dma = false,
      },
  };

  esp_err_t err;
  rmt_channel_handle_t rx_channel = NULL;

  if (this->rmt_channel_ != 255) {
    // Force specific channel
    ESP_LOGI(TAG, "Requesting fixed RMT channel %u", this->rmt_channel_);
    rx_config.flags.reserved = (rmt_channel_t) this->rmt_channel_;
    err = rmt_new_rx_channel(&rx_config, &rx_channel);
  } else {
    // Try auto-alloc first
    err = rmt_new_rx_channel(&rx_config, &rx_channel);
    if (err == ESP_ERR_NOT_FOUND) {
      ESP_LOGW(TAG, "Auto channel allocation failed, forcing channel 0");
      rx_config.flags.reserved = RMT_CHANNEL_0;
      err = rmt_new_rx_channel(&rx_config, &rx_channel);
    }
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RMT init failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  rmt_receive_config_t receive_config = {
      .signal_range_min_ns = (uint32_t) this->filter_us_ * 1000,
      .signal_range_max_ns = (uint32_t) this->idle_us_ * 1000,
  };

  err = rmt_enable(rx_channel);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RMT enable failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  this->ringbuf_ = nullptr;
  err = rmt_get_ringbuf_handle(rx_channel, &this->ringbuf_);
  if (err != ESP_OK || this->ringbuf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to get RMT ringbuffer handle: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  err = rmt_receive(rx_channel, nullptr, 0, &receive_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RMT receive start failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "Remote Receiver initialized successfully on GPIO%d", this->pin_->get_pin());
}

void RemoteReceiverComponent::loop() {
  if (this->ringbuf_ == nullptr)
    return;

  size_t size;
  rmt_item32_t *items = (rmt_item32_t *) xRingbufferReceive(this->ringbuf_, &size, 0);
  if (!items)
    return;

  size_t length = size / sizeof(rmt_item32_t);
  this->decode_rmt_(items, length);
  vRingbufferReturnItem(this->ringbuf_, (void *) items);
}

void RemoteReceiverComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Remote Receiver:");
  ESP_LOGCONFIG(TAG, "  Pin: GPIO%d", this->pin_->get_pin());
  ESP_LOGCONFIG(TAG, "  Buffer Size: %u", this->buffer_size_);
  ESP_LOGCONFIG(TAG, "  Tolerance: %u%%", this->tolerance_);
  ESP_LOGCONFIG(TAG, "  Filter (min pulse): %u us", this->filter_us_);
  ESP_LOGCONFIG(TAG, "  Idle after: %u us", this->idle_us_);
  if (this->rmt_channel_ != 255)
    ESP_LOGCONFIG(TAG, "  Forced RMT Channel: %u", this->rmt_channel_);
}

void RemoteReceiverComponent::decode_rmt_(rmt_item32_t *item, size_t len) {
  remote_base::RemoteReceiveData data;
  for (size_t i = 0; i < len; i++) {
    uint32_t duration = item[i].duration0;
    bool level = item[i].level0;
    data.items.push_back({level, duration});
    duration = item[i].duration1;
    level = item[i].level1;
    data.items.push_back({level, duration});
  }
  this->publish(data);
}

}  // namespace remote_receiver
}  // namespace esphome

#endif  // USE_ESP32
