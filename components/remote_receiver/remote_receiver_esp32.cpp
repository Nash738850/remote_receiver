#include "esphome/core/log.h"
#include "remote_receiver.h"

#ifdef USE_ESP32

#include "driver/rmt_rx.h"
#include "esp_log.h"

namespace esphome {
namespace remote_receiver {

static const char *const TAG = "remote_receiver";

// Constructor with optional RMT channel
RemoteReceiverComponent::RemoteReceiverComponent(InternalGPIOPin *pin)
    : pin_(pin), rmt_channel_(255) {}

void RemoteReceiverComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Remote Receiver on pin %u", this->pin_->get_pin());

  rmt_rx_channel_config_t rx_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 1'000'000,  // 1MHz = 1us ticks
      .mem_block_symbols = 64,
      .gpio_num = (gpio_num_t) this->pin_->get_pin(),
      .flags = {
          .invert_in = this->pin_->is_inverted(),
      },
  };

  // Allocate RMT channel
  rmt_channel_handle_t channel = nullptr;
  esp_err_t err = rmt_new_rx_channel(&rx_config, &channel);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RMT init failed: %s", esp_err_to_name(err));
    mark_failed();
    return;
  }

  this->rmt_channel_handle_ = channel;
  ESP_LOGI(TAG, "RMT receiver initialized on channel %u", this->rmt_channel_);
}

void RemoteReceiverComponent::loop() {
  // Here you can poll for data from the RMT driver if needed
}

// Added dump_config merged in
void RemoteReceiverComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Remote Receiver:");
  ESP_LOGCONFIG(TAG, "  Pin: GPIO%d", this->pin_->get_pin());
  if (this->rmt_channel_ != 255) {
    ESP_LOGCONFIG(TAG, "  Forced RMT Channel: %u", this->rmt_channel_);
  }
  ESP_LOGCONFIG(TAG, "  Buffer Size: %u", this->buffer_size_);
  ESP_LOGCONFIG(TAG, "  Tolerance: %u%%", this->tolerance_);
  ESP_LOGCONFIG(TAG, "  Filter (min pulse): %uus", this->filter_us_);
  ESP_LOGCONFIG(TAG, "  Idle after: %uus", this->idle_us_);
}

}  // namespace remote_receiver
}  // namespace esphome

#endif
