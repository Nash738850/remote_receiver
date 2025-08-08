#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#include "driver/rmt_rx.h"
#endif

namespace esphome {
namespace remote_receiver {

class RemoteReceiverComponent : public Component {
 public:
  // Constructor
  explicit RemoteReceiverComponent(InternalGPIOPin *pin);

  void setup() override;
  void loop() override;
  void dump_config() override;

  // Set RMT channel manually (optional)
  void set_rmt_channel(uint8_t channel) { this->rmt_channel_ = channel; }

  // Settings
  void set_buffer_size(uint16_t size) { this->buffer_size_ = size; }
  void set_tolerance(uint8_t tolerance) { this->tolerance_ = tolerance; }
  void set_filter_us(uint32_t filter) { this->filter_us_ = filter; }
  void set_idle_us(uint32_t idle) { this->idle_us_ = idle; }

 protected:
  InternalGPIOPin *pin_;

#ifdef USE_ESP32
  rmt_channel_handle_t rmt_channel_handle_{nullptr};
#endif

  uint8_t rmt_channel_{255};  // 255 = auto
  uint16_t buffer_size_{10000};
  uint8_t tolerance_{25};
  uint32_t filter_us_{200};
  uint32_t idle_us_{25000};
};

}  // namespace remote_receiver
}  // namespace esphome
