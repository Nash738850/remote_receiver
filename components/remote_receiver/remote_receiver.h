#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/remote_base/remote_base.h"

namespace esphome {
namespace remote_receiver {

// ----- Shared store for GPIO-ISR based capture (ESP8266 + ESP32) -----
struct RemoteReceiverComponentStore {
  static void gpio_intr(RemoteReceiverComponentStore *arg);

  // Even index = falling edge timestamp; odd index = rising edge timestamp
  volatile uint32_t *buffer{nullptr};
  volatile uint32_t buffer_write_at{0};
  uint32_t buffer_read_at{0};
  bool overflow{false};
  uint32_t buffer_size{1000};
  uint8_t filter_us{10};
  ISRInternalGPIOPin pin;
};

class RemoteReceiverComponent : public remote_base::RemoteReceiverBase,
                                public Component {
 public:
  // Keep the legacy constructor signature used by ESPHome codegen
  explicit RemoteReceiverComponent(InternalGPIOPin *pin, uint8_t /*mem_blocks*/ = 1)
      : RemoteReceiverBase(pin) {}

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Settings used by base/logging
  void set_buffer_size(uint32_t v) { this->buffer_size_ = v; }
  void set_filter_us(uint8_t v) { this->filter_us_ = v; }
  void set_idle_us(uint32_t v) { this->idle_us_ = v; }

  // Keep both tolerance setters for back-compat
  void set_tolerance(uint8_t tol, remote_base::ToleranceMode /*mode*/) { this->tolerance_ = tol; }
  void set_tolerance(uint8_t tol) { this->tolerance_ = tol; }

  // No-op but kept for back-compat with configs that set rmt_channel
  void set_rmt_channel(uint32_t /*ch*/) {}

 protected:
  // ISR-based capture state (no RMT)
  RemoteReceiverComponentStore store_;
  HighFrequencyLoopRequester high_freq_;

  // For logging/compat
  uint32_t buffer_size_{10000};
  uint8_t filter_us_{10};
  uint32_t idle_us_{25000};
  uint8_t tolerance_{25};
};

}  // namespace remote_receiver
}  // namespace esphome
