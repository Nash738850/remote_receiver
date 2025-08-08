#pragma once

#include "esphome/core/component.h"
#include "esphome/components/remote_base/remote_base.h"

#ifdef USE_ESP32
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#endif

namespace esphome {
namespace remote_receiver {

#ifdef USE_ESP8266
// (Unchanged ESP8266 store; left out here for brevity if you already have it)
struct RemoteReceiverComponentStore {
  static void gpio_intr(RemoteReceiverComponentStore *arg);

  volatile uint32_t *buffer{nullptr};
  volatile uint32_t buffer_write_at;
  uint32_t buffer_read_at{0};
  bool overflow{false};
  uint32_t buffer_size{1000};
  uint8_t filter_us{10};
  ISRInternalGPIOPin pin;
};
#endif

class RemoteReceiverComponent : public remote_base::RemoteReceiverBase,
                                public Component {
 public:
#ifdef USE_ESP32
  // Back-compat: ESPHome codegen calls (pin, mem_block_num). We ignore mem_block_num in IDF5.
  RemoteReceiverComponent(InternalGPIOPin *pin, uint8_t /*mem_block_num*/ = 1)
      : RemoteReceiverBase(pin) {}
#else
  RemoteReceiverComponent(InternalGPIOPin *pin) : RemoteReceiverBase(pin) {}
#endif

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Settings used by base/logging
  void set_buffer_size(uint32_t v) { this->buffer_size_ = v; }
  void set_filter_us(uint8_t v) { this->filter_us_ = v; }
  void set_idle_us(uint32_t v) { this->idle_us_ = v; }

  // Back-compat: some ESPHome versions call the 2-arg tolerance setter.
  void set_tolerance(uint8_t tol, remote_base::ToleranceMode /*mode*/) { this->tolerance_ = tol; }
  // Also keep a 1-arg form for convenience
  void set_tolerance(uint8_t tol) { this->tolerance_ = tol; }

  // Back-compat: YAML sometimes exposes rmt_channel; noop in IDF5 but we accept it.
  void set_rmt_channel(uint32_t /*ch*/) {}

 protected:
#ifdef USE_ESP32
  // New RMT v2 decode signature
  void decode_rmt_(const rmt_symbol_word_t *symbols, size_t len);
  rmt_channel_handle_t rx_channel_{nullptr};
  QueueHandle_t rx_queue_{nullptr};
  esp_err_t error_code_{ESP_OK};
#endif

#ifdef USE_ESP8266
  RemoteReceiverComponentStore store_;
  HighFrequencyLoopRequester high_freq_;
#endif

  // Kept for logging/compat
  uint32_t buffer_size_{10000};
  uint8_t filter_us_{10};
  uint32_t idle_us_{10000};
  uint8_t tolerance_{25};
};

}  // namespace remote_receiver
}  // namespace esphome
