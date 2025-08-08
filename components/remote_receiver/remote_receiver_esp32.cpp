#include "remote_receiver.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32

namespace esphome {
namespace remote_receiver {

static const char *const TAG = "remote_receiver.esp32";

void IRAM_ATTR HOT RemoteReceiverComponentStore::gpio_intr(RemoteReceiverComponentStore *arg) {
  const uint32_t now = micros();
  // Next index (wrap)
  const uint32_t next = (arg->buffer_write_at + 1) % arg->buffer_size;
  const bool level = arg->pin.digital_read();

  // We encode: even index => falling, odd index => rising.
  // If level doesn't match the next index parity, ignore (spurious)
  if (level != (next % 2))
    return;

  // Overflow guard
  if (next == arg->buffer_read_at)
    return;

  const uint32_t last_change = arg->buffer[arg->buffer_write_at];
  const uint32_t dt = now - last_change;
  if (dt <= arg->filter_us)
    return;

  arg->buffer[arg->buffer_write_at = next] = now;
}

void RemoteReceiverComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Remote Receiver (GPIO-ISR, no RMT)...");
  this->pin_->setup();

  auto &s = this->store_;
  s.filter_us = this->filter_us_;
  s.pin = this->pin_->to_isr();
  s.buffer_size = this->buffer_size_;

  this->high_freq_.start();

  if (s.buffer_size % 2 != 0) {
    // Make divisible by two (even indices = spaces, odd = marks)
    s.buffer_size++;
  }

  s.buffer = new uint32_t[s.buffer_size];
  memset((void *) s.buffer, 0, s.buffer_size * sizeof(uint32_t));

  // First index depends on current level; we want index parity to reflect level
  if (this->pin_->digital_read()) {
    s.buffer_write_at = s.buffer_read_at = 1;  // rising/high at start -> odd
  } else {
    s.buffer_write_at = s.buffer_read_at = 0;  // low at start -> even
  }

  this->pin_->attach_interrupt(RemoteReceiverComponentStore::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
}

void RemoteReceiverComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Remote Receiver:");
  LOG_PIN("  Pin: ", this->pin_);
  if (this->pin_->digital_read()) {
    ESP_LOGW(TAG, "Signal starts HIGH. If decoding seems inverted, use 'inverted: true' on the pin.");
  }
  ESP_LOGCONFIG(TAG, "  Buffer Size: %u", this->buffer_size_);
  ESP_LOGCONFIG(TAG, "  Tolerance: %u%%", this->tolerance_);
  ESP_LOGCONFIG(TAG, "  Filter out pulses shorter than: %u us", this->filter_us_);
  ESP_LOGCONFIG(TAG, "  Signal is done after %u us of no changes", this->idle_us_);
}

void RemoteReceiverComponent::loop() {
  auto &s = this->store_;

  // Snapshot write pointer (volatile)
  const uint32_t write_at = s.buffer_write_at;
  const uint32_t dist = (s.buffer_size + write_at - s.buffer_read_at) % s.buffer_size;

  // Need at least one rising & one falling edge
  if (dist <= 1)
    return;

  const uint32_t now = micros();
  if (now - s.buffer[write_at] < this->idle_us_) {
    // Not idle long enough yet
    return;
  }

  ESP_LOGVV(TAG, "read_at=%u write_at=%u dist=%u now=%u end=%u",
            s.buffer_read_at, write_at, dist, now, s.buffer[write_at]);

  // Skip first value (belongs to previous idle level)
  s.buffer_read_at = (s.buffer_read_at + 1) % s.buffer_size;
  uint32_t prev = s.buffer_read_at;
  s.buffer_read_at = (s.buffer_read_at + 1) % s.buffer_size;

  const uint32_t reserve = 1 + (s.buffer_size + write_at - s.buffer_read_at) % s.buffer_size;
  this->temp_.clear();
  this->temp_.reserve(reserve);

  // Determine sign based on starting index parity (even=space -> positive; odd=mark -> negative)
  int32_t multiplier = (s.buffer_read_at % 2 == 0) ? 1 : -1;

  for (uint32_t i = 0; prev != write_at; i++) {
    int32_t delta = s.buffer[s.buffer_read_at] - s.buffer[prev];
    if (uint32_t(delta) >= this->idle_us_) {
      // Long space => frame ended already
      break;
    }

    ESP_LOGVV(TAG, "  i=%u buffer[%u]=%u - buffer[%u]=%u -> %d",
              i, s.buffer_read_at, s.buffer[s.buffer_read_at], prev, s.buffer[prev], multiplier * delta);

    this->temp_.push_back(multiplier * delta);
    prev = s.buffer_read_at;
    s.buffer_read_at = (s.buffer_read_at + 1) % s.buffer_size;
    multiplier *= -1;
  }

  // Step one back and append idle separator
  s.buffer_read_at = (s.buffer_size + s.buffer_read_at - 1) % s.buffer_size;
  this->temp_.push_back(this->idle_us_ * multiplier);

  // Notify protocol dumpers/listeners
  this->call_listeners_dumpers_();
}

}  // namespace remote_receiver
}  // namespace esphome

#endif  // USE_ESP32
