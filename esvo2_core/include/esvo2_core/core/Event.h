#ifndef ESVO2_CORE_CORE_EVENT_H
#define ESVO2_CORE_CORE_EVENT_H
#include <cstdint>
#include <esvo2_core/core/timestamp.h>

namespace esvo2_core {
struct Event {
  Event(uint16_t ax, uint16_t ay, timestamp_t tsa, uint8_t p)
      : x(ax), y(ay), ts(tsa), polarity(p) {}
  uint16_t x{0};
  uint16_t y{0};
  timestamp_t ts{0};
  uint8_t polarity{0};
};
} // namespace esvo2_core
#endif // ESVO2_CORE_CORE_EVENT_H