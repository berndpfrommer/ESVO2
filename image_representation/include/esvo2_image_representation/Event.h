#ifndef ESVO2_IMAGE_REPRESENTATION__EVENT_HPP
#define ESVO2_IMAGE_REPRESENTATION__EVENT_HPP
#include <cstdint>
#include <esvo2_image_representation/timestamp.h>

namespace esvo2_image_representation {
struct Event {
  Event(uint16_t ax, uint16_t ay, timestamp_t tsa, uint8_t p)
      : x(ax), y(ay), ts(tsa), polarity(p) {}
  uint16_t x{0};
  uint16_t y{0};
  timestamp_t ts{0};
  uint8_t polarity{0};
};
} // namespace esvo2_image_representation
#endif // ESVO2_IMAGE_REPRESENTATION__EVENT_HPP