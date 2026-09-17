#ifndef ESVO2_IMAGE_REPRESENTATION__EVENT_HANDLER_H_
#define ESVO2_IMAGE_REPRESENTATION__EVENT_HANDLER_H_

#include <esvo2_image_representation/Event.h>
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace esvo2_image_representation {
class EventHandler : public event_camera_codecs::EventProcessor {
public:
  using EventPacket = event_camera_msgs::msg::EventPacket;
  using EventPacketPtr = event_camera_msgs::msg::EventPacket::ConstSharedPtr;

  EventHandler(std::vector<Event> *events, uint16_t width, uint16_t height)
      : events_(events), width_(width), height_(height) {}
  virtual ~EventHandler() {}

  // ------ EventProcessor interface overrides ------
  void finished() override { is_first_in_message_ = true; }

  void eventCD(uint64_t ts, uint16_t ex, uint16_t ey,
               uint8_t polarity) override {
    if (ex < width_ && ey < height_) {
      events_->emplace_back(ts, ex, ey, polarity);
      if (events_->size() > 1) {
        int i = events_->size() - 2;
        if ((*events_)[i].ts > ts) {
          // sort the queue to ensure chronological order
          // move elements that are later in time towards end of queue to make
          // space for the new element
          for (; (*events_)[i].ts > ts && i >= 0; i--) {
            (*events_)[i + 1] = (*events_)[i];
          }
          (*events_)[i + 1] = Event(ts, ex, ey, polarity);
        }
      }
    }
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return (true); }
  void rawData(const char *, size_t) override {};

  // ------------ own methods
  void handleMessage(const EventPacketPtr &msg) {
    if (!decoder_) {
      event_camera_codecs::DecoderFactory<EventPacket, EventHandler>
          decoderFactory;
      decoder_ = decoderFactory.getInstance(*msg);
      if (!decoder_) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("event handler"),
                            "Failed to create decoder instance for "
                                << msg->encoding);
        return;
      }
    }
    decoder_->decode(*msg, this);
  }

private:
  // ------------- member variables -------------
  std::vector<Event> *events_{nullptr};
  uint16_t width_{0};
  uint16_t height_{0};
  bool is_first_in_message_{true};
  event_camera_codecs::Decoder<EventPacket, EventHandler> *decoder_{nullptr};
};

} // namespace esvo2_image_representation

#endif // ESVO2_IMAGE_REPRESENTATION__EVENT_HANDLER_H_