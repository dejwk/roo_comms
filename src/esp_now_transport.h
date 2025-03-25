#pragma once

#include "comms.pb.h"
#include "esp_now.h"
#include "message_queue.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "roo_io/base/byte.h"
#include "roo_io/memory/memory_output_iterator.h"
#include "roo_io/net/mac_address.h"
#include "roo_logging.h"
#include "roo_scheduler.h"

namespace roo_comms {

using Magic = roo_io::byte[8];

static constexpr roo_io::byte kMagicControlMsg[8] = {
    roo_io::byte{'r'},  roo_io::byte{'o'},  roo_io::byte{'o'},
    roo_io::byte{0},    roo_io::byte{0xE1}, roo_io::byte{0xB2},
    roo_io::byte{0x88}, roo_io::byte{0x99}};

static constexpr roo_io::byte kMagicDataMsg[8] = {
    roo_io::byte{'r'},  roo_io::byte{'o'},  roo_io::byte{'o'},
    roo_io::byte{0},    roo_io::byte{0x5E}, roo_io::byte{0x0C},
    roo_io::byte{0x15}, roo_io::byte{0x03}};

struct SerializedControlMessage {
  pb_byte_t data[8 + roo_comms_ControlMessage_size];
  size_t size;
};

SerializedControlMessage SerializeControlMessage(
    const roo_comms_ControlMessage& msg, const Magic& magic);

struct SerializedDataMessage {
  pb_byte_t data[8 + roo_comms_DataMessage_size];
  size_t size;
};

SerializedDataMessage SerializeDataMessage(const roo_comms_DataMessage& msg,
                                           const Magic& magic);

void SendEspNowMessage(const esp_now_peer_info_t& peer, const void* buf,
                       size_t len);

void SendEspNowControlMessage(const esp_now_peer_info_t& peer,
                              const Magic& magic,
                              const roo_comms_ControlMessage& msg);

struct ReceivedMessage {
  roo_io::MacAddress source;
  enum Type { kControl, kData } type;
  union {
    roo_comms_ControlMessage control_msg;
    roo_comms_DataMessage data_msg;
  };
};

class Receiver {
 public:
  using ProcessorFn = std::function<void(const ReceivedMessage&)>;

  Receiver(const Magic& control_magic, const Magic& data_magic,
           roo_scheduler::Scheduler& scheduler, ProcessorFn processor_fn)
      : queue_([this]() { processor_.scheduleNow(); }),
        processor_fn_(std::move(processor_fn)),
        processor_(scheduler, [this]() { processMessages(); }),
        control_magic_{},
        data_magic_{} {
    memcpy(control_magic_, control_magic, 8);
    memcpy(data_magic_, data_magic, 8);
  }

  void handle(const uint8_t* mac_addr, const uint8_t* incoming_data,
              size_t len) {
    if (len < 8) {
      LOG(WARNING) << "Received bogus message (too short); ignoring";
      return;
    }
    if (memcmp(incoming_data, control_magic_, 8) == 0) {
      roo_comms_ControlMessage msg = roo_comms_ControlMessage_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(incoming_data + 8, len - 8);
      bool status = pb_decode(&stream, roo_comms_ControlMessage_fields, &msg);
      if (!status) {
        LOG(ERROR) << "Received a malformed message " << PB_GET_ERROR(&stream);
        return;
      }
      queue_.push(ReceivedMessage{.source = roo_io::MacAddress(mac_addr),
                                  .type = ReceivedMessage::kControl,
                                  .control_msg = std::move(msg)});
    } else if (memcmp(incoming_data, data_magic_, 8) == 0) {
      roo_comms_DataMessage msg = roo_comms_DataMessage_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(incoming_data + 8, len - 8);
      bool status = pb_decode(&stream, roo_comms_DataMessage_fields, &msg);
      if (!status) {
        LOG(ERROR) << "Received a malformed message " << PB_GET_ERROR(&stream);
        return;
      }
      queue_.push(ReceivedMessage{.source = roo_io::MacAddress(mac_addr),
                                  .type = ReceivedMessage::kData,
                                  .data_msg = std::move(msg)});
    } else {
      LOG(WARNING) << "Received message with non-matching magic; ignoring";
      return;
    }
  }

  void processMessages() {
    ReceivedMessage msg;
    while (queue_.pop(msg)) {
      processor_fn_(msg);
    }
  }

 private:
  MessageQueue<ReceivedMessage> queue_;
  ProcessorFn processor_fn_;
  roo_scheduler::SingletonTask processor_;
  Magic control_magic_;
  Magic data_magic_;
};

}  // namespace roo_comms
