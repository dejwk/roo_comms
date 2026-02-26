#pragma once

#include <functional>

#include "roo_comms.h"
#include "roo_comms/transport/message_queue.h"
#include "roo_io.h"
#include "roo_io/net/mac_address.h"
#include "roo_scheduler.h"

namespace roo_comms {

/// Receiver that validates, queues, and processes incoming messages.
class Receiver {
 public:
  /// Raw message container.
  struct Message {
    roo_io::MacAddress source;
    size_t size;
    std::unique_ptr<roo_io::byte[]> data;
  };

  using ProcessorFn = std::function<void(const Message&)>;
  using ValidatorFn = std::function<bool(const roo_io::byte* data, size_t len)>;

  /// Creates a receiver with validation and queue limits.
  Receiver(roo_scheduler::Scheduler& scheduler, ProcessorFn processor_fn,
           size_t max_queue_size, size_t min_msg_size, size_t max_msg_size,
           ValidatorFn validator_fn);

  /// Handles an incoming frame and enqueues it if valid.
  void handle(const Source& source, const void* incoming_data, size_t len);

  /// Processes queued messages.
  void processMessages();

 private:
  MessageQueue<Message> queue_;
  ProcessorFn processor_fn_;
  ValidatorFn validator_fn_;
  roo_scheduler::SingletonTask processor_;
  size_t max_quqeue_size_;
  size_t min_msg_size_;
  size_t max_msg_size_;
};

}  // namespace roo_comms