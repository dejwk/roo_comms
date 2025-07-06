#include "receiver.h"

#include "roo_logging.h"

#if !defined(MLOG_roo_esp_now_transport)
#define MLOG_roo_esp_now_transport 0
#endif

namespace roo_comms {

Receiver::Receiver(roo_scheduler::Scheduler& scheduler,
                   ProcessorFn processor_fn, size_t max_queue_size,
                   size_t min_msg_size, size_t max_msg_size,
                   ValidatorFn validator_fn)
    : queue_([this]() { processor_.scheduleNow(); }),
      processor_fn_(std::move(processor_fn)),
      validator_fn_(std::move(validator_fn)),
      processor_(scheduler, [this]() { processMessages(); }),
      max_quqeue_size_(max_queue_size),
      min_msg_size_(min_msg_size),
      max_msg_size_(max_msg_size) {}

void Receiver::handle(const Source& source, const void* incoming_data,
                      size_t len) {
  MLOG(roo_esp_now_transport)
      << "Received message of size " << len << " from " << source.addr
      << "; queue size: " << queue_.size();
  if (len < min_msg_size_) {
    LOG(WARNING) << "Received bogus message (too short: " << len
                 << " bytes); ignoring";
    return;
  }
  if (len > max_msg_size_) {
    LOG(WARNING) << "Received bogus message (too large: " << len
                 << " bytes); ignoring";
    return;
  }
  if (queue_.size() >= max_quqeue_size_) {
    LOG(WARNING) << "Received message, but queue is full; ignoring";
    return;
  }
  if (validator_fn_ != nullptr &&
      !validator_fn_((const roo::byte*)incoming_data, len)) {
    LOG(WARNING) << "Received message, but validation failed; ignoring";
    return;
  }
  std::unique_ptr<roo_io::byte[]> data(new roo::byte[len]);
  memcpy(data.get(), incoming_data, len);
  queue_.push(Message{.source = source.addr,
                      .size = len,
                      .data = std::move(data)});
}

void Receiver::processMessages() {
  Message msg;
  while (queue_.pop(msg)) {
    processor_fn_(msg);
  }
}

}  // namespace roo_comms