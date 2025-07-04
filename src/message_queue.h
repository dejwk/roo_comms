#pragma once

#include <queue>

#include "roo_scheduler.h"
#include "roo_threads/mutex.h"

namespace roo_comms {

template <typename Message>
class MessageQueue {
 public:
  MessageQueue(std::function<void()> nonempty_cb)
      : nonempty_cb_(std::move(nonempty_cb)) {}

  // Can be called from latency-sensitive contexts. Enqueues the message without
  // calling its processing code immediately.
  void push(Message msg) {
    bool was_empty = false;
    {
      roo::lock_guard<roo::mutex> lock(mutex_);
      was_empty = queue_.empty();
      queue_.push(std::move(msg));
    }
    if (was_empty) {
      nonempty_cb_();
    }
  }

  bool pop(Message &msg) {
    roo::lock_guard<roo::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    msg = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  size_t size() const { return queue_.size(); }

 private:
  mutable roo::mutex mutex_;
  std::queue<Message> queue_;
  std::function<void()> nonempty_cb_;
};

}  // namespace roo_comms
