#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <utility>

namespace esphome {
namespace tof_overdoor_ui {

struct UiResponse {
  int code{200};
  const char *content_type{"application/json"};
  std::string body;
  bool restart{false};
};

// Exactly one callback may be outstanding, including a timed-out callback
// which the main loop has not consumed yet. HTTP request pointers never enter
// this bridge. Both sides own the job until they have finished with it.
class MainThreadBridge {
 public:
  enum class State : unsigned char { WAITING, RUNNING, DONE, CANCELLED };
  struct Job {
    std::atomic<State> state{State::WAITING};
    UiResponse response;

    bool done() const { return state.load(std::memory_order_acquire) == State::DONE; }
    void cancel() {
      State expected = State::WAITING;
      state.compare_exchange_strong(expected, State::CANCELLED, std::memory_order_acq_rel);
    }
  };

  template<typename Schedule, typename Work>
  std::shared_ptr<Job> submit(Schedule schedule, Work work) {
    if (busy_.exchange(true, std::memory_order_acq_rel)) return nullptr;
    auto job = std::make_shared<Job>();
    schedule([this, job, work = std::move(work)]() mutable {
      State expected = State::WAITING;
      if (job->state.compare_exchange_strong(expected, State::RUNNING, std::memory_order_acq_rel)) {
        job->response = work();
        job->state.store(State::DONE, std::memory_order_release);
      }
      busy_.store(false, std::memory_order_release);
    });
    return job;
  }

 private:
  std::atomic<bool> busy_{false};
};

}  // namespace tof_overdoor_ui
}  // namespace esphome
