#include "components/tof_overdoor_ui/main_thread_bridge.h"
#include <cassert>
#include <functional>
#include <iostream>
#include <thread>
#include <vector>
using namespace esphome::tof_overdoor_ui;

int main() {
  MainThreadBridge bridge;
  std::vector<std::function<void()>> queue;
  auto schedule = [&](std::function<void()> callback) { queue.push_back(std::move(callback)); };
  int mutations = 0;
  auto work = [&]() { ++mutations; return UiResponse{200, "application/json", "{\"value\":1}"}; };
  auto first = bridge.submit(schedule, work);
  assert(first && !first->done() && mutations == 0 && queue.size() == 1);
  // A timed-out HTTP caller cannot keep adding scheduler jobs while main stalls.
  first->cancel();
  for (int i = 0; i < 10000; ++i) assert(!bridge.submit(schedule, work));
  assert(queue.size() == 1);
  queue.front()(); queue.clear();
  assert(mutations == 0 && !first->done());
  first.reset();
  auto second = bridge.submit(schedule, work);
  queue.front()(); queue.clear();
  assert(second->done() && second->response.body == "{\"value\":1}" && mutations == 1);
  // The HTTP side may disappear while work runs; callback owns the job.
  std::atomic<bool> running{false}, release{false};
  auto slow = bridge.submit(schedule, [&]() {
    running.store(true, std::memory_order_release);
    while (!release.load(std::memory_order_acquire)) std::this_thread::yield();
    return UiResponse{200, "text/plain", std::string(16000, 'a')};
  });
  std::thread main_loop([&]() { queue.front()(); });
  while (!running.load(std::memory_order_acquire)) std::this_thread::yield();
  slow->cancel();  // Already running: cannot revoke an action midway through it.
  slow.reset();
  assert(!bridge.submit(schedule, work));
  release.store(true, std::memory_order_release);
  main_loop.join(); queue.clear();
  // Repeated cross-thread publish/read exercises response ownership and ordering.
  for (int i = 0; i < 2000; ++i) {
    auto job = bridge.submit(schedule, [i]() { return UiResponse{200, "text/plain", std::to_string(i)}; });
    std::thread producer([&]() { queue.front()(); });
    while (!job->done()) std::this_thread::yield();
    assert(job->response.body == std::to_string(i));
    producer.join(); queue.clear();
  }
  std::cout << "UI bridge: bounded queue, cancellation, abandoned response and 2000 threaded snapshots passed\n";
}
