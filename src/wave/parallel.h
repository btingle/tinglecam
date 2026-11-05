#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace wave {

// TINGLECAM_PLANNING_WORKERS caps geometry threads, as it does for planning.
inline std::size_t geometry_thread_limit() {
  static const std::size_t limit = [] {
    if (const char* value = std::getenv("TINGLECAM_PLANNING_WORKERS")) {
      char* end = nullptr;
      const long count = std::strtol(value, &end, 10);
      if (end != value && *end == '\0' && count >= 1 && count <= 64)
        return static_cast<std::size_t>(count);
    }
    return std::size_t{std::min(8u, std::max(1u, std::thread::hardware_concurrency()))};
  }();
  return limit;
}

// Runs body(i) for every i in [begin, end). Bodies must only write state owned
// by their own index. The first exception is rethrown once all workers stop.
template <typename Body>
void parallel_for(std::size_t begin, std::size_t end, Body&& body) {
  const std::size_t count = end > begin ? end - begin : 0;
  const std::size_t threads = std::min(geometry_thread_limit(), count);
  if (threads <= 1) {
    for (std::size_t i = begin; i < end; ++i) body(i);
    return;
  }
  std::atomic<std::size_t> next{begin};
  std::exception_ptr error;
  std::mutex error_mutex;
  const auto work = [&] {
    for (std::size_t i; (i = next++) < end;) {
      try {
        body(i);
      } catch (...) {
        std::lock_guard lock(error_mutex);
        if (!error) error = std::current_exception();
        next = end;
      }
    }
  };
  std::vector<std::thread> workers;
  workers.reserve(threads - 1);
  for (std::size_t t = 1; t < threads; ++t) workers.emplace_back(work);
  work();
  for (auto& worker : workers) worker.join();
  if (error) std::rethrow_exception(error);
}

}  // namespace wave
