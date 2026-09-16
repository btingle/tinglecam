#pragma once

#include "cam/job.h"
#include "cam/parametric.h"
#include "planning/routing.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstdio>
#include <mutex>
#include <cmath>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace cam::detail {

using PlanningClock = std::chrono::steady_clock;
using Clipper2Lib::FillRule;
using Clipper2Lib::Union;

// Environment override is useful for reproducible serial/parallel benchmarks.
// The default remains bounded to eight geometry working sets.
inline std::size_t planning_worker_limit() {
  if (const char* value = std::getenv("TINGLECAM_PLANNING_WORKERS")) {
    char* end = nullptr;
    const long count = std::strtol(value, &end, 10);
    if (end != value && *end == '\0' && count >= 1 && count <= 64)
      return static_cast<std::size_t>(count);
  }
  return std::min(8u, std::max(1u, std::thread::hardware_concurrency()));
}

using LevelProgress = std::function<bool(double)>;

template <typename Function>
bool parallel_for_levels(const std::vector<std::size_t>& levels,
                         Function&& function,
                         const ProgressCallback& progress = {},
                         double from = 0.0, double to = 1.0,
                         const std::string& stage = {},
                         std::size_t worker_limit = planning_worker_limit()) {
  const auto batch_start = PlanningClock::now();
  const bool profile = levels.size() > 1 && std::getenv("TINGLECAM_PROFILE_WORKERS");
  std::vector<double> task_ms(profile ? levels.size() : 0);
  const double span = std::max(0.0, to - from);
  const std::size_t workers = std::min(std::max(std::size_t{1}, worker_limit),
                                       levels.size());
  // -1: queued, [0,1): running, 1: complete. Only each task's worker writes it.
  std::vector<std::atomic<double>> fractions(levels.size());
  for (auto& fraction : fractions) fraction.store(-1.0);
  std::atomic<bool> cancelled{false};
  const auto publish = [&] {
    if (cancelled.load()) return false;
    if (!progress) return true;
    PlanningProgress update(from);
    double completed = 0.0;
    for (const auto& fraction : fractions) {
      const double value = fraction.load(std::memory_order_relaxed);
      if (value < 0.0) continue;
      completed += value;
      if (value >= 1.0) ++update.completed_levels;
      else ++update.active_workers;
    }
    update.total_levels = levels.size();
    if (!levels.empty()) {
      update.fraction += span * completed / levels.size();
      update.active_until += span *
          (update.completed_levels + update.active_workers) / levels.size();
    } else {
      update.fraction = update.active_until = to;
    }
    if (!progress(update, stage)) cancelled.store(true);
    return !cancelled.load();
  };
  if (!publish()) return false;
  if (levels.empty()) return true;

  const auto print_profile = [&] {
    if (!profile) return;
    double sum = 0, longest = 0;
    for (double ms : task_ms) { sum += ms; longest = std::max(longest, ms); }
    const double wall = std::chrono::duration<double, std::milli>(
        PlanningClock::now() - batch_start).count();
    std::fprintf(stderr, "\nparallel_profile stage=\"%s\" levels=%zu workers=%zu wall_ms=%.3f work_ms=%.3f longest_ms=%.3f tasks_ms=",
                 stage.c_str(), levels.size(), workers, wall, sum, longest);
    for (double ms : task_ms) std::fprintf(stderr, "%.3f,", ms);
    std::fprintf(stderr, "\n");
  };

  // Serial work runs directly on the coordinator, with the same progress
  // events and no thread-launch cost. Workers never call the UI callback.
  if (workers == 1) {
    auto last_report = PlanningClock::now();
    for (std::size_t work = 0; work < levels.size(); ++work) {
      fractions[work].store(0.0);
      if (!publish()) return false;
      const LevelProgress report = [&](double value) {
        fractions[work].store(std::clamp(value, 0.0, 0.999999));
        const auto now = PlanningClock::now();
        if (now - last_report >= std::chrono::milliseconds(50)) {
          last_report = now;
          publish();
        }
        return !cancelled.load();
      };
      const auto start = PlanningClock::now();
      function(levels[work], report);
      if (profile) task_ms[work] = std::chrono::duration<double, std::milli>(PlanningClock::now() - start).count();
      if (cancelled.load()) return false;
      fractions[work].store(1.0);
      if (!publish()) return false;
    }
    print_profile();
    return true;
  }

  std::atomic<std::size_t> next{0}, finished{0};
  std::vector<std::exception_ptr> errors(levels.size());
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<std::jthread> threads;
  threads.reserve(workers);
  try {
    for (std::size_t worker = 0; worker < workers; ++worker) {
      threads.emplace_back([&] {
        while (!cancelled.load()) {
          const std::size_t work = next.fetch_add(1);
          if (work >= levels.size()) break;
          fractions[work].store(0.0);
          const LevelProgress report = [&](double value) {
            fractions[work].store(std::clamp(value, 0.0, 0.999999),
                                  std::memory_order_relaxed);
            return !cancelled.load();
          };
          const auto start = PlanningClock::now();
          try { function(levels[work], report); }
          catch (...) { errors[work] = std::current_exception(); }
          if (profile) task_ms[work] = std::chrono::duration<double, std::milli>(PlanningClock::now() - start).count();
          fractions[work].store(1.0);
          changed.notify_one();
        }
        ++finished;
        changed.notify_one();
      });
    }
    while (finished.load() < workers) {
      publish();
      std::unique_lock lock(mutex);
      changed.wait_for(lock, std::chrono::milliseconds(50),
                       [&] { return finished.load() == workers; });
    }
  } catch (...) {
    cancelled.store(true);
    throw; // jthreads join before the captured state goes out of scope.
  }
  for (auto& thread : threads) thread.join();
  if (cancelled.load()) return false;
  for (const auto& error : errors) if (error) std::rethrow_exception(error);
  print_profile();
  return publish();
}

double elapsed_milliseconds(PlanningClock::time_point start);
wave::Paths64 offset_outward(const wave::Paths64& input, double amount);
wave::Paths64 absent_subject(const wave::Paths64& stock);
void accumulate(PlanStats& into, const PlanStats& from);
void accumulate(PlanningMetrics& into, const PlanningMetrics& from);
double distance_mm(const wave::Point64& a, const wave::Point64& b);
double link_slack(double stepover);
double subdivision_cleanup_reach(const Operation& operation, double radius,
                                 bool finishing_subdivision);
wave::Paths64 open_space(const wave::Paths64& stock_slice,
                         const wave::Paths64& keep_out);

struct OperationSetup {
  double full_engagement_feed = 0.0;
  double no_engagement_feed = 0.0;
  double radius = 0.0;
  double stepover = 0.0;
  double first_bite_allowance = 0.0;
  double stepdown = 0.0;
};

bool resolve_setup(const Operation& operation, const Tool& tool,
                   const Machine& machine, const Material& material,
                   std::string& error, std::vector<std::string>* warnings,
                   OperationSetup& setup);
bool resolve_floors(const Operation& operation, const OperationSetup& setup,
                    double work_top, std::string& error,
                    std::vector<double>& floors);
void finish_job(JobPlan& plan);
wave::Paths64 stock_after_operation_at_z(
    const wave::Paths64& nominal_stock, double z,
    const OperationPlan& operation);
void append_inter_layer(wave::InterLayerTopology& into,
                        wave::InterLayerTopology from,
                        std::size_t layer_offset);
void stitch_passes(OperationPlan& plan, const OperationSetup& setup,
                   const Machine& machine, double rapid_plane);
void link_roughing_levels(OperationPlan& plan, const JobGeometry& geometry,
    const Operation& operation, const OperationSetup& setup,
    const Machine& machine, double rapid_plane, const ProgressCallback& progress,
    double progress_from, double progress_to);

}  // namespace cam::detail

namespace cam {

OperationPlan plan_subdivided_operation(
    const JobGeometry& geometry, const Operation& operation,
    const Tool& tool, const Machine& machine, const Material& material,
    const ProgressCallback& progress, double progress_from,
    double progress_to, double rapid_plane,
    const PriorOperations& prior_operations);

}  // namespace cam
