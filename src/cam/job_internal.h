#pragma once

#include "cam/job.h"
#include "cam/parametric.h"
#include "planning/routing.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace cam::detail {

using PlanningClock = std::chrono::steady_clock;
using Clipper2Lib::FillRule;
using Clipper2Lib::Union;

template <typename Function>
void parallel_for_levels(const std::vector<std::size_t>& levels,
                         Function&& function) {
  if (levels.empty()) return;
  std::atomic<std::size_t> next{0};
  const std::size_t workers = std::min<std::size_t>(4, levels.size());
  std::vector<std::thread> threads;
  threads.reserve(workers);
  for (std::size_t worker = 0; worker < workers; ++worker) {
    threads.emplace_back([&] {
      for (;;) {
        const std::size_t work = next.fetch_add(1, std::memory_order_relaxed);
        if (work >= levels.size()) return;
        function(levels[work]);
      }
    });
  }
  for (std::thread& thread : threads) thread.join();
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

}  // namespace cam::detail

namespace cam {

OperationPlan plan_subdivided_operation(
    const JobGeometry& geometry, const Operation& operation,
    const Tool& tool, const Machine& machine, const Material& material,
    const ProgressCallback& progress, double progress_from,
    double progress_to, double rapid_plane,
    const PriorOperations& prior_operations);

}  // namespace cam
