#pragma once

#include "planning/toolpath.h"

#include <cstddef>
#include <vector>

namespace wave {

// One normalized straight portion of an engaged toolpath move. Load is a
// dimensionless
// approximation: newly removed area divided by the area of a full-width slot
// over the same distance. A value of 1 is therefore approximately full radial
// engagement, while values above 1 can occur at entries and tight corners.
struct MaterialStep {
  Point64 from;
  Point64 to;
  double move_distance_begin = 0.0;
  double move_distance_end = 0.0;
  double removed_area = 0.0;
  double gouge_area = 0.0;
  double load = 0.0;
};

struct SimulatedMove {
  std::vector<MaterialStep> steps;
  Paths64 plunge_collision;
  double plunge_collision_area = 0.0;
  bool entry_unresolved = false;
};

struct SimulationCheckpoint {
  // Number of complete toolpath moves already reflected in remaining_stock.
  std::size_t moves_completed = 0;
  Paths64 remaining_stock;
};

struct SimulationStats {
  double initial_stock_area = 0.0;
  double removed_area = 0.0;
  double remaining_area = 0.0;
  double excess_area = 0.0;
  double gouge_area = 0.0;
  double maximum_load = 0.0;
  std::size_t cutting_steps = 0;
  std::size_t empty_cutting_steps = 0;
  std::size_t gouge_bounds_rejections = 0;
  std::size_t gouging_steps = 0;
  std::size_t unsafe_plunges = 0;
  std::size_t unresolved_entries = 0;
};

struct SimulationResult {
  Paths64 initial_stock;
  Paths64 final_subject;
  Paths64 remaining_stock;
  Paths64 removed_material;
  Paths64 gouged_subject;
  Paths64 excess_material;
  std::vector<SimulatedMove> moves;
  // Periodic full-stock snapshots. Playback reconstructs the few completed
  // moves after the nearest checkpoint from their stored step endpoints.
  std::vector<SimulationCheckpoint> checkpoints;
  double tool_radius = 0.0;
  double stock_inflation = 0.0;
  SimulationStats stats;
};

// Sweeps the circular tool through Cut, Step and Link moves. Retracts are above
// the work and are checked only for a stock collision at their destination.
// Entry moves cannot be subtracted until the planner supplies their real ramp
// or helix, so they are reported as unresolved.
SimulationResult simulate_material(const Paths64& raw_stock,
                                   const Paths64& raw_final_subject,
                                   const Toolpath& toolpath,
                                   double tool_radius,
                                   double simulation_stock_inflation = 0.0);

// Remaining stock at a distance along the toolpath. This is intended for UI
// playback; distances outside the path are clamped in the same way as
// toolpath_at().
Paths64 simulation_stock_at(const SimulationResult& simulation,
                            const Toolpath& toolpath,
                            double distance);

// Non-zero polygon area in world units squared. Input is unioned first so
// overlaps and holes are accounted for consistently.
double material_area(const Paths64& paths);

}  // namespace wave
