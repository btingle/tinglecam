#include "cam/profiling.h"

#include <cstdio>
#include <fstream>
#include <unordered_set>
#ifdef __GLIBC__
#include <malloc.h>
#endif
namespace cam {
namespace {
std::size_t paths_bytes(const wave::Paths64& paths) {
  std::size_t n = paths.capacity() * sizeof(wave::Path64);
  for (const auto& path : paths) n += path.capacity() * sizeof(wave::Point64);
  return n;
}
std::size_t wave_bytes(const wave::WaveResult& w) {
  std::size_t n = paths_bytes(w.subject) + paths_bytes(w.stock) +
                  paths_bytes(w.domain) + paths_bytes(w.static_boundaries) +
                  w.stock_boundary.capacity() * sizeof(wave::Point64);
  for (const auto* levels : {&w.generations, &w.reached_generations})
    for (const auto& paths : *levels) n += paths_bytes(paths);
  return n;
}
std::size_t lineage_bytes(const wave::FrontLineage& lineage) {
  std::size_t n = paths_bytes(lineage.boundary_components);
  for (const auto& generation : lineage.generations) {
    n += generation.fronts.capacity() * sizeof(wave::LeadingFront);
    for (const auto& front : generation.fronts)
      n += front.path.capacity() * sizeof(wave::Point64) +
           (front.parents.capacity() + front.children.capacity()) *
               sizeof(std::size_t);
    for (const auto& band : generation.bands) n += paths_bytes(band);
  }
  return n;
}
}  // namespace
void release_planning_scratch() {
#ifdef __GLIBC__
  // Clipper workers leave large freed arenas behind after a plan. Return idle
  // pages at the operation-range boundary, while preserving all live payloads.
  if (mallinfo2().fordblks > 64 * 1024 * 1024) malloc_trim(0);
#endif
}
void report_plan_memory(const JobPlan& plan) {
  std::vector<const wave::Heightmap*> maps;
  std::size_t motion = 0, samples = 0, geometry = 0, polygons = 0,
              checkpoints = 0;
  std::unordered_set<const wave::Paths64*> shared_polygons;
  for (const auto& op : plan.operations) {
    maps.push_back(op.stock_before.get());
    maps.push_back(op.stock_after.get());
    for (const auto& slab : op.preview_context)
      if (shared_polygons.insert(slab.region.get()).second)
        polygons += paths_bytes(*slab.region);
    for (const auto& level : op.levels) {
      for (const auto& checkpoint : level.stock_checkpoints)
        maps.push_back(checkpoint.stock.get());
      for (const auto& checkpoint : level.simulation.checkpoints)
        maps.push_back(checkpoint.heightmap.get());
      maps.push_back(level.simulation.heightmap_before.get());
      maps.push_back(level.simulation.heightmap_after.get());
      checkpoints += level.stock_checkpoints.size();
      motion += level.feeds.motion.capacity() * sizeof(MotionPhase) +
                level.feeds.segments.capacity() * sizeof(FeedSegment);
      samples +=
          level.simulation.moves.capacity() * sizeof(wave::SimulatedMove);
      for (const auto& move : level.simulation.moves)
        samples += move.steps.capacity() * sizeof(wave::MaterialStep);
      geometry += wave_bytes(level.wave) + lineage_bytes(level.lineage);
      for (const auto& stage : level.wave_stages)
        geometry += wave_bytes(stage.wave) + lineage_bytes(stage.lineage);
      geometry += level.toolpath.moves.capacity() * sizeof(wave::ToolpathMove);
      for (const auto& move : level.toolpath.moves)
        geometry += move.path.capacity() * sizeof(wave::Point64);
      for (const auto* paths :
           {&level.part_slice, &level.stock_slice, &level.incoming_stock,
            &level.simulation.initial_stock, &level.simulation.final_subject,
            &level.simulation.remaining_stock,
            &level.simulation.removed_material,
            &level.simulation.gouged_subject,
            &level.simulation.excess_material})
        polygons += paths_bytes(*paths);
    }
  }
  const auto memory = wave::Heightmap::memory_usage(maps);
  std::fprintf(
      stderr,
      "memory_profile heightmap_bytes=%zu maps=%zu pages=%zu tiles=%zu "
      "height_arrays=%zu checkpoints=%zu motion_bytes=%zu "
      "simulation_sample_bytes=%zu wave_path_bytes=%zu polygon_bytes=%zu\n",
      memory.bytes, memory.maps, memory.pages, memory.tiles,
      memory.height_arrays, checkpoints, motion, samples, geometry, polygons);
#ifdef __GLIBC__
  const auto heap = mallinfo2();
  std::fprintf(stderr,
               "allocator_memory live_heap_bytes=%zu free_heap_bytes=%zu "
               "mapped_bytes=%zu\n",
               std::size_t(heap.uordblks), std::size_t(heap.fordblks),
               std::size_t(heap.hblkhd));
#endif
#ifdef __linux__
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line))
    if (line.starts_with("VmRSS:") || line.starts_with("VmSize:") ||
        line.starts_with("VmHWM:"))
      std::fprintf(stderr, "process_memory %s\n", line.c_str());
#endif
}
}  // namespace cam
