#include "analysis.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace wave {
namespace {

using namespace Clipper2Lib;

struct Rollout {
  WaveResult wave;
  FrontLineage lineage;
  LayerTopology topology;
};

Rollout rollout(const Paths64& domain, const std::vector<WaveSeed>& seeds,
                double stepover, double softening,
                std::size_t max_iterations,
                const Paths64& static_boundaries,
                const Paths64& cached_reached,
                double tiny_split_area_factor) {
  Rollout result;
  result.wave = build_wavefront_from_seeds(
      domain, seeds, stepover, softening, max_iterations, static_boundaries,
      cached_reached);
  result.lineage = build_front_lineage(result.wave, tiny_split_area_factor);
  result.topology = build_layer_topology(result.lineage, false);
  return result;
}

WaveSeed seed_for_front(const FrontLineage& lineage, FrontId2D id) {
  WaveSeed result;
  if (id.generation >= lineage.generations.size()) return result;
  const FrontGeneration& generation = lineage.generations[id.generation];
  if (id.index >= generation.fronts.size()) return result;
  const LeadingFront& front = generation.fronts[id.index];
  result.path = front.path;
  result.closed = front.closed;
  if (front.band < generation.bands.size())
    result.initial_region = generation.bands[front.band];
  return result;
}

std::vector<WaveSeed> entries_at(const FrontLineage& lineage,
                                 const LayerTopology& topology,
                                 std::size_t evaluation_order) {
  std::vector<WaveSeed> result;
  for (const NodeEntryGroup& group :
       group_node_entries(topology, evaluation_order)) {
    for (const NodeEntry& entry : group.entries) {
      if (entry.front.generation >= lineage.generations.size() ||
          entry.front.index >=
              lineage.generations[entry.front.generation].fronts.size())
        continue;
      result.push_back(seed_for_front(lineage, entry.front));
    }
  }
  return result;
}

std::vector<WaveSeed> node_entries_at(
    const FrontLineage& lineage, const LayerTopology& topology,
    std::size_t evaluation_order, std::vector<std::size_t>* node_ids = nullptr) {
  std::vector<WaveSeed> result;
  if (node_ids) node_ids->clear();
  for (std::size_t node = 0; node < topology.nodes.size(); ++node) {
    const FrontNode& item = topology.nodes[node];
    if (item.kind != FrontNode::Kind::Propagation ||
        item.evaluation_order != evaluation_order || item.fronts.empty())
      continue;
    const FrontId2D id = item.fronts.front();
    if (id.generation >= lineage.generations.size() ||
        id.index >= lineage.generations[id.generation].fronts.size())
      continue;
    result.push_back(seed_for_front(lineage, id));
    if (node_ids) node_ids->push_back(node);
  }
  return result;
}

Paths64 root_bands(const FrontLineage& lineage,
                   const LayerTopology& topology) {
  Paths64 result;
  for (const FrontNode& node : topology.nodes) {
    if (node.kind != FrontNode::Kind::Propagation ||
        node.evaluation_order != 0)
      continue;
    for (const FrontId2D id : node.fronts) {
      if (id.generation >= lineage.generations.size()) continue;
      const FrontGeneration& generation = lineage.generations[id.generation];
      if (id.index >= generation.fronts.size()) continue;
      const std::size_t band = generation.fronts[id.index].band;
      if (band >= generation.bands.size()) continue;
      result.insert(result.end(), generation.bands[band].begin(),
                    generation.bands[band].end());
    }
  }
  return Union(result, FillRule::NonZero);
}

std::vector<std::size_t> all_indices(std::size_t count) {
  std::vector<std::size_t> result(count);
  std::iota(result.begin(), result.end(), 0);
  return result;
}

double seed_length(const WaveSeed& seed) {
  LeadingFront front;
  front.path = seed.path;
  front.closed = seed.closed;
  return front_length(front);
}

// Analysis rollouts deliberately omit the comparatively expensive exact node
// footprint reconstruction. Their wave bands already partition the newly
// reached 2D region by generation, so summing those areas gives a cheap and
// stable size estimate. A connected band can expose fronts belonging to more
// than one node at an event; divide that band's area in proportion to each
// node's front length rather than crediting the whole band more than once.
std::vector<double> approximate_node_areas(const FrontLineage& lineage,
                                           const LayerTopology& topology) {
  std::vector<double> result(topology.nodes.size(), 0.0);
  const std::size_t generation_count = std::min(
      lineage.generations.size(), topology.front_nodes.size());
  for (std::size_t generation = 0; generation < generation_count;
       ++generation) {
    const FrontGeneration& fronts = lineage.generations[generation];
    const std::vector<std::size_t>& front_nodes =
        topology.front_nodes[generation];
    std::vector<std::vector<std::pair<std::size_t, double>>> owners(
        fronts.bands.size());

    const std::size_t front_count =
        std::min(fronts.fronts.size(), front_nodes.size());
    for (std::size_t front_index = 0; front_index < front_count;
         ++front_index) {
      const LeadingFront& front = fronts.fronts[front_index];
      const std::size_t node = front_nodes[front_index];
      if (front.band >= owners.size() || node >= result.size()) continue;
      const double length = std::max(front_length(front), 0.0);
      auto& band_owners = owners[front.band];
      const auto existing = std::find_if(
          band_owners.begin(), band_owners.end(),
          [node](const auto& owner) { return owner.first == node; });
      if (existing == band_owners.end())
        band_owners.emplace_back(node, length);
      else
        existing->second += length;
    }

    for (std::size_t band = 0; band < fronts.bands.size(); ++band) {
      const auto& band_owners = owners[band];
      if (band_owners.empty()) continue;
      const double band_area =
          std::abs(Area(fronts.bands[band])) /
          (kIntegerScale * kIntegerScale);
      const double total_length = std::accumulate(
          band_owners.begin(), band_owners.end(), 0.0,
          [](double total, const auto& owner) { return total + owner.second; });
      if (total_length > 0.0) {
        for (const auto& [node, length] : band_owners)
          result[node] += band_area * length / total_length;
      } else {
        const double share = band_area / band_owners.size();
        for (const auto& [node, unused] : band_owners) {
          (void)unused;
          result[node] += share;
        }
      }
    }
  }
  return result;
}

std::size_t significant_node_count(const std::vector<double>& areas,
                                   double minimum_area,
                                   double area_fraction) {
  if (area_fraction <= 0.0) return areas.size();
  return std::count_if(areas.begin(), areas.end(),
                       [minimum_area](double area) {
                         return area >= minimum_area;
                       });
}

Paths64 cached_regions(const std::vector<WaveSeed>& entries) {
  Paths64 result;
  for (const WaveSeed& entry : entries)
    result.insert(result.end(), entry.initial_region.begin(),
                  entry.initial_region.end());
  return Union(result, FillRule::NonZero);
}

}  // namespace

WavefrontAnalysis analyze_wavefront(
    const Paths64& raw_subject, const Paths64& raw_stock, double stepover,
    double softening, double lead_in, std::size_t max_iterations,
    std::size_t max_layers, const EntryMute& muted,
    double significant_node_area_fraction,
    double tiny_split_area_factor,
    double exterior_stock_reach) {
  if (max_layers == 0)
    throw std::invalid_argument("Wavefront analysis layer limit must be positive");
  if (!std::isfinite(significant_node_area_fraction) ||
      significant_node_area_fraction < 0.0 ||
      significant_node_area_fraction > 1.0)
    throw std::invalid_argument(
        "Significant node area fraction must be between zero and one");
  if (!std::isfinite(tiny_split_area_factor) || tiny_split_area_factor < 0.0)
    throw std::invalid_argument("Tiny split area factor must be nonnegative");

  WavefrontAnalysis analysis;
  analysis.seed = seed_wavefront(raw_subject, raw_stock, stepover, softening,
                                 lead_in, muted, exterior_stock_reach);
  analysis.seed_lineage =
      build_front_lineage(analysis.seed, tiny_split_area_factor);
  if (analysis.seed.generations.empty() ||
      analysis.seed_lineage.entry_fronts.empty())
    return analysis;

  const Paths64 free_space = Difference(
      analysis.seed.domain, analysis.seed.subject, FillRule::NonZero);
  Paths64 domain = free_space;
  std::vector<WaveSeed> seeds;
  seeds.reserve(analysis.seed_lineage.entry_fronts.size());
  for (std::size_t front = 0;
       front < analysis.seed_lineage.generations.front().fronts.size(); ++front)
    seeds.push_back(seed_for_front(analysis.seed_lineage, {0, front}));

  for (std::size_t layer_index = 0;
       layer_index < max_layers && !domain.empty() && !seeds.empty();
       ++layer_index) {
    WavefrontDecisionLayer layer;
    layer.layer = layer_index;
    layer.remaining_domain = domain;
    const Paths64 cached_reached = cached_regions(seeds);
    layer.cached_reached = cached_reached;

    Rollout baseline = rollout(domain, seeds, stepover, softening,
                               max_iterations, free_space, cached_reached,
                               tiny_split_area_factor);
    std::vector<std::size_t> entry_nodes;
    layer.entries = node_entries_at(baseline.lineage, baseline.topology, 0,
                                    &entry_nodes);
    if (layer.entries.empty()) break;
    layer.groups = group_node_entries(baseline.topology, 0);
    layer.baseline_wave = baseline.wave;
    layer.baseline_lineage = baseline.lineage;
    layer.baseline_topology = baseline.topology;

    Rollout selected = baseline;
    layer.retained_entries = all_indices(layer.entries.size());

    // Node IDs are stable only inside the baseline graph. Map them once to the
    // explicit entry array used for every candidate at this layer.
    std::vector<std::size_t> node_to_entry(baseline.topology.nodes.size(),
                                           kNoTopologyId);
    for (std::size_t entry = 0; entry < entry_nodes.size(); ++entry)
      node_to_entry[entry_nodes[entry]] = entry;

    std::vector<std::size_t> candidate_indices = layer.retained_entries;
    bool has_alternative = false;
    for (std::size_t group_index = 0; group_index < layer.groups.size();
         ++group_index) {
      const NodeEntryGroup& group = layer.groups[group_index];
      if (group.entries.size() < 2) continue;
      std::vector<std::size_t> group_entries;
      for (const NodeEntry& entry : group.entries)
        if (entry.node < node_to_entry.size() &&
            node_to_entry[entry.node] != kNoTopologyId)
          group_entries.push_back(node_to_entry[entry.node]);
      if (group_entries.size() < 2) continue;

      const std::size_t longest = *std::max_element(
          group_entries.begin(), group_entries.end(), [&](std::size_t a,
                                                           std::size_t b) {
            return seed_length(layer.entries[a]) <
                   seed_length(layer.entries[b]);
          });
      has_alternative = true;
      candidate_indices.erase(
          std::remove_if(candidate_indices.begin(), candidate_indices.end(),
                         [&](std::size_t entry) {
                           return entry != longest &&
                                  std::find(group_entries.begin(),
                                            group_entries.end(), entry) !=
                                      group_entries.end();
                         }),
          candidate_indices.end());
    }

    // Every abstract volume makes the same deliberately small choice in one
    // rollout: retain all of its entries, or retain only its longest. This
    // keeps each recursive layer to one baseline and at most one candidate,
    // rather than multiplying full polygon propagations by the group count.
    if (has_alternative) {
      std::vector<WaveSeed> candidate_seeds;
      candidate_seeds.reserve(candidate_indices.size());
      for (const std::size_t entry : candidate_indices)
        candidate_seeds.push_back(layer.entries[entry]);

      Rollout candidate = rollout(domain, candidate_seeds, stepover, softening,
                                  max_iterations, free_space, cached_reached,
                                  tiny_split_area_factor);
      WavefrontStrategyTrial trial;
      trial.retained_entries = candidate_indices;
      trial.previous_node_count = selected.topology.nodes.size();
      trial.candidate_node_count = candidate.topology.nodes.size();
      trial.previous_node_areas =
          approximate_node_areas(selected.lineage, selected.topology);
      trial.candidate_node_areas =
          approximate_node_areas(candidate.lineage, candidate.topology);
      if (!trial.previous_node_areas.empty())
        trial.reference_node_area = *std::max_element(
            trial.previous_node_areas.begin(), trial.previous_node_areas.end());
      trial.minimum_significant_node_area =
          trial.reference_node_area * significant_node_area_fraction;
      trial.previous_significant_node_count = significant_node_count(
          trial.previous_node_areas, trial.minimum_significant_node_area,
          significant_node_area_fraction);
      trial.candidate_significant_node_count = significant_node_count(
          trial.candidate_node_areas, trial.minimum_significant_node_area,
          significant_node_area_fraction);
      trial.accepted = trial.candidate_significant_node_count <
                       trial.previous_significant_node_count;
      trial.wave = candidate.wave;
      trial.lineage = candidate.lineage;
      trial.topology = candidate.topology;
      if (trial.accepted) {
        layer.retained_entries = candidate_indices;
        selected = std::move(candidate);
      }
      layer.trials.push_back(std::move(trial));
    }

    layer.selected_wave = selected.wave;
    layer.selected_lineage = selected.lineage;
    layer.selected_topology = selected.topology;
    analysis.layers.push_back(std::move(layer));

    Paths64 cleared = root_bands(selected.lineage, selected.topology);
    cleared.insert(cleared.end(), cached_reached.begin(), cached_reached.end());
    cleared = Union(cleared, FillRule::NonZero);
    if (cleared.empty()) break;
    Paths64 next_domain = Difference(domain, cleared, FillRule::NonZero);
    std::vector<WaveSeed> next_seeds =
        entries_at(selected.lineage, selected.topology, 1);
    if (next_domain.empty() || next_seeds.empty()) break;
    if (paths_equal_canonical(next_domain, domain)) break;
    domain = std::move(next_domain);
    seeds = std::move(next_seeds);
  }
  return analysis;
}

std::vector<WavefrontStage> committed_wavefront_stages(
    const WavefrontAnalysis& analysis) {
  std::vector<WavefrontStage> result;
  result.reserve(analysis.layers.size());

  for (const WavefrontDecisionLayer& decision : analysis.layers) {
    const FrontLineage& source = decision.selected_lineage;
    const LayerTopology& topology = decision.selected_topology;
    if (source.generations.empty() || topology.front_nodes.size() !=
                                          source.generations.size())
      throw std::runtime_error(
          "Selected wave layer cannot be committed: lineage/topology mismatch");

    std::size_t expected_front_count = 0;
    for (const FrontNode& node : topology.nodes)
      if (node.kind == FrontNode::Kind::Propagation &&
          node.evaluation_order == 0)
        expected_front_count += node.fronts.size();

    std::vector<std::vector<std::size_t>> remap(source.generations.size());
    FrontLineage lineage;
    lineage.stepover = source.stepover;
    lineage.boundary_components = source.boundary_components;
    lineage.generations.resize(source.generations.size());
    std::vector<std::vector<std::size_t>> band_remap(source.generations.size());

    for (std::size_t generation = 0; generation < source.generations.size();
         ++generation) {
      const FrontGeneration& from = source.generations[generation];
      FrontGeneration& into = lineage.generations[generation];
      remap[generation].assign(from.fronts.size(), kNoTopologyId);
      band_remap[generation].assign(from.bands.size(), kNoTopologyId);
      for (std::size_t front = 0; front < from.fronts.size(); ++front) {
        if (front >= topology.front_nodes[generation].size()) continue;
        const std::size_t node = topology.front_nodes[generation][front];
        if (node >= topology.nodes.size() ||
            topology.nodes[node].kind != FrontNode::Kind::Propagation ||
            topology.nodes[node].evaluation_order != 0)
          continue;
        remap[generation][front] = into.fronts.size();
        into.fronts.push_back(from.fronts[front]);
        LeadingFront& retained = into.fronts.back();
        if (retained.band >= from.bands.size())
          throw std::runtime_error(
              "Selected wave layer cannot be committed: invalid front band");
        std::size_t& new_band = band_remap[generation][retained.band];
        if (new_band == kNoTopologyId) {
          new_band = into.bands.size();
          into.bands.push_back(from.bands[retained.band]);
        }
        retained.band = new_band;
      }
    }

    for (std::size_t generation = 0; generation < lineage.generations.size();
         ++generation) {
      for (std::size_t front = 0;
           front < lineage.generations[generation].fronts.size(); ++front) {
        LeadingFront& item = lineage.generations[generation].fronts[front];
        std::vector<std::size_t> parents;
        if (generation > 0)
          for (const std::size_t parent : item.parents)
            if (parent < remap[generation - 1].size() &&
                remap[generation - 1][parent] != kNoTopologyId)
              parents.push_back(remap[generation - 1][parent]);
        std::vector<std::size_t> children;
        if (generation + 1 < remap.size())
          for (const std::size_t child : item.children)
            if (child < remap[generation + 1].size() &&
                remap[generation + 1][child] != kNoTopologyId)
              children.push_back(remap[generation + 1][child]);
        item.parents = std::move(parents);
        item.children = std::move(children);
      }
    }

    // A rejected split whose fronts all belong to this committed root layer
    // remains rejected after the generation/front indices are compacted.
    for (const LineageEvent& event : source.rejected_splits) {
      if (event.generation == 0 || event.generation >= remap.size()) continue;
      LineageEvent retained = event;
      retained.previous_fronts.clear();
      retained.current_fronts.clear();
      for (const std::size_t parent : event.previous_fronts)
        if (parent < remap[event.generation - 1].size() &&
            remap[event.generation - 1][parent] != kNoTopologyId)
          retained.previous_fronts.push_back(
              remap[event.generation - 1][parent]);
      for (const std::size_t child : event.current_fronts)
        if (child < remap[event.generation].size() &&
            remap[event.generation][child] != kNoTopologyId)
          retained.current_fronts.push_back(remap[event.generation][child]);
      if (retained.previous_fronts.size() == 1 &&
          retained.current_fronts.size() >= 2)
        lineage.rejected_splits.push_back(std::move(retained));
    }

    if (!lineage.generations.empty())
      lineage.entry_fronts = lineage.generations.front().fronts;
    while (lineage.generations.size() > 1 &&
           lineage.generations.back().fronts.empty())
      lineage.generations.pop_back();

    std::size_t front_count = 0;
    for (const FrontGeneration& generation : lineage.generations)
      front_count += generation.fronts.size();
    if (front_count == 0 || front_count != expected_front_count)
      throw std::runtime_error(
          "Selected wave layer cannot be committed: root fronts were lost");

    LayerTopology committed_topology;
    try {
      committed_topology = build_layer_topology(lineage, true);
    } catch (const std::runtime_error&) {
      // Some attachment transitions cannot reconstruct endpoint rails even
      // though their swept bands are valid. The bands remain an exact record
      // of what the committed node propagated through, so use their union as
      // the display/association footprint in that case.
      committed_topology = build_layer_topology(lineage, false);
      for (FrontNode& node : committed_topology.nodes) {
        Paths64 footprint;
        for (const FrontId2D id : node.fronts) {
          if (id.generation >= lineage.generations.size()) continue;
          const FrontGeneration& generation =
              lineage.generations[id.generation];
          if (id.index >= generation.fronts.size()) continue;
          const std::size_t band = generation.fronts[id.index].band;
          if (band >= generation.bands.size()) continue;
          footprint.insert(footprint.end(), generation.bands[band].begin(),
                           generation.bands[band].end());
        }
        node.footprint = Union(footprint, FillRule::NonZero);
      }
    }
    for (const FrontNode& node : committed_topology.nodes)
      if (node.footprint.empty())
        throw std::runtime_error(
            "Selected wave layer cannot be committed: node has no footprint");

    WaveResult wave;
    // Resumed propagation operates on an abstract remaining domain. Toolpath
    // routing still needs the physical stock and protected subject.
    wave.subject = analysis.seed.subject;
    wave.stock = analysis.seed.stock;
    wave.stock_boundary = analysis.seed.stock_boundary;
    wave.stepover = analysis.seed.stepover;
    wave.softening = analysis.seed.softening;
    wave.lead_in = analysis.seed.lead_in;
    wave.lead_in_generations =
        decision.layer == 0 ? analysis.seed.lead_in_generations : 0;
    wave.muted_entries = decision.layer == 0 ? analysis.seed.muted_entries : 0;
    // Front-owned bands are enough to sequence the contours, but they are not
    // the complete propagated geometry. The final sweep of a terminating node
    // has no outgoing leading front and therefore no lineage owner. Rebuilding
    // the wave from lineage bands alone silently drops that terminal cap.
    //
    // The node footprint is the exact geometric responsibility of the
    // committed root node, including its terminal cap. Preserve any part not
    // represented by a front-owned band in the stage's final generation. Its
    // precise generation no longer affects sequencing, while retaining it in
    // the wave is essential for reached-area consumers such as cleanup
    // clipping and debug display.
    Paths64 front_owned;
    wave.generations.reserve(lineage.generations.size());
    for (const FrontGeneration& generation : lineage.generations) {
      Paths64 band;
      for (const Paths64& component : generation.bands)
        band.insert(band.end(), component.begin(), component.end());
      band = Union(band, FillRule::NonZero);
      front_owned.insert(front_owned.end(), band.begin(), band.end());
      wave.generations.push_back(std::move(band));
    }
    front_owned = Union(front_owned, FillRule::NonZero);

    Paths64 committed_region;
    for (const FrontNode& node : committed_topology.nodes) {
      if (node.kind != FrontNode::Kind::Propagation) continue;
      committed_region.insert(committed_region.end(), node.footprint.begin(),
                              node.footprint.end());
    }
    committed_region = Union(committed_region, FillRule::NonZero);
    Paths64 terminal_caps = Difference(committed_region, front_owned,
                                       FillRule::NonZero);
    if (!terminal_caps.empty() && !wave.generations.empty()) {
      wave.generations.back().insert(wave.generations.back().end(),
                                     terminal_caps.begin(),
                                     terminal_caps.end());
      wave.generations.back() =
          Union(wave.generations.back(), FillRule::NonZero);
    }

    Paths64 stage_reached;
    wave.reached_generations.reserve(wave.generations.size());
    for (const Paths64& generation : wave.generations) {
      stage_reached.insert(stage_reached.end(), generation.begin(),
                           generation.end());
      stage_reached = Union(stage_reached, FillRule::NonZero);
      wave.reached_generations.push_back(stage_reached);
    }

    result.push_back({decision.layer, std::move(wave), std::move(lineage),
                      std::move(committed_topology)});
  }
  return result;
}

CommittedWavefront commit_wavefront_analysis(
    const WavefrontAnalysis& analysis) {
  CommittedWavefront result;
  result.stages = committed_wavefront_stages(analysis);
  result.wave = analysis.seed;
  result.wave.generations.clear();
  result.wave.reached_generations.clear();
  result.wave.explicit_entry_fronts.clear();
  result.lineage.stepover = analysis.seed.stepover;

  Paths64 reached;
  for (const WavefrontStage& stage : result.stages) {
    if (result.lineage.boundary_components.empty())
      result.lineage.boundary_components = stage.lineage.boundary_components;
    else if (!paths_equal_canonical(result.lineage.boundary_components,
                                     stage.lineage.boundary_components))
      throw std::runtime_error(
          "Committed wave layers use different static boundaries");

    const std::size_t generation_offset = result.lineage.generations.size();
    const std::size_t node_offset = result.topology.nodes.size();
    const std::size_t entry_offset = result.lineage.entry_fronts.size();

    for (const FrontNode& source_node : stage.topology.nodes) {
      FrontNode node = source_node;
      node.evaluation_order = stage.analysis_layer;
      node.parents.clear();
      node.children.clear();
      for (FrontId2D& front : node.fronts)
        front.generation += generation_offset;
      result.topology.nodes.push_back(std::move(node));
    }

    for (std::size_t generation = 0;
         generation < stage.lineage.generations.size(); ++generation) {
      const FrontGeneration& source_generation =
          stage.lineage.generations[generation];
      result.lineage.generations.push_back(source_generation);

      std::vector<std::size_t> front_nodes =
          stage.topology.front_nodes[generation];
      for (std::size_t& node : front_nodes) node += node_offset;
      result.topology.front_nodes.push_back(std::move(front_nodes));

      Paths64 wave_band = stage.wave.generations[generation];
      result.wave.generations.push_back(wave_band);
      reached.insert(reached.end(), wave_band.begin(), wave_band.end());
      reached = Union(reached, FillRule::NonZero);
      result.wave.reached_generations.push_back(reached);
    }

    result.lineage.entry_fronts.insert(result.lineage.entry_fronts.end(),
                                       stage.lineage.entry_fronts.begin(),
                                       stage.lineage.entry_fronts.end());
    for (const std::size_t entry : stage.topology.entry_fronts)
      result.topology.entry_fronts.push_back(entry_offset + entry);
  }

  if (result.stages.empty()) {
    result.wave = analysis.seed;
    result.lineage = analysis.seed_lineage;
    result.topology = build_layer_topology(result.lineage);
  }
  return result;
}

}  // namespace wave
