#pragma once

#include "geometry.h"
#include "topology.h"

#include <cstddef>
#include <vector>

namespace wave {

// Nodes smaller than this fraction of the incumbent strategy's largest node
// do not contribute to strategy selection. They remain in the selected wave
// and topology; this only keeps microscopic propagation artifacts from
// turning an otherwise simpler candidate into a node-count tie.
inline constexpr double kDefaultSignificantNodeAreaFraction = 0.005;

// One deliberately small alternative to the simultaneous baseline: every
// graph-derived multi-entry volume retains only its longest entry, and all of
// those independent choices are evaluated in one propagation rollout.
struct WavefrontStrategyTrial {
  std::vector<std::size_t> retained_entries;
  // Raw topology sizes are retained for diagnostics. Selection compares the
  // significant counts below, using one threshold derived from the incumbent
  // strategy for both graphs.
  std::size_t previous_node_count = 0;
  std::size_t candidate_node_count = 0;
  std::size_t previous_significant_node_count = 0;
  std::size_t candidate_significant_node_count = 0;
  double reference_node_area = 0.0;
  double minimum_significant_node_area = 0.0;
  std::vector<double> previous_node_areas;
  std::vector<double> candidate_node_areas;
  bool accepted = false;
  WaveResult wave;
  FrontLineage lineage;
  LayerTopology topology;
};

// Everything needed to replay one recursive decision layer in the diagnostic
// viewer. The baseline and every candidate are ordinary pure wave analyses.
struct WavefrontDecisionLayer {
  std::size_t layer = 0;
  Paths64 remaining_domain;
  // Copy of the reached state immediately behind every entry at this layer.
  // Baseline and candidate rollouts share it; only their active entry curves
  // differ.
  Paths64 cached_reached;
  std::vector<WaveSeed> entries;
  std::vector<NodeEntryGroup> groups;
  WaveResult baseline_wave;
  FrontLineage baseline_lineage;
  LayerTopology baseline_topology;
  std::vector<WavefrontStrategyTrial> trials;
  std::vector<std::size_t> retained_entries;
  WaveResult selected_wave;
  FrontLineage selected_lineage;
  LayerTopology selected_topology;
};

struct WavefrontAnalysis {
  WaveResult seed;
  FrontLineage seed_lineage;
  std::vector<WavefrontDecisionLayer> layers;
};

// The production form of one committed recursive layer. Only propagation
// nodes at evaluation order zero are present; descendants belong to the next
// analysis layer and must not be sequenced twice. Stages are processed in
// order while retaining the cleared state between them.
struct WavefrontStage {
  std::size_t analysis_layer = 0;
  WaveResult wave;
  FrontLineage lineage;
  LayerTopology topology;
};

// Final per-depth representation after every recursive decision has been
// committed. `stages` retains the execution barriers used by toolpath
// sequencing; the combined wave, lineage, and topology are the corresponding
// geometry used by debug views and inter-layer association.
struct CommittedWavefront {
  std::vector<WavefrontStage> stages;
  WaveResult wave;
  FrontLineage lineage;
  LayerTopology topology;
};

// Runs the layer-by-layer node-count analysis. At each layer ordinary
// simultaneous propagation establishes the node graph and its entry groups;
// one alternative retains only the longest entry of every multi-entry group.
// A suffix graph with strictly fewer significant nodes is accepted;
// significance is measured against the incumbent graph's largest approximate
// node area. Descendants remain simultaneous, and only the winning root layer
// is committed before the next recursive analysis. Set the area fraction to
// zero to restore raw node-count comparison. `exterior_stock_reach` permits
// propagation through nearby air without changing the physical stock used by
// toolpath engagement tests.
WavefrontAnalysis analyze_wavefront(
    const Paths64& raw_subject,
    const Paths64& raw_stock,
    double stepover,
    double softening = 0.0,
    double lead_in = 0.0,
    std::size_t max_iterations = 2000,
    std::size_t max_layers = 128,
    const EntryMute& muted = {},
    double significant_node_area_fraction =
        kDefaultSignificantNodeAreaFraction,
    double tiny_split_area_factor = kDefaultTinySplitAreaFactor,
    double exterior_stock_reach = 0.0);

// Discards the speculative suffixes and returns exactly the node layer chosen
// at each recursive decision. The returned stages are suitable for sequencing
// directly into tool motion.
std::vector<WavefrontStage> committed_wavefront_stages(
    const WavefrontAnalysis& analysis);

CommittedWavefront commit_wavefront_analysis(
    const WavefrontAnalysis& analysis);

}  // namespace wave
