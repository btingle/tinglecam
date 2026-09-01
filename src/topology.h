#pragma once

#include "geometry.h"

#include <array>
#include <cstddef>
#include <limits>
#include <vector>

namespace wave {

constexpr std::size_t kNoTopologyId = std::numeric_limits<std::size_t>::max();

struct BoundaryContact {
  bool valid = false;
  // Connected ring of the final propagation-domain or protected-subject
  // boundary. This is deliberately not an input-object identifier.
  std::size_t component = kNoTopologyId;
  Point64 point = Point64(0, 0);
  // Millimetres from the component's first vertex, following its stored
  // winding, and the distance from the front endpoint to the classified point.
  double arclength = 0.0;
  double error = 0.0;
};

struct FrontId2D {
  std::size_t generation = 0;
  std::size_t index = 0;
};

struct LeadingFront {
  Path64 path;
  bool closed = false;
  // Index into FrontGeneration::bands of the connected piece of wave band this
  // front was carved out of. A piece can host several fronts, so material is
  // only credited as cleared once every front sharing it has been cut.
  std::size_t band = 0;
  std::vector<std::size_t> parents;
  std::vector<std::size_t> children;
  // Start and end contacts for an open front. Closed fronts have no endpoint
  // contacts. An invalid contact is retained and diagnosed rather than guessed.
  std::array<BoundaryContact, 2> boundary_contacts;
};

struct FrontGeneration {
  std::vector<LeadingFront> fronts;
  // The generation's wave band split into connected pieces, each an outer
  // contour followed by its holes. Grouping matters: a bare hole ring taken on
  // its own describes the material it encloses rather than the band.
  std::vector<Paths64> bands;
};

enum class LineageEventKind {
  Split,
  Merge,
  Termination,
  Birth,
};

struct LineageEvent {
  LineageEventKind kind = LineageEventKind::Split;
  std::size_t generation = 0;
  std::vector<std::size_t> previous_fronts;
  std::vector<std::size_t> current_fronts;
  Point64 position;
};

struct FrontLineage {
  std::vector<FrontGeneration> generations;
  std::vector<LineageEvent> events;
  // Stable connected boundary rings used by BoundaryContact::component.
  Paths64 boundary_components;
  // Connected propagation-source curves from generation zero. These retain
  // source identity even when the protected subject occludes the stock wall in
  // several places.
  std::vector<LeadingFront> entry_fronts;
  double stepover = 0.0;
};

struct FrontNode {
  std::size_t evaluation_order = 0;
  std::vector<FrontId2D> fronts;
  // One validated closed component. Stable open runs are bounded by their
  // first/last fronts and endpoint rails. Runs whose fronts attach to or detach
  // from a static boundary use their connected swept-band boundary instead.
  Paths64 footprint;
};

struct LayerTopology {
  std::vector<FrontNode> nodes;
  // Same shape as FrontLineage::generations; maps every front to one node.
  std::vector<std::vector<std::size_t>> front_nodes;
  // Indices into FrontLineage::entry_fronts.
  std::vector<std::size_t> entry_fronts;
};

struct LayerItemId {
  std::size_t layer = 0;
  std::size_t index = 0;
};

struct InterLayerChain {
  std::vector<LayerItemId> members;
};

struct InterLayerTopology {
  std::vector<LayerTopology> layers;
  std::vector<InterLayerChain> node_chains;
  std::vector<InterLayerChain> entry_front_chains;
  // Per-layer mappings into the chain arrays. Unmatched items use
  // kNoTopologyId.
  std::vector<std::vector<std::size_t>> node_chain_ids;
  std::vector<std::vector<std::size_t>> entry_front_chain_ids;
  // Proposed section-first order: the first encountered member of an
  // unambiguous node chain is followed down through its remaining Z members,
  // then ordinary per-layer evaluation resumes.
  std::vector<LayerItemId> node_schedule;
};

// Extracts the reached/unreached interface from each vector wave band, then
// associates consecutive front components using the same stepover offset as
// the wave propagation.
FrontLineage build_front_lineage(const WaveResult& wave);

// Collapses maximal 1-child -> 1-parent runs. A split front therefore remains
// upstream, a merge front remains downstream, and a many-to-many front is a
// singleton. Conflicting known endpoint-boundary components on a collapsed
// edge, or any footprint that does not close as one connected component, are
// internal topology errors. Validated attach/detach transitions are retained.
LayerTopology build_layer_topology(const FrontLineage& lineage);

// Entry identity down the stack, from the entry curves alone. An upper front
// claims a lower one when the whole of the lower curve falls inside an envelope
// grown around the upper, and only a mutual, unambiguous claim links them.
//
// This asks nothing of the node topology and nothing of any generation past the
// first, so a wave that has been seeded and never propagated can answer it.
// `chain_ids` receives, per layer, the chain each entry front belongs to.
std::vector<InterLayerChain> chain_entry_fronts(
    const std::vector<const FrontLineage*>& lineages,
    double entry_front_match_tolerance,
    std::vector<std::vector<std::size_t>>* chain_ids = nullptr);

// Builds exact node-area chains and tolerant entry-front chains between
// adjacent Z layers. Area containment has deliberately no geometric tolerance.
// Entry fronts use a strong whole-curve containment test inside the configured
// envelope, and both association types accept only reciprocal unique matches.
InterLayerTopology build_inter_layer_topology(
    const std::vector<const FrontLineage*>& layers,
    double entry_front_match_tolerance);

// Shorter walk between two classified positions on one closed boundary ring.
// Throws when the contacts do not name the same valid component.
Path64 boundary_arc(const FrontLineage& lineage,
                    const BoundaryContact& from,
                    const BoundaryContact& to);

// How far two separately rounded descriptions of the same wall may disagree,
// in integer units. The boundary rings come from a boolean on the machinable
// region and the propagation bands from offsetting; they agree to a couple of
// grid units, not exactly. Anything measuring one against the other has to
// allow this much, and no more.
double boundary_slack(double stepover);

Point64 representative_point(const LeadingFront& front);
double front_length(const LeadingFront& front);
const char* lineage_event_kind_name(LineageEventKind kind);

}  // namespace wave
