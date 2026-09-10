#include "topology_detail.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace wave {

using namespace Clipper2Lib;

double boundary_slack(double stepover) {
  return std::clamp(static_cast<double>(to_integer(stepover)) * 0.001, 4.0,
                    100.0);
}

using namespace detail;

Path64 boundary_arc(const FrontLineage& lineage,
                    const BoundaryContact& from,
                    const BoundaryContact& to) {
  Path64 forward = boundary_walk(lineage, from, to, true);
  Path64 reverse = boundary_walk(lineage, from, to, false);
  return path_length_integer(forward) <= path_length_integer(reverse)
             ? forward
             : reverse;
}

LayerTopology build_layer_topology(const FrontLineage& lineage,
                                   bool build_footprints) {
  LayerTopology result;
  result.front_nodes.resize(lineage.generations.size());
  std::vector<std::size_t> offsets(lineage.generations.size() + 1, 0);
  std::size_t total = 0;
  for (std::size_t generation = 0; generation < lineage.generations.size(); ++generation) {
    offsets[generation] = total;
    result.front_nodes[generation].assign(
        lineage.generations[generation].fronts.size(), kNoTopologyId);
    total += lineage.generations[generation].fronts.size();
  }
  offsets.back() = total;
  if (total == 0) return result;

  DisjointSet sets(total);
  std::vector<std::vector<bool>> rejected_splits(
      lineage.generations.size());
  for (std::size_t generation = 0; generation < lineage.generations.size();
       ++generation)
    rejected_splits[generation].assign(
        lineage.generations[generation].fronts.size(), false);
  for (const LineageEvent& event : lineage.rejected_splits) {
    if (event.kind != LineageEventKind::Split || event.generation == 0 ||
        event.generation > rejected_splits.size())
      continue;
    for (const std::size_t parent : event.previous_fronts)
      if (parent < rejected_splits[event.generation - 1].size())
        rejected_splits[event.generation - 1][parent] = true;
  }
  for (std::size_t generation = 0; generation + 1 < lineage.generations.size();
       ++generation) {
    const auto& parents = lineage.generations[generation].fronts;
    const auto& children = lineage.generations[generation + 1].fronts;
    for (std::size_t parent = 0; parent < parents.size(); ++parent) {
      if (parents[parent].children.size() == 1) {
        const std::size_t child = parents[parent].children.front();
        if (child >= children.size() || children[child].parents.size() != 1)
          continue;
        sets.join(offsets[generation] + parent,
                  offsets[generation + 1] + child);
      } else if (parents[parent].children.size() > 1 &&
                 rejected_splits[generation][parent]) {
        // The geometric fronts and their dependencies remain intact. Joining
        // them here says only that a below-resolution terminal spur does not
        // start a new propagation node.
        for (const std::size_t child : parents[parent].children)
          if (child < children.size() && children[child].parents.size() == 1)
            sets.join(offsets[generation] + parent,
                      offsets[generation + 1] + child);
      }
    }
  }

  std::vector<std::size_t> root_to_node(total, kNoTopologyId);
  for (std::size_t generation = 0; generation < lineage.generations.size(); ++generation) {
    for (std::size_t index = 0;
         index < lineage.generations[generation].fronts.size(); ++index) {
      const std::size_t root = sets.find(offsets[generation] + index);
      if (root_to_node[root] == kNoTopologyId) {
        root_to_node[root] = result.nodes.size();
        result.nodes.push_back({});
      }
      const std::size_t node = root_to_node[root];
      result.front_nodes[generation][index] = node;
      result.nodes[node].fronts.push_back({generation, index});
    }
  }

  if (build_footprints) {
    const double half_pitch =
        std::max<double>(1.0, to_integer(lineage.stepover) * 0.5);
    for (FrontNode& node : result.nodes) {
    const FrontId2D last_id = *std::max_element(
        node.fronts.begin(), node.fronts.end(),
        [](const FrontId2D& a, const FrontId2D& b) {
          return a.generation < b.generation;
        });
    const LeadingFront& last_front =
        lineage.generations[last_id.generation].fronts[last_id.index];
    const bool terminal = last_front.children.empty();
    bool has_rejected_split = false;
    for (std::size_t i = 1; i < node.fronts.size(); ++i)
      if (node.fronts[i - 1].generation == node.fronts[i].generation) {
        has_rejected_split = true;
        break;
      }
    if (!has_rejected_split)
      for (const FrontId2D id : node.fronts) {
        const LeadingFront& front =
            lineage.generations[id.generation].fronts[id.index];
        if (front.children.size() > 1 &&
            id.generation < rejected_splits.size() &&
            id.index < rejected_splits[id.generation].size() &&
            rejected_splits[id.generation][id.index]) {
          has_rejected_split = true;
          break;
        }
      }
    if (node.fronts.size() == 1 && !terminal) {
      // A singleton has no preceding/following fronts with which to enclose an
      // area, so a non-terminal singleton alone receives the half-pitch local
      // responsibility band.
      const FrontId2D id = node.fronts.front();
      const LeadingFront& front =
          lineage.generations[id.generation].fronts[id.index];
      if (front.path.size() >= 2 &&
          front.band < lineage.generations[id.generation].bands.size()) {
        const EndType end = front.closed ? EndType::Joined : EndType::Round;
        const Paths64 envelope = InflatePaths(Paths64{front.path}, half_pitch,
                                               JoinType::Round, end);
        const Paths64 footprint = Intersect(
            envelope, lineage.generations[id.generation].bands[front.band],
            FillRule::NonZero);
        node.footprint = one_closed_component(
            footprint, "Non-terminal singleton node footprint");
      }
      continue;
    }

    const Paths64 swept = member_bands(lineage, node);
    if (has_rejected_split) {
      node.footprint = one_closed_component(
          swept, "Node containing a rejected tiny split");
      continue;
    }
    std::vector<OrientedFront> open_fronts;
    std::vector<Path64> closed_fronts;
    bool has_open = false;
    bool has_closed = false;
    for (const FrontId2D id : node.fronts) {
      const LeadingFront& front =
          lineage.generations[id.generation].fronts[id.index];
      if (front.closed) {
        has_closed = true;
        closed_fronts.push_back(front.path);
      } else {
        has_open = true;
      }
    }

    // Detaching from a static boundary turns an open front into a closed one;
    // reattachment does the reverse. Such a run has no continuous pair of
    // endpoint rails. Its connected swept band is the geometric record of the
    // attachment transition, so use that boundary and require it to be one
    // closed component rather than silently inventing endpoints.
    if (has_open && has_closed) {
      node.footprint = one_closed_component(
          swept, "Attaching or detaching node footprint");
      continue;
    }

    if (has_open)
      for (const FrontId2D id : node.fronts) {
        const LeadingFront& front =
            lineage.generations[id.generation].fronts[id.index];
        open_fronts.push_back(orient_after(
            front, open_fronts.empty() ? nullptr : &open_fronts.back()));
      }

    if (open_fronts.empty()) {
      if (closed_fronts.empty())
        throw std::runtime_error("A node contains no usable front boundary");
      const Paths64 first = one_closed_component(
          Paths64{closed_fronts.front()}, "Closed node's first front");
      if (terminal) {
        node.footprint = first;
      } else {
        const Paths64 last = one_closed_component(
            Paths64{closed_fronts.back()}, "Closed node's last front");
        const bool first_contains_last = Difference(
            last, first, FillRule::NonZero).empty();
        const bool last_contains_first = Difference(
            first, last, FillRule::NonZero).empty();
        if (!first_contains_last && !last_contains_first)
          throw std::runtime_error(
              "A closed node's first and last fronts are not nested");
        node.footprint = one_closed_component(
            first_contains_last ? Difference(first, last, FillRule::NonZero)
                                : Difference(last, first, FillRule::NonZero),
            "Closed non-terminal node footprint");
      }
      continue;
    }

    std::array<Path64, 2> rails{
        Path64{open_fronts.front().path.front()},
        Path64{open_fronts.front().path.back()},
    };
    for (std::size_t i = 1; i < open_fronts.size(); ++i)
      for (std::size_t side = 0; side < 2; ++side)
        append_path(rails[side], endpoint_connection(
                                     lineage, open_fronts[i - 1],
                                     open_fronts[i], side, swept));

    if (terminal) {
      const auto& contacts = open_fronts.back().contacts;
      if (contacts[0].valid && contacts[1].valid &&
          contacts[0].component == contacts[1].component) {
        try {
          node.footprint = terminal_open_footprint(lineage, open_fronts,
                                                   rails, swept);
        } catch (const std::runtime_error& error) {
          throw std::runtime_error(
              "Terminal node " +
              std::to_string(static_cast<std::size_t>(&node -
                                                       result.nodes.data())) +
              ": " + error.what());
        }
      } else
        node.footprint = one_closed_component(
            swept, "Terminal node footprint without a static-boundary cap");
      continue;
    }

    Path64 boundary = open_fronts.front().path;
    append_path(boundary, rails[1]);
    Path64 last_reverse = open_fronts.back().path;
    std::reverse(last_reverse.begin(), last_reverse.end());
    append_path(boundary, last_reverse);
    Path64 first_return = rails[0];
    std::reverse(first_return.begin(), first_return.end());
    append_path(boundary, first_return);
    node.footprint = one_closed_component(
        Paths64{boundary}, "Open non-terminal node footprint");
    }

    for (const FrontNode& node : result.nodes) {
      if (node.footprint.empty())
        throw std::runtime_error("A node footprint has no closed boundary");
    }
  }

  // The node graph is already in generation order, so a single forward pass
  // computes its topological evaluation depth.
  const auto append_unique = [](std::vector<std::size_t>& values,
                                std::size_t value) {
    if (std::find(values.begin(), values.end(), value) == values.end())
      values.push_back(value);
  };
  for (std::size_t generation = 0; generation + 1 < lineage.generations.size();
       ++generation) {
    for (std::size_t parent = 0;
         parent < lineage.generations[generation].fronts.size(); ++parent) {
      const std::size_t parent_node = result.front_nodes[generation][parent];
      for (const std::size_t child :
           lineage.generations[generation].fronts[parent].children) {
        if (child >= result.front_nodes[generation + 1].size()) continue;
        const std::size_t child_node = result.front_nodes[generation + 1][child];
        if (child_node == parent_node) continue;
        append_unique(result.nodes[parent_node].children, child_node);
        append_unique(result.nodes[child_node].parents, parent_node);
        result.nodes[child_node].evaluation_order =
            std::max(result.nodes[child_node].evaluation_order,
                     result.nodes[parent_node].evaluation_order + 1);
      }
    }
  }

  // Material consumed between converging fronts has no leading front of its
  // own. Preserve that real topological merge as an empty terminal sink. It is
  // part of wave complexity and descendant association, but deliberately has
  // no footprint or entry front.
  for (const LineageEvent& event : lineage.events) {
    if (event.kind != LineageEventKind::MergeTermination ||
        event.generation == 0 || event.previous_fronts.size() < 2)
      continue;
    FrontNode terminal;
    terminal.kind = FrontNode::Kind::MergeTermination;
    for (const std::size_t front : event.previous_fronts) {
      if (front >= result.front_nodes[event.generation - 1].size()) continue;
      const std::size_t parent =
          result.front_nodes[event.generation - 1][front];
      append_unique(terminal.parents, parent);
    }
    if (terminal.parents.size() < 2) continue;
    const std::size_t terminal_id = result.nodes.size();
    for (const std::size_t parent : terminal.parents) {
      append_unique(result.nodes[parent].children, terminal_id);
      terminal.evaluation_order =
          std::max(terminal.evaluation_order,
                   result.nodes[parent].evaluation_order + 1);
    }
    result.nodes.push_back(std::move(terminal));
  }

  result.entry_fronts.resize(lineage.entry_fronts.size());
  std::iota(result.entry_fronts.begin(), result.entry_fronts.end(), 0);
  return result;
}

std::vector<NodeEntryGroup> group_node_entries(
    const LayerTopology& topology, std::size_t evaluation_order) {
  std::vector<NodeEntry> entries;
  for (std::size_t node = 0; node < topology.nodes.size(); ++node) {
    const FrontNode& candidate = topology.nodes[node];
    if (candidate.kind != FrontNode::Kind::Propagation ||
        candidate.evaluation_order != evaluation_order ||
        candidate.fronts.empty())
      continue;
    entries.push_back({node, candidate.fronts.front()});
  }
  if (entries.empty()) return {};

  DisjointSet groups(entries.size());
  std::vector<std::vector<std::size_t>> labels(topology.nodes.size());
  for (std::size_t entry = 0; entry < entries.size(); ++entry)
    labels[entries[entry].node].push_back(entry);

  std::vector<std::size_t> order(topology.nodes.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](std::size_t a,
                                                    std::size_t b) {
    return topology.nodes[a].evaluation_order <
           topology.nodes[b].evaluation_order;
  });
  for (const std::size_t node : order) {
    auto& incoming = labels[node];
    std::sort(incoming.begin(), incoming.end());
    incoming.erase(std::unique(incoming.begin(), incoming.end()),
                   incoming.end());
    if (incoming.size() > 1)
      for (std::size_t i = 1; i < incoming.size(); ++i)
        groups.join(incoming.front(), incoming[i]);
    for (const std::size_t child : topology.nodes[node].children) {
      if (child >= labels.size()) continue;
      labels[child].insert(labels[child].end(), incoming.begin(),
                           incoming.end());
    }
  }

  std::vector<std::size_t> root_to_group(entries.size(), kNoTopologyId);
  std::vector<NodeEntryGroup> result;
  for (std::size_t entry = 0; entry < entries.size(); ++entry) {
    const std::size_t root = groups.find(entry);
    if (root_to_group[root] == kNoTopologyId) {
      root_to_group[root] = result.size();
      result.push_back({});
    }
    result[root_to_group[root]].entries.push_back(entries[entry]);
  }
  return result;
}

// Entry identity down the stack, from the entry curves alone. An upper front
// claims a lower one when the whole of the lower curve falls inside an envelope
// grown around the upper, and the two are linked only when that claim is
// mutual and unambiguous -- a front that two others could equally claim starts
// a chain of its own rather than being assigned to a guess.
//
// This asks nothing of the node topology, and nothing of any generation past
// the first. A wave that has been seeded and not propagated already knows every
// entry it has, which is what lets an entry be chosen before a plan exists.
std::vector<InterLayerChain> chain_entry_fronts(
    const std::vector<const FrontLineage*>& lineages,
    double entry_front_match_tolerance,
    std::vector<std::vector<std::size_t>>* chain_ids) {
  if (!(entry_front_match_tolerance > 0.0) ||
      !std::isfinite(entry_front_match_tolerance))
    throw std::invalid_argument("Entry-front match tolerance must be positive");
  std::vector<std::size_t> sizes;
  std::vector<std::vector<std::size_t>> next(lineages.size());
  for (std::size_t layer = 0; layer < lineages.size(); ++layer) {
    if (!lineages[layer]) throw std::invalid_argument("Inter-layer lineage is null");
    sizes.push_back(lineages[layer]->entry_fronts.size());
    next[layer].assign(lineages[layer]->entry_fronts.size(), kNoTopologyId);
  }
  const double tolerance = static_cast<double>(
      std::max<std::int64_t>(1, to_integer(entry_front_match_tolerance)));
  for (std::size_t layer = 0; layer + 1 < lineages.size(); ++layer) {
    const auto& upper_fronts = lineages[layer]->entry_fronts;
    const auto& lower_fronts = lineages[layer + 1]->entry_fronts;
    std::vector<std::size_t> upper_count(upper_fronts.size(), 0);
    std::vector<std::size_t> lower_count(lower_fronts.size(), 0);
    std::vector<std::pair<std::size_t, std::size_t>> candidates;
    for (std::size_t a = 0; a < upper_fronts.size(); ++a) {
      const LeadingFront& upper = upper_fronts[a];
      const Paths64 envelope = InflatePaths(Paths64{upper.path}, tolerance,
                                             JoinType::Round,
                                             upper.closed ? EndType::Joined
                                                          : EndType::Round);
      for (std::size_t b = 0; b < lower_fronts.size(); ++b) {
        const LeadingFront& lower = lower_fronts[b];
        Path64 lower_path = lower.path;
        if (lower.closed && !lower_path.empty())
          lower_path.push_back(lower_path.front());
        if (!whole_open_path_inside(lower_path, envelope)) continue;
        ++upper_count[a];
        ++lower_count[b];
        candidates.push_back({a, b});
      }
    }
    for (const auto& [a, b] : candidates)
      if (upper_count[a] == 1 && lower_count[b] == 1) next[layer][a] = b;
  }
  std::vector<std::vector<std::size_t>> discarded;
  return assemble_chains(sizes, next, chain_ids ? *chain_ids : discarded);
}

InterLayerTopology build_inter_layer_topology(
    const std::vector<const FrontLineage*>& lineages,
    double entry_front_match_tolerance,
    const std::vector<const LayerTopology*>& topologies) {
  if (!(entry_front_match_tolerance > 0.0) ||
      !std::isfinite(entry_front_match_tolerance))
    throw std::invalid_argument("Entry-front match tolerance must be positive");
  InterLayerTopology result;
  result.layers.reserve(lineages.size());
  if (!topologies.empty() && topologies.size() != lineages.size())
    throw std::invalid_argument(
        "Inter-layer topology and lineage counts do not match");
  for (std::size_t layer = 0; layer < lineages.size(); ++layer) {
    const FrontLineage* lineage = lineages[layer];
    if (!lineage) throw std::invalid_argument("Inter-layer lineage is null");
    if (topologies.empty())
      result.layers.push_back(build_layer_topology(*lineage));
    else {
      if (!topologies[layer])
        throw std::invalid_argument("Inter-layer topology is null");
      if (topologies[layer]->front_nodes.size() !=
          lineage->generations.size())
        throw std::invalid_argument(
            "Inter-layer topology and lineage generations do not match");
      result.layers.push_back(*topologies[layer]);
    }
  }

  std::vector<std::size_t> node_sizes;
  std::vector<std::vector<std::size_t>> node_next(result.layers.size());
  for (const LayerTopology& layer : result.layers)
    node_sizes.push_back(layer.nodes.size());
  for (std::size_t layer = 0; layer < result.layers.size(); ++layer)
    node_next[layer].assign(result.layers[layer].nodes.size(), kNoTopologyId);

  for (std::size_t layer = 0; layer + 1 < result.layers.size(); ++layer) {
    const auto& upper = result.layers[layer].nodes;
    const auto& lower = result.layers[layer + 1].nodes;
    std::vector<std::size_t> upper_count(upper.size(), 0);
    std::vector<std::size_t> lower_count(lower.size(), 0);
    std::vector<std::pair<std::size_t, std::size_t>> candidates;
    for (std::size_t a = 0; a < upper.size(); ++a)
      for (std::size_t b = 0; b < lower.size(); ++b)
        if (upper[a].kind == FrontNode::Kind::Propagation &&
            lower[b].kind == FrontNode::Kind::Propagation &&
            upper[a].evaluation_order == lower[b].evaluation_order &&
            area_contains_with_tolerance(
                upper[a].footprint, lower[b].footprint,
                kNodeZAssociationTolerance)) {
          ++upper_count[a];
          ++lower_count[b];
          candidates.push_back({a, b});
        }
    for (const auto& [a, b] : candidates)
      if (upper_count[a] == 1 && lower_count[b] == 1) node_next[layer][a] = b;
  }
  result.node_chains =
      assemble_chains(node_sizes, node_next, result.node_chain_ids);

  result.entry_front_chains = chain_entry_fronts(lineages, entry_front_match_tolerance,
                                                 &result.entry_front_chain_ids);

  std::vector<std::vector<bool>> scheduled(result.layers.size());
  for (std::size_t layer = 0; layer < result.layers.size(); ++layer)
    scheduled[layer].assign(result.layers[layer].nodes.size(), false);
  for (std::size_t layer = 0; layer < result.layers.size(); ++layer) {
    std::vector<std::size_t> order(result.layers[layer].nodes.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return result.layers[layer].nodes[a].evaluation_order <
             result.layers[layer].nodes[b].evaluation_order;
    });
    for (const std::size_t node : order) {
      if (result.layers[layer].nodes[node].kind ==
          FrontNode::Kind::MergeTermination) {
        scheduled[layer][node] = true;
        continue;
      }
      if (scheduled[layer][node]) continue;
      const std::size_t chain = result.node_chain_ids[layer][node];
      if (chain == kNoTopologyId) {
        result.node_schedule.push_back({layer, node});
        scheduled[layer][node] = true;
        continue;
      }
      const auto& members = result.node_chains[chain].members;
      auto current = std::find_if(
          members.begin(), members.end(), [&](const LayerItemId& member) {
            return member.layer == layer && member.index == node;
          });
      for (; current != members.end(); ++current) {
        if (scheduled[current->layer][current->index]) continue;
        result.node_schedule.push_back(*current);
        scheduled[current->layer][current->index] = true;
      }
    }
  }
  return result;
}
}  // namespace wave
