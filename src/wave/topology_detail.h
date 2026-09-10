#pragma once

#include "topology.h"

#include <algorithm>
#include <numeric>

namespace wave::detail {

struct OrientedFront {
  Path64 path;
  std::array<BoundaryContact, 2> contacts;
};

class DisjointSet {
 public:
  explicit DisjointSet(std::size_t count) : parent_(count), rank_(count, 0) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }
  std::size_t find(std::size_t value) {
    if (parent_[value] != value) parent_[value] = find(parent_[value]);
    return parent_[value];
  }
  void join(std::size_t a, std::size_t b) {
    a = find(a);
    b = find(b);
    if (a == b) return;
    if (rank_[a] < rank_[b]) std::swap(a, b);
    parent_[b] = a;
    if (rank_[a] == rank_[b]) ++rank_[a];
  }

 private:
  std::vector<std::size_t> parent_;
  std::vector<unsigned char> rank_;
};

std::vector<Paths64> band_components(const Paths64& wave_band);
void extract_ring_fronts(const Path64& raw_ring, std::size_t band,
                         const Paths64& unreached,
                         const Paths64& static_boundaries,
                         double probe_pitch,
                         std::vector<LeadingFront>& fronts);
double front_probe_pitch(double stepover);
std::vector<LeadingFront> extract_leading_fronts(
    const std::vector<Paths64>& bands, const Paths64& unreached,
    const Paths64& static_boundaries, double probe_pitch);
void classify_front_contacts(std::vector<LeadingFront>& fronts,
                             const Paths64& boundaries, double tolerance);
std::vector<std::vector<double>> front_responsibility_areas(
    const FrontLineage& lineage);
void reject_tiny_splits(FrontLineage& lineage, double area_factor);
Paths64 front_reach(const LeadingFront& front, double distance,
                    double softening);
double open_intersection_length(const LeadingFront& front,
                                const Paths64& clips);
bool terminal_reaches_touch(const Paths64& lhs, const Paths64& rhs,
                            double tolerance);
Point64 average_points(const std::vector<Point64>& points);
Path64 boundary_walk(const FrontLineage& lineage,
                     const BoundaryContact& from,
                     const BoundaryContact& to, bool forward);
double path_length_integer(const Path64& path, bool closed = false);
bool whole_open_path_inside(const Path64& path, const Paths64& region);
bool area_contains_with_tolerance(const Paths64& upper, const Paths64& lower,
                                  double tolerance);
void append_path(Path64& destination, const Path64& source);
OrientedFront orient_after(const LeadingFront& front,
                           const OrientedFront* previous);
Paths64 member_bands(const FrontLineage& lineage, const FrontNode& node);
Paths64 one_closed_component(const Paths64& raw, const char* context);
Path64 endpoint_connection(const FrontLineage& lineage,
                           const OrientedFront& from,
                           const OrientedFront& to, std::size_t side,
                           const Paths64& swept);
Paths64 terminal_open_footprint(
    const FrontLineage& lineage, const std::vector<OrientedFront>& fronts,
    const std::array<Path64, 2>& rails, const Paths64& swept);
std::vector<InterLayerChain> assemble_chains(
    const std::vector<std::size_t>& layer_sizes,
    const std::vector<std::vector<std::size_t>>& next,
    std::vector<std::vector<std::size_t>>& chain_ids);

}  // namespace wave::detail
