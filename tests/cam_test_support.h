#pragma once

#include "cam/config.h"
#include "cam/feeds.h"
#include "cam/job.h"
#include "cam/post.h"

#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace cam_test {

extern int failures;

void require(bool condition, const std::string& message);
void require_near(double actual, double expected, double tolerance,
                  const std::string& message);
cam::Machine test_machine();
cam::Tool test_tool();
wave::Point64 point(double x, double y = 0.0);
wave::Path64 box(double x0, double y0, double x1, double y1);
bool inside(const wave::Point64& at, const wave::Paths64& region);
double area(const wave::Paths64& paths);

struct SyntheticCut {
  wave::Toolpath toolpath;
  wave::SimulationResult simulation;
};

SyntheticCut straight_cut(const std::vector<double>& loads);
wave::FrontLineage single_front_lineage(const wave::Path64& front,
                                        const wave::Path64& band);
wave::FrontLineage railed_node(double band_inset);

void check_engagement_feeds();
void check_feed_compatible_merge();
void check_path_resampling();
void check_level_floors();
void check_axial_allowance();
void check_boundary_rail_slack();
void check_entry_front_selection();
void check_box_stock(const std::filesystem::path& part_mesh);
void check_engagement_limits(const std::filesystem::path& part_mesh);
void check_multi_operation_plan(const std::filesystem::path& data,
                                const std::filesystem::path& part_mesh);
void check_lsjig_subdivision_cleanup(const std::filesystem::path& data,
                                     const std::filesystem::path& jig_mesh);
void check_inter_layer_topology();
void check_general_traversal_and_contours();
void check_serialisation();
void check_user_settings();
void check_operation_stock_handoff();
void check_schedule_timing();
void check_validation();
void check_planning_error_details();
void check_shipped_data(const std::filesystem::path& data);
void check_post_output();

}  // namespace cam_test
