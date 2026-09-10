#include "cam_test_support.h"

int main(int argc, char** argv) {
  using namespace cam_test;
  try {
    check_engagement_feeds();
    check_feed_compatible_merge();
    check_path_resampling();
    check_level_floors();
    check_axial_allowance();
    check_boundary_rail_slack();
    check_entry_front_selection();
    check_inter_layer_topology();
    check_general_traversal_and_contours();
    check_serialisation();
    check_operation_stock_handoff();
    check_schedule_timing();
    check_validation();
    check_planning_error_details();
    check_post_output();
    if (argc > 1) check_shipped_data(argv[1]);
    if (argc > 2) check_box_stock(argv[2]);
    if (argc > 2) check_engagement_limits(argv[2]);
    if (argc > 2) check_multi_operation_plan(argv[1], argv[2]);
    if (argc > 3)
      check_lsjig_subdivision_cleanup(
          argv[1], std::filesystem::path(argv[3]) / "LsJig1.obj");
  } catch (const std::exception& error) {
    std::cerr << "Test threw: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  if (failures > 0) {
    std::cerr << failures << " check(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All checks passed.\n";
  return EXIT_SUCCESS;
}
