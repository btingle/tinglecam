#pragma once
#include "cam/job.h"

namespace cam {
// Height-only splits share immutable polygon storage. XY clipping allocates only
// the intersected piece, leaving the original operation's layers untouched.
std::vector<StockSlab> stock_inside(const std::vector<StockSlab>& stock,
    const wave::Paths64& footprint, double bottom, double top);
std::vector<StockSlab> stock_outside(const std::vector<StockSlab>& stock,
    const wave::Paths64& footprint, double bottom, double top);
std::vector<StockSlab> preview_before_level(const OperationPlan& operation, std::size_t level);
std::vector<StockSlab> active_stock(const JobGeometry& geometry,
    const PriorOperations& previous, int grid,
    const std::vector<double>& extra_boundaries = {});
}
