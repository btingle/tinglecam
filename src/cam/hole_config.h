#pragma once
#include "cam/job.h"
#include <nlohmann/json_fwd.hpp>
namespace cam {
nlohmann::json hole_parameters_json(const HoleParameters&);
HoleParameters hole_parameters_from_json(const nlohmann::json&);
}
