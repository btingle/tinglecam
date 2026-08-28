#pragma once

#include <optional>
#include <string>
#include <vector>

namespace cam {

enum class MaterialGroup {
  Aluminium,
  Steel,
  Stainless,
  Titanium,
  Brass,
  Plastic,
  Wood,
  Composite,
  Foam,
};

enum class ChipClass {
  Gummy,
  Short,
  Abrasive,
};

struct Material {
  std::string id;
  std::string name;
  std::string notes;
  MaterialGroup group = MaterialGroup::Aluminium;
  std::string hardness;  // free text: "HB 95", "HRC 30"

  // Specific cutting energy, J/mm^3. A thousand of these is the tangential
  // force per square millimetre of chip section as it is usually tabulated, in
  // N/mm^2. This is the whole bridge between removal rate and spindle power.
  double specific_cutting_energy = 0.0;

  // Kienzle form of the same thing, for when a constant is too crude: the
  // specific force at a one-millimetre chip, in N/mm^2 as it is always quoted,
  // and the exponent by which it falls as the chip thickens. Zero disables it
  // and the constant above is used.
  double kienzle_kc11 = 0.0;
  double kienzle_mc = 0.0;

  // A user derate on the tool's book figures for this material, not a physical
  // property: the cutting data on each tool is already keyed by material, so
  // these default to one and exist to be turned down when a particular batch or
  // setup wants taking gently.
  double vc_multiplier = 1.0;
  double fz_multiplier = 1.0;
  ChipClass chip_class = ChipClass::Short;
  bool work_hardening = false;
};

// Specific cutting energy at a given mean chip thickness, J/mm^3. Falls back to
// the constant when no Kienzle coefficients are supplied. A thin chip costs far
// more energy per unit volume than a thick one, which is exactly the regime a
// chip-thinned adaptive path spends its time in, so the constant form
// underestimates power at low engagement.
double specific_energy(const Material& material, double chip_thickness);

const char* material_group_name(MaterialGroup value);
const char* chip_class_name(ChipClass value);

std::optional<MaterialGroup> material_group_from_name(const std::string& text);
std::optional<ChipClass> chip_class_from_name(const std::string& text);

std::vector<std::string> validate(const Material& material);

struct MaterialLibrary {
  std::vector<Material> materials;

  const Material* find(const std::string& id) const;
  Material* find(const std::string& id);
  bool upsert(Material material);
};

}  // namespace cam
