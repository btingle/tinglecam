// Exercise the actual tool-builder widgets without a display or GPU backend.
#define main tinglecam_application_main
#include "../src/tinglecam.cpp"
#undef main
#include "imgui_internal.h"

#include <iostream>
#include <limits>

namespace {

void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void frame(App& app) {
  ImGui::NewFrame();
  ImGui::SetNextWindowPos(ImVec2(20, 20));
  tool_builder(app);
  ImGui::Render();
}

void click(App& app, ImVec2 position) {
  auto& io = ImGui::GetIO();
  io.AddMousePosEvent(position.x, position.y);
  frame(app);
  io.AddMouseButtonEvent(0, true);
  frame(app);
  io.AddMouseButtonEvent(0, false);
  frame(app);
}

void type_number(App& app, const char* label, int row, const char* text) {
  auto* window = ImGui::FindWindowByName("Tool builder");
  const auto& style = ImGui::GetStyle();
  const float line = ImGui::GetFrameHeightWithSpacing();
  click(app, ImVec2(window->Pos.x + style.WindowPadding.x + 40,
                   window->Pos.y + window->TitleBarHeight +
                       style.WindowPadding.y + row * line +
                       ImGui::GetFrameHeight() * 0.5f));
  check(ImGui::GetActiveID() == window->GetID(label),
        "A single click must activate numeric text entry");
  auto& io = ImGui::GetIO();
  io.AddKeyEvent(ImGuiMod_Ctrl, true);
  io.AddKeyEvent(ImGuiKey_A, true);
  frame(app);
  io.AddKeyEvent(ImGuiKey_A, false);
  io.AddKeyEvent(ImGuiMod_Ctrl, false);
  frame(app);
  io.AddInputCharactersUTF8(text);
  frame(app);
  io.AddKeyEvent(ImGuiKey_Enter, true);
  frame(app);
  io.AddKeyEvent(ImGuiKey_Enter, false);
  frame(app);
}

void save(App& app) {
  const auto* window = ImGui::FindWindowByName("Tool builder");
  click(app, ImVec2(window->Pos.x + ImGui::GetStyle().WindowPadding.x + 25,
                   window->DC.CursorMaxPos.y - ImGui::GetFrameHeight() * 0.5f));
}

template <typename Function>
void must_fail(Function function) {
  bool threw = false;
  try {
    function();
  } catch (const std::exception&) {
    threw = true;
  }
  check(threw, "Invalid library operation must report an error");
}

void run(const std::filesystem::path& directory) {
  App app;
  app.tools_file = directory / "tools.json";
  app.builder_open = true;
  app.builder_is_new = true;
  app.builder_tool.name = "Fractional cutter";
  // New tool intentionally has no ID, as in the Libraries tab.
  frame(app);
  frame(app);
  type_number(app, "Diameter (mm)", 1, "3.175");
  type_number(app, "Flute length (mm)", 2, "25.1234567");
  check(app.builder_tool.diameter == 3.175, "Exact diameter must be editable");
  check(app.builder_tool.flute_length == 25.1234567,
        "Numeric entry must retain more than three decimal places");
  app.has_plan = true;
  app.gcode = "old program";
  save(app);
  check(!app.builder_open, "New tool Save button must be enabled");
  check(!app.has_plan && app.gcode.empty(), "Saving must invalidate the old plan");
  auto disk = cam::load_tools(app.tools_file);
  check(disk.tools.size() == 1 && disk.tools.front().id == "fractional-cutter",
        "New tool must persist with an automatically generated ID");
  check(disk.tools.front().diameter == 3.175 &&
            disk.tools.front().flute_length == 25.1234567,
        "Exact numeric values must survive saving and loading");

  app.builder_tool = disk.tools.front();
  app.builder_is_new = true;
  app.builder_open = true;
  frame(app);
  frame(app);
  save(app);
  check(!app.builder_open && app.tools.find("fractional-cutter-2"),
        "Copy must create a distinct tool even with the same name");

  app.builder_tool = app.tools.tools.front();
  app.builder_tool.name = "Renamed cutter";
  app.builder_tool.diameter = 4.125;
  app.builder_is_new = false;
  app.builder_open = true;
  frame(app);
  frame(app);
  save(app);
  check(!app.builder_open && app.tools.tools.size() == 2 &&
            app.tools.find("fractional-cutter")->diameter == 4.125,
        "Edit must update the same ID without adding another tool");

  const std::string original = cam::read_file(app.tools_file);
  app.builder_tool.diameter = 0;
  app.builder_open = true;
  frame(app);
  frame(app);
  save(app);
  check(app.builder_open && cam::read_file(app.tools_file) == original,
        "Invalid dimensions must disable saving");
  app.builder_tool.diameter = std::numeric_limits<double>::infinity();
  check(!cam::validate(app.builder_tool).empty(), "Infinite dimensions are invalid");

  app.builder_tool.diameter = 7.25;
  const auto actual_file = app.tools_file;
  app.tools_file = actual_file / "not-a-directory.json";
  frame(app);
  frame(app);
  save(app);
  check(app.builder_open && app.banner_is_error &&
            app.tools.find("fractional-cutter")->diameter == 4.125 &&
            cam::read_file(actual_file) == original,
        "Failed save must leave the editor open and preserve memory and disk");
  app.tools_file = actual_file;
  app.builder_open = false;

  app.tools_import_file = (directory / "import.json").string();
  cam::ToolLibrary imported;
  cam::Tool tool = app.tools.tools.front();
  tool.diameter = 8.375;
  imported.upsert(tool);
  tool.id = "imported-tool";
  imported.upsert(tool);
  cam::save_tools(app.tools_import_file, imported);
  import_tools(app);
  check(app.tools.tools.size() == 3 && app.tools.find("fractional-cutter-2") &&
            app.tools.find("fractional-cutter")->diameter == 8.375 &&
            cam::load_tools(actual_file).find("imported-tool"),
        "Import must add and replace IDs, retain other tools, and persist");

  const std::string after_import = cam::read_file(actual_file);
  for (const std::string bad : {"{", "{}", "{\"tools\":{}}",
                               "{\"tools\":[{\"id\":\"bad\"}]}"}) {
    cam::write_file(app.tools_import_file, bad);
    must_fail([&] { import_tools(app); });
    check(app.tools.tools.size() == 3 && cam::read_file(actual_file) == after_import,
          "Malformed imports must preserve the active library");
  }
  imported.tools.push_back(imported.tools.front());
  cam::save_tools(app.tools_import_file, imported);
  must_fail([&] { import_tools(app); });
  std::filesystem::remove(app.tools_import_file);
  must_fail([&] { import_tools(app); });

  imported.tools.pop_back();
  cam::save_tools(actual_file, imported);
  app.has_plan = true;
  reload_tools(app);
  check(app.tools.tools.size() == 2 && !app.has_plan,
        "Reload must replace the in-memory library and invalidate planning");
  cam::write_file(actual_file, "not JSON");
  must_fail([&] { reload_tools(app); });
  check(app.tools.tools.size() == 2, "Failed reload must retain the current tools");
  cam::save_tools(actual_file, {});
  reload_tools(app);
  check(app.tools.tools.empty(), "An explicitly empty library must remain empty");
}

void check_warning_wrapping() {
  const std::vector<std::string> warnings{
      "Stepdown reduced from 2 mm to 1.5875 mm: the selected tool's material "
      "cutting data limits axial engagement to half its diameter, even though "
      "its flute length is 25 mm. Edit the tool in Libraries to change this limit."};
  for (int frame_index = 0; frame_index < 2; ++frame_index) {
    ImGui::NewFrame();
    ImGui::SetNextWindowSize(ImVec2(300, 500));
    ImGui::Begin("Narrow warning panel");
    show_warnings("test-warnings", warnings);
    const ImRect text = GImGui->LastItemData.Rect;
    check(text.GetHeight() > ImGui::GetTextLineHeight() * 2,
          "Long warnings must wrap onto multiple lines");
    check(text.Max.x <= ImGui::GetCurrentWindow()->WorkRect.Max.x + 1,
          "Warning text must fit within a narrow panel");
    ImGui::End();
    ImGui::Render();
  }
}

// The view cube, driven the way a hand drives it: a real press and release at a
// real pixel, and the camera that comes out of it. Every piece of it is a place
// where a sign or an axis can be transposed without looking wrong -- the cube
// still draws, it just sends the camera somewhere else -- so what is checked is
// the whole round trip: click the face that says Top, and end up looking down.
void cube_frame(App& app, ImVec2 canvas) {
  ImGui::NewFrame();
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(canvas);
  ImGui::Begin("view cube", nullptr, ImGuiWindowFlags_NoDecoration);
  view_cube(app, ImGui::GetWindowDrawList(), ImVec2(0, 0), canvas, true);
  ImGui::End();
  ImGui::Render();
}

void near(float a, float b, const char* message, float tolerance = 1e-4f) {
  check(std::abs(a - b) <= tolerance, message);
}

void check_view_cube() {
  const ImVec2 canvas(600, 600);
  auto& io = ImGui::GetIO();

  // Where a cell of a face lands on screen, worked out the way the cube draws
  // it rather than by reaching into it.
  const auto cell_at = [&](const App& app, int axis, int sign, int across,
                           int down) {
    const Basis basis = orbit_basis(app.orbit.yaw, app.orbit.pitch);
    const CubeFace face = cube_face(axis, sign);
    const float u = static_cast<float>(across) * 2.0f / 3.0f;
    const float v = static_cast<float>(down) * 2.0f / 3.0f;
    const Vec3 at{face.normal.x + face.across.x * u + face.down.x * v,
                  face.normal.y + face.across.y * u + face.down.y * v,
                  face.normal.z + face.across.z * u + face.down.z * v};
    const ImVec2 middle = cube_centre(ImVec2(0, 0), canvas);
    const float scale = kCubeBox * 0.5f / kCubeFit;
    return ImVec2{
        middle.x +
            (at.x * basis.right.x + at.y * basis.right.y + at.z * basis.right.z) *
                scale,
        middle.y -
            (at.x * basis.up.x + at.y * basis.up.y + at.z * basis.up.z) * scale};
  };

  const auto press = [&](App& app, ImVec2 position) {
    io.AddMousePosEvent(position.x, position.y);
    cube_frame(app, canvas);
    io.AddMouseButtonEvent(0, true);
    cube_frame(app, canvas);
    io.AddMouseButtonEvent(0, false);
    cube_frame(app, canvas);
    // The turn is eased over about a quarter second; run it out.
    advance_camera(app, 1.0);
  };

  const auto looks_from = [&](const App& app, float x, float y, float z,
                              const char* message) {
    const Basis basis = orbit_basis(app.orbit.yaw, app.orbit.pitch);
    const float length = std::sqrt(x * x + y * y + z * z);
    near(basis.toward.x, x / length, message, 2e-3f);
    near(basis.toward.y, y / length, message, 2e-3f);
    near(basis.toward.z, z / length, message, 2e-3f);
  };

  // Straight down. The clamp used to stop a little short of it, so this is
  // also the check that the poles are reachable at all.
  App app;
  press(app, cell_at(app, 2, 1, 0, 0));
  near(app.orbit.pitch, kQuarterTurn, "The top view is exactly a quarter turn");
  looks_from(app, 0, 0, 1, "Clicking Top looks straight down");

  // And that the basis has not gone degenerate there, which is the reason a
  // clamp short of the pole is usually there in the first place.
  const Basis pole = orbit_basis(app.orbit.yaw, app.orbit.pitch);
  near(pole.right.x * pole.up.x + pole.right.y * pole.up.y +
           pole.right.z * pole.up.z, 0.0f, "Right and up stay perpendicular");
  near(std::sqrt(pole.up.x * pole.up.x + pole.up.y * pole.up.y +
                 pole.up.z * pole.up.z), 1.0f, "Up stays a unit vector");
  near(std::sqrt(pole.toward.x * pole.toward.x + pole.toward.y * pole.toward.y +
                 pole.toward.z * pole.toward.z), 1.0f,
       "Toward stays a unit vector");

  // The named axis views, each clicked on the face carrying its name, against
  // the axis directions the origin picker uses for the same words.
  struct Named { int axis, sign; float x, y, z; const char* name; };
  static const Named kNamed[] = {
      {0, 1, 1, 0, 0, "Right"},  {0, -1, -1, 0, 0, "Left"},
      {1, 1, 0, 1, 0, "Front"},  {1, -1, 0, -1, 0, "Back"},
      {2, 1, 0, 0, 1, "Top"},    {2, -1, 0, 0, -1, "Bottom"},
  };
  for (const Named& named : kNamed) {
    App named_app;
    // Turn to the face first, or it is on the far side of the cube and there
    // is nothing there to click.
    aim_camera(named_app, {named.x, named.y, named.z});
    advance_camera(named_app, 1.0);
    press(named_app, cell_at(named_app, named.axis, named.sign, 0, 0));
    check(std::string(cube_face_name(named.axis, named.sign)) == named.name,
          "Each face carries the origin picker's name for its direction");
    looks_from(named_app, named.x, named.y, named.z,
               "Clicking a named face looks from that direction");
  }

  // A corner cell is the isometric between its three faces.
  App corner;
  press(corner, cell_at(corner, 2, 1, 1, 1));
  looks_from(corner, 1, 1, 1, "A corner of the cube is the isometric view");

  // An edge cell is the view halfway between its two faces.
  App edge;
  press(edge, cell_at(edge, 2, 1, 1, 0));
  looks_from(edge, 1, 0, 1, "An edge of the cube splits its two faces");

  // Azimuth roams, so the angles naming one direction can be many turns from
  // the ones the camera happens to hold. The turn taken has to be the short
  // way round rather than several revolutions back to where it already was.
  App wound;
  wound.orbit.yaw = 0.2f + 6.0f * kQuarterTurn * 2.0f;  // three turns on
  aim_camera(wound, {0, 1, 0});
  check(std::abs(wound.orbit.to_yaw - wound.orbit.from_yaw) <= kQuarterTurn * 2.0f,
        "A snap turns by at most half a revolution");
  advance_camera(wound, 1.0);
  looks_from(wound, 0, 1, 0, "A wound-on azimuth still arrives at the view");

  // A press that turns into a drag is an abandoned click, not a snap: the cube
  // sits inside the canvas, and letting a stray drag across it throw the camera
  // somewhere would make the corner of every view a hazard.
  App dragged;
  const float before_yaw = dragged.orbit.yaw;
  const float before_pitch = dragged.orbit.pitch;
  const ImVec2 from = cell_at(dragged, 2, 1, 0, 0);
  io.AddMousePosEvent(from.x, from.y);
  cube_frame(dragged, canvas);
  io.AddMouseButtonEvent(0, true);
  cube_frame(dragged, canvas);
  io.AddMousePosEvent(from.x + 40.0f, from.y + 25.0f);
  cube_frame(dragged, canvas);
  io.AddMouseButtonEvent(0, false);
  cube_frame(dragged, canvas);
  check(!dragged.orbit.turning, "A drag off the cube does not snap the camera");
  near(dragged.orbit.yaw, before_yaw, "A drag off the cube leaves the azimuth");
  near(dragged.orbit.pitch, before_pitch,
       "A drag off the cube leaves the elevation");
  io.AddMousePosEvent(-1.0f, -1.0f);
  cube_frame(dragged, canvas);
}

}  // namespace

int main() {
  const auto directory = std::filesystem::temp_directory_path() /
      ("tinglecam-tool-tests-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  ImGui::CreateContext();
  auto& io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.DisplaySize = ImVec2(1280, 1000);
  io.DeltaTime = 1.0f / 60.0f;
  unsigned char* pixels;
  int width, height;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  int result = EXIT_SUCCESS;
  try {
    run(directory);
    check_warning_wrapping();
    check_view_cube();
    std::cout << "Tool library, warning, and view cube GUI checks passed.\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    result = EXIT_FAILURE;
  }
  ImGui::DestroyContext();
  std::filesystem::remove_all(directory);
  return result;
}
