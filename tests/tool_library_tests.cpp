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
    std::cout << "Tool library GUI and persistence checks passed.\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    result = EXIT_FAILURE;
  }
  ImGui::DestroyContext();
  std::filesystem::remove_all(directory);
  return result;
}
