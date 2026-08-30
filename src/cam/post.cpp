#include "cam/post.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>

namespace cam {
namespace {

// A writer that knows one machine's dialect, so the emission code below reads
// as motion rather than as a pile of conditionals.
class Writer {
 public:
  Writer(const Machine& machine, PostResult& result)
      : machine_(machine), result_(result) {}

  void comment(const std::string& text) {
    switch (machine_.controller) {
      case Controller::Fanuc:
      case Controller::Haas:
      case Controller::Mach3:
        line("(" + sanitise(text) + ")");
        break;
      default:
        line("; " + text);
        break;
    }
  }

  void blank() { out_ << '\n'; }

  void line(const std::string& text) {
    if (text.empty()) {
      out_ << '\n';
      return;
    }
    if (machine_.line_numbers) {
      out_ << 'N' << number_ << ' ';
      number_ += std::max(machine_.line_number_increment, 1);
    }
    out_ << text << '\n';
    ++result_.line_count;
  }

  // Coordinates are only emitted when they actually move, which is most of what
  // keeps a file readable and small.
  void rapid(double x, double y, double z) { motion("G0", x, y, z, 0.0); }
  void feed(double x, double y, double z, double rate) { motion("G1", x, y, z, rate); }

  void reset_modal() {
    has_position_ = false;
    last_feed_ = -1.0;
    last_motion_.clear();
  }

  std::string take() { return out_.str(); }

  std::string coordinate(double value) const {
    const double scaled = machine_.output_inches ? value / 25.4 : value;
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%.*f", machine_.decimals, scaled);
    std::string text = buffer;
    // Trim trailing zeros so the file does not carry noise, but keep one digit.
    if (text.find('.') != std::string::npos) {
      while (text.size() > 1 && text.back() == '0') text.pop_back();
      if (!text.empty() && text.back() == '.') text.pop_back();
    }
    if (text == "-0") text = "0";
    return text;
  }

 private:
  static std::string sanitise(const std::string& text) {
    std::string clean;
    clean.reserve(text.size());
    for (const char character : text)
      clean.push_back(character == '(' || character == ')' ? '_' : character);
    return clean;
  }

  bool differs(double a, double b) const {
    const double tolerance = std::pow(10.0, -machine_.decimals) * 0.5;
    return std::abs(a - b) > tolerance;
  }

  void motion(const std::string& code, double x, double y, double z, double rate) {
    std::string text;
    if (code != last_motion_) {
      text = code;
      last_motion_ = code;
    }
    const auto append = [&](char axis, double value) {
      if (!text.empty()) text += ' ';
      text += axis;
      text += coordinate(value);
    };
    if (!has_position_ || differs(x, position_x_)) append('X', x);
    if (!has_position_ || differs(y, position_y_)) append('Y', y);
    if (!has_position_ || differs(z, position_z_)) append('Z', z);
    if (code == "G1" && rate > 0.0) {
      const double rounded = std::round(rate);
      if (last_feed_ < 0.0 || std::abs(rounded - last_feed_) >= 1.0) {
        if (!text.empty()) text += ' ';
        char buffer[32];
        std::snprintf(buffer, sizeof buffer, "F%.0f",
                      machine_.output_inches ? rounded / 25.4 : rounded);
        text += buffer;
        last_feed_ = rounded;
      }
    }
    position_x_ = x;
    position_y_ = y;
    position_z_ = z;
    has_position_ = true;
    if (text.empty()) return;  // nothing moved and nothing changed
    line(text);
    if (code == "G0") ++result_.rapid_moves;
    else ++result_.cutting_moves;
  }

  const Machine& machine_;
  PostResult& result_;
  std::ostringstream out_;
  int number_ = 10;
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double position_z_ = 0.0;
  bool has_position_ = false;
  double last_feed_ = -1.0;
  std::string last_motion_;
};

std::string spindle_word(const Machine& machine, double rpm) {
  char buffer[64];
  std::snprintf(buffer, sizeof buffer, "M3 S%.0f", std::round(rpm));
  (void)machine;
  return buffer;
}

}  // namespace

PostResult post_job(const JobPlan& plan,
                    const Job& job,
                    const ToolLibrary& tools,
                    const MachineLibrary& machines) {
  PostResult result;
  if (!plan.ok()) {
    result.error = plan.error;
    return result;
  }
  if (plan.operations.empty()) {
    result.error = "Nothing was planned";
    return result;
  }

  // Every operation must agree on the machine: one program goes to one control.
  const Machine* machine = nullptr;
  for (const OperationPlan& operation : plan.operations) {
    if (!operation.ok()) continue;
    const Operation* source = nullptr;
    for (const Operation& candidate : job.operations)
      if (candidate.name == operation.operation_name) source = &candidate;
    if (!source) continue;
    const Machine* found = machines.find(source->machine_id);
    if (!found) continue;
    if (machine && machine->id != found->id) {
      result.error = "The job's operations are for different machines";
      return result;
    }
    machine = found;
  }
  if (!machine) {
    result.error = "No machine could be resolved for this job";
    return result;
  }

  // The clearance and retract planes have to be above everything the job cuts.
  // A job whose depths were set before the models were loaded can easily name a
  // retract inside the work, and a plunge to a Z above where the tool already
  // is would drive it up through the part.
  double highest_cut = -std::numeric_limits<double>::infinity();
  for (const OperationPlan& operation : plan.operations)
    for (const LevelPlan& level : operation.levels)
      if (!level.nothing_to_cut)
        highest_cut = std::max(highest_cut, level.floor_z + level.axial_depth);

  Writer writer(*machine, result);
  double clearance = job.clearance_z + job.origin_z;
  double retract = job.retract_z + job.origin_z;
  if (std::isfinite(highest_cut)) {
    const double top = highest_cut + job.origin_z;
    if (retract < top + 0.5 || clearance < top + 1.0) {
      result.warnings.push_back(
          "The retract and clearance planes were at or below the top of the cut; "
          "they have been raised above it");
      retract = std::max(retract, top + 2.0);
      clearance = std::max(clearance, top + 10.0);
    }
    if (clearance < retract) clearance = retract + 5.0;
  }
  // ---- Preamble ---------------------------------------------------------
  writer.comment(job.name.empty() ? std::string("TingleCAM") : job.name);
  writer.comment(std::string("Machine: ") + machine->name);
  if (machine->specifications_unverified)
    writer.comment("Machine limits in this profile are unverified placeholders");
  writer.comment("Units: millimetres, absolute");
  if (!machine->program_header.empty()) {
    std::istringstream stream(machine->program_header);
    std::string text;
    while (std::getline(stream, text)) writer.line(text);
  }
  writer.line(machine->output_inches ? "G20" : "G21");
  writer.line("G90");
  writer.line("G94");
  writer.line("G17");
  if (!machine->path_blending.empty()) writer.line(machine->path_blending);
  writer.line(job.work_offset.empty() ? "G54" : job.work_offset);

  std::string active_tool;
  bool spindle_running = false;
  bool coolant_running = false;

  for (const OperationPlan& operation : plan.operations) {
    if (!operation.ok()) {
      writer.blank();
      writer.comment("Skipped " + operation.operation_name + ": " + operation.error);
      result.warnings.push_back("Skipped " + operation.operation_name + ": " +
                                operation.error);
      continue;
    }
    const Operation* source = nullptr;
    for (const Operation& candidate : job.operations)
      if (candidate.name == operation.operation_name) source = &candidate;
    if (!source) continue;
    const Tool* tool = tools.find(operation.tool_id);
    if (!tool) continue;

    writer.blank();
    writer.comment("Operation: " + operation.operation_name);
    writer.comment("Tool " + std::to_string(tool->tool_number) + " - " + tool->name);
    writer.comment("Cutter " + writer.coordinate(tool->diameter) + " mm");

    if (active_tool != tool->id) {
      writer.line("G0 Z" + writer.coordinate(clearance));
      if (spindle_running) {
        writer.line("M5");
        spindle_running = false;
      }
      switch (machine->tool_change) {
        case ToolChange::Automatic:
          writer.line("T" + std::to_string(tool->tool_number) + " M6");
          break;
        case ToolChange::SemiAutomatic:
          writer.line("T" + std::to_string(tool->tool_number));
          writer.line("M6");
          break;
        case ToolChange::Manual:
          writer.comment("Fit tool " + std::to_string(tool->tool_number) + ": " +
                         tool->name);
          writer.line("M0");
          break;
      }
      if (machine->length_offset_mode == LengthOffsetMode::G43WithH)
        writer.line("G43 H" + std::to_string(tool->length_offset));
      writer.reset_modal();
      active_tool = tool->id;
    }

    writer.line(spindle_word(*machine, operation.spindle_rpm));
    spindle_running = true;
    if (machine->spindle_spinup_seconds > 0.0) {
      char buffer[48];
      std::snprintf(buffer, sizeof buffer, "G4 P%.1f", machine->spindle_spinup_seconds);
      writer.line(buffer);
    }

    const CuttingData* data = tool->data_for(source->material_id);
    const bool wants_coolant =
        data && (data->coolant == CoolantNeed::Required ||
                 data->coolant == CoolantNeed::Recommended);
    if (wants_coolant && !coolant_running) {
      if (machine->coolant_flood) { writer.line("M8"); coolant_running = true; }
      else if (machine->coolant_mist) { writer.line("M7"); coolant_running = true; }
      else if (data->coolant == CoolantNeed::Required)
        result.warnings.push_back(operation.operation_name +
                                  ": the tool asks for coolant this machine has not got");
    }

    for (const LevelPlan& level : operation.levels) {
      if (level.nothing_to_cut || level.feeds.segments.empty()) continue;
      const double cut_z = level.floor_z + job.origin_z;
      writer.blank();
      writer.comment("Depth " + writer.coordinate(level.floor_z) + " mm, cutting " +
                     writer.coordinate(level.axial_depth) + " mm");

      bool at_depth = false;
      for (const FeedSegment& segment : level.feeds.segments) {
        const double from_x = wave::to_world(segment.from.x) + job.origin_x;
        const double from_y = wave::to_world(segment.from.y) + job.origin_y;
        const double to_x = wave::to_world(segment.to.x) + job.origin_x;
        const double to_y = wave::to_world(segment.to.y) + job.origin_y;

        if (segment.kind == wave::MoveKind::Retract) {
          if (at_depth) {
            // Straight up where it stands, then across. Lifting and traversing
            // together would drag the cutter through the wall it is leaving.
            writer.rapid(from_x, from_y, cut_z);
            writer.rapid(from_x, from_y, retract);
            at_depth = false;
          }
          writer.rapid(to_x, to_y, retract);
          continue;
        }

        if (!at_depth) {
          // Across at height first, then straight down. Combining the two into
          // one diagonal rapid would take the tool through whatever stands
          // between where it was and where it is going.
          writer.rapid(from_x, from_y, retract);
          const double plunge = data ? data->plunge_feed : 100.0;
          writer.feed(from_x, from_y, cut_z, std::max(plunge, 1.0));
          at_depth = true;
        }

        // The rate is the schedule's own: a disengaged piece at depth is a
        // feed at the machine's cutting maximum rather than a rapid, and it is
        // timed at that same figure, so the estimate cannot drift from what
        // this writes.
        writer.feed(to_x, to_y, cut_z, segment.rate);
      }

      if (at_depth) {
        // Leave the pass by lifting where it finished rather than dragging out.
        const double last_x =
            wave::to_world(level.feeds.segments.back().to.x) + job.origin_x;
        const double last_y =
            wave::to_world(level.feeds.segments.back().to.y) + job.origin_y;
        writer.rapid(last_x, last_y, retract);
      }
    }

    writer.line("G0 Z" + writer.coordinate(clearance));
  }

  // ---- Postamble --------------------------------------------------------
  writer.blank();
  if (coolant_running) writer.line("M9");
  if (spindle_running) writer.line("M5");
  writer.line("G0 Z" + writer.coordinate(clearance));
  if (machine->tool_change == ToolChange::Automatic ||
      machine->controller == Controller::LinuxCnc)
    writer.line("G0 X" + writer.coordinate(machine->tool_change_x) + " Y" +
                writer.coordinate(machine->tool_change_y));
  if (!machine->program_footer.empty()) {
    std::istringstream stream(machine->program_footer);
    std::string text;
    while (std::getline(stream, text)) writer.line(text);
  }
  writer.line("M30");

  result.text = writer.take();
  result.estimated_minutes = plan.stats.cutting_minutes + plan.stats.rapid_minutes +
                             plan.stats.tool_change_minutes;

  if (machine->specifications_unverified)
    result.warnings.push_back(
        "This machine profile carries unverified limits; check the feeds against "
        "the machine before running the file");
  return result;
}

}  // namespace cam
