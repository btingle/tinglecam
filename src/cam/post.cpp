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
  void arc(double x, double y, double z, double i, double j,
           bool clockwise, double rate) {
    const std::string code = clockwise ? "G2" : "G3";
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
    // I/J are offsets from the arc start and must be emitted even for a full
    // circle whose XY endpoint is identical to its start.
    append('I', i);
    append('J', j);
    if (rate > 0.0) {
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
    line(text);
    ++result_.cutting_moves;
  }

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
  for (std::size_t operation_index = 0;
       operation_index < plan.operations.size(); ++operation_index) {
    const OperationPlan& operation = plan.operations[operation_index];
    if (!operation.ok()) continue;
    const std::string machine_id =
        !operation.machine_id.empty()
            ? operation.machine_id
            : (operation_index < job.operations.size()
                   ? job.operations[operation_index].machine_id
                   : std::string());
    const Machine* found = machines.find(machine_id);
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

  // The one rapid plane has to be above everything in the way. A 2D slice
  // cannot certify a lower traverse, so disconnected moves always use it. It is
  // taken from the plan rather than worked out here: the planner costs every
  // lift against this height while choosing whether to lift at all, so the post
  // must not be the place that forms a second opinion about it.
  Writer writer(*machine, result);
  const double rapid_plane = plan.rapid_plane + job.origin_z;
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

  for (std::size_t operation_index = 0;
       operation_index < plan.operations.size(); ++operation_index) {
    const OperationPlan& operation = plan.operations[operation_index];
    if (!operation.ok()) {
      writer.blank();
      writer.comment("Skipped " + operation.operation_name + ": " + operation.error);
      result.warnings.push_back("Skipped " + operation.operation_name + ": " +
                                operation.error);
      continue;
    }
    const Operation* source = operation_index < job.operations.size()
                                  ? &job.operations[operation_index]
                                  : nullptr;
    const Tool* tool = tools.find(operation.tool_id);
    if (!tool || !source) continue;

    const int tool_number = source->tool_number > 0 ? source->tool_number :
        static_cast<int>(tool - tools.tools.data()) + 1;
    const int length_offset = source->length_offset > 0 ? source->length_offset : tool_number;
    const std::string tool_assignment = tool->id + ":" + std::to_string(tool_number) +
                                        ":" + std::to_string(length_offset);
    writer.blank();
    writer.comment("Operation: " + operation.operation_name);
    writer.comment("Tool " + std::to_string(tool_number) + " - " + tool->name);
    writer.comment("Cutter " + writer.coordinate(tool->diameter) + " mm");

    if (active_tool != tool_assignment) {
      writer.line("G0 Z" + writer.coordinate(rapid_plane));
      if (spindle_running) {
        writer.line("M5");
        spindle_running = false;
      }
      switch (machine->tool_change) {
        case ToolChange::Automatic:
          writer.line("T" + std::to_string(tool_number) + " M6");
          break;
        case ToolChange::SemiAutomatic:
          writer.line("T" + std::to_string(tool_number));
          writer.line("M6");
          break;
        case ToolChange::Manual:
          writer.comment("Fit tool " + std::to_string(tool_number) + ": " +
                         tool->name);
          writer.line("M0");
          break;
      }
      if (machine->length_offset_mode == LengthOffsetMode::G43WithH)
        writer.line("G43 H" + std::to_string(length_offset));
      writer.reset_modal();
      active_tool = tool_assignment;
    }

    writer.line(spindle_word(*machine, operation.spindle_rpm));
    spindle_running = true;
    if (machine->spindle_spinup_seconds > 0.0) {
      char buffer[48];
      std::snprintf(buffer, sizeof buffer, "G4 P%.1f", machine->spindle_spinup_seconds);
      writer.line(buffer);
    }

    if (!source->coolant && coolant_running) {
      writer.line("M9");
      coolant_running = false;
    }
    if (source->coolant && !coolant_running) {
      if (machine->coolant_flood) { writer.line("M8"); coolant_running = true; }
      else if (machine->coolant_mist) { writer.line("M7"); coolant_running = true; }
      else result.warnings.push_back(operation.operation_name +
          ": coolant requested but unavailable on this machine");
    }

    // Carried across the passes rather than reset at each one, because a pass
    // can now be reached without lifting: the pass above ends over this one's
    // first point, at its own floor, and the only move between them is the
    // descent. Where that has not happened these stay exactly as they were --
    // false at the top of every pass -- and the tool crosses at the plane.
    bool at_depth = false;
    bool rapid_descent = false;
    bool motion_positioned = false;
    double depth_z = 0.0;
    const double plunge_rate = std::max(source->plunge_feed, 1.0);
    for (const LevelPlan& level : operation.levels) {
      if (level.nothing_to_cut ||
          (level.feeds.segments.empty() && level.feeds.motion.empty())) continue;
      const double cut_z = level.floor_z + job.origin_z;
      writer.blank();
      // The posted depth, not the model's. Every other number in this file is
      // in the program's coordinates and a comment that is not agrees with
      // nothing the operator can read off the machine.
      writer.comment("Depth " + writer.coordinate(cut_z) + " mm, cutting " +
                     writer.coordinate(level.axial_depth) + " mm");

      if (level.explicit_motion || level.feeds.authoritative_motion) {
        if (level.feeds.authoritative_motion && !motion_positioned &&
            !level.feeds.motion.empty()) {
          // The first schedule starts above its entry point. Establish that
          // XY at clearance before executing its vertical descent.
          const auto& first = level.feeds.motion.front();
          writer.rapid(wave::to_world(first.from.x) + job.origin_x,
                       wave::to_world(first.from.y) + job.origin_y, rapid_plane);
          motion_positioned = true;
        }
        // XYZ phases are authoritative for helices: flattening these back to
        // FeedSegment would turn the entry into a vertical plunge.
        for (const MotionPhase& phase : level.feeds.motion) {
          const double x = wave::to_world(phase.to.x) + job.origin_x;
          const double y = wave::to_world(phase.to.y) + job.origin_y;
          const double z = phase.to_z + job.origin_z;
          if (phase.rapid) writer.rapid(x, y, z);
          else if (phase.arc)
            writer.arc(x, y, z,
                       wave::to_world(phase.arc_center.x - phase.from.x),
                       wave::to_world(phase.arc_center.y - phase.from.y),
                       phase.clockwise, phase.rate);
          else writer.feed(x, y, z, phase.rate);
        }
        at_depth = false;
        continue;
      }

      // The opening retract of a pass reached at depth is the one thing that
      // must not be written out: it exists to prove the descent after it lands
      // in cleared air, and writing it would lift the tool back to the plane
      // that staying down was the whole point of avoiding. What it proves is
      // still taken from it.
      bool opening_retract = level.feeds.begins_at_depth && at_depth;
      for (const FeedSegment& segment : level.feeds.segments) {
        const double from_x = wave::to_world(segment.from.x) + job.origin_x;
        const double from_y = wave::to_world(segment.from.y) + job.origin_y;
        const double to_x = wave::to_world(segment.to.x) + job.origin_x;
        const double to_y = wave::to_world(segment.to.y) + job.origin_y;

        if (segment.kind == wave::MoveKind::Retract) {
          if (opening_retract) {
            rapid_descent = segment.safe_retract;
            continue;
          }
          if (at_depth) {
            // Straight up where it stands, then across. Lifting and traversing
            // together would drag the cutter through the wall it is leaving.
            writer.rapid(from_x, from_y, depth_z);
            writer.rapid(from_x, from_y, rapid_plane);
            at_depth = false;
          }
          writer.rapid(to_x, to_y, rapid_plane);
          rapid_descent = segment.safe_retract;
          continue;
        }
        opening_retract = false;

        if (at_depth && depth_z != cut_z) {
          // Reached without lifting: the cutter already stands over this pass's
          // first point, one pass depth above it.
          if (rapid_descent) writer.rapid(from_x, from_y, cut_z);
          else writer.feed(from_x, from_y, cut_z, plunge_rate);
          depth_z = cut_z;
          rapid_descent = false;
        } else if (!at_depth) {
          // Across at height first, then straight down. Combining the two into
          // one diagonal rapid would take the tool through whatever stands
          // between where it was and where it is going.
          writer.rapid(from_x, from_y, rapid_plane);
          if (rapid_descent && segment.kind != wave::MoveKind::Entry) {
            // Retract landings are checked against the simulated remaining
            // stock. Where that footprint is clear, returning to depth is a
            // positioning move rather than a cutting plunge.
            writer.rapid(from_x, from_y, cut_z);
          } else {
            writer.feed(from_x, from_y, cut_z, plunge_rate);
          }
          at_depth = true;
          depth_z = cut_z;
          rapid_descent = false;
        }

        // The rate is the schedule's own: a disengaged piece at depth is a
        // feed at the machine's cutting maximum rather than a rapid, and it is
        // timed at that same figure, so the estimate cannot drift from what
        // this writes.
        writer.feed(to_x, to_y, cut_z, segment.rate);
      }

      if (at_depth && !level.feeds.ends_at_depth) {
        // Leave the pass by lifting where it finished rather than dragging out.
        const double last_x =
            wave::to_world(level.feeds.segments.back().to.x) + job.origin_x;
        const double last_y =
            wave::to_world(level.feeds.segments.back().to.y) + job.origin_y;
        writer.rapid(last_x, last_y, rapid_plane);
        at_depth = false;
      }
    }

    writer.line("G0 Z" + writer.coordinate(rapid_plane));
  }

  // ---- Postamble --------------------------------------------------------
  writer.blank();
  if (coolant_running) writer.line("M9");
  if (spindle_running) writer.line("M5");
  writer.line("G0 Z" + writer.coordinate(rapid_plane));
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

PostResult post_operation(const JobPlan& plan,
                          const Job& job,
                          const ToolLibrary& tools,
                          const MachineLibrary& machines,
                          std::size_t operation_index) {
  return post_operations(plan, job, tools, machines, {operation_index});
}

PostResult post_operations(const JobPlan& plan,
                           const Job& job,
                           const ToolLibrary& tools,
                           const MachineLibrary& machines,
                           const std::vector<std::size_t>& operation_indices) {
  PostResult result;
  if (operation_indices.empty()) {
    result.error = "No operations were selected";
    return result;
  }

  JobPlan selected_plan;
  selected_plan.rapid_plane = plan.rapid_plane;
  Job selected_job = job;
  selected_job.operations.clear();
  std::vector<std::size_t> sorted_indices = operation_indices;
  std::sort(sorted_indices.begin(), sorted_indices.end());
  sorted_indices.erase(std::unique(sorted_indices.begin(), sorted_indices.end()),
                       sorted_indices.end());
  for (const std::size_t index : sorted_indices) {
    if (index >= plan.operations.size() || index >= job.operations.size()) {
      result.error = "Operation index is out of range";
      return result;
    }
    if (!plan.operations[index].planned || !plan.operations[index].ok()) {
      result.error = "A selected operation has no valid plan";
      return result;
    }
    selected_plan.operations.push_back(plan.operations[index]);
    selected_job.operations.push_back(job.operations[index]);
  }
  summarise_job(selected_plan);
  selected_plan.error.clear();
  return post_job(selected_plan, selected_job, tools, machines);
}

}  // namespace cam
