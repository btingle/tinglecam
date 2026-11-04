#include "cam/post.h"
#include <map>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace cam {
namespace {

// A writer that knows one machine's dialect, so the emission code below reads
// as motion rather than as a pile of conditionals.
class Writer {
 public:
  Writer(const Machine& machine, PostResult& result, ExportFlavor flavor)
      : machine_(machine), result_(result), roland_(flavor==ExportFlavor::RolandMdx40a) {}

  void comment(const std::string& text) {
    if(roland_) {
      std::string clean=sanitise(text);
      for(char& c:clean) if(static_cast<unsigned char>(c)<32 || static_cast<unsigned char>(c)>126) c='_';
      line("(" + clean + ")");
      return;
    }
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
    if (machine_.line_numbers && text!="%") {
      out_ << 'N' << number_ << ' ';
      number_ += std::max(machine_.line_number_increment, 1);
    }
    std::string formatted=text;
    if(roland_ && formatted.size()>=2 && (formatted[0]=='G' || formatted[0]=='M') &&
        formatted[1]>='0' && formatted[1]<='9' &&
        (formatted.size()==2 || formatted[2]==' ')) formatted.insert(1,"0");
    out_ << formatted << '\n';
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
        std::snprintf(buffer, sizeof buffer, roland_?"F%.3f":"F%.0f",
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
    if (roland_ && text.find('.') == std::string::npos) text += ".0";
    return text;
  }

 private:
  static std::string sanitise(const std::string& text) {
    std::string clean;
    clean.reserve(text.size());
    for (const char character : text)
      clean.push_back(character == '(' || character == ')' || character == '\n' ||
                         character == '\r' || character == '%' ? '_' : character);
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
        std::snprintf(buffer, sizeof buffer, roland_?"F%.3f":"F%.0f",
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
  bool roland_;
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
                    const Job& document,
                    const ToolLibrary& tools,
                    const MachineLibrary& machines,
                    ExportFlavor flavor) {
  PostResult result;
  const bool roland=flavor==ExportFlavor::RolandMdx40a;
  if (!plan.ok()) {
    result.error = plan.error;
    return result;
  }
  if (plan.operations.empty()) {
    result.error = "Nothing was planned";
    return result;
  }

  std::size_t first_valid=0;
  while(first_valid<plan.operations.size() && !plan.operations[first_valid].ok()) ++first_valid;
  if(first_valid>=plan.operations.size() || first_valid>=document.operations.size()) {
    result.error="No valid operation has a matching setup";return result;
  }
  Job job=document;
  std::string active_setup;
  std::map<std::string,Job> contexts;
  try {
    for(std::size_t i=0;i<document.operations.size() && i<plan.operations.size();++i) {
      const auto& source=document.operations[i];
      if(!plan.operations[i].ok() || document.setups.empty() || contexts.contains(source.setup_id)) continue;
      contexts.emplace(source.setup_id,job_for_setup(document,source.setup_id));
    }
    if(!document.setups.empty() && !document.operations.empty()) {
      active_setup=document.operations[first_valid].setup_id;
      job=contexts.at(active_setup);
      for(const auto& [id,context]:contexts)
        if(context.machine_id!=job.machine_id) {
          result.error="Setups using different machines must be exported separately; select operations from one machine";
          return result;
        }
    }
  } catch(const std::exception& e) { result.error=e.what(); return result; }
  const Machine* machine = machines.find(job.machine_id);
  if (!machine) {
    result.error = job.machine_id.empty() ? "The job has no machine"
                                          : "Unknown machine: " + job.machine_id;
    return result;
  }
  const std::vector<ToolAssignment> tool_table = resolved_tool_table(document);

  // The one rapid plane has to be above everything in the way. A 2D slice
  // cannot certify a lower traverse, so disconnected moves always use it. It is
  // taken from the plan rather than worked out here: the planner costs every
  // lift against this height while choosing whether to lift at all, so the post
  // must not be the place that forms a second opinion about it.
  Writer writer(*machine, result, flavor);
  double rapid_plane = (plan.operations[first_valid].rapid_plane > 0 || !document.setups.empty()
      ? plan.operations[first_valid].rapid_plane : plan.rapid_plane) + job.origin_z;
  // ---- Preamble ---------------------------------------------------------
  if(roland) writer.line("%");
  writer.comment(job.name.empty() ? std::string("TingleCAM") : job.name);
  if(roland) writer.comment("Export flavor: Roland MDX-40A. Fit the tool before starting");
  writer.comment(std::string("Machine: ") + machine->name);
  if(const auto* setup=find_setup(document,active_setup)) writer.comment("Setup: "+setup->name);
  if (machine->specifications_unverified)
    writer.comment("Machine limits in this profile are unverified placeholders");
  writer.comment(machine->output_inches?"Units: inches, absolute":"Units: millimetres, absolute");
  if (!roland && !machine->program_header.empty()) {
    std::istringstream stream(machine->program_header);
    std::string text;
    while (std::getline(stream, text)) writer.line(text);
  }
  writer.line(machine->output_inches ? "G20" : "G21");
  writer.line("G90");
  if(!roland) writer.line("G94");
  writer.line("G17");
  if (!roland && !machine->path_blending.empty()) writer.line(machine->path_blending);
  if(roland) writer.line("G40");
  if(roland && !job.work_offset.empty() && job.work_offset!="G54" &&
      job.work_offset!="G55" && job.work_offset!="G56" && job.work_offset!="G57" &&
      job.work_offset!="G58" && job.work_offset!="G59") {
    result.error="Roland MDX-40A export requires a G54–G59 work coordinate system";
    return result;
  }
  writer.line(job.work_offset.empty() ? "G54" : job.work_offset);

  std::string active_tool;
  bool spindle_running = false;
  bool coolant_running = false;
  double active_rpm=-1;

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
    const Operation* source = operation_index < document.operations.size()
                                  ? &document.operations[operation_index]
                                  : nullptr;
    if(!source) continue;
    if(!document.setups.empty() && source->setup_id!=active_setup) {
      writer.line("G0 Z"+writer.coordinate(rapid_plane));
      if(coolant_running) { writer.line("M9"); coolant_running=false; }
      if(spindle_running) { writer.line("M5"); spindle_running=false; }
      job=contexts.at(source->setup_id);
      const auto* setup=find_setup(document,source->setup_id);
      writer.blank(); writer.comment("Setup: "+(setup?setup->name:source->setup_id));
      if(job.pause_on_entry) writer.line("M0");
      if(roland && job.work_offset!="G54" && job.work_offset!="G55" && job.work_offset!="G56" &&
         job.work_offset!="G57" && job.work_offset!="G58" && job.work_offset!="G59") {
        result.error="Roland MDX-40A export requires a G54–G59 work coordinate system"; return result;
      }
      writer.line(job.work_offset.empty()?"G54":job.work_offset);
      writer.reset_modal(); active_setup=source->setup_id;
    }
    rapid_plane=(!document.setups.empty()?operation.rapid_plane:plan.rapid_plane)+job.origin_z;
    writer.blank();writer.comment("Operation: "+operation.operation_name);
    const auto activate_tool=[&](const Tool* tool,double rpm) {
      const ToolAssignment* assignment = find_tool_assignment(tool_table, tool->id);
      const int tool_number = assignment ? assignment->number : 1;
      const int length_offset = assignment ? assignment->length_offset : tool_number;
      const std::string tool_assignment = tool->id + ":" + std::to_string(tool_number) +
                                          ":" + std::to_string(length_offset);

      const bool changed=active_tool!=tool_assignment;
      if(changed) {
        if(roland && !active_tool.empty())
          throw std::runtime_error("Roland MDX-40A export requires one tool per file. Export checked operations using the same tool.");
        writer.comment("Tool " + std::to_string(tool_number) + " - " + tool->name);
        writer.comment("Cutter " + writer.coordinate(tool->diameter) + " mm");
        if(coolant_running) {writer.line("M9");coolant_running=false;}
        writer.line("G0 Z" + writer.coordinate(rapid_plane));
        if (spindle_running) {
          writer.line("M5");
          spindle_running = false;
        }
        if(roland) {
          writer.comment("Fit tool " + std::to_string(tool_number) + ": " + tool->name);
        } else switch (machine->tool_change) {
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
        if (!roland && machine->length_offset_mode == LengthOffsetMode::G43WithH)
          writer.line("G43 H" + std::to_string(length_offset));
        writer.reset_modal();
        active_tool = tool_assignment;
      }

      if(changed || !spindle_running || active_rpm!=rpm) {
        writer.line(spindle_word(*machine,rpm));
        spindle_running=true;active_rpm=rpm;
        if(machine->spindle_spinup_seconds>0) {
          char buffer[48];
          if(roland) std::snprintf(buffer,sizeof buffer,"G04 P%.0f",std::ceil(machine->spindle_spinup_seconds*1000));
          else std::snprintf(buffer,sizeof buffer,"G4 P%.1f",machine->spindle_spinup_seconds);
          writer.line(buffer);
        }
      }

      if (!source->coolant && coolant_running) {
        writer.line("M9");
        coolant_running = false;
      }
      if (source->coolant && !coolant_running) {
        if (!roland && machine->coolant_flood) { writer.line("M8"); coolant_running = true; }
        else if (!roland && machine->coolant_mist) { writer.line("M7"); coolant_running = true; }
        else result.warnings.push_back(operation.operation_name +
            ": coolant requested but unavailable on this machine");
      }

      return changed;
    };

    bool motion_positioned = false;
    for (const LevelPlan& level : operation.levels) {
      if (level.nothing_to_cut || level.feeds.motion.empty()) continue;
      const auto* tool=tools.find(level.tool_id.empty()?operation.tool_id:level.tool_id);
      if(!tool) {result.error="Unknown tool in planned hole level";return result;}
      try {
        if(activate_tool(tool,level.feeds.stats.spindle_rpm>0?level.feeds.stats.spindle_rpm:operation.spindle_rpm))
          motion_positioned=false;
      } catch(const std::runtime_error& error) {
        result.error=error.what();return result;
      }
      const double cut_z = level.floor_z + job.origin_z;
      writer.blank();
      // The posted depth, not the model's. Every other number in this file is
      // in the program's coordinates and a comment that is not agrees with
      // nothing the operator can read off the machine.
      writer.comment("Depth " + writer.coordinate(cut_z) + " mm, cutting " +
                     writer.coordinate(level.axial_depth) + " mm");

      if (!motion_positioned) {
        // The first schedule starts above its entry point. Establish that XY
        // at clearance before executing its vertical descent.
        const auto& first = level.feeds.motion.front();
        writer.rapid(wave::to_world(first.from.x) + job.origin_x,
                     wave::to_world(first.from.y) + job.origin_y, rapid_plane);
        motion_positioned = true;
      }
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
    }

    writer.line("G0 Z" + writer.coordinate(rapid_plane));
  }

  // ---- Postamble --------------------------------------------------------
  writer.blank();
  if (coolant_running) writer.line("M9");
  if (spindle_running) writer.line("M5");
  writer.line("G0 Z" + writer.coordinate(rapid_plane));
  if (!roland && (machine->tool_change == ToolChange::Automatic ||
      machine->controller == Controller::LinuxCnc))
    writer.line("G0 X" + writer.coordinate(machine->tool_change_x) + " Y" +
                writer.coordinate(machine->tool_change_y));
  if (!roland && !machine->program_footer.empty()) {
    std::istringstream stream(machine->program_footer);
    std::string text;
    while (std::getline(stream, text)) writer.line(text);
  }
  writer.line("M30");
  if(roland) writer.line("%");

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
                          std::size_t operation_index,
                          ExportFlavor flavor) {
  return post_operations(plan, job, tools, machines, {operation_index}, flavor);
}

PostResult post_operations(const JobPlan& plan,
                           const Job& job,
                           const ToolLibrary& tools,
                           const MachineLibrary& machines,
                           const std::vector<std::size_t>& operation_indices,
                           ExportFlavor flavor) {
  PostResult result;
  if (operation_indices.empty()) {
    result.error = "No operations were selected";
    return result;
  }

  JobPlan selected_plan;
  selected_plan.rapid_plane = plan.rapid_plane;
  Job selected_job = job;
  // Numbered as the whole job numbers them.
  selected_job.tools = resolved_tool_table(job);
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
  return post_job(selected_plan, selected_job, tools, machines, flavor);
}

}  // namespace cam
