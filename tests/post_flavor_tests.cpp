#include "cam/post.h"

#include <iostream>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
void check(bool value,const std::string& message) {
  if(!value) throw std::runtime_error(message);
}
void check_roland(const std::string& text) {
  check(text.starts_with("%\n") && text.ends_with("M30\n%\n"),"Unnumbered program markers wrap the program");
  check(text.find(';')==std::string::npos,"No semicolon comments");
  const std::set<std::string> g_codes{"G00","G01","G02","G03","G04","G17","G20","G21","G40","G54","G55","G56","G57","G58","G59","G90"};
  const std::set<std::string> m_codes{"M03","M05","M30"};
  const std::regex word("([A-Z])([-+]?[0-9]+(?:\\.[0-9]+)?)");
  std::istringstream lines(text);std::string line;
  while(std::getline(lines,line)) {
    if(line.empty() || line=="%" || line.starts_with('(')) continue;
    if(const auto comment=line.find('(');comment!=std::string::npos) line.erase(comment);
    for(auto it=std::sregex_iterator(line.begin(),line.end(),word);it!=std::sregex_iterator();++it) {
      const auto address=(*it)[1].str(),value=(*it)[2].str(),token=it->str();
      if(address=="G") check(g_codes.contains(token),"Unsupported Roland G-code: "+token);
      else if(address=="M") check(m_codes.contains(token),"Unsupported Roland M-code: "+token);
      else if(address=="P" || address=="N" || address=="S")
        check(value.find('.')==std::string::npos,"Integer argument required: "+token);
      else {
        check(address=="X" || address=="Y" || address=="Z" || address=="I" || address=="J" || address=="F",
              "Unexpected address: "+address);
        check(value.find('.')!=std::string::npos,"Explicit decimal required: "+token);
      }
    }
  }
}
void run() {
  cam::Tool tool;tool.id="drill";tool.name="Drill";tool.diameter=3.175;
  auto second_tool=tool;second_tool.id="mill";
  const cam::ToolLibrary tools{{tool,second_tool}};
  cam::Machine machine;machine.id="machine";machine.name="Test machine";
  machine.spindle_spinup_seconds=4;
  cam::MachineLibrary machines{{machine}};
  cam::Job job;job.name="Test (job)\n%";job.machine_id=machine.id;
  cam::Operation operation;operation.tool_id=tool.id;operation.name="Drilling";
  job.operations={operation};
  cam::JobPlan plan;plan.rapid_plane=10;
  cam::OperationPlan planned;planned.planned=true;planned.operation_name=operation.name;
  planned.tool_id=tool.id;planned.spindle_rpm=12000;
  cam::LevelPlan level;level.floor_z=-2;level.axial_depth=2;
  cam::MotionPhase phase;phase.from={0,0};phase.to={wave::to_integer(-46),wave::to_integer(-34)};
  phase.from_z=10;phase.to_z=-1;phase.rate=500;
  level.feeds.motion.push_back(phase);
  phase.from=phase.to;phase.from_z=-1;phase.to_z=-2;
  phase.arc=true;phase.arc_center={wave::to_integer(-47),wave::to_integer(-34)};
  level.feeds.motion.push_back(phase);
  phase.arc=false;phase.from_z=-2;phase.to_z=10;phase.rapid=true;
  level.feeds.motion.push_back(phase);
  planned.levels.push_back(level);plan.operations.push_back(planned);
  const auto native=cam::post_job(plan,job,tools,machines);
  check(native.ok() && native.text.starts_with("; "),"Default flavor retains machine comments");
  check(native.text.find("G94\n")!=std::string::npos && native.text.find("M0\n")!=std::string::npos,
        "Default flavor retains its feed mode and manual tool pause");
  const auto roland=cam::ExportFlavor::RolandMdx40a;
  auto output=cam::post_job(plan,job,tools,machines,roland);
  check(output.ok(),output.error);check_roland(output.text);
  check(output.text.find("G04 P4000\n")!=std::string::npos,"Four-second dwell uses integer milliseconds");
  check(output.text.find("X-46.0 Y-34.0 Z-1.0")!=std::string::npos,"Coordinates retain millimetre values");
  check(output.text.find("F500.000")!=std::string::npos,"Feed has explicit decimal units");
  check(output.text.find("I-1.0 J0.0")!=std::string::npos,"Helical arc offsets retain explicit decimals");

  auto& m=machines.machines.front();m.line_numbers=true;m.tool_change=cam::ToolChange::Automatic;
  m.length_offset_mode=cam::LengthOffsetMode::G43WithH;m.path_blending="G64 P0.02";
  m.program_header="G94";m.program_footer="M0";m.coolant_flood=true;
  job.operations.front().coolant=true;
  output=cam::post_job(plan,job,tools,machines,roland);
  check(output.ok(),output.error);check_roland(output.text);
  check(output.text.find("G43")==std::string::npos && output.text.find("G64")==std::string::npos,
        "Machine-specific offsets and blending are omitted");
  m.output_inches=true;
  output=cam::post_job(plan,job,tools,machines,roland);
  check(output.ok(),output.error);check_roland(output.text);
  check(output.text.find("G20\n")!=std::string::npos && output.text.find("F19.685")!=std::string::npos,
        "Inch output converts feed units and keeps decimal precision");
  m.output_inches=false;m.line_numbers=false;

  job.operations.push_back(operation);plan.operations.push_back(planned);
  output=cam::post_operations(plan,job,tools,machines,{1,0,1},roland);
  check(output.ok(),output.error);check_roland(output.text);
  job.operations[1].tool_id=second_tool.id;plan.operations[1].tool_id=second_tool.id;
  output=cam::post_job(plan,job,tools,machines,roland);
  check(!output.ok() && output.text.empty() && output.error.find("one tool per file")!=std::string::npos,
        "Reject multi-tool export without emitting an executable partial program");
  output=cam::post_operation(plan,job,tools,machines,1,roland);
  check(output.ok(),output.error);check_roland(output.text);
  job.work_offset="G54.1 P1";
  check(!cam::post_operation(plan,job,tools,machines,1,roland).ok(),"Reject unsupported work offsets");
}
}
int main() {
  try {run();std::cout<<"Export flavor checks passed\n";return 0;}
  catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
