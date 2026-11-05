#include "cam/job.h"

#include <atomic>
#include <chrono>
#include <set>
#include <stdexcept>

namespace cam {
std::string new_document_id() {
  static std::atomic<std::uint64_t> serial{0};
  static const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
  return "id-" + std::to_string(stamp) + "-" + std::to_string(++serial);
}
const Setup* find_setup(const Job& job, const std::string& id) {
  for (const auto& setup : job.setups) if(setup.id == id) return &setup;
  return nullptr;
}
void ensure_setup_ids(Job& job) {
  if(job.setups.empty()) {
    Setup setup;
    static_cast<SetupContext&>(setup)=job;
    setup.id=new_document_id(); setup.name="Setup 1";
    job.setups.push_back(std::move(setup));
  }
  for(auto& setup:job.setups) if(setup.id.empty()) setup.id=new_document_id();
  for(auto& operation:job.operations) {
    if(operation.id.empty()) operation.id=new_document_id();
    if(operation.setup_id.empty()) operation.setup_id=job.setups.front().id;
  }
}
SetupContext resolve_setup_context(const Job& job, const std::string& id) {
  if(job.setups.empty()) {
    if(!id.empty()) throw std::runtime_error("Unknown setup: "+id);
    return job;
  }
  const auto* setup=find_setup(job,id);
  if(!setup) throw std::runtime_error("Unknown setup: "+id);
  SetupContext context=*setup;
  const auto parent=[&](const std::string& source) {
    const auto* previous=find_setup(job,source);
    if(!previous || previous>=setup)
      throw std::runtime_error("Setup inheritance must reference an earlier setup: "+source);
    return resolve_setup_context(job,source);
  };
  if(!setup->properties_from_setup.empty()) {
    const auto p=parent(setup->properties_from_setup);
    // Explicit geometry and property inheritance are independent.
    context.machine_id=p.machine_id; context.material_id=p.material_id;
    context.rapid_clearance_height=p.rapid_clearance_height;
    context.plunge_clearance_height=p.plunge_clearance_height;
    context.origin_x=p.origin_x; context.origin_y=p.origin_y; context.origin_z=p.origin_z;
    context.origin_relative_to_part=p.origin_relative_to_part;
    context.work_offset=p.work_offset;
  }
  if(!setup->part_from_setup.empty()) {
    const auto p=parent(setup->part_from_setup);
    context.part_mesh=p.part_mesh;
    // Local placement stays independent of the inherited source resource.
  }
  if(!setup->fixtures_from_setup.empty()) {
    auto inherited=parent(setup->fixtures_from_setup).fixtures;
    inherited.insert(inherited.end(),context.fixtures.begin(),context.fixtures.end());
    context.fixtures=std::move(inherited);
  }
  if(context.machine_id.empty()) context.machine_id=job.machine_id;
  if(context.material_id.empty()) context.material_id=job.material_id;
  return context;
}
Job job_for_setup(const Job& job, const std::string& id) {
  Job result=job;
  static_cast<SetupContext&>(result)=resolve_setup_context(job,id);
  result.setups.clear();
  result.operations.clear();
  for(const auto& operation:job.operations)
    if(job.setups.empty() || operation.setup_id==id) result.operations.push_back(operation);
  return result;
}
std::vector<std::string> validate_setups(const Job& job) {
  std::vector<std::string> errors;
  std::set<std::string> setup_ids, operation_ids;
  for(const auto& setup:job.setups) {
    if(setup.id.empty() || !setup_ids.insert(setup.id).second)
      errors.push_back("Setups require unique nonempty IDs");
    try { (void)resolve_setup_context(job,setup.id); }
    catch(const std::exception& e) {errors.push_back(e.what());}
  }
  std::size_t latest=0;
  for(std::size_t i=0;i<job.operations.size();++i) {
    const auto& op=job.operations[i];
    if(!op.id.empty() && !operation_ids.insert(op.id).second)
      errors.push_back("Duplicate operation ID: "+op.id);
    if(job.setups.empty()) continue;
    const auto* setup=find_setup(job,op.setup_id);
    if(!setup) { errors.push_back("Operation references a missing setup: "+op.name); continue; }
    const auto position=std::size_t(setup-job.setups.data());
    if(position<latest) errors.push_back("Operations must remain grouped in setup order");
    latest=position;
    SetupContext context;
    try { context=resolve_setup_context(job,setup->id); }
    catch(const std::exception&) { continue; }
    if(!context.stock_from_operation.empty()) {
      std::size_t source=0;
      while(source<i && job.operations[source].id!=context.stock_from_operation) ++source;
      if(source==i || job.operations[source].setup_id==op.setup_id)
        errors.push_back("Inherited stock must reference an operation in an earlier setup: "+op.name);
      if(!context.stock_transfer.preserves_columns())
        errors.push_back("Inherited stock rotation must preserve the heightmap Z axis; 90 degree X/Y rotations require conversion");
    }
  }
  return errors;
}
}  // namespace cam
