#include "gui/app_ui_api.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition,const char* text) {if(!condition) throw std::runtime_error(text);}
void near(double a,double b,const char* text) {check(std::abs(a-b)<1e-7,text);}
cam::FeedSchedule peck_schedule() {
  cam::FeedSchedule schedule;
  const auto motion=[&](double from,double to,bool rapid) {
    cam::MotionPhase phase;phase.from=phase.to={1000,2000};phase.from_z=from;phase.to_z=to;
    phase.length=std::abs(to-from);phase.rate=rapid?600:60;phase.rapid=rapid;
    phase.kind=rapid?wave::MoveKind::Retract:wave::MoveKind::Entry;
    schedule.motion.push_back(phase);
  };
  motion(20,11,true);motion(11,8,false);motion(8,11,true);
  motion(11,8.1,true);motion(8.1,6,false);
  cam::MotionPhase dwell;dwell.from=dwell.to={1000,2000};dwell.from_z=dwell.to_z=6;
  dwell.dwell_seconds=.5;dwell.rate=60;dwell.kind=wave::MoveKind::Entry;
  schedule.motion.push_back(dwell);motion(6,20,true);
  return schedule;
}
void setup(tinglecam_gui::App& app,bool machine_time,int passes=1) {
  app.has_plan=true;app.playback.playing=true;app.playback.real_time=machine_time;
  app.playback.speed=2;app.selected_plan_operation=0;app.selected_level=0;
  app.plan.operations.resize(1);
  for(int i=0;i<passes;++i) {
    cam::LevelPlan level;level.explicit_motion=true;level.feeds=peck_schedule();
    app.plan.operations.front().levels.push_back(std::move(level));
  }
}
cam::MotionCursor cursor(const tinglecam_gui::App& app) {
  return cam::feed_motion_at_seconds(app.plan.operations[app.selected_plan_operation].levels[app.selected_level].feeds,
                                     app.playback.motion_seconds);
}
void fixed_speed() {
  using namespace tinglecam_gui;
  App app;setup(app,false);
  advance_playback(app,6.75);
  near(cursor(app).z,9.5,"Fixed-speed preview must show the upward peck retract");
  check(cursor(app).rapid,"Upward peck is a rapid");
  check(app.playback.playing,"Zero XY distance must not finish the drilling pass");
  near(app.playback.distance,0,"Material-removal XY cursor may remain stationary");
  near(app.playback.motion_seconds,4.05,"Fixed XYZ speed maps onto the commanded timeline");
  App framed;setup(framed,false);
  for(int i=0;i<135;++i)advance_playback(framed,.05);
  near(framed.playback.motion_seconds,app.playback.motion_seconds,"Frame subdivision must not change preview position");
  App dwell;setup(dwell,false);advance_playback(dwell,10.25);
  near(cursor(dwell).z,6,"Dwell holds the cutter at the bottom at fixed speed");
  near(dwell.playback.motion_seconds,6.84,"Bottom dwell must consume view time");
  advance_playback(dwell,.5);near(cursor(dwell).z,6.5,"Cutter retracts only after bottom dwell completes");
  App followed;setup(followed,false,2);advance_playback(followed,18);
  check(followed.selected_level==1,"Fixed-speed playback follows into the next drilling pass");
  near(cursor(followed).z,19,"Unused preview time carries correctly across passes");
  App completed;setup(completed,false);advance_playback(completed,17.5);
  check(!completed.playback.playing,"Whole fixed-speed peck pass finishes at the expected time");
  near(cursor(completed).z,20,"Finished drilling returns to clearance");
}
void machine_time_and_seek() {
  using namespace tinglecam_gui;
  App app;setup(app,true);advance_playback(app,4.05);
  near(cursor(app).z,9.5,"Machine-time peck retract follows the commanded motion");
  seek_playback(app,4.05);near(cursor(app).z,9.5,"Scrubbing can seek within a vertical peck");
  seek_playback(app,6.84);near(cursor(app).z,6,"Scrubbing can seek inside a dwell");
  const double before=app.playback.motion_seconds;app.playback.real_time=false;
  advance_playback(app,0);near(app.playback.motion_seconds,before,"Switching speed modes must preserve the current peck/dwell");
  seek_playback(app,100);near(cursor(app).z,20,"End seek clamps to the full XYZ timeline");
  seek_playback(app,-1);near(cursor(app).z,20,"Restart seek returns to the opening clearance position");
  App followed;setup(followed,true,2);followed.playback.time_scale=2;
  const double total=cam::feed_schedule_seconds(peck_schedule());advance_playback(followed,(total+.5)/2);
  check(followed.selected_level==1,"Machine time follows drilling passes");
  near(followed.playback.motion_seconds,.5,"Scaled unused time must carry into the next pass");
}
void helper_edge_cases() {
  cam::FeedSchedule dwell;cam::MotionPhase phase;phase.dwell_seconds=2;dwell.motion.push_back(phase);
  auto midway=cam::advance_feed_playback(dwell,0,.5,100);
  near(midway.motion_seconds,.5,"A dwell-only pass must not be skipped");
  auto done=cam::advance_feed_playback(dwell,midway.motion_seconds,2.5,100);
  near(done.motion_seconds,2,"Dwell-only pass completes");near(done.unused_view_seconds,1,"Dwell returns unused view time");
  cam::FeedSchedule empty;auto skipped=cam::advance_feed_playback(empty,0,1,2);
  near(skipped.unused_view_seconds,1,"Empty pass returns all view time");
  cam::FeedSchedule legacy;cam::FeedSegment segment;segment.length=10;segment.rate=60;legacy.segments.push_back(segment);
  auto end=cam::advance_feed_playback(legacy,0,5.2,2);
  near(end.motion_seconds,10,"Legacy planar schedules remain supported");near(end.unused_view_seconds,.2,"Legacy schedule returns unused view time");
}
}
int main() {
 try {fixed_speed();machine_time_and_seek();helper_edge_cases();
  std::cout<<"GUI playback preserves pecks, dwell, scrubbing, XYZ speed and pass transitions.\n";
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
