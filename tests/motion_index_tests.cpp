#include "cam/feeds.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

void check(bool v,const char* text) {if(!v)throw std::runtime_error(text);}
void near(double a,double b) {check(std::abs(a-b)<1e-7,"Indexed cursor differs from linear reference");}
int main() {
 try {
  cam::FeedSchedule reference;
  for(int i=0;i<2000;++i) {
   cam::MotionPhase p;
   p.from={i*1000,0};p.to={(i+1)*1000,0};p.from_z=i%5;p.to_z=(i+1)%5;
   p.length=i%11==0?0:std::hypot(1,p.to_z-p.from_z);p.rate=60*(1+i%9);
   p.path_distance_begin=i;p.path_distance_end=i+1;p.rapid=i%3==0;
   if(i%7==0) p.fixture_contacts.push_back({.2,.6,{},{}});
   reference.motion.push_back(p);
  }
  auto indexed=reference;cam::index_feed_motion(indexed);
  near(cam::feed_schedule_seconds(reference),cam::feed_schedule_seconds(indexed));
  near(cam::feed_motion_length(reference),cam::feed_motion_length(indexed));
  const auto compare=[&](double seconds) {
   auto a=cam::feed_motion_at_seconds(reference,seconds), b=cam::feed_motion_at_seconds(indexed,seconds);
   check(a.position==b.position && a.rapid==b.rapid,"Indexed cursor changes position or motion kind");
   check(a.fixture_contact==b.fixture_contact,"Indexed playback changes fixture contact state");
   near(a.z,b.z);near(a.path_distance,b.path_distance);near(a.motion_distance,b.motion_distance);
   near(a.phase_distance,b.phase_distance);near(a.rate,b.rate);
   near(cam::feed_seconds_to(reference,a.path_distance),cam::feed_seconds_to(indexed,a.path_distance));
  };
  compare(-1);compare(0);compare(cam::feed_schedule_seconds(indexed)+100);
  for(auto t:indexed.motion_seconds) {compare(t);compare(t+1e-9);compare(t-1e-9);}
  for(int i=0;i<3000;++i)compare(cam::feed_schedule_seconds(indexed)*i/2999.0);
  indexed.motion.pop_back();reference.motion.pop_back(); // Size changes use the linear fallback.
  compare(200);cam::index_feed_motion(indexed);compare(200);
  cam::FeedSchedule vertical;
  const double begins[]{0,0,1,1,2},ends[]{0,1,1,2,2},lengths[]{10,1,2,1,10};
  for (int i=0;i<5;++i) {
    cam::MotionPhase p;p.path_distance_begin=begins[i];p.path_distance_end=ends[i];
    p.length=lengths[i];p.rate=60;vertical.motion.push_back(p);
  }
  auto indexed_vertical=vertical;cam::index_feed_motion(indexed_vertical);
  near(cam::feed_seconds_to(indexed_vertical,1),13);
  for (double d : {0.,.5,1.,1.+1e-9,2.,3.})
    near(cam::feed_seconds_to(vertical,d),cam::feed_seconds_to(indexed_vertical,d));
  for (auto& p : reference.motion) p.path_distance_begin=p.path_distance_end=0;
  indexed=reference;cam::index_feed_motion(indexed);
  near(cam::feed_seconds_to(reference,1),cam::feed_seconds_to(indexed,1));
  reference.motion[10].path_distance_end=100;
  indexed=reference;cam::index_feed_motion(indexed);
  check(indexed.motion_seconds.empty(),"Unordered custom schedules must keep the linear fallback");
  compare(1);
  std::cout<<"Motion index agrees with linear lookup at boundaries and during travel.\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
