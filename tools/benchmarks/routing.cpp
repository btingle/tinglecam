#include "planning/routing.h"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace {
wave::Path64 box(int x,int y,int w,int h) {
  return {{x,y},{x+w,y},{x+w,y+h},{x,y+h}};
}
}
int main() {
 try {
  wave::Paths64 holes;
  for(int y=0;y<6;++y) for(int x=0;x<6;++x)
    holes.push_back(box(12000+x*14000,12000+y*14000,4000,4000));
  const auto region=Clipper2Lib::Difference(wave::Paths64{box(0,0,100000,100000)},holes,
                                          Clipper2Lib::FillRule::NonZero);
  const auto start=std::chrono::steady_clock::now();
  wave::Router router(region,4);
  const auto built=std::chrono::steady_clock::now();
  std::uint64_t hash=1469598103934665603ull;
  std::size_t points=0;
  for(int i=0;i<32;++i) {
    const int y=2000+i*3000;
    const auto route=router.route({2000,y},{98000,100000-y});
    if(!route)throw std::runtime_error("Expected a route through the obstacle grid");
    points+=route->size();
    for(const auto& p:*route) for(const auto value:{p.x,p.y}) {
      hash^=std::uint64_t(value);hash*=1099511628211ull;
    }
  }
  const auto done=std::chrono::steady_clock::now();
  std::cout<<"corners="<<router.corner_count()<<" build_ms="
           <<std::chrono::duration<double,std::milli>(built-start).count()
           <<" routes_32_ms="<<std::chrono::duration<double,std::milli>(done-built).count()
           <<" route_points="<<points<<" checksum="<<hash<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
