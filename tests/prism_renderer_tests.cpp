#include "rendering/prism_renderer.h"
#include <iostream>
#include <stdexcept>
using namespace wave::prism;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
void rectangle(Geometry& g,float x0,float y0,float x1,float y1){
  g.triangles.push_back({{x0,y0,0},{x1,y0,0},{x1,y1,0}});
  g.triangles.push_back({{x0,y0,0},{x1,y1,0},{x0,y1,0}});
}
bool solid(const std::vector<std::uint8_t>& p,int x,int y){return p[(y*64+x)*4]!=14;}
int main(){
  auto* r=create();int status=0;
  try{
    auto ring=std::make_shared<Geometry>();
    rectangle(*ring,-8,-8,8,-3);rectangle(*ring,-8,3,8,8);
    rectangle(*ring,-8,-3,-3,3);rectangle(*ring,3,-3,8,3);
    auto base=std::make_shared<Geometry>();rectangle(*base,-8,-8,8,8);
    Camera camera{{0,0,0},{1,0,0},{0,1,0},{0,0,1},{0,0,1},2,100,32,32};
    update(r,{{ring,2,10}});
    auto pixels=render_host(r,camera,64,64);
    require(!solid(pixels,32,32),"Hole must remain empty without triangulated walls");
    require(solid(pixels,20,32),"Ring face must be visible");
    require(!solid(pixels,4,4),"Outside remains empty");
    update(r,{{ring,2,10},{base,0,2}});
    pixels=render_host(r,camera,64,64);
    require(solid(pixels,32,32),"Lower slab must be visible through upper hole");
    require(stats(r).layer_builds==2,"Two unique layers build two BVHs");
    update(r,{{ring,2,10},{base,0,2}});
    require(stats(r).top_builds==2 && stats(r).top_refits==0,"Unchanged scene does not rebuild or refit");
    update(r,{{ring,4,10},{base,0,4}});
    require(stats(r).layer_builds==2 && stats(r).top_refits==1,"Height change only refits top-level BVH");
    update(r,{{base,0,4},{base,4,10}});
    require(stats(r).layer_builds==2,"Height aliases share one geometry BVH");
    pixels=render_host(r,camera,64,64);
    for(int y=18;y<46;++y)for(int x=18;x<46;++x)
      require(pixels[(y*64+x)*4]==pixels[(32*64+32)*4],"Internal triangle diagonals must not change cap normals");
    auto changed=std::make_shared<Geometry>();rectangle(*changed,-6,-6,6,6);
    update(r,{{base,0,4},{changed,4,10}});
    require(stats(r).layer_builds==3,"Only changed layer rebuilds");
    Camera side{{0,0,5},{0,1,0},{0,0,1},{1,0,0},{1,0,0},2,100,32,32};
    pixels=render_host(r,side,64,64);
    require(solid(pixels,32,32),"Horizontal rays hit prism sides");
    require(!solid(pixels,32,10),"Prisms must not extend below their bottom Z");
    require(!solid(pixels,32,54),"Prisms must not extend above their top Z");
    update(r,{});pixels=render_host(r,camera,64,64);
    require(!solid(pixels,32,32),"Empty scene renders background");
    std::cout<<"Prism intersections, holes, normals and independent BVH updates passed.\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';status=1;}
  destroy(r);return status;
}
