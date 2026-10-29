#include "prism_renderer.h"
#include "wave/cpu_bvh.h"
#include <atomic>
#include <map>
#include <set>
#include <stdexcept>
#include <thread>

namespace wave::prism {
namespace {
Vec3f add(Vec3f a,Vec3f b) {return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3f mul(Vec3f a,float b) {return {a.x*b,a.y*b,a.z*b};}
float dot(Vec3f a,Vec3f b) {return a.x*b.x+a.y*b.y+a.z*b.z;}
bool plane(Vec3f n,float b,const cpu::Ray& ray,float& enter,float& leave,Vec3f& normal) {
  float numerator=b-dot(n,ray.origin),denominator=dot(n,ray.direction);
  if(std::abs(denominator)<1e-12f) return numerator>=-1e-6f;
  float t=numerator/denominator;
  if(denominator<0) {if(t>enter) {enter=t;normal=n;}}
  else leave=std::min(leave,t);
  return enter<=leave;
}
bool hit_prism(const Triangle3f& tri,const cpu::Ray& ray,float& t,Vec3f& normal) {
  float enter=-INFINITY,leave=ray.max;
  if(!plane({0,0,1},1,ray,enter,leave,normal) || !plane({0,0,-1},0,ray,enter,leave,normal)) return false;
  const Vec3f p[3]={tri.a,tri.b,tri.c};
  for(int j=0;j<3;++j) {
    auto a=p[j],b=p[(j+1)%3]; Vec3f n{b.y-a.y,a.x-b.x,0};
    float length=std::sqrt(dot(n,n)); if(length<1e-12f) return false; n=mul(n,1/length);
    if(!plane(n,n.x*a.x+n.y*a.y,ray,enter,leave,normal)) return false;
  }
  if(enter<ray.min || enter>leave || enter>=ray.max) return false;
  t=enter;return true;
}
struct CachedGeometry {std::shared_ptr<const Geometry> source; cpu::Bvh bvh;};
}
struct Renderer {
  std::map<const Geometry*,CachedGeometry> geometries;
  std::vector<Layer> layers;
  std::vector<const CachedGeometry*> instances;
  cpu::Bvh top;
  Stats counters;
};
Renderer* create() {return new Renderer;}
void destroy(Renderer* r) {delete r;}
Stats stats(const Renderer* r) {return r->counters;}
void update(Renderer* r,const std::vector<Layer>& input) {
  std::vector<Layer> layers;
  for(const auto& l:input) if(l.top>l.bottom && l.geometry && !l.geometry->triangles.empty()) layers.push_back(l);
  if(layers.size()==r->layers.size() && std::equal(layers.begin(),layers.end(),r->layers.begin(),[](const auto& a,const auto& b){
    return a.geometry==b.geometry && a.bottom==b.bottom && a.top==b.top;
  })) return;
  std::set<const Geometry*> live; std::vector<cpu::Box> boxes; r->instances.clear();
  for(const auto& l:layers) {
    auto key=l.geometry.get(); live.insert(key);
    auto [it,inserted]=r->geometries.try_emplace(key);
    if(inserted) {
      it->second.source=l.geometry; std::vector<cpu::Box> triangles;
      for(const auto& t:l.geometry->triangles) {auto b=cpu::bounds(t);b.lo.z=0;b.hi.z=1;triangles.push_back(b);}
      it->second.bvh.build(triangles); ++r->counters.layer_builds;
    }
    r->instances.push_back(&it->second);
    auto b=it->second.bvh.bounds();b.lo.z=l.bottom;b.hi.z=l.top;boxes.push_back(b);
  }
  if(layers.size()!=r->layers.size()) {r->top.build(boxes);if(!layers.empty()) ++r->counters.top_builds;}
  else {r->top.refit(boxes);if(!layers.empty()) ++r->counters.top_refits;}
  r->layers=std::move(layers);
  for(auto it=r->geometries.begin();it!=r->geometries.end();) if(!live.count(it->first)) it=r->geometries.erase(it);else ++it;
}
struct Interop {};
Interop* register_buffer(unsigned) {return nullptr;}
void unregister_buffer(Interop* p) {delete p;}
void render(Renderer*,Interop*,const Camera&,int,int) {throw std::logic_error("CPU renderer uses render_host");}
std::vector<std::uint8_t> render_host(Renderer* r,const Camera& c,int w,int h) {
  if(w<=0 || h<=0 || !(c.scale>0)) throw std::invalid_argument("Invalid render dimensions or scale");
  std::vector<std::uint8_t> pixels(std::size_t(w)*h*4);
  std::atomic<int> next_row{0};
  const auto worker=[&] {
    for(int y;(y=next_row.fetch_add(4))<h;) for(int yy=y;yy<std::min(y+4,h);++yy) for(int x=0;x<w;++x) {
      float px=(x+.5f-c.pan_x)/c.scale,py=(h-yy-.5f-c.pan_y)/c.scale;
      auto origin=add(add(add(c.centre,mul(c.right,px)),mul(c.up,-py)),mul(c.toward,c.reach));
      cpu::Ray ray{origin,mul(c.toward,-1),0,2*c.reach}; Vec3f normal{};bool hit=false;
      r->top.query([&](const cpu::Box& b){return cpu::intersects(b,ray);},[&](unsigned id){
        const auto& l=r->layers[id];const auto& g=*r->instances[id];float height=l.top-l.bottom;
        auto local=ray;local.origin.z=(local.origin.z-l.bottom)/height;local.direction.z/=height;
        g.bvh.query([&](const cpu::Box& b){return cpu::intersects(b,local);},[&](unsigned primitive){
          float t;Vec3f n;
          if(hit_prism(g.source->triangles[primitive],local,t,n)) {local.max=t;normal=n;hit=true;}
          return true;
        });ray.max=local.max;return true;
      });
      auto offset=(std::size_t(yy)*w+x)*4;
      if(hit) {
        float shade=.42f+.58f*std::max(0.f,dot(normal,c.light));
        pixels[offset]=255*.54f*shade;pixels[offset+1]=255*.61f*shade;pixels[offset+2]=255*.70f*shade;
      } else {pixels[offset]=14;pixels[offset+1]=17;pixels[offset+2]=22;}
      pixels[offset+3]=255;
    }
  };
  unsigned count=std::min({std::max(1u,std::thread::hardware_concurrency()),16u,unsigned((h+3)/4)});
  if(std::size_t(w)*h<65536) count=1;
  std::vector<std::jthread> workers;
  for(unsigned i=1;i<count;++i) workers.emplace_back(worker);
  worker();
  // Join before moving the pixel storage into the return value.
  workers.clear();return pixels;
}
}
