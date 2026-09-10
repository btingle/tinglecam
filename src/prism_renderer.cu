#include "prism_renderer.h"
#include <cuBQL/bvh.h>
#include <cuBQL/traversal/rayQueries.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace wave::prism {
using cuBQL::vec3f; using cuBQL::box3f; using cuBQL::bvh3f; using cuBQL::ray3f;
static void check(cudaError_t status,const char* context) {
  if(status!=cudaSuccess) throw std::runtime_error(std::string(context)+": "+cudaGetErrorString(status));
}
struct GpuLayer { bvh3f bvh; const Triangle3f* triangles; float bottom,top; };
struct GeometryGpu {
  std::shared_ptr<const Geometry> source;
  Triangle3f* triangles=nullptr; box3f* boxes=nullptr; bvh3f bvh{};
  ~GeometryGpu(){ cuBQL::cuda::free(bvh);cudaFree(triangles);cudaFree(boxes); }
};
struct Renderer {
  std::map<const Geometry*,std::unique_ptr<GeometryGpu>> geometries;
  std::vector<Layer> layers;
  GpuLayer* gpu_layers=nullptr; box3f* boxes=nullptr; bvh3f top{};
  uchar4* output=nullptr;std::size_t output_capacity=0;
  Stats counters;
  ~Renderer(){cuBQL::cuda::free(top);cudaFree(gpu_layers);cudaFree(boxes);cudaFree(output);}
};
__host__ __device__ static vec3f v(Vec3f p){return {p.x,p.y,p.z};}
__global__ void triangle_bounds(box3f* boxes,const Triangle3f* triangles,unsigned count){
  unsigned i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=count)return;
  const auto t=triangles[i];
  boxes[i]=box3f().extend(vec3f(t.a.x,t.a.y,0)).extend(vec3f(t.b.x,t.b.y,0))
      .extend(vec3f(t.c.x,t.c.y,0)).extend(vec3f(t.a.x,t.a.y,1));
}
__global__ void layer_bounds(box3f* boxes,const GpuLayer* layers,unsigned count){
  unsigned i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=count)return;
  boxes[i]=layers[i].bvh.nodes[0].bounds;boxes[i].lower.z=layers[i].bottom;boxes[i].upper.z=layers[i].top;
}
// Intersect five half-spaces, not an extruded triangle mesh. Plane normals
// point out of the prism; the plane that establishes entry supplies shading.
__device__ bool plane(vec3f n,float b,const ray3f& ray,float& enter,float& leave,vec3f& normal){
  float numerator=b-cuBQL::dot(n,ray.origin),denominator=cuBQL::dot(n,ray.direction);
  if(fabsf(denominator)<1e-12f)return numerator>=-1e-6f;
  float t=numerator/denominator;
  if(denominator<0){if(t>enter){enter=t;normal=n;}}
  else leave=fminf(leave,t);
  return enter<=leave;
}
__device__ bool hit_prism(const Triangle3f& tri,const ray3f& ray,float& t,vec3f& normal){
  float enter=-CUDART_INF_F,leave=ray.tMax;
  if(!plane({0,0,1},1,ray,enter,leave,normal) || !plane({0,0,-1},0,ray,enter,leave,normal))return false;
  const Vec3f p[3]={tri.a,tri.b,tri.c};
  for(int j=0;j<3;++j){
    auto a=p[j],b=p[(j+1)%3];vec3f n(b.y-a.y,a.x-b.x,0);
    float length=sqrtf(cuBQL::dot(n,n));if(length<1e-12f)return false;n=n*(1/length);
    if(!plane(n,n.x*a.x+n.y*a.y,ray,enter,leave,normal))return false;
  }
  if(enter<ray.tMin || enter>leave || enter>=ray.tMax)return false;
  t=enter;return true;
}
__global__ void trace(uchar4* output,int width,int height,Camera camera,bvh3f top,const GpuLayer* layers,bool empty){
  int x=blockIdx.x*blockDim.x+threadIdx.x,y=blockIdx.y*blockDim.y+threadIdx.y;
  if(x>=width || y>=height)return;
  uchar4 colour=make_uchar4(14,17,22,255);
  if(!empty){
    float px=(x+.5f-camera.pan_x)/camera.scale;
    float py=(height-y-.5f-camera.pan_y)/camera.scale;
    vec3f origin=v(camera.centre)+v(camera.right)*px-v(camera.up)*py+v(camera.toward)*camera.reach;
    ray3f ray(origin,-v(camera.toward),0,2*camera.reach);
    vec3f normal(0);bool hit=false;
    auto layer_hit=[&](unsigned id){
      const GpuLayer layer=layers[id];float h=layer.top-layer.bottom;
      ray3f local=ray;local.origin.z=(local.origin.z-layer.bottom)/h;local.direction.z/=h;
      auto triangle_hit=[&](unsigned primitive){
        float t;vec3f n;
        if(hit_prism(layer.triangles[primitive],local,t,n)){
          local.tMax=t;normal=n;hit=true;
        }
        return local.tMax;
      };
      cuBQL::shrinkingRayQuery::forEachPrim(triangle_hit,layer.bvh,local);
      ray.tMax=local.tMax;return ray.tMax;
    };
    cuBQL::shrinkingRayQuery::forEachPrim(layer_hit,top,ray);
    if(hit){float shade=.42f+.58f*fmaxf(0,cuBQL::dot(normal,v(camera.light)));
      colour=make_uchar4(255*.54f*shade,255*.61f*shade,255*.70f*shade,255);}
  }
  output[y*width+x]=colour;
}
Renderer* create(){return new Renderer;}
void destroy(Renderer* r){delete r;}
Stats stats(const Renderer* r){return r->counters;}
void update(Renderer* r,const std::vector<Layer>& input){
  std::vector<Layer> layers;
  for(const auto& layer:input)if(layer.top>layer.bottom && layer.geometry && !layer.geometry->triangles.empty())layers.push_back(layer);
  bool same=layers.size()==r->layers.size() && std::equal(layers.begin(),layers.end(),r->layers.begin(),[](const auto& a,const auto& b){
    return a.geometry==b.geometry && a.bottom==b.bottom && a.top==b.top;
  });
  if(same)return;
  std::vector<GpuLayer> packed;std::set<const Geometry*> live;
  for(const auto& layer:layers){
    const auto key=layer.geometry.get();live.insert(key);
    if(!r->geometries.count(key)){
      auto g=std::make_unique<GeometryGpu>();g->source=layer.geometry;
      const auto& triangles=layer.geometry->triangles;const unsigned count=triangles.size();
      check(cudaMalloc(&g->triangles,count*sizeof(Triangle3f)),"allocate prism triangles");
      check(cudaMalloc(&g->boxes,count*sizeof(box3f)),"allocate prism bounds");
      check(cudaMemcpy(g->triangles,triangles.data(),count*sizeof(Triangle3f),cudaMemcpyHostToDevice),"upload prism triangles");
      triangle_bounds<<<(count+255)/256,256>>>(g->boxes,g->triangles,count);
      check(cudaGetLastError(),"prism bounds kernel");
      cuBQL::gpuBuilder(g->bvh,g->boxes,count,cuBQL::BuildConfig(4));
      r->geometries.emplace(key,std::move(g));++r->counters.layer_builds;
    }
    const auto& g=*r->geometries.at(key);
    packed.push_back({g.bvh,g.triangles,layer.bottom,layer.top});
  }
  const bool count_changed=layers.size()!=r->layers.size();
  if(count_changed){
    cuBQL::cuda::free(r->top);r->top={};cudaFree(r->gpu_layers);r->gpu_layers=nullptr;cudaFree(r->boxes);r->boxes=nullptr;
    if(!packed.empty()){
      check(cudaMalloc(&r->gpu_layers,packed.size()*sizeof(GpuLayer)),"allocate layer instances");
      check(cudaMalloc(&r->boxes,packed.size()*sizeof(box3f)),"allocate layer bounds");
    }
  }
  if(!packed.empty()){
    check(cudaMemcpy(r->gpu_layers,packed.data(),packed.size()*sizeof(GpuLayer),cudaMemcpyHostToDevice),"upload layer instances");
    layer_bounds<<<(packed.size()+255)/256,256>>>(r->boxes,r->gpu_layers,packed.size());
    if(count_changed){cuBQL::gpuBuilder(r->top,r->boxes,packed.size(),cuBQL::BuildConfig(1));++r->counters.top_builds;}
    else {cuBQL::cuda::refit(r->top,r->boxes);++r->counters.top_refits;}
  }
  r->layers=std::move(layers);
  for(auto it=r->geometries.begin();it!=r->geometries.end();)if(!live.count(it->first))it=r->geometries.erase(it);else ++it;
}
struct Interop{cudaGraphicsResource* resource=nullptr;};
Interop* register_buffer(unsigned buffer){
  auto p=std::make_unique<Interop>();auto status=cudaGraphicsGLRegisterBuffer(&p->resource,buffer,cudaGraphicsRegisterFlagsWriteDiscard);
  if(status!=cudaSuccess){cudaGetLastError();return nullptr;}return p.release();
}
void unregister_buffer(Interop* p){if(p){cudaGraphicsUnregisterResource(p->resource);delete p;}}
static void launch(Renderer* r,uchar4* output,const Camera& c,int w,int h){
  trace<<<dim3((w+15)/16,(h+15)/16),dim3(16,16)>>>(output,w,h,c,r->top,r->gpu_layers,r->layers.empty());
  check(cudaGetLastError(),"prism raycast kernel");
}
void render(Renderer* r,Interop* p,const Camera& c,int w,int h){
  check(cudaGraphicsMapResources(1,&p->resource),"map raycast image");
  try{
    uchar4* output;std::size_t bytes;
    check(cudaGraphicsResourceGetMappedPointer(reinterpret_cast<void**>(&output),&bytes,p->resource),"map raycast pixels");
    if(bytes<std::size_t(w)*h*4)throw std::runtime_error("Raycast pixel buffer too small");
    launch(r,output,c,w,h);
  }catch(...){cudaGraphicsUnmapResources(1,&p->resource);throw;}
  check(cudaGraphicsUnmapResources(1,&p->resource),"unmap raycast image");
}
std::vector<std::uint8_t> render_host(Renderer* r,const Camera& c,int w,int h){
  std::size_t count=std::size_t(w)*h;
  if(count>r->output_capacity){cudaFree(r->output);r->output=nullptr;r->output_capacity=0;check(cudaMalloc(&r->output,count*4),"allocate raycast image");r->output_capacity=count;}
  launch(r,r->output,c,w,h);std::vector<std::uint8_t> pixels(count*4);
  check(cudaMemcpy(pixels.data(),r->output,count*4,cudaMemcpyDeviceToHost),"read raycast image");return pixels;
}
}
