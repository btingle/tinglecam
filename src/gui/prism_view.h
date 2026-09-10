#pragma once

// CPU triangulation cache and GL presentation. All prism bounds, BVH builds,
// refits, traversal and ray intersections run in CUDA.
class PrismView {
  struct Cached {
    std::shared_ptr<const Paths64> region;
    std::shared_ptr<const wave::prism::Geometry> geometry;
    std::size_t generation=0;
    bool failed=false;
  };
  std::unordered_map<std::uint64_t,Cached> cache_;
  std::size_t generation_=0, failures_=0;
  wave::prism::Renderer* renderer_=nullptr;
  wave::prism::Interop* interop_=nullptr;
  GLuint image_=0,pbo_=0;
  int width_=0,height_=0;
  bool warned_staging_=false;
 public:
  std::size_t failed_layers() const {return failures_;}
  template<class P,class L>
  GLuint render(const std::vector<cam::StockSlab>& stock,const P& p,const L& light,
                ImVec2 top_left,ImVec2 size,float reach){
    if(!renderer_)renderer_=wave::prism::create();
    ++generation_;failures_=0;
    std::vector<wave::prism::Layer> layers;
    for(const auto& slab:stock){
      if(slab.region->empty() || slab.top<=slab.bottom)continue;
      std::uint64_t key=1469598103934665603ull;
      const auto mix=[&](std::uint64_t n){key=(key^n)*1099511628211ull;};
      for(const auto& ring:*slab.region){mix(ring.size());for(const auto& pt:ring){mix(pt.x);mix(pt.y);}}
      // Verify equality as well as the hash, so collisions cannot alias stock.
      while(cache_.count(key) && *cache_.at(key).region!=*slab.region)++key;
      auto it=cache_.find(key);
      if(it==cache_.end()){
        Paths64 triangles;
        auto region=Clipper2Lib::Union(*slab.region,Clipper2Lib::FillRule::NonZero);
        auto status=Clipper2Lib::Triangulate(region,triangles);
        if(status!=Clipper2Lib::TriangulateResult::success && status!=Clipper2Lib::TriangulateResult::no_polygons){
          // Same cleanup used by the original layer renderer, applied to the
          // whole filled slice, not differences between neighbouring caps.
          region=drawable_region(*slab.region,std::max(double(reach)/1600,.05));
          triangles.clear();status=Clipper2Lib::Triangulate(region,triangles);
        }
        bool failed=status!=Clipper2Lib::TriangulateResult::success && status!=Clipper2Lib::TriangulateResult::no_polygons;
        auto geometry=std::make_shared<wave::prism::Geometry>();
        if(failed)std::fprintf(stderr,"Prism triangulation failed for slab Z %.6f to %.6f (Clipper status %d); other layers retained.\n",slab.bottom,slab.top,int(status));
        else for(auto& t:triangles){
          if(t.size()!=3 || Clipper2Lib::Area(t)==0)continue;
          if(Clipper2Lib::Area(t)<0)std::reverse(t.begin(),t.end());
          const auto point=[](const Point64& q){return wave::Vec3f{float(wave::to_world(q.x)),float(wave::to_world(q.y)),0};};
          geometry->triangles.push_back({point(t[0]),point(t[1]),point(t[2])});
        }
        it=cache_.emplace(key,Cached{slab.region,geometry,generation_,failed}).first;
      }
      it->second.generation=generation_;
      if(it->second.failed)++failures_;
      layers.push_back({it->second.geometry,float(slab.bottom),float(slab.top)});
    }
    wave::prism::update(renderer_,layers);
    for(auto it=cache_.begin();it!=cache_.end();)if(it->second.generation!=generation_)it=cache_.erase(it);else ++it;
    int w=std::max(1,int(size.x)),h=std::max(1,int(size.y));
    glActiveTexture(GL_TEXTURE0);
    if(!image_){glGenTextures(1,&image_);glGenBuffers(1,&pbo_);}
    glBindTexture(GL_TEXTURE_2D,image_);
    if(w!=width_ || h!=height_){
      wave::prism::unregister_buffer(interop_);interop_=nullptr;
      glBindBuffer(GL_PIXEL_UNPACK_BUFFER,0);
      glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,w,h,0,GL_RGBA,GL_UNSIGNED_BYTE,nullptr);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
      glBindBuffer(GL_PIXEL_UNPACK_BUFFER,pbo_);glBufferData(GL_PIXEL_UNPACK_BUFFER,std::size_t(w)*h*4,nullptr,GL_STREAM_DRAW);
      glBindBuffer(GL_PIXEL_UNPACK_BUFFER,0);
      interop_=wave::prism::register_buffer(pbo_);width_=w;height_=h;
      if(!interop_ && !warned_staging_){std::fprintf(stderr,"CUDA/OpenGL sharing unavailable; staging raycast pixels through host memory. Raycasting remains on GPU.\n");warned_staging_=true;}
    }
    const auto vec=[](const auto& x){return wave::Vec3f{x.x,x.y,x.z};};
    wave::prism::Camera camera{vec(p.model_centre),vec(p.right),vec(p.up),vec(p.toward),vec(light),p.scale,reach,p.centre.x-top_left.x,p.centre.y-top_left.y};
    if(interop_){
      wave::prism::render(renderer_,interop_,camera,w,h);
      glBindBuffer(GL_PIXEL_UNPACK_BUFFER,pbo_);
      glTexSubImage2D(GL_TEXTURE_2D,0,0,0,w,h,GL_RGBA,GL_UNSIGNED_BYTE,nullptr);
      glBindBuffer(GL_PIXEL_UNPACK_BUFFER,0);
    }else{
      auto pixels=wave::prism::render_host(renderer_,camera,w,h);
      glBindBuffer(GL_PIXEL_UNPACK_BUFFER,0);
      glTexSubImage2D(GL_TEXTURE_2D,0,0,0,w,h,GL_RGBA,GL_UNSIGNED_BYTE,pixels.data());
    }
    return image_;
  }
  void release(){
    wave::prism::unregister_buffer(interop_);interop_=nullptr;
    wave::prism::destroy(renderer_);renderer_=nullptr;
    glDeleteTextures(1,&image_);glDeleteBuffers(1,&pbo_);image_=pbo_=0;
  }
};
