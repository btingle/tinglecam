#pragma once

#include "mesh_slicer.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

namespace wave::cpu {
inline float axis(Vec3f p, int i) { return i == 0 ? p.x : i == 1 ? p.y : p.z; }
struct Box {
  Vec3f lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  void extend(Vec3f p) {
    lo = {std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
    hi = {std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
  }
  void extend(const Box& b) { extend(b.lo); extend(b.hi); }
};
inline Box bounds(const Triangle3f& t) { Box b; b.extend(t.a); b.extend(t.b); b.extend(t.c); return b; }
struct Ray { Vec3f origin, direction; float min = 0, max = INFINITY; };
inline bool intersects(const Box& b, const Ray& ray) {
  float enter=ray.min, leave=ray.max;
  for(int a=0;a<3;++a) {
    const float o=axis(ray.origin,a), d=axis(ray.direction,a);
    if(d==0) { if(o<axis(b.lo,a) || o>axis(b.hi,a)) return false; }
    else {
      float x=(axis(b.lo,a)-o)/d, y=(axis(b.hi,a)-o)/d;
      if(x>y) std::swap(x,y);
      enter=std::max(enter,x); leave=std::min(leave,y);
      if(enter>leave) return false;
    }
  }
  return true;
}
// Balanced median BVH: O(n log n) construction, immutable traversal and
// caller-owned query state. Nodes and primitive IDs are contiguous in memory.
class Bvh {
  struct Node { Box box; std::uint32_t begin=0,end=0,left=0,right=0; };
  std::vector<Node> nodes_;
  std::vector<std::uint32_t> ids_;
  std::uint32_t build_node(const std::vector<Box>& boxes, unsigned begin, unsigned end) {
    const auto index=static_cast<unsigned>(nodes_.size()); nodes_.push_back({});
    Box box, centres;
    for(unsigned i=begin;i<end;++i) {
      const auto& b=boxes[ids_[i]]; box.extend(b);
      centres.extend(Vec3f{(b.lo.x+b.hi.x)*.5f,(b.lo.y+b.hi.y)*.5f,(b.lo.z+b.hi.z)*.5f});
    }
    nodes_[index].box=box; nodes_[index].begin=begin; nodes_[index].end=end;
    if(end-begin>4) {
      Vec3f span{centres.hi.x-centres.lo.x,centres.hi.y-centres.lo.y,centres.hi.z-centres.lo.z};
      int a=span.y>span.x?1:0; if(span.z>axis(span,a)) a=2;
      unsigned mid=begin+(end-begin)/2;
      std::nth_element(ids_.begin()+begin,ids_.begin()+mid,ids_.begin()+end,[&](auto x,auto y){
        return axis(boxes[x].lo,a)+axis(boxes[x].hi,a)<axis(boxes[y].lo,a)+axis(boxes[y].hi,a);
      });
      auto left=build_node(boxes,begin,mid), right=build_node(boxes,mid,end);
      nodes_[index].left=left; nodes_[index].right=right;
    }
    return index;
  }
  template<class Predicate,class Visitor>
  bool visit(unsigned index, Predicate& predicate, Visitor& visitor, std::size_t& budget) const {
    if(budget==0) return false;
    --budget;
    const auto& n=nodes_[index]; if(!predicate(n.box)) return true;
    if(!n.left) { for(auto i=n.begin;i<n.end;++i) if(!visitor(ids_[i])) return false; return true; }
    return visit(n.left,predicate,visitor,budget) && visit(n.right,predicate,visitor,budget);
  }
 public:
  void build(const std::vector<Box>& boxes) {
    nodes_.clear(); ids_.resize(boxes.size()); std::iota(ids_.begin(),ids_.end(),0u);
    nodes_.reserve(boxes.size()*2); if(!boxes.empty()) build_node(boxes,0,boxes.size());
  }
  void refit(const std::vector<Box>& boxes) {
    for(auto it=nodes_.rbegin();it!=nodes_.rend();++it) {
      Box b;
      if(it->left) { b.extend(nodes_[it->left].box); b.extend(nodes_[it->right].box); }
      else for(auto i=it->begin;i<it->end;++i) b.extend(boxes[ids_[i]]);
      it->box=b;
    }
  }
  const Box& bounds() const { return nodes_.front().box; }
  template<class Predicate,class Visitor> void query(Predicate predicate, Visitor visitor,
      std::size_t budget=std::numeric_limits<std::size_t>::max()) const {
    if(!nodes_.empty()) visit(0,predicate,visitor,budget);
  }
};
inline double cross(Vec3f a,Vec3f b,Vec3f p) {
  return (double(b.x)-a.x)*(double(p.y)-a.y)-(double(b.y)-a.y)*(double(p.x)-a.x);
}
inline bool point_in_xy(const Triangle3f& t,Vec3f p) {
  double a=cross(t.a,t.b,p),b=cross(t.b,t.c,p),c=cross(t.c,t.a,p);
  return (a>=-1e-5 && b>=-1e-5 && c>=-1e-5) || (a<=1e-5 && b<=1e-5 && c<=1e-5);
}
inline double height(const Triangle3f& t,Vec3f p) {
  double area=cross(t.a,t.b,t.c);
  return t.a.z + cross(t.a,p,t.c)/area*(double(t.b.z)-t.a.z)
                 + cross(t.a,t.b,p)/area*(double(t.c.z)-t.a.z);
}
} // namespace wave::cpu
