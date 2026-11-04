#pragma once
namespace wave::heightfield {
inline constexpr const char* fullscreen_vertex = R"GLSL(
#version 150
void main() {
  vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";
inline constexpr const char* heightmap_fragment = R"GLSL(
#version 150
uniform samplerBuffer bounds_data;
uniform isamplerBuffer tree_data;
uniform isamplerBuffer tile_data;
uniform samplerBuffer heights;
uniform vec3 centre, right, up, toward, light;
uniform vec2 pan, viewport, grid_origin;
uniform ivec2 grid_size;
uniform float scale, reach, pitch;
uniform bool empty_grid;
out vec4 colour;
vec3 illuminate(vec3 normal, vec3 base);

// Exact intersection with a column/tile box; parallel and inside rays are
// handled without reciprocal infinities. All intervals are in world distance.
bool box_hit(vec3 ro, vec3 rd, vec3 lo, vec3 hi, float limit, bool surface,
             out float enter, out float leave, out vec3 normal) {
  if (hi.z <= lo.z) return false;
  enter = 0.0; leave = limit; normal = vec3(0);
  vec3 exit_normal = vec3(0);
  for (int k = 0; k < 3; ++k) {
    if (abs(rd[k]) < 1e-12) {
      if (ro[k] < lo[k] || ro[k] > hi[k]) return false;
    } else {
      float a = (lo[k] - ro[k]) / rd[k];
      float b = (hi[k] - ro[k]) / rd[k];
      float n = -1.0;
      if (a > b) { float c = a; a = b; b = c; n = 1.0; }
      if (a >= enter) { enter = a; normal = vec3(0); normal[k] = n; }
      if (b < leave) { leave = b; exit_normal = vec3(0); exit_normal[k] = -n; }
      if (enter > leave) return false;
    }
  }
  if (surface && enter == 0.0 && dot(normal, normal) == 0.0) {
    enter = leave; normal = exit_normal;
  }
  return enter < limit && leave >= 0.0;
}
float height_at(int mode, int offset, ivec2 cell) {
  int i = mode == 3 ? cell.y * 32 + cell.x :
          mode == 1 ? cell.x : mode == 2 ? cell.y : 0;
  return texelFetch(heights, offset + i).r;
}
// Samples remain compressed on the GPU. Reconstruction reads through tile
// boundaries without building or uploading a dense display mesh.
vec2 cell_heights(ivec2 cell) {
  cell = clamp(cell, ivec2(0), grid_size - 1);
  ivec2 tile = cell / 32;
  ivec4 d = texelFetch(tile_data, tile.y * ((grid_size.x + 31) / 32) + tile.x);
  ivec2 local = cell - tile * 32;
  return vec2(height_at(d.z, d.w, local), height_at(d.x, d.y, local));
}
// Most patches lie inside one compressed tile. Decode its descriptor once
// for all four corners; seams and clamped borders use the general sampler.
vec2 tile_heights(ivec4 d, ivec2 local) {
  return vec2(height_at(d.z,d.w,local),height_at(d.x,d.y,local));
}
void patch_heights(ivec2 base, out vec2 a, out vec2 b, out vec2 c, out vec2 d) {
  ivec2 lo=clamp(base,ivec2(0),grid_size-1);
  ivec2 hi=clamp(base+ivec2(1),ivec2(0),grid_size-1);
  ivec2 tile=lo/32;
  if(all(equal(tile,hi/32))) {
    ivec4 descriptor=texelFetch(tile_data,tile.y*((grid_size.x+31)/32)+tile.x);
    lo-=tile*32;hi-=tile*32;
    a=tile_heights(descriptor,lo);
    b=tile_heights(descriptor,ivec2(hi.x,lo.y));
    c=tile_heights(descriptor,ivec2(lo.x,hi.y));
    d=tile_heights(descriptor,hi);
  } else {
    a=cell_heights(base);b=cell_heights(base+ivec2(1,0));
    c=cell_heights(base+ivec2(0,1));d=cell_heights(base+ivec2(1));
  }
}
bool occupied(vec2 h) { return !any(isnan(h)) && h.y > h.x; }
bool continuous_edge(ivec2 a, ivec2 b, float ha, float hb, bool top) {
  float jump = hb - ha;
  if (abs(jump) <= 2.0 * pitch) return true;
  // A steep slope continues in neighbouring samples; a vertical step between
  // plateaus does not. Avoid bridging steps just because their cells touch.
  ivec2 delta = b - a;
  vec2 l = cell_heights(a - delta), r = cell_heights(b + delta);
  float before = ha - (top ? l.y : l.x);
  float after = (top ? r.y : r.x) - hb;
  return (occupied(l) && before * jump > 0.0 &&
          abs(before) >= .3 * abs(jump) && abs(before) <= 3.0 * abs(jump)) ||
         (occupied(r) && after * jump > 0.0 &&
          abs(after) >= .3 * abs(jump) && abs(after) <= 3.0 * abs(jump));
}
vec4 coefficients(ivec2 base, ivec2 owner, vec4 z, bvec4 valid,
                  float fallback, bool top, out bvec4 connected) {
  vec4 jumps = vec4(z.y-z.x,z.w-z.z,z.z-z.x,z.w-z.y);
  if (all(valid) && all(lessThanEqual(abs(jumps),vec4(2.0*pitch)))) {
    connected=bvec4(true);
    return vec4(z.x,z.y-z.x,z.z-z.x,z.w-z.z-z.y+z.x);
  }
  bvec4 edges = bvec4(
      valid.x && valid.y && continuous_edge(base, base + ivec2(1, 0), z.x, z.y, top),
      valid.z && valid.w && continuous_edge(base + ivec2(0, 1), base + ivec2(1), z.z, z.w, top),
      valid.x && valid.z && continuous_edge(base, base + ivec2(0, 1), z.x, z.z, top),
      valid.y && valid.w && continuous_edge(base + ivec2(1, 0), base + ivec2(1), z.y, z.w, top));
  if (all(edges)) {
    connected=bvec4(true);
    return vec4(z.x,z.y-z.x,z.z-z.x,z.w-z.z-z.y+z.x);
  }
  connected = bvec4(false);
  ivec2 local = clamp(owner - base, ivec2(0), ivec2(1));
  connected[local.y * 2 + local.x] = true;
  for (int i = 0; i < 3; ++i) {
    if (edges.x && (connected.x || connected.y)) { connected.x = true; connected.y = true; }
    if (edges.y && (connected.z || connected.w)) { connected.z = true; connected.w = true; }
    if (edges.z && (connected.x || connected.z)) { connected.x = true; connected.z = true; }
    if (edges.w && (connected.y || connected.w)) { connected.y = true; connected.w = true; }
  }
  vec4 result = z;
  for (int i = 0; i < 4; ++i) if (!connected[i]) {
    float sum = 0.0, count = 0.0;
    for (int axis = 0; axis < 2; ++axis) {
      int neighbour = i ^ (axis == 0 ? 1 : 2);
      if (connected[neighbour]) { sum += z[neighbour]; count += 1.0; }
    }
    result[i] = count > 0.0 ? sum / count : fallback;
  }
  return vec4(result.x, result.y - result.x, result.z - result.x,
              result.w - result.z - result.y + result.x);
}
float value(vec4 c, vec2 uv) { return c.x + dot(c.yz, uv) + c.w * uv.x * uv.y; }
bool cap_root(float root, float duration, vec3 start, vec3 rd, ivec2 base,
              vec4 c, vec4 other, bool top, float begin,
              inout float closest, inout vec3 normal) {
  if (root < -1e-6 || root > duration + 1e-6) return false;
  root = clamp(root, 0.0, duration);
  float t = begin + root;
  if (t >= closest) return false;
  vec2 uv = (start.xy + rd.xy * root) / pitch - (vec2(base) + .5);
  float height = value(c, uv), opposite = value(other, uv);
  if (top ? height <= opposite : height >= opposite) return false;
  vec2 gradient = (c.yz + c.w * uv.yx) / pitch;
  normal = normalize(vec3(-gradient, 1.0)) * (top ? 1.0 : -1.0);
  closest = t;
  return true;
}
bool cap_hit(vec3 start, vec3 rd, ivec2 base, vec4 c, vec4 other, bool top,
             float begin, float end, inout float closest, inout vec3 normal) {
  vec2 uv = start.xy / pitch - (vec2(base) + .5), v = rd.xy / pitch;
  float A = -c.w * v.x * v.y;
  float B = rd.z - dot(c.yz, v) - c.w * dot(uv, v.yx);
  float C = start.z - value(c, uv);
  bool hit = false;
  if (abs(A) < 1e-10) {
    if (abs(B) > 1e-10)
      hit = cap_root(-C / B, end - begin, start, rd, base, c, other, top,
                     begin, closest, normal);
  } else {
    float discriminant = B * B - 4.0 * A * C;
    if (discriminant >= 0.0) {
      float q = -.5 * (B + (B >= 0.0 ? 1.0 : -1.0) * sqrt(discriminant));
      if (abs(q) > 1e-15) {
        hit = cap_root(q / A, end - begin, start, rd, base, c, other, top,
                       begin, closest, normal);
        hit = cap_root(C / q, end - begin, start, rd, base, c, other, top,
                       begin, closest, normal) || hit;
      } else hit = cap_root(-B / (2.0 * A), end - begin, start, rd, base,
                             c, other, top, begin, closest, normal);
    }
  }
  return hit;
}
// Stable quadratic/linear roots in ray distance; unused slots are unbounded.
vec2 equation_roots(float a, float b, float c) {
  if (abs(a)<1e-10) return vec2(abs(b)>1e-10 ? -c/b : 1e30,1e30);
  float discriminant=b*b-4.0*a*c;
  if(discriminant<0.0) return vec2(1e30);
  float q=-.5*(b+(b>=0.0?1.0:-1.0)*sqrt(discriminant));
  if(abs(q)<1e-15) return vec2(-b/(2.0*a),1e30);
  vec2 result=vec2(q/a,c/q);
  return vec2(min(result.x,result.y),max(result.x,result.y));
}
vec4 material_at(vec4 caps[8], bvec4 valid, vec2 uv, float z) {
  vec4 material=vec4(0);
  for(int i=0;i<4;++i) if(valid[i]) {
    float low=value(caps[4+i],uv),high=value(caps[i],uv);
    material[i]=high>low && z>=low-1e-6 && z<=high+1e-6?1.0:0.0;
  }
  return material;
}
vec4 material_field(vec4 samples) {
  return vec4(samples.x,samples.y-samples.x,samples.z-samples.x,
              samples.w-samples.z-samples.y+samples.x);
}
// Most wall intervals lie entirely between cap levels. If no connected cap
// group straddles the ray's height range, its material pattern is constant;
// solve just one quadratic, without constructing cap sheets or event roots.
bool stable_slice(ivec2 base, vec2 raw[4], bvec4 valid, float low, float high,
                  out vec4 field) {
  bvec4 below_top=bvec4(false),above_bottom=bvec4(false);
  vec4 material=vec4(0);
  for(int i=0;i<4;++i) if(valid[i]) {
    if((low<=raw[i].x && high>=raw[i].x) ||
       (low<=raw[i].y && high>=raw[i].y)) return false;
    below_top[i]=high<raw[i].y;above_bottom[i]=low>raw[i].x;
    material[i]=below_top[i] && above_bottom[i]?1.0:0.0;
  }
  for(int edge=0;edge<4;++edge) {
    int a=edge<2?edge*2:edge-2,b=edge<2?a+1:a+2;
    if(!valid[a] || !valid[b]) continue;
    ivec2 p=base+ivec2(a%2,a/2),q=base+ivec2(b%2,b/2);
    if(below_top[a]!=below_top[b] && continuous_edge(p,q,raw[a].y,raw[b].y,true)) return false;
    if(above_bottom[a]!=above_bottom[b] && continuous_edge(p,q,raw[a].x,raw[b].x,false)) return false;
  }
  field=material_field(material);return true;
}
bool slice_hit(vec4 field, vec3 start, vec3 rd, ivec2 base,
               float begin, float end, vec3 front,
               inout float closest, inout vec3 normal) {
  vec2 uv=start.xy/pitch-(vec2(base)+.5),v=rd.xy/pitch;
  if(begin>0.0 && value(field,uv)>=.5) {
    closest=begin;normal=front;return true;
  }
  vec2 roots=equation_roots(field.w*v.x*v.y,
      dot(field.yz,v)+field.w*dot(uv,v.yx),value(field,uv)-.5);
  float t=roots.x>=-1e-6?roots.x:roots.y;
  if(t< -1e-6 || t>end-begin+1e-6 || begin+t>=closest) return false;
  t=clamp(t,0.0,end-begin);
  vec2 gradient=-(field.yz+field.w*(uv+v*t).yx);
  if(dot(gradient,gradient)<1e-12) return false;
  closest=begin+t;normal=normalize(vec3(gradient,0));return true;
}
// Intersect the 50% material isoline between samples, rather than square cell
// edges. Connected cap sheets extend to the same boundary, so walls and caps
// meet without a display mesh, including holes, steps and raised undersides.
bool wall_patch_hit(vec3 start, vec3 rd, ivec2 base, vec2 raw[4], bvec4 valid,
                    float begin, float end, vec3 front,
                    inout float closest, inout vec3 normal) {
  vec4 caps[8];
  bvec4 assigned_top=bvec4(false),assigned_bottom=bvec4(false);
  vec4 top_samples=vec4(raw[0].y,raw[1].y,raw[2].y,raw[3].y);
  vec4 bottom_samples=vec4(raw[0].x,raw[1].x,raw[2].x,raw[3].x);
  vec2 uv=start.xy/pitch-(vec2(base)+.5),v=rd.xy/pitch;
  float duration=end-begin;
  for(int side=0;side<2;++side) for(int i=0;i<4;++i) {
    if(!valid[i] || (side==0?assigned_top[i]:assigned_bottom[i])) continue;
    bool top=side==0;
    bvec4 connected;
    vec4 c=coefficients(base,base+ivec2(i%2,i/2),
                       top?top_samples:bottom_samples,valid,
                       top?raw[i].y:raw[i].x,top,connected);
    for(int k=0;k<4;++k) if(valid[k] && connected[k]) {
      caps[side*4+k]=c;
      if(top) assigned_top[k]=true;else assigned_bottom[k]=true;
    }
  }
  vec4 initial=material_field(material_at(caps,valid,uv,start.z));
  if(begin>0.0 && value(initial,uv)>=.5) {
    closest=begin;normal=front;return true;
  }
  bool hit=false,cap_crossing=false;
  for(int side=0;side<2;++side) for(int i=0;i<4;++i) {
    if(!valid[i]) continue;
    vec4 c=caps[side*4+i];
    vec2 roots=equation_roots(-c.w*v.x*v.y,
        rd.z-dot(c.yz,v)-c.w*dot(uv,v.yx),start.z-value(c,uv));
    for(int k=0;k<2;++k) {
      float t=roots[k];
      if(t<0.0 || t>duration) continue;
      cap_crossing=true;
      if(begin+t>=closest) continue;
      vec2 at=uv+v*t;
      vec4 field=material_field(material_at(caps,valid,at,start.z+rd.z*t));
      if(value(field,at)<.5-1e-6) continue;
      vec2 gradient=(c.yz+c.w*at.yx)/pitch;
      closest=begin+t;normal=normalize(vec3(-gradient,1))*(side==0?1.0:-1.0);
      hit=true;
    }
  }
  // Away from caps, a single quadratic is enough. Only intervals crossing a
  // cap can change their occupancy pattern along the ray; there we test the
  // seven distinct isolines (the other seven are their complements).
  for(int pattern=1;pattern<=7;++pattern) {
    vec4 field=initial;
    if(cap_crossing) field=material_field(vec4(
        (pattern&1)!=0?1.0:0.0,(pattern&2)!=0?1.0:0.0,
        (pattern&4)!=0?1.0:0.0,0));
    vec2 roots=equation_roots(field.w*v.x*v.y,
        dot(field.yz,v)+field.w*dot(uv,v.yx),value(field,uv)-.5);
    for(int k=0;k<2;++k) {
      float t=roots[k];
      if(t< -1e-6 || t>duration+1e-6 || begin+t>=closest) continue;
      t=clamp(t,0.0,duration);
      vec2 at=uv+v*t;
      vec4 actual=cap_crossing?
          material_field(material_at(caps,valid,at,start.z+rd.z*t)):initial;
      if(abs(value(actual,at)-.5)>1e-5) continue;
      vec2 gradient=-(actual.yz+actual.w*at.yx);
      if(dot(gradient,gradient)<1e-12) continue;
      closest=begin+t;normal=normalize(vec3(gradient,0));hit=true;
    }
    if(!cap_crossing) break;
  }
  return hit;
}
bool interval_hit(vec3 ro, vec3 rd, ivec2 base, float begin, float end,
                  vec3 front, inout float closest, inout vec3 normal) {
  vec3 start = ro + rd * begin;
  vec2 a,b,c,d;patch_heights(base,a,b,c,d);
  bvec4 valid=bvec4(occupied(a),occupied(b),occupied(c),occupied(d));
  if(!any(valid)) return false;
  vec4 limits=vec4(1e30,-1e30,1e30,-1e30);
  vec2 raw[4];raw[0]=a;raw[1]=b;raw[2]=c;raw[3]=d;
  for(int i=0;i<4;++i) if(valid[i]) {
    limits.x=min(limits.x,raw[i].x);limits.y=max(limits.y,raw[i].x);
    limits.z=min(limits.z,raw[i].y);limits.w=max(limits.w,raw[i].y);
  }
  float finish_z=ro.z+rd.z*end;
  float ray_low=min(start.z,finish_z),ray_high=max(start.z,finish_z);
  if(ray_low>limits.w+1e-6 || ray_high<limits.x-1e-6) return false;
  if(all(valid) && begin>0.0 && start.z>=limits.y && start.z<=limits.z && limits.z>limits.y) {
    closest=begin;normal=front;return true;
  }
  // Missing samples always require occupancy reconstruction, which does not
  // depend on a nearest-cell owner. Skip unused connected-cap coefficients.
  if(!all(valid)) {
    vec4 slice;
    if(stable_slice(base,raw,valid,ray_low,ray_high,slice))
      return slice_hit(slice,start,rd,base,begin,end,front,closest,normal);
    return wall_patch_hit(start,rd,base,raw,valid,begin,end,front,closest,normal);
  }
  bvec4 bottom_connected,top_connected;
  vec4 low_samples=vec4(a.x,b.x,c.x,d.x), high_samples=vec4(a.y,b.y,c.y,d.y);
  vec4 bottom=coefficients(base,base,low_samples,valid,a.x,false,bottom_connected);
  vec4 top=coefficients(base,base,high_samples,valid,a.y,true,top_connected);
  if(!all(bottom_connected) || !all(top_connected)) {
    vec4 slice;
    if(stable_slice(base,raw,valid,ray_low,ray_high,slice))
      return slice_hit(slice,start,rd,base,begin,end,front,closest,normal);
    return wall_patch_hit(start,rd,base,raw,valid,begin,end,front,closest,normal);
  }
  vec2 uv = start.xy / pitch - (vec2(base) + .5);
  float low = value(bottom, uv), high = value(top, uv);
  if (begin > 0.0 && high > low && start.z >= low && start.z <= high) {
    closest = begin; normal = front; return true;
  }
  bool hit=false;
  if(ray_high>=limits.z-1e-6 && ray_low<=limits.w+1e-6)
    hit=cap_hit(start,rd,base,top,bottom,true,begin,end,closest,normal);
  if(ray_high>=limits.x-1e-6 && ray_low<=limits.y+1e-6)
    hit=cap_hit(start,rd,base,bottom,top,false,begin,end,closest,normal)||hit;
  return hit;
}
bool leaf_hit(bool flat_tile, vec3 ro, vec3 rd, vec3 lo, vec3 hi,
              inout float closest, out vec3 normal) {
  float a, b; vec3 n;
  if (flat_tile) {
    if (!box_hit(ro, rd, lo, hi, closest, true, a, b, n)) return false;
    closest = a; normal = n; return true;
  }
  if (!box_hit(ro, rd, lo, hi, closest, false, a, b, n)) return false;
  // March directly between sample-centre planes. These bound the bilinear
  // patches; integer cell edges inside a patch do not change its surface.
  ivec2 patch=ivec2(floor((ro.xy+rd.xy*a)/pitch-.5));
  ivec2 step=ivec2(sign(rd.xy));
  vec2 next=vec2(1e30),delta=vec2(1e30);
  for(int k=0;k<2;++k) if(step[k]!=0) {
    float edge=(float(patch[k]+(step[k]>0?1:0))+.5)*pitch;
    next[k]=max(a,(edge-ro[k])/rd[k]);
    delta[k]=pitch/abs(rd[k]);
  }
  float entered=a;vec3 front=n;
  // A tile spans at most 33 patches on either axis, including its border halo.
  for(int i=0;i<68;++i) {
    float boundary=min(min(next.x,next.y),min(b,closest));
    if(interval_hit(ro,rd,patch,entered,boundary,front,closest,normal)) return true;
    if(boundary>=b || boundary>=closest) break;
    bool sx=next.x<=next.y,sy=next.y<=next.x;
    front=sx?vec3(-step.x,0,0):vec3(0,-step.y,0);
    if(sx) {patch.x+=step.x;next.x+=delta.x;}
    if(sy) {patch.y+=step.y;next.y+=delta.y;}
    entered=boundary;
  }
  return false;
}
vec2 sample_gradient(ivec2 cell, bool top) {
  cell = clamp(cell, ivec2(0), grid_size - 1);
  vec2 h = cell_heights(cell);
  float centre = top ? h.y : h.x;
  vec2 gradient = vec2(0);
  for (int axis = 0; axis < 2; ++axis) {
    ivec2 delta = axis == 0 ? ivec2(1, 0) : ivec2(0, 1);
    vec2 l = cell_heights(cell - delta), r = cell_heights(cell + delta);
    float low = top ? l.y : l.x, high = top ? r.y : r.x;
    bool left = cell[axis] > 0 && occupied(l) && continuous_edge(cell-delta,cell,low,centre,top);
    bool right = cell[axis]+1 < grid_size[axis] && occupied(r) && continuous_edge(cell,cell+delta,centre,high,top);
    if (left && right) gradient[axis] = (high - low) / (2.0 * pitch);
    else if (left) gradient[axis] = (centre - low) / pitch;
    else if (right) gradient[axis] = (high - centre) / pitch;
  }
  return gradient;
}
vec3 wall_normal(vec3 position, vec3 geometric) {
  // The four weighted gradients share separable triangular kernels. Blend
  // their X/Y weights and moments first, then accumulate one gradient rather
  // than four. This is the same dense filter, with the same radius and samples.
  int radius=int(clamp(ceil(1.0/(scale*pitch)),2.0,4.0));
  vec2 coordinates=position.xy/pitch-.5;
  ivec2 base=ivec2(floor(coordinates));
  vec2 uv=clamp(coordinates-vec2(base),vec2(0),vec2(1));
  vec2 gradient=vec2(0);
  float extent=float(radius+1);
  for(int y=0;y<10;++y) {
    if(y>2*radius+1) break;
    float py=float(y-radius);
    float wy0=max(0.0,extent-abs(py)),wy1=max(0.0,extent-abs(py-1.0));
    float wy=mix(wy0,wy1,uv.y),my=mix(py*wy0,(py-1.0)*wy1,uv.y);
    for(int x=0;x<10;++x) {
      if(x>2*radius+1) break;
      ivec2 cell=base+ivec2(x-radius,y-radius);
      if(any(lessThan(cell,ivec2(0))) || any(greaterThanEqual(cell,grid_size))) continue;
      vec2 h=cell_heights(cell);
      if(!occupied(h) || position.z<h.x || position.z>h.y) continue;
      float px=float(x-radius);
      float wx0=max(0.0,extent-abs(px)),wx1=max(0.0,extent-abs(px-1.0));
      float wx=mix(wx0,wx1,uv.x),mx=mix(px*wx0,(px-1.0)*wx1,uv.x);
      gradient-=vec2(mx*wy,wx*my);
    }
  }
  if(dot(gradient,gradient)<1e-8) return geometric;
  vec3 result=normalize(vec3(gradient,0));
  return dot(result,geometric)>-.05?result:geometric;
}
vec3 shading_normal(vec2 position, vec3 geometric) {
  if (abs(geometric.z) < 1e-6) return geometric;
  ivec2 owner = clamp(ivec2(floor(position / pitch)), ivec2(0), grid_size - 1);
  vec2 h = cell_heights(owner);
  if (!occupied(h)) return geometric;
  bool top = geometric.z > 0.0;
  ivec2 base = ivec2(floor(position / pitch - .5));
  vec2 a,b,c,d;patch_heights(base,a,b,c,d);
  bvec4 selected;
  vec4 z = top ? vec4(a.y,b.y,c.y,d.y) : vec4(a.x,b.x,c.x,d.x);
  coefficients(base,owner,z,bvec4(occupied(a),occupied(b),occupied(c),occupied(d)),
               top ? h.y : h.x,top,selected);
  vec2 fallback = vec2(0);
  if (!all(selected)) fallback = sample_gradient(owner,top);
  vec2 gradients[4];
  for (int i=0;i<4;++i)
    gradients[i] = selected[i] ? sample_gradient(base+ivec2(i%2,i/2),top) : fallback;
  vec2 uv = clamp(position / pitch - (vec2(base)+.5),vec2(0),vec2(1));
  vec2 gradient = mix(mix(gradients[0],gradients[1],uv.x),
                      mix(gradients[2],gradients[3],uv.x),uv.y);
  return normalize(vec3(-gradient,1)) * (top ? 1.0 : -1.0);
}
bool node_hit(int node, vec3 ro, vec3 rd, float limit, out float entry) {
  float leave; vec3 normal;
  return box_hit(ro, rd, texelFetch(bounds_data, node * 2).xyz,
                 texelFetch(bounds_data, node * 2 + 1).xyz,
                 limit, false, entry, leave, normal);
}
void main() {
  colour = vec4(14.0/255.0, 17.0/255.0, 22.0/255.0, 1);
  gl_FragDepth = 1.0;
  if (empty_grid) return;
  vec2 p = vec2(gl_FragCoord.x, viewport.y - gl_FragCoord.y) - pan;
  vec3 ro = centre + right * (p.x / scale) - up * (p.y / scale) + toward * reach;
  ro.xy -= grid_origin;
  vec3 rd = -toward;
  float closest = 2.0 * reach;
  vec3 normal = vec3(0);
  bool hit = false;
  int stack[32]; int count = 0, node = 0;
  bool check_bounds = true;
  while (true) {
    ivec4 info = texelFetch(tree_data, node);
    if (info.z >= 0) {
      // Leaf intersection already tests its bounds and computes entry normals.
      vec3 n;
      if (leaf_hit(info.w != 0, ro, rd, texelFetch(bounds_data, node * 2).xyz,
                   texelFetch(bounds_data, node * 2 + 1).xyz, closest, n)) {
        hit = true; normal = n;
      }
    } else {
      // Descend into the nearer child immediately: its bounds were just tested.
      // Deferred internal nodes still need a new test against the latest hit.
      float entry;
      if (!check_bounds || node_hit(node,ro,rd,closest,entry)) {
        float a, b;
        bool ha = node_hit(info.x, ro, rd, closest, a);
        bool hb = node_hit(info.y, ro, rd, closest, b);
        if (ha || hb) {
          if (ha && hb) {
            bool left_first=a<=b;
            stack[count++]=left_first?info.y:info.x;
            node=left_first?info.x:info.y;
          } else node=ha?info.x:info.y;
          check_bounds=false;
          continue;
        }
      }
    }
    if (count==0) break;
    node=stack[--count];
    check_bounds=true;
  }
  // Evaluate derivatives before hit-dependent branches, including background
  // lanes in the fragment quad. Fine cap normals can use these derivatives;
  // walls use the filtered occupancy gradient below.
  vec3 position = ro + rd * closest;
  vec3 dx = dFdx(position), dy = dFdy(position);
  if (hit) {
    vec3 screen=cross(dx,dy);
    bool screen_valid=pitch*scale<1.5 && dot(screen,screen)>1e-12 &&
        length(dx)<4.0/scale && length(dy)<4.0/scale;
    if(screen_valid) {
      screen=normalize(screen);
      if(dot(screen,toward)<0.0) screen=-screen;
    }
    // Subpixel caps already span several simulation samples per pixel.
    // Use derivatives where credible; retain sampled cap normals at close-up
    // views and creases. Wall gradients filter a small XY neighbourhood.
    if(abs(normal.z)<1e-6) {
      normal=wall_normal(position,normal);
    } else {
      if(screen_valid && dot(screen,normal)>.85) normal=screen;
      else normal=shading_normal(position.xy,normal);
    }
    colour = vec4(illuminate(normal, vec3(.34, .45, .58)), 1);
    gl_FragDepth = closest / (2.0 * reach);
  }
}
)GLSL";
inline constexpr const char* mesh_vertex = R"GLSL(
#version 150
in vec3 position;
in vec3 normal;
uniform vec3 centre, right, up, toward, offset;
uniform vec2 pan, viewport;
uniform float scale, reach;
out vec3 surface_normal;
void main() {
  vec3 r = position + offset - centre;
  vec2 pixel = pan + vec2(dot(r, right), -dot(r, up)) * scale;
  gl_Position = vec4(pixel.x * 2.0 / viewport.x - 1.0,
                     1.0 - pixel.y * 2.0 / viewport.y,
                     -dot(r, toward) / reach, 1);
  surface_normal = normal;
}
)GLSL";
inline constexpr const char* mesh_fragment = R"GLSL(
#version 150
in vec3 surface_normal;
uniform vec3 light, right, up, toward;
uniform vec4 base_colour;
out vec4 colour;
vec3 illuminate(vec3 normal, vec3 base);
void main() {
  colour = vec4(illuminate(normalize(surface_normal), base_colour.rgb), base_colour.a);
}
)GLSL";
// Shared lighting for the reconstructed stock, meshes and cutter.
inline constexpr const char* surface_lighting = R"GLSL(
vec3 illuminate(vec3 normal, vec3 base) {
  vec3 n = normalize(normal);
  vec3 key = normalize(light);
  vec3 fill = normalize(-.65 * right + .25 * up + .6 * toward);
  vec3 half_direction = normalize(key + toward);
  float diffuse = max(0.0, dot(n, key));
  float bounce = max(0.0, dot(n, fill));
  float highlight = pow(max(0.0, dot(n, half_direction)), 28.0) * .28;
  float rim = pow(1.0 - max(0.0, dot(n, toward)), 4.0) * .035;
  vec3 illumination = vec3(.21, .23, .26) +
      vec3(.76, .72, .65) * diffuse + vec3(.22, .27, .34) * bounce;
  return base * illumination + vec3(highlight + rim);
}
)GLSL";
}  // namespace wave::heightfield
