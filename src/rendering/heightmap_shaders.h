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
bool leaf_hit(int tile, vec3 ro, vec3 rd, vec3 lo, vec3 hi,
              inout float closest, out vec3 normal) {
  float a, b; vec3 n;
  if (!box_hit(ro, rd, lo, hi, closest, false, a, b, n)) return false;
  ivec4 desc = texelFetch(tile_data, tile);
  if (desc.x == 0 && desc.z == 0) {
    if (!box_hit(ro, rd, lo, hi, closest, true, a, b, n)) return false;
    closest = a; normal = n; return true;
  }
  ivec2 begin = ivec2(round(lo.xy / pitch));
  ivec2 end = min(begin + ivec2(32), grid_size);
  ivec2 cell = clamp(ivec2(floor((ro.xy + rd.xy * a) / pitch)), begin, end - 1);
  ivec2 step = ivec2(sign(rd.xy));
  vec2 next = vec2(1e30), delta = vec2(1e30);
  for (int k = 0; k < 2; ++k) if (step[k] != 0) {
    float edge = float(cell[k] + (step[k] > 0 ? 1 : 0)) * pitch;
    next[k] = max(a, (edge - ro[k]) / rd[k]);
    delta[k] = pitch / abs(rd[k]);
  }
  // A line crosses at most 63 XY cells in a 32x32 tile (including corners).
  for (int i = 0; i < 66; ++i) {
    ivec2 local = cell - begin;
    float top = height_at(desc.x, desc.y, local);
    float bottom = height_at(desc.z, desc.w, local);
    float t, exit_t; vec3 hit_normal;
    if (!isnan(top) && !isnan(bottom) && top > bottom &&
        box_hit(ro, rd, vec3(vec2(cell) * pitch, bottom),
                vec3(vec2(cell + 1) * pitch, top), closest, true,
                t, exit_t, hit_normal)) {
      closest = t; normal = hit_normal; return true;
    }
    float boundary = min(next.x, next.y);
    if (boundary > b || boundary >= closest) break;
    bool sx = next.x <= next.y, sy = next.y <= next.x;
    if (sx) { cell.x += step.x; next.x += delta.x; }
    if (sy) { cell.y += step.y; next.y += delta.y; }
    if (any(lessThan(cell, begin)) || any(greaterThanEqual(cell, end))) break;
  }
  return false;
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
  int stack[32]; int count = 1; stack[0] = 0;
  while (count > 0) {
    int node = stack[--count]; float entry;
    if (!node_hit(node, ro, rd, closest, entry)) continue;
    ivec4 info = texelFetch(tree_data, node);
    if (info.z >= 0) {
      vec3 n;
      if (leaf_hit(info.z, ro, rd, texelFetch(bounds_data, node * 2).xyz,
                   texelFetch(bounds_data, node * 2 + 1).xyz, closest, n)) {
        hit = true; normal = n;
      }
    } else {
      float a, b;
      bool ha = node_hit(info.x, ro, rd, closest, a);
      bool hb = node_hit(info.y, ro, rd, closest, b);
      if (ha && hb) {
        if (a <= b) { stack[count++] = info.y; stack[count++] = info.x; }
        else { stack[count++] = info.x; stack[count++] = info.y; }
      } else if (ha) stack[count++] = info.x;
      else if (hb) stack[count++] = info.y;
    }
  }
  if (hit) {
    float shade = .42 + .58 * max(0.0, dot(normal, light));
    colour = vec4(vec3(.34, .45, .58) * shade, 1);
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
uniform vec3 light;
uniform vec4 base_colour;
out vec4 colour;
void main() {
  float shade = .42 + .58 * max(0.0, dot(normalize(surface_normal), light));
  colour = vec4(base_colour.rgb * shade, base_colour.a);
}
)GLSL";
}  // namespace wave::heightfield
