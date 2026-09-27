#include "mesh.h"

#include <cmath>

#include "be.h"

namespace geom {
namespace {

/** Double, so rounding cannot move a triangle across a classification threshold. */
struct Vec {
  double x, y, z;
};

Vec vertex_at(const uint8_t* table, int32_t index) {
  const uint8_t* p = table + static_cast<size_t>(index) * kVertexBytes;
  return {be_f32(p), be_f32(p + 4), be_f32(p + 8)};
}

void widen(Mesh& mesh, const Vec& v) {
  const double xyz[3] = {v.x, v.y, v.z};
  for (int i = 0; i < 3; ++i) {
    const float f = static_cast<float>(xyz[i]);
    if (mesh.empty) {
      mesh.box[i] = f;
      mesh.box[i + 3] = f;
    } else {
      if (f < mesh.box[i]) mesh.box[i] = f;
      if (f > mesh.box[i + 3]) mesh.box[i + 3] = f;
    }
  }
  mesh.empty = false;
}

void push(std::vector<float>& into, const Vec& a, const Vec& b, const Vec& c) {
  const Vec* three[3] = {&a, &b, &c};
  for (const Vec* v : three) {
    into.push_back(static_cast<float>(v->x));
    into.push_back(static_cast<float>(v->y));
    into.push_back(static_cast<float>(v->z));
  }
}

}  // namespace

Face classify(double normal_y) {
  // `cBgW_CheckBGround` / `cBgW_CheckBRoof`; not symmetrical.
  if (normal_y >= 0.5) return Face::Ground;
  if (normal_y < -0.8) return Face::Roof;
  return Face::Wall;
}

uint32_t group_info(const Groups& groups, uint16_t group) {
  if (groups.rows == nullptr || groups.count <= 0) return 0;
  uint32_t info = 0;
  uint32_t at = group;
  // Capped against a `m_parent` cycle.
  for (int step = 0; step < 32; ++step) {
    if (at >= static_cast<uint32_t>(groups.count)) break;
    const size_t off = static_cast<size_t>(at) * kGrpBytes;
    if (off + kGrpBytes > groups.bytes) break;
    const uint8_t* row = groups.rows + off;
    info |= be32(row + 0x30);
    const uint16_t parent = be16(row + 0x24);
    // `m_parent` is 0xFFFF at the root.
    if (parent == 0xFFFF || parent == at) break;
    at = parent;
  }
  return info;
}

uint8_t ground_code(const Attributes& attributes, uint16_t row) {
  if (attributes.rows == nullptr || row >= attributes.count) return 0;
  const size_t off = static_cast<size_t>(row) * kTiBytes;
  if (off + kTiBytes > attributes.bytes) return 0;
  return static_cast<uint8_t>((be32(attributes.rows + off + 4) >> 21) & 0x1F);
}

bool append(Mesh& mesh, const uint8_t* verts, size_t verts_bytes, int32_t vert_count,
            const uint8_t* tris, size_t tris_bytes, int32_t tri_count, const Groups& groups,
            const Attributes& attributes) {
  if (vert_count <= 0 || vert_count >= kMaxVerts) return false;
  if (tri_count <= 0 || tri_count >= kMaxTris) return false;
  if (verts_bytes < static_cast<size_t>(vert_count) * kVertexBytes) return false;
  if (tris_bytes < static_cast<size_t>(tri_count) * kTriBytes) return false;

  for (int32_t i = 0; i < tri_count; ++i) {
    const uint8_t* t = tris + static_cast<size_t>(i) * kTriBytes;
    const uint16_t i0 = be16(t), i1 = be16(t + 2), i2 = be16(t + 4);
    if (i0 >= vert_count || i1 >= vert_count || i2 >= vert_count) continue;

    // Walls too: a water group's steep faces are not walls.
    if (groups.rows != nullptr &&
        (group_info(groups, be16(t + 8)) & kNotSolid) != 0) {
      ++mesh.not_solid;
      continue;
    }

    const Vec v0 = vertex_at(verts, i0), v1 = vertex_at(verts, i1), v2 = vertex_at(verts, i2);

    const double ax = v1.x - v0.x, ay = v1.y - v0.y, az = v1.z - v0.z;
    const double bx = v2.x - v0.x, by = v2.y - v0.y, bz = v2.z - v0.z;
    const double nx = ay * bz - az * by;
    const double ny = az * bx - ax * bz;
    const double nz = ax * by - ay * bx;
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    const double unit_y = len < 1e-9 ? 0.0 : ny / len;

    switch (classify(unit_y)) {
      case Face::Ground:
        // Pads codes for ground that was added without them.
        mesh.ground_code.resize(mesh.ground.size() / 9, 0);
        push(mesh.ground, v0, v1, v2);
        mesh.ground_code.push_back(ground_code(attributes, be16(t + 6)));
        break;
      case Face::Roof: push(mesh.roof, v0, v1, v2); break;
      case Face::Wall: push(mesh.wall, v0, v1, v2); break;
    }
    widen(mesh, v0);
    widen(mesh, v1);
    widen(mesh, v2);
  }
  return true;
}

void split_seabed(Mesh& mesh) {
  const double cut = static_cast<double>(kSeaSurface) - static_cast<double>(kSwimDepth);
  std::vector<float> kept, kept_source;
  std::vector<uint8_t> kept_code;
  kept.reserve(mesh.ground.size());
  kept_source.reserve(mesh.ground.size());
  auto fan = [](std::vector<float>& into, std::vector<uint8_t>* codes, uint8_t code,
                const std::vector<Vec>& poly) {
    for (size_t i = 1; i + 1 < poly.size(); ++i) {
      push(into, poly[0], poly[i], poly[i + 1]);
      if (codes != nullptr) codes->push_back(code);
    }
  };
  const std::vector<float>& g = mesh.ground;
  for (size_t t = 0; t + 8 < g.size(); t += 9) {
    const size_t tri = t / 9;
    const uint8_t code = tri < mesh.ground_code.size() ? mesh.ground_code[tri] : 0;
    const Vec v[3] = {{g[t], g[t + 1], g[t + 2]}, {g[t + 3], g[t + 4], g[t + 5]},
                      {g[t + 6], g[t + 7], g[t + 8]}};
    const bool over[3] = {v[0].y >= cut, v[1].y >= cut, v[2].y >= cut};
    const size_t pieces_before = kept.size();
    if (over[0] && over[1] && over[2]) {
      push(kept, v[0], v[1], v[2]);
      push(kept_source, v[0], v[1], v[2]);
      kept_code.push_back(code);
      continue;
    }
    if (!over[0] && !over[1] && !over[2]) { push(mesh.seabed, v[0], v[1], v[2]); continue; }
    // Sutherland-Hodgman against y = cut, both sides at once.
    std::vector<Vec> up, down;
    for (int i = 0; i < 3; ++i) {
      const Vec& p = v[i];
      const Vec& q = v[(i + 1) % 3];
      (over[i] ? up : down).push_back(p);
      if (over[i] != over[(i + 1) % 3]) {
        const double f = (cut - p.y) / (q.y - p.y);
        const Vec m{p.x + (q.x - p.x) * f, cut, p.z + (q.z - p.z) * f};
        up.push_back(m);
        down.push_back(m);
      }
    }
    fan(kept, &kept_code, code, up);
    for (size_t p = pieces_before; p < kept.size(); p += 9) push(kept_source, v[0], v[1], v[2]);
    fan(mesh.seabed, nullptr, code, down);
  }
  mesh.ground = std::move(kept);
  mesh.ground_source = std::move(kept_source);
  mesh.ground_code = std::move(kept_code);
}

void seabed_of(const std::string& stage, Mesh& mesh) {
  if (stage == "sea") split_seabed(mesh);
}

}  // namespace geom
