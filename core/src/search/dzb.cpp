#include "dzb.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "../geom/be.h"
#include "../geom/dzb.h"

namespace search {
namespace {

using namespace geom::bgd;

using geom::kBlkBytes;
using geom::kGrpBytes;
using geom::kTiBytes;
using geom::kTreeBytes;
using geom::kTriBytes;
using geom::kVertexBytes;

/** Bounds a garbage count read from a process mid-load or a bad file. */
constexpr int32_t kMaxRows = 600000;

/** One table's bytes, or null when the file does not hold a table of that shape there. */
const uint8_t* table_at(const std::vector<uint8_t>& dzb, uint32_t offset, int32_t count,
                        uint32_t row_bytes) {
  if (count < 0 || count > kMaxRows) return nullptr;
  if (count == 0) return dzb.data();
  if (offset >= dzb.size()) return nullptr;
  const uint64_t span = static_cast<uint64_t>(count) * row_bytes;
  if (span > dzb.size() - offset) return nullptr;
  return dzb.data() + offset;
}

}  // namespace

bool room_of(const std::vector<uint8_t>& dzb, tww_engine::RoomDzb* out, std::string* why) {
  if (dzb.size() < kHeaderBytes) {
    *why = "the collision is too short to hold a header";
    return false;
  }

  geom::bgd::Tables at;
  at.v_num = geom::be_s32(dzb.data() + kVertCount);
  at.t_num = geom::be_s32(dzb.data() + kTriCount);
  at.b_num = geom::be_s32(dzb.data() + kBlkCount);
  at.tree_num = geom::be_s32(dzb.data() + kTreeCount);
  at.g_num = geom::be_s32(dzb.data() + kGrpCount);
  at.ti_num = geom::be_s32(dzb.data() + kTiCount);

  at.v = table_at(dzb, geom::be32(dzb.data() + kVertTable), at.v_num, kVertexBytes);
  at.t = table_at(dzb, geom::be32(dzb.data() + kTriTable), at.t_num, kTriBytes);
  at.b = table_at(dzb, geom::be32(dzb.data() + kBlkTable), at.b_num, kBlkBytes);
  at.tree = table_at(dzb, geom::be32(dzb.data() + kTreeTable), at.tree_num, kTreeBytes);
  at.g = table_at(dzb, geom::be32(dzb.data() + kGrpTable), at.g_num, kGrpBytes);
  at.ti = table_at(dzb, geom::be32(dzb.data() + kTiTable), at.ti_num, kTiBytes);
  if (at.v == nullptr || at.t == nullptr || at.b == nullptr || at.tree == nullptr ||
      at.g == nullptr || at.ti == nullptr) {
    *why = "the collision does not describe itself";
    return false;
  }
  return room_of(at, out, why);
}

bool room_of(const geom::bgd::Tables& at, tww_engine::RoomDzb* out, std::string* why) {
  const int32_t counts[6] = {at.v_num, at.t_num, at.b_num, at.tree_num, at.g_num, at.ti_num};
  const uint8_t* spans[6] = {at.v, at.t, at.b, at.tree, at.g, at.ti};
  for (int i = 0; i < 6; ++i) {
    if (counts[i] < 0 || counts[i] > kMaxRows || (counts[i] > 0 && spans[i] == nullptr)) {
      *why = "the collision does not describe itself";
      return false;
    }
  }

  const int32_t v_num = at.v_num, t_num = at.t_num, b_num = at.b_num;
  const int32_t tree_num = at.tree_num, g_num = at.g_num, ti_num = at.ti_num;
  const uint8_t *v = at.v, *t = at.t, *b = at.b, *tree = at.tree, *g = at.g, *ti = at.ti;

  /* `cM3d_CalcPla` reads `pm_vtx_tbl[tri.vtx0]` unbounded; a room read mid-load may be bad. */
  for (int32_t i = 0; i < t_num; ++i) {
    const uint8_t* row = t + static_cast<size_t>(i) * kTriBytes;
    for (int k = 0; k < 3; ++k) {
      if (geom::be16(row + k * 2) >= v_num) {
        *why = "the collision names a vertex it does not have";
        return false;
      }
    }
  }

  tww_engine::RoomDzb& d = *out;

  d.v_tbl.resize(static_cast<size_t>(v_num));
  for (int32_t i = 0; i < v_num; ++i) {
    const uint8_t* row = v + static_cast<size_t>(i) * kVertexBytes;
    d.v_tbl[i].x = geom::be_f32(row + 0);
    d.v_tbl[i].y = geom::be_f32(row + 4);
    d.v_tbl[i].z = geom::be_f32(row + 8);
  }

  d.t_tbl.resize(static_cast<size_t>(t_num));
  for (int32_t i = 0; i < t_num; ++i) {
    const uint8_t* row = t + static_cast<size_t>(i) * kTriBytes;
    d.t_tbl[i].vtx0 = geom::be16(row + 0);
    d.t_tbl[i].vtx1 = geom::be16(row + 2);
    d.t_tbl[i].vtx2 = geom::be16(row + 4);
    /* `dBgW::ChkPolyThrough` reads both; zeroing them is not neutral. */
    d.t_tbl[i].id = geom::be16(row + 6);
    d.t_tbl[i].grp = geom::be16(row + 8);
  }

  d.b_tbl.resize(static_cast<size_t>(b_num));
  for (int32_t i = 0; i < b_num; ++i) {
    d.b_tbl[i].startTri = geom::be16(b + static_cast<size_t>(i) * kBlkBytes);
  }

  /* `mChild[8]` and `mBlock` are a union: on a leaf the block index is `mChild[0]`. */
  d.tree_tbl.resize(static_cast<size_t>(tree_num));
  for (int32_t i = 0; i < tree_num; ++i) {
    const uint8_t* row = tree + static_cast<size_t>(i) * kTreeBytes;
    d.tree_tbl[i].mFlag = geom::be16(row + 0);
    d.tree_tbl[i].mParent = geom::be16(row + 2);
    for (int k = 0; k < 8; ++k) {
      d.tree_tbl[i].mChild[k] = geom::be16(row + 4 + k * 2);
    }
  }

  d.g_tbl.resize(static_cast<size_t>(g_num));
  for (int32_t i = 0; i < g_num; ++i) {
    const uint8_t* row = g + static_cast<size_t>(i) * kGrpBytes;
    cBgD_Grp_t& grp = d.g_tbl[i];
    /* A string-table offset in the file, a `char*` in RAM; no collision path reads it. */
    grp.m_name = nullptr;
    grp.m_scale.x = geom::be_f32(row + 0x04);
    grp.m_scale.y = geom::be_f32(row + 0x08);
    grp.m_scale.z = geom::be_f32(row + 0x0C);
    grp.m_rotation.x = static_cast<s16>(geom::be16(row + 0x10));
    grp.m_rotation.y = static_cast<s16>(geom::be16(row + 0x12));
    grp.m_rotation.z = static_cast<s16>(geom::be16(row + 0x14));
    grp.m_translation.x = geom::be_f32(row + 0x18);
    grp.m_translation.y = geom::be_f32(row + 0x1C);
    grp.m_translation.z = geom::be_f32(row + 0x20);
    grp.m_parent = geom::be16(row + 0x24);
    grp.m_next_sibling = geom::be16(row + 0x26);
    grp.m_first_child = geom::be16(row + 0x28);
    grp.m_room_id = geom::be16(row + 0x2A);
    grp.m_first_vtx_idx = geom::be16(row + 0x2C);
    grp.m_tree_idx = geom::be16(row + 0x2E);
    /* `dBgW::ChkGrpThrough` tests this against 0x80700 (water, lava, doku, light). */
    grp.m_info = geom::be32(row + 0x30);
  }

  d.ti_tbl.resize(static_cast<size_t>(ti_num));
  for (int32_t i = 0; i < ti_num; ++i) {
    const uint8_t* row = ti + static_cast<size_t>(i) * kTiBytes;
    d.ti_tbl[i].mPolyInf0 = geom::be32(row + 0);
    d.ti_tbl[i].mPolyInf1 = geom::be32(row + 4);
    d.ti_tbl[i].mPolyInf2 = geom::be32(row + 8);
    d.ti_tbl[i].mPolyInf3 = geom::be32(row + 12);
  }

  return true;
}

Slab slab(double y_at_origin, double grade_x, double grade_z, double half_extent,
          uint8_t ground_code) {
  /* Winding 0 -> 1 -> 2 gives an up normal under `cM3d_VectorProduct`; reversed, it is a roof. */
  const float e = static_cast<float>(half_extent);
  const float y00 = static_cast<float>(y_at_origin - grade_x * half_extent - grade_z * half_extent);
  const float y01 = static_cast<float>(y_at_origin - grade_x * half_extent + grade_z * half_extent);
  const float y10 = static_cast<float>(y_at_origin + grade_x * half_extent - grade_z * half_extent);
  const float y11 = static_cast<float>(y_at_origin + grade_x * half_extent + grade_z * half_extent);

  Slab s;
  s.room.v_tbl.resize(4);
  s.room.v_tbl[0].x = -e; s.room.v_tbl[0].y = y00; s.room.v_tbl[0].z = -e;
  s.room.v_tbl[1].x = -e; s.room.v_tbl[1].y = y01; s.room.v_tbl[1].z =  e;
  s.room.v_tbl[2].x =  e; s.room.v_tbl[2].y = y10; s.room.v_tbl[2].z = -e;
  s.room.v_tbl[3].x =  e; s.room.v_tbl[3].y = y11; s.room.v_tbl[3].z =  e;

  s.room.t_tbl.resize(2);
  s.room.t_tbl[0].vtx0 = 0; s.room.t_tbl[0].vtx1 = 1; s.room.t_tbl[0].vtx2 = 2;
  s.room.t_tbl[1].vtx0 = 3; s.room.t_tbl[1].vtx1 = 2; s.room.t_tbl[1].vtx2 = 1;
  for (int i = 0; i < 2; ++i) {
    s.room.t_tbl[i].id = 0;
    s.room.t_tbl[i].grp = 0;
  }

  s.room.b_tbl.resize(1);
  s.room.b_tbl[0].startTri = 0;

  /* One leaf node: `mFlag` bit 0. */
  s.room.tree_tbl.resize(1);
  s.room.tree_tbl[0].mFlag = 1;
  s.room.tree_tbl[0].mParent = 0xFFFF;
  s.room.tree_tbl[0].mBlock = 0;

  s.room.g_tbl.resize(1);
  cBgD_Grp_t& g = s.room.g_tbl[0];
  std::memset(&g, 0, sizeof(g));
  g.m_name = nullptr;
  g.m_scale.x = g.m_scale.y = g.m_scale.z = 1.0f;
  g.m_parent = 0xFFFF;
  g.m_next_sibling = 0xFFFF;
  g.m_first_child = 0xFFFF;
  g.m_room_id = 0;
  g.m_first_vtx_idx = 0;
  g.m_tree_idx = 0;
  g.m_info = 0;

  s.room.ti_tbl.resize(1);
  std::memset(&s.room.ti_tbl[0], 0, sizeof(s.room.ti_tbl[0]));
  s.room.ti_tbl[0].mPolyInf1 = static_cast<uint32_t>(ground_code & 0x1F) << 21;

  const float nine[2][9] = {
      {-e, y00, -e, -e, y01, e, e, y10, -e},
      {e, y11, e, e, y10, -e, -e, y01, e},
  };
  s.mesh.ok = true;
  s.mesh.meshes = 1;
  s.mesh.empty = false;
  for (int i = 0; i < 2; ++i) {
    s.mesh.ground.insert(s.mesh.ground.end(), nine[i], nine[i] + 9);
    s.mesh.ground_code.push_back(ground_code);
  }
  s.mesh.box[0] = -e; s.mesh.box[1] = std::min(std::min(y00, y01), std::min(y10, y11));
  s.mesh.box[2] = -e; s.mesh.box[3] = e;
  s.mesh.box[4] = std::max(std::max(y00, y01), std::max(y10, y11));
  s.mesh.box[5] = e;

  const double len = std::sqrt(grade_x * grade_x + grade_z * grade_z + 1.0);
  s.nx = -grade_x / len;
  s.ny = 1.0 / len;
  s.nz = -grade_z / len;
  s.d = s.ny * y_at_origin;
  return s;
}

}  // namespace search
