/* DZB vertex and triangle tables decoded into drawable triangles; the layout is the same in RAM
 * and on disc.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace geom {

/** Row widths. `cBgD_Vtx_t` is three f32; `cBgD_Tri_t` is five u16: vtx0, vtx1, vtx2, plane_id
 *  (the attribute row), group. `cBgD_Grp_t`'s first word is a `char*` in RAM, an offset on disc. */
constexpr uint32_t kVertexBytes = 12;
constexpr uint32_t kTriBytes = 10;
constexpr uint32_t kBlkBytes = 2;    // u16 startTri
constexpr uint32_t kTreeBytes = 20;  // mFlag, mParent, then eight children or one block
constexpr uint32_t kGrpBytes = 0x34;
constexpr uint32_t kTiBytes = 16;  // four u32

/** Sanity caps on counts read mid-load or off a bad file. */
constexpr int32_t kMaxVerts = 300000;
constexpr int32_t kMaxTris = 600000;

/** `cBgW_CheckB*`, by face normal y. */
enum class Face { Ground, Wall, Roof };

Face classify(double normal_y);

/** `cBgD_Grp_t::m_info` bits tested by `dBgW::ChkGrpThrough` (`d_bg_w.cpp`). */
constexpr uint32_t kGrpWater = 0x00100;
constexpr uint32_t kGrpLava = 0x00200;   // `Yogan`
constexpr uint32_t kGrpPoison = 0x00400; // `Doku`
constexpr uint32_t kGrpLight = 0x80000;

/** Triangles in these groups are dropped. `kGrpLight` is deliberately left out. */
constexpr uint32_t kNotSolid = kGrpWater | kGrpLava | kGrpPoison;

/** Absent (the default) means no group is water. */
struct Groups {
  const uint8_t* rows = nullptr;
  size_t bytes = 0;
  int32_t count = 0;
};

/** A group's `m_info` OR'd with every ancestor's: `ChkGrpThrough` only answers at tree depth 2, so
 *  the deciding bit can sit on an ancestor of the triangle's leaf group. */
uint32_t group_info(const Groups& groups, uint16_t group);

/** `cBgD_Ti_t` rows. Absent means every triangle is on code 0. */
struct Attributes {
  const uint8_t* rows = nullptr;
  size_t bytes = 0;
  int32_t count = 0;
};

/** `dBgS::GetGroundCode` (`d_bg_s.cpp`): bits 21-25 of the row's second word; 0 past the table. */
uint8_t ground_code(const Attributes& attributes, uint16_t row);

/** `posMoveFromFootPos` treats a frame on stairs as level ground. */
constexpr uint8_t kGroundStairs = 8;

/** Nine world-space floats a triangle. Fields are meaningful only when `ok`. */
struct Mesh {
  bool ok = false;
  std::string why;
  std::vector<float> ground, wall, roof;
  /** One per `ground` triangle; may be shorter, and a missing entry is code 0. */
  std::vector<uint8_t> ground_code;
  /** The uncut game triangle each `ground` piece came from, for its plane: a plane recomputed from
   *  a cut vertex far from the origin gives heights the game never does. Empty when nothing was cut. */
  std::vector<float> ground_source;
  /** Min x, y, z then max x, y, z; zero with no triangles. */
  float box[6] = {0, 0, 0, 0, 0, 0};
  /** Tables this came from. */
  int meshes = 0;
  /** Triangles dropped as water, lava or poison. */
  int not_solid = 0;
  /** See `split_seabed`. */
  std::vector<float> seabed;
  /** No vertex seen yet, so `box` is unset. */
  bool empty = true;
};

/** Appends one vertex and triangle table (raw big-endian, counts in entries, checked against the
 *  byte lengths). False when they do not describe a mesh. */
bool append(Mesh& mesh, const uint8_t* verts, size_t verts_bytes, int32_t vert_count,
            const uint8_t* tris, size_t tris_bytes, int32_t tri_count,
            const Groups& groups = Groups(), const Attributes& attributes = Attributes());

/* The sea stage's ocean floor is ordinary solid ground (the water is an actor, not collision).
   Link swims once the water over him is deeper than `daPy_HIO_swim_c0::m.field_0x24` (90), so
   ground deeper than that under the y = 0 surface is never stood on. */
constexpr float kSeaSurface = 0.0f;
constexpr float kSwimDepth = 90.0f;

/** Moves `mesh.ground` below `kSeaSurface - kSwimDepth` into `mesh.seabed`, cutting triangles that
 *  cross it. */
void split_seabed(Mesh& mesh);

/** `split_seabed` on the sea stage only. */
void seabed_of(const std::string& stage, Mesh& mesh);

}  // namespace geom
