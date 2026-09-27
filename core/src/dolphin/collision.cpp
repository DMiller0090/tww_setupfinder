#include "collision.h"

#include <vector>

#include "../geom/be.h"
#include "../geom/dzb.h"
#include "../geom/mesh.h"

namespace dolphin {

using namespace geom::bgd;

namespace {

/* `cBgS_ChkElm m_chk_element[256]`. */
constexpr int kSlots = 256;
constexpr uint32_t kSlotStride = 0x14;
constexpr uint32_t kElemBgw = 0x00;    // cBgW* m_bgw_base_ptr
constexpr uint32_t kElemFlags = 0x04;  // u32 m_flags, bit 0 = in use
constexpr uint32_t kElemActor = 0x0C;  // fopAc_ac_c* m_actor_ptr

/* `dStage_roomStatus_c`. */
constexpr uint32_t kRoomStatusStride = 0x114;
constexpr uint32_t kRoomStatusBgw = 0x110;  // dBgW* mpBgW
constexpr int kRooms = 64;

/* `cBgW::mFlags`. */
constexpr uint32_t kBgwFlags = 0x6C;
constexpr uint8_t kMoveBg = 0x01;
constexpr uint8_t kGlobal = 0x20;

/* `cBgW`. The world vertex table, since a moving object's DZB `m_v_tbl` is local. */
constexpr uint32_t kBgwVtx = 0x90;  // cBgD_Vtx_t* pm_vtx_tbl
constexpr uint32_t kBgwBgd = 0x94;  // cBgD_t* pm_bgd

bool in_mem1(uint32_t p) { return p >= kMem1Start && p < kMem1Start + kMem1Size; }

/** Appends one registered mesh. False when the slot holds none. */
bool read_mesh(const Mem& mem, uint32_t bgw, Room& room) {
  uint32_t bgd = 0, vtx_table = 0, tri_table = 0;
  int32_t vert_count = 0, tri_count = 0;
  if (!mem.u32(bgw + kBgwBgd, &bgd) || !in_mem1(bgd)) return false;
  if (!mem.u32(bgw + kBgwVtx, &vtx_table) || !in_mem1(vtx_table)) return false;
  if (!mem.s32(bgd + kVertCount, &vert_count) || !mem.s32(bgd + kTriCount, &tri_count) ||
      !mem.u32(bgd + kTriTable, &tri_table)) {
    return false;
  }
  if (vert_count <= 0 || vert_count >= geom::kMaxVerts) return false;
  if (tri_count <= 0 || tri_count >= geom::kMaxTris) return false;
  if (!in_mem1(tri_table)) return false;

  std::vector<uint8_t> verts(static_cast<size_t>(vert_count) * geom::kVertexBytes);
  std::vector<uint8_t> tris(static_cast<size_t>(tri_count) * geom::kTriBytes);
  if (!mem.read(vtx_table, verts.data(), verts.size())) return false;
  if (!mem.read(tri_table, tris.data(), tris.size())) return false;

  // Groups (water) and attributes (stairs) are optional; one that will not read is left out.
  std::vector<uint8_t> grps;
  geom::Groups groups;
  int32_t grp_count = 0;
  uint32_t grp_table = 0;
  if (mem.s32(bgd + kGrpCount, &grp_count) && mem.u32(bgd + kGrpTable, &grp_table) &&
      grp_count > 0 && grp_count < geom::kMaxTris && in_mem1(grp_table)) {
    grps.resize(static_cast<size_t>(grp_count) * geom::kGrpBytes);
    if (mem.read(grp_table, grps.data(), grps.size())) {
      groups.rows = grps.data();
      groups.bytes = grps.size();
      groups.count = grp_count;
    }
  }
  std::vector<uint8_t> tis;
  geom::Attributes attributes;
  int32_t ti_count = 0;
  uint32_t ti_table = 0;
  if (mem.s32(bgd + kTiCount, &ti_count) && mem.u32(bgd + kTiTable, &ti_table) &&
      ti_count > 0 && ti_count < geom::kMaxTris && in_mem1(ti_table)) {
    tis.resize(static_cast<size_t>(ti_count) * geom::kTiBytes);
    if (mem.read(ti_table, tis.data(), tis.size())) {
      attributes.rows = tis.data();
      attributes.bytes = tis.size();
      attributes.count = ti_count;
    }
  }

  return geom::append(room, verts.data(), verts.size(), vert_count, tris.data(), tris.size(),
                      tri_count, groups, attributes);
}

/** One of `cBgD_t`'s tables. False for a bad count, an address outside MEM1, or overrun. */
struct Follow {
  const Read* read;
  bool at(uint32_t bgd, uint32_t count_field, uint32_t table_field, uint32_t row,
          std::vector<uint8_t>* bytes, int32_t* count) const {
    uint8_t head[8];
    if (!(*read)(bgd + count_field, head, 4)) return false;
    if (!(*read)(bgd + table_field, head + 4, 4)) return false;
    const int32_t rows = geom::be_s32(head);
    const uint32_t table = geom::be32(head + 4);
    if (rows < 0 || rows > geom::kMaxTris) return false;
    *count = rows;
    bytes->clear();
    if (rows == 0) return true;
    const uint64_t span = static_cast<uint64_t>(rows) * row;
    if (!in_mem1(table) || span > kMem1Start + kMem1Size - table) return false;
    bytes->resize(static_cast<size_t>(span));
    return (*read)(table, bytes->data(), bytes->size());
  }
};

}  // namespace

geom::bgd::Tables Dzb::located() const {
  geom::bgd::Tables at;
  at.v = v.data();    at.v_num = v_num;
  at.t = t.data();    at.t_num = t_num;
  at.b = b.data();    at.b_num = b_num;
  at.tree = tree.data();  at.tree_num = tree_num;
  at.g = g.data();    at.g_num = g_num;
  at.ti = ti.data();  at.ti_num = ti_num;
  return at;
}

Dzb dzb_at(const Read& read, uint32_t bgw) {
  Dzb out;
  uint8_t word[4];
  if (!read(bgw + kBgwBgd, word, 4)) {
    out.why = "the game's collision could not be read";
    return out;
  }
  const uint32_t bgd = geom::be32(word);
  if (!in_mem1(bgd)) {
    out.why = "the game has no collision loaded for that room";
    return out;
  }

  /* Pointers in RAM (`cBgS::ConvDzb`), so followed. Vertices are the header's `m_v_tbl`, not
     `cBgW::pm_vtx_tbl`: the same for a room, as the engine assumes. */
  const Follow follow = {&read};
  if (!follow.at(bgd, kVertCount, kVertTable, geom::kVertexBytes, &out.v, &out.v_num) ||
      !follow.at(bgd, kTriCount, kTriTable, geom::kTriBytes, &out.t, &out.t_num) ||
      !follow.at(bgd, kBlkCount, kBlkTable, geom::kBlkBytes, &out.b, &out.b_num) ||
      !follow.at(bgd, kTreeCount, kTreeTable, geom::kTreeBytes, &out.tree, &out.tree_num) ||
      !follow.at(bgd, kGrpCount, kGrpTable, geom::kGrpBytes, &out.g, &out.g_num) ||
      !follow.at(bgd, kTiCount, kTiTable, geom::kTiBytes, &out.ti, &out.ti_num)) {
    out.why = "the game's collision does not describe itself - is it still loading?";
    return out;
  }

  // Decoded from the same bytes, not a second read.
  geom::Groups groups;
  groups.rows = out.g.data();
  groups.bytes = out.g.size();
  groups.count = out.g_num;
  if (!geom::append(out.mesh, out.v.data(), out.v.size(), out.v_num, out.t.data(), out.t.size(),
                    out.t_num, groups)) {
    out.why = "the game's collision does not describe itself - is it still loading?";
    return out;
  }
  out.mesh.meshes = 1;
  out.mesh.ok = true;
  out.ok = true;
  return out;
}

Dzb dzb_of(const Mem& mem, int room) {
  Dzb out;
  if (room < 0 || room >= kRooms) {
    out.why = "the game is not in a room this build can read";
    return out;
  }
  uint32_t bgw = 0;
  if (!mem.u32(addr::kRoomStatus + static_cast<uint32_t>(room) * kRoomStatusStride + kRoomStatusBgw,
               &bgw) ||
      !in_mem1(bgw)) {
    out.why = "the game has no collision for the room it is in";
    return out;
  }
  return dzb_at([&mem](uint32_t at, void* dst, size_t n) { return mem.read(at, dst, n); }, bgw);
}

Room room_of(const Mem& mem, int room) {
  Room out;
  // `getBgW(roomNo)`; zero leaves only what moves.
  uint32_t room_mesh = 0;
  if (room >= 0 && room < kRooms) {
    mem.u32(addr::kRoomStatus + static_cast<uint32_t>(room) * kRoomStatusStride + kRoomStatusBgw,
            &room_mesh);
  }

  for (int slot = 0; slot < kSlots; ++slot) {
    const uint32_t elem = addr::kBgS + static_cast<uint32_t>(slot) * kSlotStride;
    uint32_t flags = 0, bgw = 0, actor = 0;
    if (!mem.u32(elem + kElemFlags, &flags)) continue;
    if (!(flags & 1)) continue;  // `cBgS_ChkElm::ChkUsed`
    if (!mem.u32(elem + kElemBgw, &bgw) || !in_mem1(bgw)) continue;

    uint8_t bgw_flags = 0;
    if (!mem.u8(bgw + kBgwFlags, &bgw_flags)) continue;
    mem.u32(elem + kElemActor, &actor);
    // Some actor-owned meshes never set `MOVE_BG_e`.
    const bool moves = (bgw_flags & kMoveBg) != 0 || (in_mem1(actor) && !(bgw_flags & kGlobal));
    if (bgw != room_mesh && !moves) continue;

    if (read_mesh(mem, bgw, out)) ++out.meshes;
  }
  if (out.meshes == 0) {
    out.why = "the game has no collision registered - is it still loading?";
    return out;
  }
  out.ok = true;
  return out;
}

namespace {

bool read_u32(const Read& read, uint32_t at, uint32_t* out) {
  uint8_t b[4];
  if (!read(at, b, sizeof(b))) return false;
  *out = geom::be32(b);
  return true;
}

/* `stage_camera_class`: count, pointer to 0x14-byte rows with a 16-char name. */
bool camera_list_at(const Read& read, uint32_t list, std::vector<std::string>* out) {
  out->clear();
  if (list == 0) return true;
  if (!in_mem1(list)) return false;
  uint32_t num = 0, rows = 0;
  if (!read_u32(read, list, &num) || !read_u32(read, list + 4, &rows)) return false;
  if (num == 0) return true;
  if (num > 255 || !in_mem1(rows)) return false;
  std::vector<uint8_t> bytes(static_cast<size_t>(num) * 0x14);
  if (!read(rows, bytes.data(), bytes.size())) return false;
  for (uint32_t i = 0; i < num; ++i) {
    const char* name = reinterpret_cast<const char*>(&bytes[static_cast<size_t>(i) * 0x14]);
    size_t n = 0;
    while (n < 16 && name[n] != 0) ++n;
    out->push_back(std::string(name, n));
  }
  return true;
}

}  // namespace

bool camera_names_at(const Read& read, int room, disc::CameraNames* out) {
  *out = disc::CameraNames();
  if (room < 0 || room >= kRooms) return false;
  uint32_t stage_list = 0, stag = 0, room_list = 0;
  const uint32_t room_dt =
      addr::kRoomStatus + static_cast<uint32_t>(room) * kRoomStatusStride;
  if (!read_u32(read, addr::kStageData + 0x04, &stage_list) ||
      !read_u32(read, addr::kStageData + 0x48, &stag) ||
      !read_u32(read, room_dt + 0x2C, &room_list)) {
    return false;
  }
  if (stag != 0) {
    if (!in_mem1(stag)) return false;
    uint8_t id = 0xFF;
    if (!read(stag + 0x08, &id, 1)) return false;
    out->stage_id = id;
  }
  return camera_list_at(read, stage_list, &out->stage_list) &&
         camera_list_at(read, room_list, &out->room_list);
}

bool camera_names_of(const Mem& mem, int room, disc::CameraNames* out) {
  return camera_names_at([&mem](uint32_t at, void* dst, size_t n) { return mem.read(at, dst, n); },
                         room, out);
}

}  // namespace dolphin
