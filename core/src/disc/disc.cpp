#include "disc.h"

#include "../geom/be.h"
#include "../geom/dzb.h"
#include "rarc.h"

namespace disc {
namespace {

constexpr char kStages[] = "res/Stage";

using namespace geom::bgd;

std::string room_path(const std::string& stage, int room) {
  return std::string(kStages) + "/" + stage + "/Room" + std::to_string(room) + ".arc";
}

/** `Room<n>.arc` -> `n`, or -1. */
int room_number(const std::string& name) {
  if (name.size() < 9) return -1;
  if (name.compare(0, 4, "Room") != 0) return -1;
  if (name.compare(name.size() - 4, 4, ".arc") != 0) return -1;
  int n = 0;
  for (size_t i = 4; i + 4 < name.size(); ++i) {
    if (name[i] < '0' || name[i] > '9') return -1;
    n = n * 10 + (name[i] - '0');
    if (n > 255) return -1;
  }
  return name.size() > 8 ? n : -1;
}

}  // namespace

std::vector<Where> rooms_on(const Iso& iso) {
  std::vector<Where> found;
  for (const std::string& path : iso.under(kStages)) {
    // `res/Stage/<stage>/Room<n>.arc`, nothing deeper.
    const size_t stage_at = std::string(kStages).size() + 1;
    const size_t slash = path.find('/', stage_at);
    if (slash == std::string::npos) continue;
    if (path.find('/', slash + 1) != std::string::npos) continue;
    const int n = room_number(path.substr(slash + 1));
    if (n < 0) continue;
    Where w;
    w.stage = path.substr(stage_at, slash - stage_at);
    w.room = n;
    found.push_back(std::move(w));
  }
  return found;
}

bool dzb_of(const Iso& iso, const std::string& stage, int room, std::vector<uint8_t>* out,
            std::string* why) {
  std::vector<uint8_t> archive;
  if (!iso.file(room_path(stage, room), &archive)) {
    *why = "the disc has no " + stage + " room " + std::to_string(room);
    return false;
  }
  if (!from_rarc(archive, ".dzb", out)) {
    *why = "that room has no collision in it";
    return false;
  }
  return true;
}

namespace {

/* Chunk list: u32 count, then per chunk tag[4], u32 entries, u32 offset (`d_stage.cpp`). */
bool chunk_of(const std::vector<uint8_t>& file, const char* tag, uint32_t* count, uint32_t* at) {
  if (file.size() < 4) return false;
  const uint32_t n = geom::be32(&file[0]);
  for (uint32_t i = 0; i < n; ++i) {
    const size_t row = 4 + static_cast<size_t>(i) * 12;
    if (row + 12 > file.size()) return false;
    if (std::memcmp(&file[row], tag, 4) == 0) {
      *count = geom::be32(&file[row + 4]);
      *at = geom::be32(&file[row + 8]);
      return true;
    }
  }
  return false;
}

/* `stage_camera2_data_class`: 0x14-byte entries, name in the first 16. Stage files use `CAMR` or
   `RCAM`, room files `RCAM`; `dStage_cameraInit` runs per chunk in order, so the last one wins. */
std::vector<std::string> camera_list(const std::vector<uint8_t>& file, bool stage_file) {
  std::vector<std::string> out;
  if (file.size() < 4) return out;
  const uint32_t n = geom::be32(&file[0]);
  for (uint32_t c = 0; c < n; ++c) {
    const size_t head = 4 + static_cast<size_t>(c) * 12;
    if (head + 12 > file.size()) break;
    const bool rcam = std::memcmp(&file[head], "RCAM", 4) == 0;
    const bool camr = stage_file && std::memcmp(&file[head], "CAMR", 4) == 0;
    if (!rcam && !camr) continue;
    const uint32_t count = geom::be32(&file[head + 4]);
    const uint32_t at = geom::be32(&file[head + 8]);
    out.clear();
    for (uint32_t i = 0; i < count; ++i) {
      const size_t row = at + static_cast<size_t>(i) * 0x14;
      if (row + 0x14 > file.size()) break;
      const char* name = reinterpret_cast<const char*>(&file[row]);
      out.push_back(std::string(name, strnlen(name, 16)));
    }
  }
  return out;
}

}  // namespace

bool camera_names_of(const Iso& iso, const std::string& stage, int room, CameraNames* out,
                     std::string* why) {
  std::vector<uint8_t> archive, dzs, dzr;
  if (!iso.file(std::string(kStages) + "/" + stage + "/Stage.arc", &archive) ||
      !from_rarc(archive, ".dzs", &dzs)) {
    *why = "the disc has no stage file for " + stage;
    return false;
  }
  if (!iso.file(room_path(stage, room), &archive)) {
    *why = "the disc has no " + stage + " room " + std::to_string(room);
    return false;
  }
  from_rarc(archive, ".dzr", &dzr);
  *out = CameraNames();
  uint32_t count = 0, at = 0;
  if (chunk_of(dzs, "STAG", &count, &at) && count > 0 && at + 9 <= dzs.size()) {
    out->stage_id = dzs[at + 8];
  }
  out->stage_list = camera_list(dzs, true);
  out->room_list = camera_list(dzr, false);
  return true;
}

geom::Mesh room_of(const Iso& iso, const std::string& stage, int room) {
  geom::Mesh out;
  std::vector<uint8_t> dzb;
  if (!dzb_of(iso, stage, room, &dzb, &out.why)) {
    return out;
  }
  if (dzb.size() < kHeaderBytes) {
    out.why = "that room's collision is too short to read";
    return out;
  }

  // Offsets here, pointers in RAM: `cBgS::ConvDzb` adds the file's address and nothing else.
  const int32_t vert_count = geom::be_s32(dzb.data() + kVertCount);
  const int32_t tri_count = geom::be_s32(dzb.data() + kTriCount);
  const uint32_t verts_at = geom::be32(dzb.data() + kVertTable);
  const uint32_t tris_at = geom::be32(dzb.data() + kTriTable);
  if (verts_at >= dzb.size() || tris_at >= dzb.size()) {
    out.why = "that room's collision does not describe itself";
    return out;
  }
  // Groups (water) and attributes (stairs) are optional; an unreadable table is treated as absent.
  geom::Groups groups;
  const int32_t grp_count = geom::be_s32(dzb.data() + kGrpCount);
  const uint32_t grps_at = geom::be32(dzb.data() + kGrpTable);
  if (grp_count > 0 && grps_at < dzb.size() &&
      static_cast<uint64_t>(grp_count) * geom::kGrpBytes <= dzb.size() - grps_at) {
    groups.rows = dzb.data() + grps_at;
    groups.bytes = dzb.size() - grps_at;
    groups.count = grp_count;
  }
  geom::Attributes attributes;
  const int32_t ti_count = geom::be_s32(dzb.data() + kTiCount);
  const uint32_t tis_at = geom::be32(dzb.data() + kTiTable);
  if (ti_count > 0 && tis_at < dzb.size() &&
      static_cast<uint64_t>(ti_count) * geom::kTiBytes <= dzb.size() - tis_at) {
    attributes.rows = dzb.data() + tis_at;
    attributes.bytes = dzb.size() - tis_at;
    attributes.count = ti_count;
  }

  if (!geom::append(out, dzb.data() + verts_at, dzb.size() - verts_at, vert_count,
                    dzb.data() + tris_at, dzb.size() - tris_at, tri_count, groups,
                    attributes)) {
    out.why = "that room's collision does not describe itself";
    return out;
  }
  out.meshes = 1;
  out.ok = true;
  return out;
}

}  // namespace disc
