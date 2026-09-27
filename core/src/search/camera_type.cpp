#include "camera_type.h"

#include <cstring>

#include "d/d_camera.h"
#include "engine/session.h"

namespace search {
namespace {

/* `dCamera_c::mvBGTypes`, d_cam_type2.cpp:9-45, in order. */
const char* const kCamMoveBgTypes[] = {
    "????",       "Field",      "Dungeon",    "Plain",      "DungeonDown",    "DungeonUp",
    "DungeonCorner", "Jump",    "DungeonWide", "Room",      "FieldCushion",   "OverLook",
    "Corridor",   "Subject",    "DungeonPassage", "Cliff",  "Cliff2",         "MajTower",
    "Boss01",     "Boss02",     "Gamoss",     "MiniIsland", "Amoss",          "Cafe",
    "P_Ganon1",   "P_Ganon2",   "WindBoss",   "P_Ganon3",   "G_BedRoom",      "G_Roof",
    "G_BedRoom2", "Boss04",     "WindHall",   "BigBird",    "DStairs",
};
const int kCamMoveBgCount = sizeof(kCamMoveBgTypes) / sizeof(kCamMoveBgTypes[0]);

/* `GetCameraTypeFromCameraName`. */
int by_name(const std::string& name) {
  for (int i = 0; i < dCamera_c::type_num; ++i) {
    if (name == dCamera_c::types[i].name) return i;
  }
  return -1;
}

/* As `GetCameraTypeFromMapToolID` reads a list. */
bool entry(const std::vector<std::string>& list, int id, std::string* name) {
  if (id < 0 || id >= static_cast<int>(list.size())) return false;
  *name = list[id];
  return true;
}

int resolved(const std::string& name, std::string* why) {
  if (name == "Keep") {
    *why = "the floor's camera is Keep, which holds a type a posed room does not know";
    return -1;
  }
  const int type = by_name(name);
  if (type < 0) *why = "no camera type is named " + name;
  return type;
}

/* As `dCamera_c`'s constructor: `Field` when STAG's id reaches no entry. */
int stage_type(const disc::CameraNames& names, std::string* why) {
  std::string name;
  if (!entry(names.stage_list, names.stage_id, &name)) name = "Field";
  return resolved(name, why);
}

}  // namespace

int camera_type(const disc::CameraNames& names, const FloorCamera& floor, std::string* why) {
  std::string name;
  if (floor.id == 0x1FF) {
    *why = "the camera found no floor near enough under him, which holds the type it had";
    return -1;
  }
  if (floor.id != 0xFF) {
    const std::vector<std::string>& list = floor.room >= 0 ? names.room_list : names.stage_list;
    if (!entry(list, floor.id, &name)) return stage_type(names, why);
    return resolved(name, why);
  }
  if (floor.cam_move_bg > 0 && floor.cam_move_bg < kCamMoveBgCount) {
    return resolved(kCamMoveBgTypes[floor.cam_move_bg], why);
  }
  return stage_type(names, why);
}

FloorCamera floor_camera(const tww_engine::Session& session) {
  const tww_engine::Session::CameraFacts f = session.cameraFacts();
  FloorCamera out;
  out.id = f.room_cam_idx;
  out.room = f.room_no;
  out.cam_move_bg = f.cam_move_bg;
  return out;
}

}  // namespace search
