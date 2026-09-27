/* The camera type a room runs under a standing player: the on-foot, no-event half of
 * `dCamera_c::nextType`. `Keep` and id 0x1FF are refused, since a posed room has no prior type. */
#pragma once

#include <string>

#include "../disc/disc.h"

namespace tww_engine {
struct Session;
}

namespace search {

/** `mRoomMapToolCameraIdx`, `mRoomNo` and `m350`. `id` 0xFF is none, 0x1FF too far above the
 *  floor; `room` -1 means `id` indexes the stage's list. */
struct FloorCamera {
  int id = 0xFF;
  int room = -1;
  int cam_move_bg = 0;
};

/** The `dCamera_c::types` index, or -1 with `why` set for the Logs. */
int camera_type(const disc::CameraNames& names, const FloorCamera& floor, std::string* why);

/** Read off the session's camera after its last frame. */
FloorCamera floor_camera(const tww_engine::Session& session);

}  // namespace search
