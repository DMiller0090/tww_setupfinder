/* A room's collision off the disc image.
 *
 * The stage's `MULT` is not applied: `daBg_c::createHeap` sets the mesh with a null base matrix, so
 * the DZB vertices on the disc are the ones in RAM. Moving actors' collision is not in any room file.
 */
#pragma once

#include <string>
#include <vector>

#include "../geom/mesh.h"
#include "gcm.h"

namespace disc {

/** The release whose addresses the rest of the core knows. */
constexpr char kKnownDisc[] = "GZLJ01";

struct Where {
  std::string stage;
  int room = 0;
};

/** Every room on the disc, in the file table's order. No archive is opened. */
std::vector<Where> rooms_on(const Iso& iso);

/** `ok` is false with `why` when there is no such room or it has no collision file. */
geom::Mesh room_of(const Iso& iso, const std::string& stage, int room);

/** The room archive's `.dzb` member, undecoded. */
bool dzb_of(const Iso& iso, const std::string& stage, int room, std::vector<uint8_t>* out,
            std::string* why);

/** The inputs `dCamera_c::dCamera_c` and `nextType` use to pick a camera type.
 *
 *  - `stage_id`: `STAG` `mCameraMapToolID` (byte 0x08), an index into `stage_list`; 0xFF or out of
 *    range means `Field`.
 *  - `stage_list`: the stage file's `CAMR` or `RCAM`.
 *  - `room_list`: the room file's `RCAM`; missing is an empty list, not a failure. */
struct CameraNames {
  int stage_id = 0xFF;
  std::vector<std::string> stage_list;
  std::vector<std::string> room_list;
};
bool camera_names_of(const Iso& iso, const std::string& stage, int room, CameraNames* out,
                     std::string* why);

}  // namespace disc
