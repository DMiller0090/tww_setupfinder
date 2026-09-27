/* A room's `.dzb` collision decoded into `tww_engine::RoomDzb`, the six tables of `cBgD_t`. */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "../geom/dzb.h"
#include "../geom/mesh.h"
#include "engine/room.h"

namespace search {

/** From a file's bytes, every count and offset bounds-checked. `why` is for the Logs. */
bool room_of(const std::vector<uint8_t>& dzb, tww_engine::RoomDzb* out, std::string* why);

/** From tables already located in RAM, where `cBgS::ConvDzb` has turned the offsets into pointers.
 *  Counts and vertex indices are still checked. `why` is for the Logs. */
bool room_of(const geom::bgd::Tables& tables, tww_engine::RoomDzb* out, std::string* why);

/** A tilted slab of ground as engine tables and as a mesh, for calibration with no disc.
 *  The plane is `y = y_at_origin + grade_x * x + grade_z * z`; `half_extent` bounds x and z. */
struct Slab {
  tww_engine::RoomDzb room;
  geom::Mesh mesh;
  double nx = 0, ny = 1, nz = 0, d = 0;
  double at(double x, double z) const { return (d - nx * x - nz * z) / ny; }
};

/** `ground_code` goes into both the attribute row and the mesh. 8 is stairs. */
Slab slab(double y_at_origin, double grade_x, double grade_z, double half_extent = 32768.0,
          uint8_t ground_code = 0);

}  // namespace search
