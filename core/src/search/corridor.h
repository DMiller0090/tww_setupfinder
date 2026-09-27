/* The xz region the grid is built over: the start-to-target line widened by the widest single move.
 * It selects triangles; it does not prune the search. */
#pragma once

#include <vector>

#include "../geom/mesh.h"
#include "target.h"

namespace search {

/** Every xz point within `half_width` of the segment (ax, az)-(bx, bz), plus the goal box when
 *  `boxed`. */
struct Corridor {
  double ax = 0, az = 0;
  double bx = 0, bz = 0;
  double half_width = 0;

  double min_x = 0, min_z = 0, max_x = 0, max_z = 0;

  /** A range or freed-axis goal's box. False for a plain point, which must select exactly as the
   *  stadium alone does. */
  bool boxed = false;
  double gx0 = 0, gz0 = 0, gx1 = 0, gz1 = 0;

  /** Distance to the spine or the goal box, whichever is nearer. */
  double from_spine(double x, double z) const;
  bool holds(double x, double z) const { return from_spine(x, z) <= half_width; }
};

/** A negative `widest_move` is taken as zero. */
Corridor corridor(double start_x, double start_z, double target_x, double target_z,
                  double widest_move);

/** A plain point takes the overload above; a range or freed axis adds the goal's box, clamped to
 *  `room` (`geom::Mesh::box`). */
Corridor corridor(double start_x, double start_z, const Target& target, const float room[6],
                  double widest_move);

/** Triangles a corridor reaches, nine floats each. Roofs are not selected. */
struct Selection {
  std::vector<float> ground, wall;
  /** Shorter than `ground` for a hand-made selection; a triangle past its end is code 0. */
  std::vector<uint8_t> ground_code;
  /** The game triangle each was cut from; empty means each is its own. */
  std::vector<float> ground_source;

  int ground_held = 0, ground_total = 0;
  int wall_held = 0, wall_total = 0;
};

/** Held when the filled xz triangle comes within `half_width` of the spine. */
Selection select(const geom::Mesh& mesh, const Corridor& corridor);

}  // namespace search
