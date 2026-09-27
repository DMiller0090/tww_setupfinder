/* The corridor as a flat grid of ground triangles, so a frame's floor is an array index. A swept
 * path touching a marked cell goes to the engine; thresholds err toward marking. */
#pragma once

#include <cstdint>
#include <vector>

#include "corridor.h"

namespace search {

/** Why a cell is marked; one bit a reason so hand-offs are counted per reason. */
enum Mark : uint8_t {
  kNoGround = 1u << 0,
  /** Ground normals disagree within the cell or with a neighbour. */
  kNormal = 1u << 1,
  /** A height step within the cell or against a neighbour: `procFall`'s case. */
  kHeight = 1u << 2,
  kSlope = 1u << 3,
};

struct Limits {
  double cell = 50.0;
  /** The cell widens rather than exceed this. */
  int max_cells = 4000000;
  /** `1 - dot` of two face normals; 0.02 is about eleven degrees. */
  double normal_apart = 0.02;
  /** Not measured: the drop that hands a frame to `procFall`. */
  double height_step = 50.0;
  /** Not measured. Ground whose normal y is below this is marked. */
  double slope_normal_y = 0.9;
  /** Link's wall cylinder radius (`WallCyl`). */
  double reach = 35.0;
  /** The highest of Link's three cylinders. */
  double body_height = 125.0;
};

struct Grid {
  /** `Limits::cell`, widened if `max_cells` bit. */
  double cell = 0;
  double origin_x = 0, origin_z = 0;
  int nx = 0, nz = 0;

  /** `(nx * nz) + 1` offsets; cell c holds `tri[start[c] .. start[c + 1])`. */
  std::vector<int> start;
  /** Triangle indices into `Selection::ground`. */
  std::vector<int> tri;
  /** Indices into `Selection::wall`, each in every cell its xz box grown by `reach` touches. */
  std::vector<int> wall_start, wall_tri;
  int wall_cells = 0, worst_wall_cell = 0;

  double reach = 0;
  double body_height = 0;

  std::vector<uint8_t> mark;
  /** nx, ny, nz, d per ground triangle; unit normals, y positive. */
  std::vector<double> plane;

  /** Ground height range over the cell; meaningless under `kNoGround`. */
  std::vector<float> low, high;

  /** `marked` counts cells; the per-reason counts can sum past it. */
  int cells = 0, marked = 0;
  int marked_no_ground = 0, marked_normal = 0, marked_height = 0, marked_slope = 0;
  int worst_cell = 0;
  double per_cell = 0;

  int at(int ix, int iz) const { return iz * nx + ix; }
  bool inside(int ix, int iz) const { return ix >= 0 && iz >= 0 && ix < nx && iz < nz; }
};

Grid build(const Selection& selection, const Corridor& corridor, const Limits& limits = Limits());

}  // namespace search
