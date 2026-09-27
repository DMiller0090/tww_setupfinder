/* The approximation: where a sequence of moves ends, cheaply, in pure fp. It orders candidates and,
 * with the greedy switch on, may discard one; it never admits an answer, the engine does. A path
 * near a marked cell or a wall goes to the engine whole, so there is no bonk model here. */
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "catalogue.h"
#include "corridor.h"
#include "grid.h"
#include "engine/room.h"

namespace search {

/** One frame's step on flat ground, in the entry facing's frame: `ahead` along it, `side` 90
 *  degrees clockwise. A step, not a running total. */
struct BaseFrame {
  double ahead = 0, side = 0;
  /** The part of the step carried by `speedF`, the only part the ground angle scales; the rest is
   *  the animation root's. */
  double speed_ahead = 0, speed_side = 0;
  double rise = 0;
  /** Height above the floor at frame end; greater than zero is airborne and takes no slope
   *  correction. */
  double above = 0;
  /** The facing at frame end, as an s16 delta from the entry facing. */
  int turn = 0;
};

/** One move's measurement. `driven` false has its reason in `why`: priced by hand, blocked, or it
 *  dispatched into an arm the engine records rather than runs (which reports `ok` with zero
 *  displacement). */
struct BaseMove {
  std::string id;
  bool driven = false;
  std::string why;

  /** What the search pays: the catalogue's cost unless a person set another. */
  int frames = 0;
  int engine_frames = 0;
  /** A person's price less the measured frames, added to the engine's run rather than written over
   *  it, since that run can be longer on slopes. */
  int surcharge = 0;

  /** `Move::stands`: how many moves this row stands for. */
  int stands = 1;

  /** The net, the sum of `step`. */
  double ahead = 0, side = 0;
  int turn = 0;
  /** The furthest any frame got from the start, in xz. */
  double reach = 0;

  std::vector<BaseFrame> step;

  /** The airborne frames, half-open, or two -1s. Flight time depends on the ground, so `step_move`
   *  re-integrates the arc with `gravity`, the per-frame change in `rise`, read off the run. */
  int air_from = -1, air_to = -1;
  double gravity = 0;
};

/** The whole roster measured once per search. */
struct BaseTable {
  std::vector<BaseMove> move;
  int driven = 0, not_driven = 0;
  /** The widest net displacement among the driven moves; zero when none. */
  double widest = 0;

  const BaseMove* of(const std::string& id) const;
};

/** Drive the roster on flat ground and measure every frame. `combos` goes to `roster()`. */
BaseTable base_table(bool combos = true);

/** A copy of the table with a person's prices, frames by move id. Unnamed moves and prices below
 *  one frame keep what was measured. */
BaseTable priced(const BaseTable& measured, const std::map<std::string, int>& costs);

/** What is under a point. `found` false with `inside` true is a cell with no floor; `inside` false
 *  is off the grid, a counted corridor-leave. */
struct Floor {
  bool inside = false;
  bool found = false;
  double y = 0;
  /** The chosen triangle's unit normal. */
  double nx = 0, ny = 1, nz = 0;
  int tri = -1;
  /** `geom::Mesh::ground_code`. */
  uint8_t code = 0;
  int cell = -1;
  uint8_t mark = 0;
};

/** `dBgS_Acch::GroundCheck` adds `m_ground_check_offset - m_ground_up_h` (60.0f - 0.0f). */
const double kGroundProbe = 60.0;

/** The ground under (x, z) as `cBgW::GroundCrossRp` chooses: the highest plane at or below the
 *  raised probe. */
Floor floor_at(const Grid& grid, const Selection& selection, double x, double z, double probe_y);

/** Whether any wall's xz projection comes within `radius` of the segment and overlaps the body's
 *  height (`y` up by `Grid::body_height`). Conservative: may say yes when nothing is touched,
 *  never the reverse. `radius` negative is `Grid::reach`; wider than that answers true. */
bool wall_in_reach(const Grid& grid, const Selection& selection, double x0, double z0, double x1,
                   double z1, double y, double radius = -1.0);

/** One move's measured slope correction, following `daPy_lk_c`:
 *
 *      ang = cM_atan2s(|n_xz| * cM_scos(cM_atan2s(n_x, n_z) - facing), n_y)
 *      scale  = 1 + ground_share * (cM_scos(ang) - 1)
 *      ang < 0:   scale *= up_level - up_slope * grade
 *
 *  `cM_atan2s` indexes its table at `(int)(ratio * 1024)`, so a grade under 1/1024 is level.
 *  Stored data, never updated during a run. */
struct MoveCal {
  std::string id;
  /** How much of the move the ground angle prices, 0 to 1. */
  double ground_share = 0.0;
  double up_level = 1.0;
  double up_slope = 0.0;

  /** The fit's remaining error in units, worst and mean, over `frames` sloped frames. */
  double worst = 0.0, mean = 0.0;
  int rows = 0;
  int frames = 0;
};

/** A move's engine end point per plane, in the move's frame, for what no slope scale carries (the
 *  IK's planted foot, `posMoveFromFootPos`). `ga` is the grade along the facing, `gc` across it.
 *  Rows are downhill, a level-angle band, then uphill; each node's `key` names its smooth piece,
 *  and a cell whose corners disagree is not answered. */
struct PlaneTable {
  std::string id;
  /** The steepest grade, and the grade of the first off-band row. */
  double max_grade = 0.0, edge = 0.0;
  /** Rows each side of the band (`2 * rows + 1` in all), and columns. */
  int rows = 0, cols = 0;
  /** Per node, row-major. `key` 255 is a node the engine could not run; `extra_frames` is over the
   *  flat count. */
  std::vector<float> ahead, side;
  std::vector<uint8_t> key;
  std::vector<int8_t> extra_frames;
  double worst = 0.0, mean = 0.0;
  int cells = 0, answered = 0;

  /** False outside the table or across a switch. `level_angle` and `uphill` are the game's s16 for
   *  the plane being zero, and negative. */
  bool at(double ga, double gc, bool level_angle, bool uphill, double* end_ahead, double* end_side,
          int* frames) const;
};

/** The whole calibration. A move with no row takes no correction. */
struct Calibration {
  std::vector<MoveCal> move;
  /** The steepest grade the fitting ladder reached; 0 unknown. */
  double ladder = 0.0;
  /** Rungs each way the charge was re-measured over; 0 unknown. */
  int charge_steps = 0;
  /** From `bench/plane_table.cpp`. */
  std::vector<PlaneTable> plane;
  const PlaneTable* plane_of(const std::string& id) const;

  /** The discard threshold for one move, scaled by its sloped frames. */
  double threshold(const std::string& id, int sloped_frames) const;
  /** The same; a `tabled` end takes its plane table's worst. */
  double threshold(const std::string& id, int sloped_frames, bool tabled) const;

  const MoveCal* of(const std::string& id) const;

  /** As `bench/calibrate.cpp` last wrote it. */
  static Calibration measured();
};

/** What one move did to a state. A handed-off move's end state is still filled in, for ordering;
 *  `why` holds the `Mark` bits that caused it. */
struct Stepped {
  bool ok = false;
  bool handed_off = false;
  uint8_t why = 0;
  /** The path left the grid. */
  bool left = false;

  /** Landed from the air onto ground that is not level; which frame it lands on is a discrete
   *  event, so the engine decides. */
  bool air_landing = false;

  /** A wall was within reach of a frame, not necessarily touched. */
  bool wall_near = false;

  /** Travelled over slope with no calibration rows, so its charge would be an unmeasured zero. */
  bool unmeasured = false;

  double x = 0, y = 0, z = 0;
  int facing = 0;
  /** The cost here, which moves with the arc's flight time. */
  int frames = 0;
  /** Frames over ground that is not level. */
  int sloped_frames = 0;
  /** Frames that took the uphill arm. */
  int uphill_frames = 0;
  /** Sloped frames on stairs (ground code 8), priced as level by `posMoveFromFootPos` but still
   *  charged; a move with any gets no plane table. */
  int stairs_frames = 0;

  /** The end came from the move's plane table. */
  bool tabled = false;
  /** The plane every grounded frame stood on (`Grid::plane`); `planes` is 0 none yet, 1 one, 2 more. */
  double plane[4] = {0, 0, 0, 0};
  int planes = 0;
  /** The last grounded frame's triangle, so the same one skips the compare. */
  int plane_tri = -1;
};

/** How much of the room a step reads. `None` applies the flat table at a facing with nothing
 *  handed off; walls need floors, so the three are ordered. */
enum class Collision { None, Floors, Solid };

/** Apply one measured move to a state over a built grid, without `tww_engine`. */
Stepped step_move(const BaseMove& move, const Grid& grid, const Selection& selection,
                  const Calibration& cal, double x, double y, double z, int facing,
                  Collision collision = Collision::Solid);

/** The same with `cal.of` and `cal.plane_of` already found, either null. */
Stepped step_move(const BaseMove& move, const Grid& grid, const Selection& selection,
                  const MoveCal* mc, const PlaneTable* pt, double x, double y, double z,
                  int facing, Collision collision = Collision::Solid);

/** `Calibration::threshold` with the rows already found. */
double charge_of(const MoveCal* mc, const PlaneTable* pt, int sloped_frames, bool tabled);

}  // namespace search
