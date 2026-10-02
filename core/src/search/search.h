/* The search: a depth-first walk over the move catalogue that returns candidates, never answers.
 * Every candidate still has to be verified by `tww_engine` before it is called an answer.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "approx.h"
#include "cup_tape.h"
#include "cam_clear.h"
#include "grid.h"
#include "target.h"

namespace search {

/** One step of a candidate. `row` indexes `BaseTable::move`; `steps` is the signed C up turn step
 *  count on the turn row and zero elsewhere (`turn_steps`). */
struct Edge {
  int row = -1;
  int steps = 0;
  /** The L taps an L chain presses before its turnaround, and 0 on every other row. */
  int taps = 0;

  bool operator==(const Edge& o) const {
    return row == o.row && steps == o.steps && taps == o.taps;
  }
  bool operator<(const Edge& o) const {
    return row != o.row ? row < o.row : steps != o.steps ? steps < o.steps : taps < o.taps;
  }
};

/** Whether the target is Link or the item he carries overhead. */
enum class Aim { Player, Overhead };

/** Where the target is measured from. Aimed at the item it is `daDitem_c::set_pos`'s offset
 *  (30 across, 20 ahead) turned by `facing`. */
void aim_point(const struct Question& q, double x, double z, int facing, double* ax, double* az);

/** The largest xz distance the aim offset can put between the aim and Link's feet. */
double aim_reach(const struct Question& q);

/** The aim's height above Link's feet: the held item's height in `daDitem_c::set_pos`, else 0. */
double aim_lift(const struct Question& q);

/** The held item's position, `daDitem_c::set_pos` line for line: the offset turned by Link's angle
 *  x, facing and angle z, added to his position in f32. The one place the offset is applied. */
void item_point(double x, double y, double z, int angle_x, int shape_y, int angle_z, double* ax,
                double* ay, double* az);

/** Per facing, whether some f32 Link position puts the item on the target's own f32. Asked only
 *  aimed at the item at a tolerance of zero on a point or a line; empty means every facing can. */
std::vector<uint8_t> item_facings(const struct Question& q);

/** -1 when the first place is nearer (by distance, then gap), 1 when the second is, 0 on a tie. */
int closeness(double distance_a, double gap_a, double distance_b, double gap_b);

/** How far `off` is past `noise`, or zero within it. */
double beyond_error(double off, double noise);

/** The steps-left bound: true when no plan under the state can be recorded or beat
 *  `farthest_closest`. */
struct StepsLeft {
  double distance = 0.0;
  int steps = 0;
  double travel = 0.0;
  double swing = 0.0;
  double tolerance = 0.0;
  double consult = 0.0;
  double charge = 0.0;
  double farthest_closest = 0.0;

  bool cannot_help() const {
    const double left = static_cast<double>(steps);
    const double lower = distance - left * travel - swing;
    return lower > tolerance + consult + left * charge && lower > farthest_closest;
  }
};

/** What is asked. `frames` is a cost bound explored completely (or `Found::exhausted` is false),
 *  never a count of answers. Empty `moves` is every steppable row. */
struct Question {
  double start_x = 0, start_y = 0, start_z = 0;
  int start_facing = 0;
  /** Stick input is camera-relative, so a turnaround can only aim at `camera + k * 0x4000`.
   *  `has_camera` false means no turnaround can be aimed. */
  bool has_camera = false;
  int start_camera = 0;
  Target target;
  EndFacing end_facing;
  /** World units. Also sets the dominance quantum (`Quanta`). */
  double tolerance = 0;
  int frames = 0;
  /** Move cap, or zero for none. Prunes by depth only; it is not a cost bound. */
  int steps = 0;
  /** No plan of fewer moves is recorded, driven or kept as closest; shorter states are still
   *  walked. A shorter state still cuts a costlier one on the same state, so a plan inside the span
   *  that ends where a shorter one did is not reported. Zero records the
   *  start itself when it is near. */
  int fewest = 0;
  /** The same for a plan taking fewer frames: walked, never recorded, driven or kept as closest. */
  int least_frames = 0;
  std::vector<std::string> moves;

  Collision collision = Collision::Solid;

  /** Camera-clear field for turnaround blocks; checked whatever `collision` says. Null skips it. */
  const CamField* camera_clear = nullptr;

  /** A prune on where each move leaves Link, never applied to the start. Each side is optional. */
  struct Box {
    bool has_xmin = false, has_xmax = false, has_zmin = false, has_zmax = false;
    double xmin = 0, xmax = 0, zmin = 0, zmax = 0;

    bool holds(double x, double z) const {
      if (has_xmin && x < xmin) return false;
      if (has_xmax && x > xmax) return false;
      if (has_zmin && z < zmin) return false;
      if (has_zmax && z > zmax) return false;
      return true;
    }
    bool any() const { return has_xmin || has_xmax || has_zmin || has_zmax; }
  };
  Box bounds;

  /** Applied per state, because the item offset turns with the facing a move ends on. */
  Aim aim = Aim::Player;

  /** On, candidates reached only through the model's error allowance are discarded before the
   *  consult, which can lose a real answer. The walk itself does not read it. */
  bool greedy = false;

  /** Test-only switch to check that the distance bound never cuts a recorded state. */
  bool distance_bound = true;

  /** Test-only switch to check that dropping childless states the item cannot land from loses
   *  no plan (`item_facings`). */
  bool drop_unlandable = true;

  /** The consult radius per move, in multiples of the f32 position quantum (`consult_floor`). At
   *  zero no model state lands on the target and nothing is confirmed. */
  double check_range = 4.0;

  /** Threads in flight. Cannot change the answer: the split into work items does not read it and
   *  each item is walked against its own dominance table. 0 or 1 runs on the calling thread. */
  int cores = 1;

  /** Shortlist memory ceiling in bytes, or zero for none. Reaching it clears `Found::exhausted`. */
  long long memory = 0;
};

/** The dominance quantum, derived from the question:
 *    cell   = tolerance / sqrt(2), floored at the f32 position quantum at the start's magnitude;
 *    facing = tolerance * 65536 / (2 * pi * reach), floored at 1, `reach` the widest chosen move's.
 *  `camera`: whether the camera is part of the signature. Only the room's camera check reads the
 *  camera a move starts from; every model step reseats or keeps it, so with no check two states
 *  a camera apart have the same future.
 *  `slots`: the dominance table's size, a power of two from `Question::memory` alone (never the
 *  thread count, D13), so a costlier twin is not let through for want of room. */
struct Quanta {
  double cell = 0;
  int facing = 1;
  bool camera = true;
  size_t slots = size_t(1) << 14;
};

Quanta quanta_for(const Question& question, const BaseTable& base, const std::vector<int>& rows);

/** `left_corridor` counts paths off the grid: not expanded, but still offered as candidates. */
struct Counters {
  long long generated = 0;
  long long expanded = 0;
  long long bound_pruned = 0;
  long long steps_pruned = 0;
  /** States walked by the pre-run dives, in no other count. */
  long long dived = 0;
  long long dominance_kills = 0;
  long long key_collapses = 0;
  long long left_corridor = 0;
  long long outside_bounds = 0;
  /** Children with no children of their own at a facing the item cannot land from, dropped
   *  before they are stepped. */
  long long unlandable = 0;
  /** Candidates past the tolerance but inside the model's error: what `greedy` would discard. */
  long long allowed = 0;
  /** States kept as the closest reached rather than as near. */
  long long kept_closest = 0;
  long long camera_met = 0;
  /** Shortlisted plans spliced onto a path the dominance test deleted on a tie. */
  long long merged_orders = 0;
  long long held_bytes = 0;
  /** Set once the shortlist turns a plan away for want of memory. */
  bool memory_full = false;
};

/** The camera field level a move reads: the follow camera's for the B moves, the manual camera's
 *  for C-down at `taps`. -1 for a seat with no camera or one this does not know. */
int camera_level(const CamField& field, Seat seat, int taps);

/** One candidate, not an answer until the engine says so. `handed_off` is set when any move on the
 *  path left the model's trusted region; verification runs regardless. */
struct Candidate {
  std::vector<Edge> path;
  double x = 0, y = 0, z = 0;
  int facing = 0;
  int frames = 0;
  /** The model's xz distance to the target. */
  double distance = 0;
  /** `Target::bytes_off` less the path's model error (`beyond_error`); breaks ties on distance. */
  double gap = 0;
  /** The model's measured error over this path: `Calibration::threshold` per move, scaled by its
   *  sloped frames. Zero on level ground. */
  double allowance = 0;
  /** How near the model must come before the engine is asked; never less than `allowance`. */
  double consult = 0;
  bool handed_off = false;
};

/** The run. `exhausted` false means the bound was not explored completely. */
struct Found {
  std::vector<Candidate> candidate;
  /** The closest states reached, consulted but never answers; kept apart from `candidate`. */
  std::vector<Candidate> closest;
  Counters count;
  Quanta quanta;
  bool exhausted = true;
  /** Question move ids the table cannot step (`BaseMove::why` says why). */
  std::vector<std::string> unstepped;
  /** The admissible bound's rate, units a frame. */
  double rate = 0;
  /** The bound's per-frame error allowance (`widest_slack`). */
  double slack = 0;
};

/** The fastest any chosen move covers ground, units a frame, taken off each move's reach and
 *  widened for extra air over ground that falls; the frame bound is admissible only while it is. */
double fastest_rate(const BaseTable& base, const std::vector<int>& rows, const Grid& grid,
                    const Calibration* cal = nullptr);

/** An upper bound on how far one move can carry Link in xz on this room; both distance bounds
 *  rest on it. `steepest` is `ground_steepest`. */
double travel_of(const BaseMove& row, double drop, const MoveCal* mc, const PlaneTable* pt,
                 double steepest);
double ground_drop(const Grid& grid);
double ground_steepest(const Grid& grid);

/** The most error a chosen move can add to the recording test per frame (slope plus check range),
 *  which keeps the frame bound admissible over the widened near set. */
double widest_slack(const Question& question, const BaseTable& base, const std::vector<int>& rows,
                    const Calibration& cal, const Grid& grid, double rate);

/** Whether every ground plane prices at angle 0 (`cM_atan2s`) at every facing. */
bool level_ground(const Grid& grid);

/** At least 1; a launch can land early on rising ground, so not always the table's count. */
int fewest_frames(const BaseMove& row, const Calibration& cal, bool level);

/** The steppable rows a question names, in table order; empty `moves` is every one. */
std::vector<int> rows_for(const Question& question, const BaseTable& base);

/** `Found::unstepped`. */
std::vector<std::string> unstepped_for(const Question& question, const BaseTable& base);

/** Steps each way for the C up turn row (half of `Move::stands`), zero for every other row. A run
 *  of N steps costs `(frames - 1) + N` frames and turns `turn * N`; consecutive turns are one edge. */
int turn_steps(const BaseMove& row);

/** A replacement for `step_move`, used only by the recall audit. Null keeps the walk off
 *  `tww_engine` entirely. `ok` false skips the move. */
struct Stepper {
  virtual ~Stepper() {}
  virtual Stepped step(const BaseMove& row, double x, double y, double z, int facing) const = 0;
};

/** Progress callbacks. `through` is parts of a million of the tree and only rises; returning
 *  false stops the walk with `Found::exhausted` false. */
struct Watching {
  virtual ~Watching() {}
  virtual bool walked(const Counters& count, int through, size_t candidates) = 0;

  /** Each recorded candidate, in record order, on the walker's thread with no lock held. */
  virtual void found(const Candidate& one) { (void)one; }

  /** True: `Found::candidate` stays empty and the candidate ceiling never ends the run. */
  virtual bool takes_every_candidate() { return false; }

  /** Whether to assemble a partial run for `so_far`. */
  virtual bool wants_so_far() { return false; }

  /** A prefix of the finished run, ranked the same, with the candidate list cut short. */
  virtual void so_far(const Found& run) { (void)run; }
};

/** Where the search and the verification read the camera's ways out of the view: halfway from the
 *  start to the target's middle, a freed axis staying at the start. */
cup_tape::Spot camera_spot(const Question& q);

/** The model's room and tables, so a verification can aim a plan's remaining moves from where the
 *  engine put Link (`can_still_land`). */
struct Model {
  const BaseTable* base = nullptr;
  const Grid* grid = nullptr;
  const Selection* selection = nullptr;
  /** Per table row, its calibration and plane table, either null. */
  std::vector<const MoveCal*> cal_of;
  std::vector<const PlaneTable*> plane_of;
};

Model model_for(const BaseTable& base, const Grid& grid, const Selection& selection,
                const Calibration& cal);

/** Whether `path`'s moves from `from` on, stepped by the model from `pose` at (`x`, `y`, `z`), can
 *  still end within the tolerance plus those moves' own measured error: the near test, owing only
 *  the moves not yet driven. True wherever the model cannot say (a move it hands to the engine, a
 *  camera it cannot aim, an address target). */
bool can_still_land(const Question& question, const Model& model, const std::vector<Edge>& path,
                    size_t from, const Pose& pose, double x, double y, double z);

/** Where a stopped run is kept and where a run carries on from (`checkpoint.h`). A run is kept only
 *  when it is stopped: each walk in flight with its place, its dominance table and what it found,
 *  and the run's totals. A resume continues those walks where they stood, so a run stopped and
 *  resumed answers what one run straight through does. */
struct Pausing {
  /** A folder to keep a stopped run in, or empty. */
  std::string save;
  /** A folder a run was kept in, or empty. */
  std::string resume;
  /** What the question was, so a run is never resumed as another. Not the thread count. */
  std::string identity;
  /** Set when `resume` could not be used, with why; the run then does not start. */
  std::string refused;
  /** Set when the stopped run was written to `save`. */
  bool saved = false;
};

Found search_tree(const Question& question, const BaseTable& base, const Grid& grid,
                  const Selection& selection, const Calibration& cal,
                  const Stepper* instead = nullptr, Watching* watch = nullptr,
                  Pausing* pausing = nullptr);

/** The answer key: the sorted edges, so reorderings of the same moves collapse. */
std::vector<Edge> canonical(const std::vector<Edge>& path);

}  // namespace search
