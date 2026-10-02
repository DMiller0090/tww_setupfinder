/* Re-runs each candidate through `tww_engine` and admits only on the engine's end position, never
 * the approximation's. A path of k moves is k runs, each seeded bit-exactly from where the last
 * came to rest (`Session` cannot be forked). */
#pragma once

#include <string>
#include <vector>

#include "search.h"
#include "engine/room.h"

namespace search {

/** `CouldNotRun` means the port cannot say, not no. */
enum class Outcome { Confirmed, Refuted, CouldNotRun };

/** Where one move of a consult left Link, as the engine put him. `frames` is the engine's cost for
 *  that move, which can differ from the table's. */
struct Reached {
  double x = 0, y = 0, z = 0;
  int facing = 0;
  int frames = 0;
  /** The camera yaw after this move; a turnaround aims at `camera + k * 0x4000`. Meaningless
   *  unless `Consult::had_camera`. */
  int camera = 0;
};

/** One consult, which is also one row of the error catalogue. */
struct Consult {
  std::vector<Edge> path;
  Outcome outcome = Outcome::CouldNotRun;
  /** Which move could not be run and why, for the Logs. */
  std::string why;

  double x = 0, y = 0, z = 0;
  /** `x, z` unless an item is aimed at, then where the item ended. `distance` is measured on it. */
  double aim_x = 0, aim_z = 0;
  double aim_y = 0;
  int facing = 0;
  /** `kInputDelay` a move plus the frames stepped. */
  int frames = 0;
  /** The engine's xz distance to the target; this admits or refutes. */
  double distance = 0;

  double predicted = 0;
  int predicted_frames = 0;
  /** `Candidate::allowance`. */
  double allowance = 0;
  /** `predicted - distance`, signed. */
  double error = 0;

  /** One entry a move the engine ran; shorter than `path` when the chain stopped early. */
  std::vector<Reached> stop;

  /** A flag because a yaw of zero is a real camera. */
  bool had_camera = false;

  /** Frames priced by hand rather than run (the C up turn). */
  int stated_frames = 0;

  /** Moves stepped; on `CouldNotRun`, where the chain stopped. */
  int drives = 0;
  long long engine_frames = 0;
};

struct VerifyCounters {
  long long consults = 0;
  long long confirmed = 0;
  long long refuted = 0;
  long long could_not_run = 0;
  /** Only ever nonzero with the greedy switch on. */
  long long discarded = 0;
  long long engine_frames = 0;
  /** Kept apart from `consults`: they are never answers. */
  long long closest_consults = 0;
};

/** Whether an engine landing reaches the question's target. Tolerance zero compares f32 bits; an
 *  address is also held to its typed bytes (`Target::holds_bytes`). */
bool reaches(const Question& question, double aim_x, double aim_y, double aim_z);

/** -1 when nothing was confirmed. */
double consults_per_confirmed(const VerifyCounters& count);

/** `answer` indexes `consult` in hand-in order, which is not a ranking. */
struct Verified {
  std::vector<Consult> consult;
  /** `Found::closest`, driven; never indexed by `answer`. */
  std::vector<Consult> closest;
  std::vector<size_t> answer;
  VerifyCounters count;
  bool greedy = false;
};

/** Called after each consult; returning false stops and keeps what was answered. */
struct Consulting {
  virtual ~Consulting() {}
  virtual bool consulted(const Verified& sofar, size_t of) = 0;
};

/** `room` must be the collision the candidates were searched over, and not null. With `model`,
 *  a candidate is driven only while its remaining moves can still land (`can_still_land`); one
 *  that cannot is not a consult, as one outside the near set is not. */
Verified verify(const Found& found, const Question& question, const BaseTable& base,
                const tww_engine::RoomDzb* room, Consulting* watch = nullptr,
                const Model* model = nullptr);

/** This thread's drive memo: a move driven from the same inputs is read back, not re-driven. */
struct DriveMemo {
  long long hits = 0;
  long long misses = 0;
};
DriveMemo drive_memo();

/** Turns this thread's memo on or off and empties it; returns the previous setting. */
bool remember_drives(bool on);

/** The recall check: the same `search_tree` with every move stepped by the engine instead of the
 *  model, diffed against `verified`. `lost` is not a recall figure when `skipped` is non-empty or
 *  `complete` is false. Drives the engine per generated state, so shallow questions only. */
struct Recall {
  bool complete = false;
  long long drives = 0;
  long long truth_generated = 0;
  std::vector<std::vector<Edge>> truth;
  std::vector<std::vector<Edge>> lost;
  /** Lost answers holding a camera move, which the truth walk runs without the camera field. */
  std::vector<std::vector<Edge>> behind_camera;
  std::vector<std::vector<Edge>> gained;
  std::vector<std::string> skipped;
};

Recall recall_audit(const Question& question, const BaseTable& base, const Grid& grid,
                    const Selection& selection, const Calibration& cal, const Verified& verified,
                    const tww_engine::RoomDzb* room);

/** Collapses consults with exactly equal position, item and facing, keeping the fewest frames (a
 *  tie keeps the first). Order is preserved. */
std::vector<Consult> one_per_landing(const std::vector<Consult>& consults);

}  // namespace search
