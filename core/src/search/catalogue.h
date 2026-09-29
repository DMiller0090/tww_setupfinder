/* The move catalogue: what a move is to the search, and the buttons that perform one. Ids match
 * `app/src/core/moves.ts`. A move starts and ends at a standstill and needs no frame-perfect
 * timing. Bearings are world angles, not camera-relative bytes.
 *
 * A move's cost = kInputDelay + the engine frames from the first action frame through the first
 * frame back at WAIT / FREE_WAIT with no speed left. Sheathing and drawing cost nothing. */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "engine/room.h"
#include "engine/session.h"

namespace search {

/** The console's two-frame controller latency, which the engine does not model. */
const int kInputDelay = 2;

/** What a move requires of the equip state. */
enum class Sword { Away, Out, Any };

/** The four camera-relative stick presses, as offsets from the camera. */
const int kTurnCardinals[4] = {0x0000, 0x4000, 0x8000, 0xC000};

/** `checkNextMode` (4487): `cLib_distanceAngleS(m34E8, current.angle.y) > 0x7800`. */
const int kTurnReversalGate = 0x7800;

/** `cLib_distanceAngleS`. */
inline int dist_angle_s(int a, int b) {
  const int d = static_cast<int16_t>(static_cast<uint16_t>(a - b));
  return d < 0 ? -d : d;
}

/** The world bearing of the cardinal that reverses a standstill at `facing`. False means the state
 *  is wrong: a camera the game reached is always within 0x0800 of the facing. */
inline bool turn_bearing(int facing, int camera, int* world) {
  for (int i = 0; i < 4; ++i) {
    const int w = (camera + kTurnCardinals[i]) & 0xFFFF;
    /* At a standstill the travel angle is the facing. */
    if (dist_angle_s(w, facing) > kTurnReversalGate) {
      if (world) *world = w;
      return true;
    }
  }
  return false;
}

/** `has_camera` is a flag because a yaw of zero is a real camera. */
struct Pose {
  int facing = 0;
  int camera = 0;
  bool has_camera = false;
  /** The C up turn just left: +1 raised the facing, -1 lowered it, 0 otherwise. */
  int cup_dir = 0;
  /** State-dependent frames on top of the row's own; written by `model_step`. */
  int frames = 0;
  /** L taps this step presses; an input only, cleared on output. */
  int taps = 0;
};

/** The model's one rule for the camera, shared by the search and the verification. The C up turn
 *  seats the camera on its end facing; a way out of the view seats it where the tap reads it
 *  (`cup_exit`, `l_chain`); every other move keeps it, and an ESS turn turns onto it. False when the move cannot be aimed from
 *  this state. `steps` is the C up turn's run length, else 1. */
bool model_step(const std::string& id, int turn_per_step, int steps, const Pose& from, Pose* to);

/** A row's camera behaviour, resolved once per row so the walk avoids string compares. */
enum class Seat {
  Kept,
  /** The C up turn. */
  OnTheFacing,
  /** Ways out of the C up view, each a turnaround tapped after it; elsewhere, its own C up press. */
  LeavesByCDown,
  LeavesByBSettle,
  LeavesByBEarly,
  /** L held until the camera is back on the facing, C-down before L is let go, then `Pose::taps`
   *  taps with C-down held, then the turnaround. */
  LHeldCDown,
  /** The ESS turns: C up, the C-down exit, then C-down and the stick at ESS strength held; the
   *  facing turns in place onto the exit's camera plus the stick's offset, which does not move. */
  EssUp,
  EssLeft,
  EssRight,
};

inline bool is_ess(Seat seat) {
  return seat == Seat::EssUp || seat == Seat::EssLeft || seat == Seat::EssRight;
}

/** The stick's bearing off the camera. */
inline int ess_offset(Seat seat) {
  return seat == Seat::EssLeft ? 0x4000 : seat == Seat::EssRight ? 0xC000 : 0x0000;
}

/** Stick byte 158, the middle of the band from 146 (the least read) to 170 (the most that gives no
 *  speed from a standstill: `setSpeedAndAngleNormal` walks only past 0.5). */
const float kEssDistance = 0.27777779f;

/** Frames the ESS stick must be held to close `gap` (target less facing), stepped as
 *  `setSpeedAndAngleNormal` does; at least one. */
int ess_hold(int gap);

/** The price `moves.ts` shows: the median, reseat and lag included, over every facing. */
int ess_median(Seat seat);

/** The reseat an ESS turn starts with: the C-down exit's csangle at `facing` (`cup_dir` as
 *  `Pose`) and its fewest frames to the stick. False where the exit table holds no promise. */
bool ess_reseat(int facing, int cup_dir, int* csangle, int* frames);

/** The ESS turn's target, or false where it would not turn (no camera, already there) or would
 *  reverse instead (`checkNextMode`'s `> 0x7800` gate sends that to procWaitTurn). */
bool ess_target(Seat seat, int facing, bool has_camera, int camera, int* target);

inline bool leaves_the_view(Seat seat) {
  return seat == Seat::LeavesByCDown || seat == Seat::LeavesByBSettle ||
         seat == Seat::LeavesByBEarly || seat == Seat::LHeldCDown;
}

inline bool takes_taps(Seat seat) {
  return seat == Seat::LeavesByCDown || seat == Seat::LHeldCDown;
}

/** Whether the turnaround reads a camera the room can block (`cam_clear.h`). */
inline bool runs_camera(Seat seat) { return leaves_the_view(seat); }

/** `model_step` for `leaves_the_view` seats. `frames` is the wait, the exit and the taps. */
bool leave_the_view(Seat seat, const Pose& from, Pose* to);

Seat seat_of(const std::string& id);

/** Inline because the walk calls it per generated state. The string overload calls this. */
inline bool model_step(Seat seat, int turn_per_step, int steps, const Pose& from, Pose* to) {
  Pose out = from;
  out.facing = (from.facing & 0xFFFF);
  out.camera = (from.camera & 0xFFFF);
  out.cup_dir = 0;
  out.frames = 0;
  out.taps = 0;

  if (seat == Seat::OnTheFacing) {
    out.facing = ((from.facing + turn_per_step * steps) & 0xFFFF);
    out.camera = out.facing;
    out.has_camera = true;
    const int net = turn_per_step * steps;
    out.cup_dir = net > 0 ? 1 : net < 0 ? -1 : 0;
  } else if (leaves_the_view(seat)) {
    if (!leave_the_view(seat, from, &out)) return false;
  } else if (is_ess(seat)) {
    int seated = 0, reseat = 0, target = 0;
    if (!ess_reseat(out.facing, from.cup_dir, &seated, &reseat) ||
        !ess_target(seat, out.facing, true, seated, &target)) {
      return false;
    }
    /* The row is priced at a one-frame hold. */
    out.frames =
        reseat + ess_hold(static_cast<int16_t>(static_cast<uint16_t>(target - out.facing))) - 1;
    out.camera = seated;
    out.has_camera = true;
    out.facing = target;
  } else {
    out.facing = ((from.facing + turn_per_step * steps) & 0xFFFF);
  }

  if (to) *to = out;
  return true;
}

/** One frame of input for `tww_engine::Pad`. `buttons` is `daPy_lk_c::BTN_*`; `drive()` derives
 *  the rising edge. `lock` is `dAttention_c`'s, set together with `BTN_L`. */
struct Press {
  uint8_t buttons = 0;
  bool lock = false;
  /** World angle; kept on a centred stick so `m3578` reads no turn. */
  int32_t bearing = 0;
  /** `mStickDistance`. */
  float distance = 0.0f;
};

/** `stands` is how many moves the row stands for (the C up turn is 200); `presses` is a combo's
 *  length. */
struct Move {
  std::string id;
  std::string type;
  Sword sword = Sword::Any;
  int stands = 1;
  int presses = 0;

  /** Written onto the facing rather than pressed; only the C up turn, which is not driven. */
  int rotates = 0;
  /** A hand-set price; non-zero only where `rotates` is. */
  int stated_frames = 0;

  /** Empty exactly when `stated_frames` is set. */
  std::vector<Press> action;

  /** Non-empty when driving this move hits a `JUT_ASSERT` that aborts the process; says what has
   *  to change. */
  std::string blocked;

  /** No cardinal reverses the facing from the given camera; `action` is empty and `drive()`
   *  refuses it. */
  bool unaimed = false;

  bool is_stated() const { return stated_frames > 0; }
};

/** The named moves in `moves.ts` order, then every distinct sword combo when `combos`. A null
 *  `camera` aims the turnaround at `facing + 0x8000`. */
std::vector<Move> roster(int facing, bool combos = true, const int* camera = 0);

bool move_of(const std::string& id, int facing, Move* out, const int* camera = 0);

/** What a driven move did, read off the run. */
struct Drive {
  /** Not stepped: `Move::blocked`. */
  bool blocked = false;
  /** Not stepped: `Move::unaimed`. */
  bool unaimed = false;
  /** False when `run_proc` had no body for `stopped_in`. */
  bool ok = false;
  /** Came back to a standstill inside the coast. */
  bool rested = false;
  int stopped_in = 0;

  int frames = 0;
  int engine_frames = 0;
  int action_frames = 0;

  /** From `init.pos` as handed in, read off the player. */
  double dx = 0, dy = 0, dz = 0;
  /** The furthest xz distance any frame reached from the start. */
  double reach = 0;

  int facing_in = 0, facing_out = 0;
  /** `current.angle.x` and `.z`, which `daDitem_c::set_pos` turns the item's offset by. */
  int angle_x_out = 0, angle_z_out = 0;

  /** Procs walked, in order, without repeats. */
  std::vector<int> procs;

  /** STUB boundary rows reached (PARTIAL ones fire on clean frames, so are left out). */
  std::vector<std::string> stubs;

  /** The whole boundary report, every grade. */
  struct Boundary {
    std::string name;
    std::string grade;
    int hits = 0;
  };
  std::vector<Boundary> boundary;

  /** The unimplemented dispatcher arm reached (`next_init`, `next_init_calls`). Nothing else
   *  reports it: a slash here runs `ok`, `rested`, and moves nothing. */
  int dispatch_arm = 0;
  int dispatch_calls = 0;

  /** A flag because a yaw of zero is a real camera. */
  bool had_camera = false;
  int camera_in = 0, camera_out = 0;
};

struct Watcher {
  virtual ~Watcher() {}
  /** `frame` counts engine frames from 1. */
  virtual void frame(int frame, const daPy_lk_c& lk, bool action) = 0;
};

/** Steps one move from `init`, then coasts on a neutral stick until Link rests. A null `dzb` has
 *  no floor. A stated move is refused with `frames` at its stated value. A non-null `camera` runs
 *  a real `dCamera_c` every frame and reports `camera_out`; it does not change the move. */
Drive drive(const Move& move, const tww_engine::Init& init, const tww_engine::RoomDzb* dzb,
            const tww_engine::RunOptions& opts = tww_engine::RunOptions(), int coast_max = 96,
            Watcher* watch = 0, bool force_blocked = false, const int* camera = 0);

/** Byte-for-byte table comparison; an address says nothing. */
bool same_room(const tww_engine::RoomDzb& a, const tww_engine::RoomDzb& b);

/** For instruments only: steps a blocked move anyway, which may abort the process. */
const bool kForceBlocked = true;

}  // namespace search
