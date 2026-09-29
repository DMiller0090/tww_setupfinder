#include "cup_exit.h"

#include <cstdint>

namespace cup_exit {
namespace {

/** A raising turn; offsets are from the end facing. */
struct Row {
  int16_t cdown;
  uint8_t cdown_wait;
  int16_t settled;
  uint8_t settle;
  int16_t first;
  uint8_t window;
  uint8_t b_wait;
  uint8_t doubts;
};

#define cup_row(cdown, cdown_wait, settled, settle, first, window, b_wait, doubts) \
  {cdown, cdown_wait, settled, settle, first, window, b_wait, doubts},
#define still_row(...)
const Row kRows[65536] = {
#include "cup_exit_table.inc"
};
#undef cup_row
#undef still_row

/* Second pass over the same .inc: the rows for a view left with no turn. */
#define cup_row(...)
#define still_row(cdown, cdown_wait, settled, settle, first, window, b_wait, doubts) \
  {cdown, cdown_wait, settled, settle, first, window, b_wait, doubts},
const Row kStill[65536] = {
#include "cup_exit_table.inc"
};
#undef cup_row
#undef still_row

/** The exit's own frames after the wait. */
const int kCDownHeld = 6;
const int kViewEnds = 2;
const int kFirstTap = 3;
/** Frames after B for the follow camera to settle, as the table was measured. */
const int kSettleOut = 400;

}  // namespace

bool leave(int facing, int dir, Way way, Leave* out) {
  facing &= 0xFFFF;
  /* A lowering turn mirrors the raising one ending on -F: offsets negated, frames the same. */
  const int end = dir < 0 ? (0x10000 - facing) & 0xFFFF : facing;
  const Row& r = dir == 0 ? kStill[facing] : kRows[end];
  if (r.doubts != 0) return false;
  const int sign = dir < 0 ? -1 : 1;
  Leave l;
  if (way == Way::CDown) {
    l.csangle = facing + sign * r.cdown;
    l.wait = r.cdown_wait;
    l.frames = r.cdown_wait + kCDownHeld;
  } else if (way == Way::Settled) {
    l.csangle = facing + sign * r.settled;
    l.wait = r.b_wait;
    l.frames = r.b_wait + kViewEnds + r.settle;
  } else {
    l.csangle = facing + sign * r.first;
    l.wait = r.b_wait;
    l.frames = r.b_wait + kFirstTap;
    l.window = r.first == r.settled && r.settle == 0 ? 0 : r.window;
  }
  l.csangle &= 0xFFFF;
  if (out) *out = l;
  return true;
}

namespace {

int exit_value(const cup_tape::Spot& at, int facing, int dir, Way way, int wait) {
  using cup_tape::Exit;
  const int steps = dir == 0 ? 0 : cup_tape::kSteps;
  if (way == Way::CDown) {
    return cup_tape::exit_series(facing, dir, steps, wait, Exit::CDown, cup_tape::kCDownHeldFor,
                                 30, &at)
        .back();
  }
  if (way == Way::Settled) {
    return cup_tape::exit_series(facing, dir, steps, wait, Exit::B, 0, kSettleOut, &at).back();
  }
  /* The first frame after the view ends. */
  return cup_tape::exit_series(facing, dir, steps, wait, Exit::B, 0, 3, &at)[2];
}

}  // namespace

bool leave_at(const cup_tape::Spot& at, int facing, int dir, Way way, Leave* out) {
  Leave l;
  if (!leave(facing, dir, way, &l)) return false;
  facing &= 0xFFFF;
  const int v = exit_value(at, facing, dir, way, l.wait);
  if (exit_value(at, facing, dir, way, cup_tape::kLongWait) != v) return false;
  l.csangle = v & 0xFFFF;
  if (out) *out = l;
  return true;
}

}  // namespace cup_exit
