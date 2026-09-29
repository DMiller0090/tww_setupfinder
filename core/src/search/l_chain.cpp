#include "l_chain.h"

#include <cstdint>
#include <vector>

#include "cup_exit.h"

namespace l_chain {
namespace {

/** Values are offsets from the row's facing. */
struct Row {
  int16_t value[kTaps + 1];
  uint8_t base;
  uint8_t doubts;
};

/* The .inc holds all three tables; each pass keeps one sort of row. Tap rows' slot 0 is unused. */
#define held_row(v0, v1, v2, v3, v4, v5, v6, v7, v8, base, doubts) \
  {{v0, v1, v2, v3, v4, v5, v6, v7, v8}, base, doubts},
#define tap_row(...)
#define still_tap_row(...)
const Row kHeld[65536] = {
#include "l_chain_table.inc"
};
#undef held_row
#undef tap_row
#undef still_tap_row

#define held_row(...)
#define tap_row(v1, v2, v3, v4, v5, v6, v7, v8, base, doubts) \
  {{0, v1, v2, v3, v4, v5, v6, v7, v8}, base, doubts},
#define still_tap_row(...)
const Row kRaising[65536] = {
#include "l_chain_table.inc"
};
#undef held_row
#undef tap_row
#undef still_tap_row

#define held_row(...)
#define tap_row(...)
#define still_tap_row(v1, v2, v3, v4, v5, v6, v7, v8, base, doubts) \
  {{0, v1, v2, v3, v4, v5, v6, v7, v8}, base, doubts},
const Row kStill[65536] = {
#include "l_chain_table.inc"
};
#undef held_row
#undef tap_row
#undef still_tap_row

/** Frames per tap: one of C-down alone, two with L. */
const int kTapEvery = 3;

}  // namespace

bool held(int facing, int taps, int* csangle, int* frames) {
  facing &= 0xFFFF;
  if (taps < 0 || taps > kTaps) return false;
  const Row& r = kHeld[facing];
  if (r.doubts != 0) return false;
  if (taps > 0 && r.value[taps] == r.value[taps - 1]) return false;
  if (csangle) *csangle = (facing + r.value[taps]) & 0xFFFF;
  if (frames) *frames = r.base + kTapEvery * taps;
  return true;
}

bool cdown(int facing, int dir, int taps, int* csangle, int* frames) {
  facing &= 0xFFFF;
  if (taps < 1 || taps > kTaps) return false;
  /* A lowering turn is the raising turn's mirror image. */
  const int end = dir < 0 ? (0x10000 - facing) & 0xFFFF : facing;
  const Row& r = dir == 0 ? kStill[facing] : kRaising[end];
  if (r.doubts != 0) return false;
  if (taps > 1 && r.value[taps] == r.value[taps - 1]) return false;
  const int sign = dir < 0 ? -1 : 1;
  if (csangle) *csangle = (facing + sign * r.value[taps]) & 0xFFFF;
  if (frames) *frames = r.base + kTapEvery * taps;
  return true;
}

namespace {

/** The yaw the camera settles on after `taps` taps, from a C-down exit left after `wait`. */
int tapped(const cup_tape::Spot& at, int facing, int dir, int wait, int taps) {
  const int steps = dir == 0 ? 0 : cup_tape::kSteps;
  int from = 0;
  std::vector<cup_tape::Frame> t = cup_tape::tape_of(facing, dir == 0 ? 1 : dir, steps, wait,
                                                     cup_tape::Exit::CDown, 6, 0, &from);
  const int turned = t.back().facing;
  cup_tape::add_taps(&t, t.size(), turned, taps, cup_tape::kTapGap, cup_tape::kTapHeld);
  return cup_tape::run(t, -1, &at).yaw.back();
}

struct TapArgs {
  cup_tape::Spot at;
  int facing, dir, wait, taps;
};

/** The chain's value at the table's wait, or -1 where a long wait differs or the last tap does not
 *  move the camera (as `cdown`). */
int run_taps(const void* p) {
  const TapArgs& a = *static_cast<const TapArgs*>(p);
  const int v = tapped(a.at, a.facing, a.dir, a.wait, a.taps);
  if (tapped(a.at, a.facing, a.dir, cup_tape::kLongWait, a.taps) != v) return -1;
  if (a.taps > 1 && tapped(a.at, a.facing, a.dir, a.wait, a.taps - 1) == v) return -1;
  return v;
}

}  // namespace

bool cdown_at(const cup_tape::Spot& at, int facing, int dir, int taps, int* csangle, int* frames) {
  facing &= 0xFFFF;
  int f = 0;
  if (!cdown(facing, dir, taps, nullptr, &f)) return false;
  cup_exit::Leave l;
  if (!cup_exit::leave(facing, dir, cup_exit::Way::CDown, &l)) return false;
  const TapArgs args = {at, facing, dir, l.wait, taps};
  const int v = cup_exit::kept(cup_exit::key('t', at, facing, dir, taps), run_taps, &args);
  if (v < 0) return false;
  if (csangle) *csangle = v & 0xFFFF;
  if (frames) *frames = f;
  return true;
}

}  // namespace l_chain
