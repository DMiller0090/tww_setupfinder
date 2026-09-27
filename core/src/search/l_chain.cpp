#include "l_chain.h"

#include <cstdint>

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

}  // namespace l_chain
