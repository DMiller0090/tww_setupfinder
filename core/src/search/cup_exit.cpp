#include "cup_exit.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <unordered_map>

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

struct ExitArgs {
  cup_tape::Spot at;
  int facing, dir, wait;
  Way way;
};

/** The exit at the table's wait, or -1 where a long wait exits differently. */
int run_exit(const void* p) {
  const ExitArgs& a = *static_cast<const ExitArgs*>(p);
  const int v = exit_value(a.at, a.facing, a.dir, a.way, a.wait);
  return exit_value(a.at, a.facing, a.dir, a.way, cup_tape::kLongWait) == v ? v : -1;
}

std::mutex g_kept_lock;
std::unordered_map<std::string, int> g_kept;
std::atomic<long long> g_rows(0), g_ns(0);
/** Past this many the cache is emptied; a row costs a few ms to run again. */
const size_t kKeptMost = 2000000;

}  // namespace

Computed computed() {
  Computed c;
  c.rows = g_rows.load();
  c.ns = g_ns.load();
  return c;
}

std::string key(char what, const cup_tape::Spot& at, int a, int b, int c) {
  char k[25];
  std::memcpy(k, &what, 1);
  std::memcpy(k + 1, &at.x, 4);
  std::memcpy(k + 5, &at.y, 4);
  std::memcpy(k + 9, &at.z, 4);
  std::memcpy(k + 13, &a, 4);
  std::memcpy(k + 17, &b, 4);
  std::memcpy(k + 21, &c, 4);
  return std::string(k, sizeof k);
}

int kept(const std::string& k, int (*run)(const void*), const void* args) {
  {
    std::lock_guard<std::mutex> hold(g_kept_lock);
    const auto at = g_kept.find(k);
    if (at != g_kept.end()) return at->second;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const int v = run(args);
  g_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - t0)
              .count();
  ++g_rows;
  std::lock_guard<std::mutex> hold(g_kept_lock);
  if (g_kept.size() >= kKeptMost) g_kept.clear();
  g_kept.emplace(k, v);
  return v;
}

bool leave_at(const cup_tape::Spot& at, int facing, int dir, Way way, Leave* out) {
  Leave l;
  if (!leave(facing, dir, way, &l)) return false;
  facing &= 0xFFFF;
  const ExitArgs args = {at, facing, dir, l.wait, way};
  const int v = kept(key('e', at, facing, dir, static_cast<int>(way)), run_exit, &args);
  if (v < 0) return false;
  l.csangle = v & 0xFFFF;
  if (out) *out = l;
  return true;
}

}  // namespace cup_exit
