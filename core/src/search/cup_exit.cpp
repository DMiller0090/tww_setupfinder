#include "cup_exit.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "checkpoint.h"
#include "profile.h"

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

std::atomic<long long> g_rows(0), g_ns(0);

/** Every row one spot can need: the three ways out and the C-down chain's taps, by direction and
 *  facing. A slot is empty, being run, or its value plus `kStored`. */
struct Rows {
  static const int kSlots = (3 + kChainTaps) * 3 * 65536;
  static const int32_t kEmpty = 0;
  static const int32_t kRunning = -1;
  static const int32_t kStored = 2;

  cup_tape::Spot at;
  std::unique_ptr<std::atomic<int32_t>[]> slot;
  std::atomic<long long> filled{0};

  explicit Rows(const cup_tape::Spot& spot)
      : at(spot), slot(new std::atomic<int32_t>[static_cast<size_t>(kSlots)]) {
    for (int i = 0; i < kSlots; ++i) slot[static_cast<size_t>(i)].store(kEmpty);
  }
  bool is(const cup_tape::Spot& s) const {
    return std::memcmp(&at.x, &s.x, 4) == 0 && std::memcmp(&at.y, &s.y, 4) == 0 &&
           std::memcmp(&at.z, &s.z, 4) == 0;
  }
};

/** The last few spots' rows. A thread keeps the table it last used, so the common case takes no
 *  lock; a table dropped from here lives on until no thread holds it. */
std::mutex g_spots_lock;
std::vector<std::shared_ptr<Rows> > g_spots;
const size_t kSpotsKept = 4;
thread_local std::shared_ptr<Rows> t_rows;

Rows& rows_at(const cup_tape::Spot& at) {
  if (t_rows && t_rows->is(at)) return *t_rows;
  std::lock_guard<std::mutex> hold(g_spots_lock);
  for (size_t i = 0; i < g_spots.size(); ++i) {
    if (g_spots[i]->is(at)) {
      t_rows = g_spots[i];
      return *t_rows;
    }
  }
  if (g_spots.size() >= kSpotsKept) g_spots.erase(g_spots.begin());
  g_spots.push_back(std::make_shared<Rows>(at));
  t_rows = g_spots.back();
  return *t_rows;
}

}  // namespace

Computed computed() {
  Computed c;
  c.rows = g_rows.load();
  c.ns = g_ns.load();
  return c;
}

int kept(const cup_tape::Spot& at, Kept what, int facing, int dir, int which,
         int (*run)(const void*), const void* args) {
  const int band = what == Kept::Exit ? which : 3 + which - 1;
  if (band < 0 || band >= 3 + kChainTaps || dir < -1 || dir > 1) return run(args);
  Rows& rows = rows_at(at);
  std::atomic<int32_t>& slot =
      rows.slot[(static_cast<size_t>(band) * 3 + static_cast<size_t>(dir + 1)) * 65536 +
                static_cast<size_t>(facing & 0xFFFF)];
  int32_t v = slot.load(std::memory_order_acquire);
  if (v >= Rows::kStored - 1) {
    profile::Timed finding(profile::kExitFind, false);
    return v - Rows::kStored;
  }
  if (v == Rows::kEmpty && slot.compare_exchange_strong(v, Rows::kRunning)) {
    int got = 0;
    {
      profile::Timed running(profile::kExitRow);
      const auto t0 = std::chrono::steady_clock::now();
      got = run(args);
      g_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now() - t0)
                  .count();
    }
    ++g_rows;
    slot.store(got + Rows::kStored, std::memory_order_release);
    profile::exit_rows_held(static_cast<size_t>(++rows.filled));
    return got;
  }
  /* Another thread is running this row; it takes a few ms. */
  const uint64_t asked = profile::on() ? profile::cycles() : 0;
  while ((v = slot.load(std::memory_order_acquire)) == Rows::kRunning) std::this_thread::yield();
  if (asked != 0) profile::add(profile::kExitWait, profile::cycles() - asked);
  return v - Rows::kStored;
}

namespace {
const uint32_t kRowsMagic = 0x53574F52;  // "ROWS"
}  // namespace

bool save_rows(const cup_tape::Spot& at, const std::string& path) {
  Rows& rows = rows_at(at);
  checkpoint::Writer w(path);
  w.pod(kRowsMagic);
  w.pod(checkpoint::kVersion);
  w.pod(at);
  w.pod(static_cast<int32_t>(Rows::kSlots));
  std::vector<uint32_t> index;
  std::vector<int32_t> value;
  for (int i = 0; i < Rows::kSlots; ++i) {
    const int32_t v = rows.slot[static_cast<size_t>(i)].load(std::memory_order_acquire);
    if (v < Rows::kStored - 1) continue;
    index.push_back(static_cast<uint32_t>(i));
    value.push_back(v);
  }
  w.pods(index);
  w.pods(value);
  return w.close();
}

bool load_rows(const std::string& path) {
  checkpoint::Reader r(path);
  if (r.pod<uint32_t>() != kRowsMagic || r.pod<uint32_t>() != checkpoint::kVersion) return false;
  const cup_tape::Spot at = r.pod<cup_tape::Spot>();
  if (r.pod<int32_t>() != Rows::kSlots) return false;
  std::vector<uint32_t> index;
  std::vector<int32_t> value;
  r.pods(&index);
  r.pods(&value);
  if (!r.ok() || index.size() != value.size()) return false;
  Rows& rows = rows_at(at);
  for (size_t i = 0; i < index.size(); ++i) {
    if (index[i] >= static_cast<uint32_t>(Rows::kSlots) || value[i] < Rows::kStored - 1) continue;
    int32_t empty = Rows::kEmpty;
    if (rows.slot[index[i]].compare_exchange_strong(empty, value[i])) ++rows.filled;
  }
  return true;
}

bool leave_at(const cup_tape::Spot& at, int facing, int dir, Way way, Leave* out) {
  Leave l;
  if (!leave(facing, dir, way, &l)) return false;
  facing &= 0xFFFF;
  const ExitArgs args = {at, facing, dir, l.wait, way};
  const int v = kept(at, Kept::Exit, facing, dir, static_cast<int>(way), run_exit, &args);
  if (v < 0) return false;
  l.csangle = v & 0xFFFF;
  if (out) *out = l;
  return true;
}

}  // namespace cup_exit
