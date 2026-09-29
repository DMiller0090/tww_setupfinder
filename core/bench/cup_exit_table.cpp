/* Writes `src/search/cup_exit_table.inc` (and, with `--chains`, `l_chain_table.inc`): the camera
 * yaw each C up exit settles on per end facing, off the engine's camera over a console-shaped tape.
 */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "disc/link.h"
#include "search/cup_tape.h"
#include "boundary/game_boundary.h"
#include "d/actor/d_a_player_main.h"
#include "engine/session.h"
#include "m_Do/m_Do_controller_pad.h"

namespace {

using cup_tape::Exit;
using cup_tape::Frame;
using cup_tape::add_taps;
using cup_tape::exit_series;
using cup_tape::kCDownHeldFor;
using cup_tape::kChainTail;
using cup_tape::kLongWait;
using cup_tape::kSteps;
using cup_tape::kTapGap;
using cup_tape::kTapHeld;
using cup_tape::tape_of;

/** Camera style by mode, for the header: manual (12), view (4), follow (0). */
std::atomic<int> g_style[16];

std::vector<int> run_tape(const std::vector<Frame>& tape, int seat = -1) {
  const cup_tape::Run r = cup_tape::run(tape, seat);
  for (int m = 0; m < 16; ++m) {
    if (r.style[m] >= 0) g_style[m] = r.style[m];
  }
  return r.yaw;
}

/** The engine's first flat stretch after B runs one frame longer than the console's, so `settle`
 *  and `window` have one frame taken off. */
struct Entry {
  int cdown = 0;
  int cdown_wait = 0;
  int settled = 0;   // where B settles
  int settle = 0;    // frames from the view's end to the settle
  int first = 0;     // the first flat stretch after B
  int window = 0;    // that stretch's length
  int b_wait = 0;
  int doubts = 0;    // `Doubt` bits
};

enum Doubt {
  kBNeverSteady = 1,      // no wait short of `kLongWait` exits by B as the long one does
  kCDownNeverSteady = 2,
  kCDownHeld = 4,         // C-down held 6 or 40 frames differs from 10
  kBNotFlat = 8,          // B still moving in the last `kFlat` frames
};

const int kOut = 400;
const int kFlat = 60;

void read_b(const std::vector<int>& y, Entry* e) {
  // y[0] is the press and y[1] the view's last frame; the view has ended from y[2].
  e->settled = y.back();
  size_t from = y.size() - 1;
  while (from > 2 && y[from - 1] == e->settled) --from;
  e->first = y[2];
  size_t len = 0;
  while (2 + len < y.size() && y[2 + len] == e->first) ++len;
  if (from <= 2) {
    // No creep: a window with no end is written 0.
    e->settle = 0;
    e->window = 0;
  } else {
    e->settle = static_cast<int>(from) - 2 - 1;
    e->window = static_cast<int>(len) - 1;
  }
}

bool flat_tail(const std::vector<int>& y, size_t n) {
  for (size_t i = y.size() - n; i < y.size(); ++i) {
    if (y[i] != y.back()) return false;
  }
  return true;
}

/** The shortest wait from which `same(w)` holds for it and the two after it. */
template <typename Same>
int shortest_wait(int from, Same same) {
  for (int w = from; w + 2 < kLongWait; ++w) {
    if (same(w) && same(w + 1) && same(w + 2)) return w;
  }
  return -1;
}

Entry measure(int end, int dir, int steps = kSteps) {
  Entry e;
  const std::vector<int> ref = exit_series(end, dir, steps, kLongWait, Exit::B, 0, kOut);
  read_b(ref, &e);
  if (!flat_tail(ref, kFlat)) e.doubts |= kBNotFlat;
  // Compared from the view's end to a few frames past the settle.
  const size_t seen = std::min(ref.size(), static_cast<size_t>(e.settle + 2 + 1 + 10));
  const int from = steps > 0 ? 0 : 4;
  e.b_wait = shortest_wait(from, [&](int w) {
    const std::vector<int> y = exit_series(end, dir, steps, w, Exit::B, 0,
                                           static_cast<int>(seen));
    return std::equal(ref.begin() + 2, ref.begin() + seen, y.begin() + 2);
  });
  if (e.b_wait < 0) e.doubts |= kBNeverSteady;

  e.cdown = exit_series(end, dir, steps, kLongWait, Exit::CDown, kCDownHeldFor, 30).back();
  e.cdown_wait = shortest_wait(from, [&](int w) {
    return exit_series(end, dir, steps, w, Exit::CDown, kCDownHeldFor, 30).back() == e.cdown;
  });
  if (e.cdown_wait < 0) {
    e.doubts |= kCDownNeverSteady;
  } else if (exit_series(end, dir, steps, e.cdown_wait, Exit::CDown, 6, 34).back() != e.cdown ||
             exit_series(end, dir, steps, e.cdown_wait, Exit::CDown, 40, 30).back() != e.cdown) {
    e.doubts |= kCDownHeld;
  }
  return e;
}

int s16_of(int v) { return static_cast<int16_t>(static_cast<uint16_t>(v)); }

template <typename F>
void on_threads(int threads, int count, F body) {
  std::atomic<int> next(0);
  std::vector<std::thread> pool;
  for (int t = 0; t < threads; ++t) {
    pool.emplace_back([&]() {
      for (int i = next++; i < count; i = next++) body(i);
    });
  }
  for (std::thread& th : pool) th.join();
}

int series(char** argv) {
  const Exit exit = argv[6][0] == 'C' ? Exit::CDown : Exit::B;
  const std::vector<int> y = exit_series(std::atoi(argv[2]), std::atoi(argv[3]),
                                         std::atoi(argv[4]), std::atoi(argv[5]), exit,
                                         std::atoi(argv[7]), std::atoi(argv[8]));
  for (size_t i = 0; i < y.size(); ++i) std::printf("%d %d\n", static_cast<int>(i), y[i]);
  return 0;
}

/** Sampled end facings, two per index, spread by a Fibonacci hash. */
int sample(int i) {
  return static_cast<int>((static_cast<uint32_t>(i / 2) * 2654435761u) & 0xFFFF);
}

bool same_entry(const Entry& a, const Entry& b) {
  return a.cdown == b.cdown && a.cdown_wait == b.cdown_wait && a.settled == b.settled &&
         a.settle == b.settle && a.first == b.first && a.window == b.window &&
         a.b_wait == b.b_wait && a.doubts == b.doubts;
}

/** Exits 1 if another turn length changes a value or lengthens a wait. */
int steps_check(int samples, int threads) {
  const int lengths[] = {1, 4, 60};
  std::atomic<int> differ(0);
  on_threads(threads, samples * 2, [&](int i) {
    const int end = sample(i);
    const int dir = i % 2 ? -1 : 1;
    const Entry ref = measure(end, dir);
    for (int n : lengths) {
      const Entry e = measure(end, dir, n);
      const bool values = e.cdown == ref.cdown && e.settled == ref.settled &&
                          e.first == ref.first && e.settle == ref.settle &&
                          e.window == ref.window && e.doubts == ref.doubts;
      const bool waits = e.b_wait <= ref.b_wait && e.cdown_wait <= ref.cdown_wait;
      if (!values || !waits) {
        std::printf("end %d dir %+d: %d steps differ from %d\n", end, dir, n, kSteps);
        differ++;
      }
    }
  });
  std::printf("%d end facings, both directions, turns of 1, 4 and 60 steps against %d: %d "
              "differ\n",
              samples, kSteps, differ.load());
  return differ.load() == 0 ? 0 : 1;
}

/** Exits 1 if any wait between the table's and `kLongWait` exits differently. */
int waits_check(int samples, int threads) {
  std::atomic<int> differ(0);
  on_threads(threads, samples * 2, [&](int i) {
    const int end = sample(i);
    const int dir = i % 2 ? -1 : 1;
    const Entry e = measure(end, dir);
    const std::vector<int> ref = exit_series(end, dir, kSteps, kLongWait, Exit::B, 0, kOut);
    for (int w = e.b_wait; w <= kLongWait && e.b_wait >= 0; ++w) {
      const std::vector<int> y = exit_series(end, dir, kSteps, w, Exit::B, 0, kOut);
      if (!std::equal(ref.begin() + 2, ref.end(), y.begin() + 2)) {
        std::printf("end %d dir %+d: B after %d differs, the table says %d\n", end, dir, w,
                    e.b_wait);
        differ++;
        break;
      }
    }
    for (int w = e.cdown_wait; w <= kLongWait && e.cdown_wait >= 0; ++w) {
      if (exit_series(end, dir, kSteps, w, Exit::CDown, kCDownHeldFor, 30).back() != e.cdown) {
        std::printf("end %d dir %+d: C-down after %d differs, the table says %d\n", end, dir, w,
                    e.cdown_wait);
        differ++;
        break;
      }
    }
  });
  std::printf("%d end facings, both directions, every wait from the table's to %d: %d differ\n",
              samples, kLongWait, differ.load());
  return differ.load() == 0 ? 0 : 1;
}

/** The raising turn at `end` and the lowering turn at `-end` are one another's mirror image. */
bool mirrors(const Entry& up, const Entry& down, int end) {
  const int m = (0x10000 - end) & 0xFFFF;
  return s16_of(up.cdown - end) == -s16_of(down.cdown - m) &&
         s16_of(up.settled - end) == -s16_of(down.settled - m) &&
         s16_of(up.first - end) == -s16_of(down.first - m) && up.settle == down.settle &&
         up.window == down.window && up.b_wait == down.b_wait &&
         up.cdown_wait == down.cdown_wait && up.doubts == down.doubts;
}

int generate(const char* path, int threads) {
  const int count = 65536 * 2;
  std::vector<Entry> table(count);
  const auto t0 = std::chrono::steady_clock::now();
  on_threads(threads, count, [&](int i) { table[i] = measure(i >> 1, (i & 1) ? -1 : 1); });
  std::vector<Entry> still(65536);
  on_threads(threads, 65536, [&](int i) { still[i] = measure(i, 0, 0); });
  const double secs =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  int worst = 0, settle_max = 0, window_min = 1000, window_max = 0, b_wait_max = 0;
  int cdown_wait_max = 0;
  int unmirrored = 0;
  int by_doubt[4] = {0, 0, 0, 0};
  for (int i = 0; i < count; ++i) {
    const Entry& e = table[i];
    const int end = i >> 1;
    worst = std::max({worst, std::abs(s16_of(e.cdown - end)), std::abs(s16_of(e.settled - end)),
                      std::abs(s16_of(e.first - end))});
    settle_max = std::max(settle_max, e.settle);
    b_wait_max = std::max(b_wait_max, e.b_wait);
    cdown_wait_max = std::max(cdown_wait_max, e.cdown_wait);
    if (e.first != e.settled) window_min = std::min(window_min, e.window);
    window_max = std::max(window_max, e.window);
    for (int b = 0; b < 4; ++b) {
      if (e.doubts & (1 << b)) by_doubt[b]++;
    }
    if (i % 2 == 0 && !mirrors(e, table[((0x10000 - end) & 0xFFFF) * 2 + 1], end)) unmirrored++;
  }
  int still_doubted = 0, still_worst = 0, still_settle = 0, still_window = 0;
  int still_b_wait = 0, still_cdown_wait = 0;
  for (int end = 0; end < 65536; ++end) {
    const Entry& e = still[end];
    if (e.doubts) still_doubted++;
    still_worst = std::max({still_worst, std::abs(s16_of(e.cdown - end)),
                            std::abs(s16_of(e.settled - end)), std::abs(s16_of(e.first - end))});
    still_settle = std::max(still_settle, e.settle);
    still_window = std::max(still_window, e.window);
    still_b_wait = std::max(still_b_wait, e.b_wait);
    still_cdown_wait = std::max(still_cdown_wait, e.cdown_wait);
  }
  std::printf("with no turn: furthest off %d, longest settle %d, longest wait from the press: "
              "B %d, C-down %d; doubted %d\n",
              still_worst, still_settle, still_b_wait, still_cdown_wait, still_doubted);
  worst = std::max(worst, still_worst);
  settle_max = std::max(settle_max, still_settle);
  window_max = std::max(window_max, still_window);
  b_wait_max = std::max(b_wait_max, still_b_wait);
  cdown_wait_max = std::max(cdown_wait_max, still_cdown_wait);
  std::printf("%d entries on %d threads in %.1f s; styles: manual %d, view %d, follow %d\n",
              count, threads, secs, g_style[12].load(), g_style[4].load(), g_style[0].load());
  std::printf("furthest off the facing %d, longest settle %d, shortest stretch %d\n", worst,
              settle_max, window_min);
  std::printf("longest wait before B %d, before C-down %d\n", b_wait_max, cdown_wait_max);
  std::printf("doubts: B never steady %d, C-down never steady %d, C-down held %d, B moving %d\n",
              by_doubt[0], by_doubt[1], by_doubt[2], by_doubt[3]);
  std::printf("raising turns that are not the lowering turn's mirror: %d\n", unmirrored);
  if (worst > 32767 || settle_max > 255 || b_wait_max > 255 || cdown_wait_max > 255 ||
      window_max > 255) {
    std::printf("a value does not fit the table's fields; not written\n");
    return 1;
  }
  if (unmirrored != 0) {
    std::printf("the table is written as one direction and read as both; not written\n");
    return 1;
  }

  FILE* f = std::fopen(path, "wb");
  if (!f) {
    std::printf("cannot write %s\n", path);
    return 1;
  }
  std::fprintf(f,
               "/* GENERATED by `setupcore_cup_exit`. Do not edit by hand - re-run the "
               "instrument.\n"
               " *\n"
               " * One row per facing a raising C up turn ends on, 0 to 65535. A lowering turn\n"
               " * ending on F is the raising turn ending on -F, mirrored: its values negated,\n"
               " * its frame counts the same. The generator refuses to write the table if any\n"
               " * entry is not.\n"
               " *\n"
               " * Each row: the C-down value off the end facing and the wait before C-down;\n"
               " * where B settles off the end facing and the settle in console frames; the first\n"
               " * flat stretch off the end facing and its length in console frames; the wait\n"
               " * before B; the doubts `bench/cup_exit_table.cpp` names.\n"
               " *\n"
               " * Then one `still_row` per facing for the view entered and left with no turn,\n"
               " * the same fields, its waits counted from the C up press.\n"
               " *\n"
               " * Conditions: tww_engine's Field camera (type 7) in styles %d manual, %d in the\n"
               " * view and %d following; open flat ground; the stick neutral through the exit;\n"
               " * a %d-step turn; C-down held %d frames.\n"
               " */\n",
               g_style[12].load(), g_style[4].load(), g_style[0].load(), kSteps, kCDownHeldFor);
  for (int end = 0; end < 65536; ++end) {
    const Entry& e = table[end * 2];
    std::fprintf(f, "cup_row(%d,%d,%d,%d,%d,%d,%d,%d)\n", s16_of(e.cdown - end), e.cdown_wait,
                 s16_of(e.settled - end), e.settle, s16_of(e.first - end), e.window, e.b_wait,
                 e.doubts);
  }
  for (int end = 0; end < 65536; ++end) {
    const Entry& e = still[end];
    std::fprintf(f, "still_row(%d,%d,%d,%d,%d,%d,%d,%d)\n", s16_of(e.cdown - end), e.cdown_wait,
                 s16_of(e.settled - end), e.settle, s16_of(e.first - end), e.window, e.b_wait,
                 e.doubts);
  }
  std::fclose(f);
  return 0;
}

/* L chains: with C-down held, each L tap moves the manual camera one rung. N taps cost
   `base + kTapEvery * N`, `base` the worst over N. */
const int kTaps = 8;
const int kTapEvery = kTapGap + kTapHeld;
const int kHold = 20;
const int kHoldWithC = 2;

struct Chain {
  int value[kTaps + 1] = {0};  // after n taps; [0] is the start
  int base = 0;
  int doubts = 0;
};

enum ChainDoubt {
  kStartMatters = 1,  // seats a quarter circle apart differ
  kWaitMatters = 2,   // a longer wait before the C-down exit differs
  kTapMatters = 4,    // a longer tap or gap differs
};

/** The final yaw of `y` and the first index from which it holds. */
void settled_at(const std::vector<int>& y, int* value, int* from) {
  *value = y.back();
  int i = static_cast<int>(y.size()) - 1;
  while (i > 0 && y[static_cast<size_t>(i) - 1] == *value) --i;
  *from = i;
}

Chain held_chain(int facing, int seat, int gap = kTapGap, int held = kTapHeld) {
  Chain c;
  std::vector<Frame> t;
  for (int i = 0; i < kHold - kHoldWithC; ++i) t.push_back({facing, 0.0f, false, 0.0f, true});
  for (int i = 0; i < kHoldWithC; ++i) t.push_back({facing, 0.0f, false, -1.0f, true});
  const size_t after = t.size();
  for (int n = 0; n <= kTaps; ++n) {
    add_taps(&t, after, facing, n, gap, held);
    int v = 0, from = 0;
    settled_at(run_tape(t, seat), &v, &from);
    c.value[n] = v;
    c.base = std::max(c.base, from + 1 - kTapEvery * n);
  }
  return c;
}

/** `dir` 0 is no turn; `wait` as in `tape_of`. */
Chain cdown_chain(int end, int dir, int wait, int gap = kTapGap, int held = kTapHeld) {
  Chain c;
  const int steps = dir == 0 ? 0 : kSteps;
  int at = 0;
  const std::vector<Frame> exit = tape_of(end, dir == 0 ? 1 : dir, steps, wait, Exit::CDown,
                                          6, 0, &at);
  const int facing = exit.back().facing;
  for (int n = 1; n <= kTaps; ++n) {
    std::vector<Frame> t = exit;
    add_taps(&t, t.size(), facing, n, gap, held);
    int v = 0, from = 0;
    settled_at(run_tape(t), &v, &from);
    c.value[n] = v;
    // Counted from the C-down press, the exit's first frame.
    c.base = std::max(c.base, from - at + 1 - kTapEvery * n);
  }
  return c;
}

bool same_values(const Chain& a, const Chain& b, int from) {
  for (int n = from; n <= kTaps; ++n) {
    if (a.value[n] != b.value[n]) return false;
  }
  return true;
}

int chains(const char* path, int threads) {
  const auto t0 = std::chrono::steady_clock::now();
  std::vector<Chain> held(65536), raising(65536), still(65536);
  on_threads(threads, 65536, [&](int f) {
    Chain c = held_chain(f, f);
    const Chain across = held_chain(f, (f + 0x8000) & 0xFFFF);
    const Chain side = held_chain(f, (f + 0x4000) & 0xFFFF);
    if (!same_values(c, across, 0) || !same_values(c, side, 0)) c.doubts |= kStartMatters;
    c.base = std::max({c.base, across.base, side.base});
    held[f] = c;
  });
  on_threads(threads, 65536 * 2, [&](int i) {
    const int end = i >> 1;
    const int dir = (i & 1) ? 0 : 1;
    const Entry e = measure(end, dir, dir == 0 ? 0 : kSteps);
    Chain c;
    if (e.doubts != 0 || e.cdown_wait < 0) {
      c.doubts |= kWaitMatters;
    } else {
      c = cdown_chain(end, dir, e.cdown_wait);
      if (!same_values(c, cdown_chain(end, dir, e.cdown_wait + 5), 1)) c.doubts |= kWaitMatters;
    }
    (dir == 0 ? still : raising)[end] = c;
  });
  const double secs =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  int doubted[3] = {0, 0, 0}, base_max[3] = {0, 0, 0}, worst = 0;
  std::vector<Chain>* all[3] = {&held, &raising, &still};
  for (int k = 0; k < 3; ++k) {
    for (int f = 0; f < 65536; ++f) {
      const Chain& c = (*all[k])[f];
      if (c.doubts) doubted[k]++;
      base_max[k] = std::max(base_max[k], c.base);
      for (int n = k == 0 ? 0 : 1; n <= kTaps; ++n) {
        worst = std::max(worst, std::abs(s16_of(c.value[n] - f)));
      }
    }
  }
  std::printf("chains on %d threads in %.1f s\n", threads, secs);
  std::printf("held L: doubted %d, longest base %d\n", doubted[0], base_max[0]);
  std::printf("C-down after a turn: doubted %d, longest base %d\n", doubted[1], base_max[1]);
  std::printf("C-down with no turn: doubted %d, longest base %d\n", doubted[2], base_max[2]);
  std::printf("furthest off the facing %d\n", worst);
  if (worst > 32767 || base_max[0] > 255 || base_max[1] > 255 || base_max[2] > 255) {
    std::printf("a value does not fit the table's fields; not written\n");
    return 1;
  }

  FILE* f = std::fopen(path, "wb");
  if (!f) {
    std::printf("cannot write %s\n", path);
    return 1;
  }
  std::fprintf(f,
               "/* GENERATED by `setupcore_cup_exit --chains`. Do not edit by hand - re-run the\n"
               " * instrument.\n"
               " *\n"
               " * The L chains. Every value is off the row's facing.\n"
               " *\n"
               " * `held_row`, one per facing: the value after 0 to %d taps from held L, the base\n"
               " * frames from L's first frame, the doubts.\n"
               " * `tap_row`, one per facing a raising C up turn ends on, and `still_tap_row`, one\n"
               " * per facing for the view with no turn: the value after 1 to %d taps from the\n"
               " * C-down exit, the base frames from the C-down press, the doubts. A lowering\n"
               " * turn is the raising turn ending on -F, mirrored, as in `cup_exit_table.inc`.\n"
               " *\n"
               " * Conditions: L held %d frames, C-down pushed on its last %d; each tap %d frame(s)\n"
               " * of C-down alone and %d of C-down with L; a frame costs base + %d per tap.\n"
               " */\n",
               kTaps, kTaps, kHold, kHoldWithC, kTapGap, kTapHeld, kTapEvery);
  for (int k = 0; k < 3; ++k) {
    const char* name = k == 0 ? "held_row" : k == 1 ? "tap_row" : "still_tap_row";
    for (int facing = 0; facing < 65536; ++facing) {
      const Chain& c = (*all[k])[facing];
      std::fprintf(f, "%s(", name);
      for (int n = k == 0 ? 0 : 1; n <= kTaps; ++n) {
        std::fprintf(f, "%d,", s16_of(c.value[n] - facing));
      }
      std::fprintf(f, "%d,%d)\n", c.base, c.doubts);
    }
  }
  std::fclose(f);
  return 0;
}

/** Exits 1 if another tap length or gap changes a chain, or a lowering chain is not the mirror. */
int chains_check(int samples, int threads) {
  std::atomic<int> differ(0), mirrored(0);
  on_threads(threads, samples, [&](int i) {
    const int f = sample(i * 2);
    const Chain h = held_chain(f, f);
    const Entry e = measure(f, 1);
    const Chain c = e.doubts == 0 ? cdown_chain(f, 1, e.cdown_wait) : Chain();
    for (int held = 2; held <= 6; held += 2) {
      for (int gap = 1; gap <= 8; gap += 3) {
        if (!same_values(h, held_chain(f, f, gap, held), 0) ||
            (e.doubts == 0 && !same_values(c, cdown_chain(f, 1, e.cdown_wait, gap, held), 1))) {
          std::printf("facing %d: a tap of %d with a gap of %d differs\n", f, held, gap);
          differ++;
          return;
        }
      }
    }
    const int m = (0x10000 - f) & 0xFFFF;
    const Entry em = measure(m, -1);
    if (e.doubts == 0 && em.doubts == 0) {
      const Chain low = cdown_chain(m, -1, em.cdown_wait);
      for (int n = 1; n <= kTaps; ++n) {
        if (s16_of(low.value[n] - m) != -s16_of(c.value[n] - f)) {
          std::printf("facing %d: the lowering chain is not the mirror at %d taps\n", f, n);
          mirrored++;
          return;
        }
      }
    }
  });
  std::printf("%d facings: %d differ by the tap or the gap, %d lowering chains not mirrored\n",
              samples, differ.load(), mirrored.load());
  return differ.load() == 0 && mirrored.load() == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  {
    std::string why;
    if (!disc::install_link_from_settings(&why)) std::fprintf(stderr, "link: %s\n", why.c_str());
  }
  /* Capped so the machine stays responsive. */
  constexpr int kMostThreads = 6;
  const int cores =
      std::min(kMostThreads, static_cast<int>(std::max(1u, std::thread::hardware_concurrency())));
  auto threads_at = [&](int i) {
    return argc > i ? std::max(1, std::min(kMostThreads, std::atoi(argv[i]))) : cores;
  };
  if (argc >= 9 && std::strcmp(argv[1], "--series") == 0) return series(argv);
  if (argc >= 3 && std::strcmp(argv[1], "--steps") == 0) {
    return steps_check(std::atoi(argv[2]), threads_at(3));
  }
  if (argc >= 3 && std::strcmp(argv[1], "--waits") == 0) {
    return waits_check(std::atoi(argv[2]), threads_at(3));
  }
  if (argc >= 3 && std::strcmp(argv[1], "--chains") == 0) {
    return chains(argv[2], threads_at(3));
  }
  if (argc >= 4 && std::strcmp(argv[1], "--chain") == 0) {
    const int f = std::atoi(argv[2]) & 0xFFFF, dir = std::atoi(argv[3]);
    Chain c;
    if (dir == 2) {
      c = held_chain(f, f);
    } else {
      const Entry e = measure(f, dir, dir == 0 ? 0 : kSteps);
      c = cdown_chain(f, dir, e.cdown_wait);
      c.value[0] = e.cdown;
    }
    for (int n = 0; n <= kTaps; ++n) std::printf("%d ", c.value[n]);
    std::printf("base %d\n", c.base);
    return 0;
  }
  if (argc >= 3 && std::strcmp(argv[1], "--chains-check") == 0) {
    return chains_check(std::atoi(argv[2]), threads_at(3));
  }
  if (argc >= 2 && argv[1][0] != '-') {
    return generate(argv[1], threads_at(2));
  }
  std::printf("setupcore_cup_exit <out.inc> [threads]\n"
              "setupcore_cup_exit --series <end> <dir> <steps> <wait> <B|C> <held> <out>\n"
              "setupcore_cup_exit --steps <samples> [threads]\n"
              "setupcore_cup_exit --waits <samples> [threads]\n"
              "setupcore_cup_exit --chains <out.inc> [threads]\n"
              "setupcore_cup_exit --chains-check <samples> [threads]\n");
  return 2;
}
