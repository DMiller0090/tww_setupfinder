/* The csangle a turnaround reads after leaving a C up turn, from a compiled-in table. Assumes the
 * Field camera, open flat ground, and the stick neutral through the exit. */
#pragma once

#include <string>

#include "cup_tape.h"

namespace cup_exit {

enum class Way {
  /** C-down held, then the tap. */
  CDown,
  /** B, then the tap once the follow camera has settled. */
  Settled,
  /** B, then the tap inside the first flat stretch after the view ends. */
  Stretch,
};

struct Leave {
  int csangle = 0;
  /** Best case, from the turn's stick release to the frame before the tap, wait included. */
  int frames = 0;
  /** Frames the tap may land on and read `csangle`; 0 is unbounded. */
  int window = 0;
  int wait = 0;
};

/** A turn ending on `facing`; `dir` +1 raised, -1 lowered, 0 no turn (then `frames` and `wait`
 *  count from the C up press). False where the table holds no promise. */
bool leave(int facing, int dir, Way way, Leave* out);

/** `leave` with the csangle off the engine's camera, Link standing at `at`; the frames stay the
 *  table's. Also false where the table's wait and a long one exit differently there. */
bool leave_at(const cup_tape::Spot& at, int facing, int dir, Way way, Leave* out);

/** Rows run through the engine so far, this process, and the time they took. */
struct Computed {
  long long rows = 0;
  long long ns = 0;
};
Computed computed();

/** The most C-down chain taps a kept row holds (`l_chain::kTaps`). */
const int kChainTaps = 8;

/** A kept row: a way out of the view (`which` is the `Way`), or the C-down chain after `which`
 *  taps. */
enum class Kept { Exit, Taps };

/** A value run through the engine once per spot and row and kept for every thread, read without a
 *  lock; `run` fills a miss, and a thread asking for a row another is running waits for it. */
int kept(const cup_tape::Spot& at, Kept what, int facing, int dir, int which,
         int (*run)(const void*), const void* args);

/** Writes every row kept for `at` to `path` (a checkpoint); false on a write error. */
bool save_rows(const cup_tape::Spot& at, const std::string& path);

/** Reads rows `save_rows` wrote back into the table for their spot; false when unreadable. */
bool load_rows(const std::string& path);

}  // namespace cup_exit
