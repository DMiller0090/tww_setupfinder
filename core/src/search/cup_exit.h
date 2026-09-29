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

/** A value run through the engine once per key and kept, for every thread; `run` fills a miss. */
int kept(const std::string& key, int (*run)(const void*), const void* args);

/** A `kept` key: what is kept, the spot's bits and three numbers. */
std::string key(char what, const cup_tape::Spot& at, int a, int b, int c);

}  // namespace cup_exit
