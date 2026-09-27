/* The csangle a turnaround reads after leaving a C up turn, from a compiled-in table. Assumes the
 * Field camera, open flat ground, and the stick neutral through the exit. */
#pragma once

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

}  // namespace cup_exit
