/* The console-shaped C up tapes the exit tables are built from, run through the engine's camera.
 * Shared by `bench/cup_exit_table.cpp`, which writes the tables at the origin, and the
 * verification, which runs them where Link stands. */
#pragma once

#include <vector>

namespace cup_tape {

/** One frame; `cstick_y` -1 is held down. */
struct Frame {
  int facing;
  float stick_x;
  bool in_view;
  float cstick_y;
  bool l = false;
};

/** Where Link stands. Far from the origin the eye rounds to coarser floats, and the game's does. */
struct Spot {
  float x, y, z;
};

/** Field camera on flat ground at Link's height; the facing is forced frame by frame. */
struct Run {
  std::vector<int> yaw;
  /** Camera style by mode; -1 where the mode was never entered. */
  int style[16];
};

/** One camera yaw per frame, the camera seated at `seat` (-1: the first facing). With no `at`,
 *  Link is at the origin. */
Run run(const std::vector<Frame>& tape, int seat = -1, const Spot* at = nullptr);

/** Tapes `run` has driven so far, this process, all threads. */
unsigned long long runs();

enum class Exit { B, CDown };

/** A C up turn ending on `end` (`dir` +1 raises the facing) and its exit. `wait` counts from the
 *  stick's release, or from the C up press when `steps` is 0. `*exit_at` is the exit's first frame. */
std::vector<Frame> tape_of(int end, int dir, int steps, int wait, Exit exit, int held, int out,
                           int* exit_at);

/** Yaws from the exit's first frame on. */
std::vector<int> exit_series(int end, int dir, int steps, int wait, Exit exit, int held, int out,
                             const Spot* at = nullptr);

/** `n` taps with C-down held, from index `at` of `t`, then C-down one frame more and a tail. */
void add_taps(std::vector<Frame>* t, size_t at, int facing, int n, int gap, int held);

/** The turn length and C-down hold the tables are measured with. */
const int kSteps = 17;
const int kCDownHeldFor = 10;
const int kTapGap = 1;
const int kTapHeld = 2;
const int kChainTail = 40;
/** A wait long enough that every exit has settled. */
const int kLongWait = 40;

}  // namespace cup_tape
