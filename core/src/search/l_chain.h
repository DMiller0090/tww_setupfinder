/* The csangle a turnaround reads after N L taps with C-down held, from a compiled-in table. */
#pragma once

namespace l_chain {

const int kTaps = 8;

/** Held-L chain at `facing`: the csangle after `taps` taps, and `frames` from L's first frame to
 *  the frame before the turnaround's tap. False where the table holds no promise or the tap would
 *  not move the camera. */
bool held(int facing, int taps, int* csangle, int* frames);

/** C-down chain after a turn ending on `facing`; `dir` +1 raising, -1 lowering, 0 no turn; `taps`
 *  from 1; `frames` from the C-down press. False as for `held`. */
bool cdown(int facing, int dir, int taps, int* csangle, int* frames);

}  // namespace l_chain
