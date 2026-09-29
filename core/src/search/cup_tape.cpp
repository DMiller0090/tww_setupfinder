#include "cup_tape.h"

#include <algorithm>

#include "boundary/game_boundary.h"
#include "d/actor/d_a_player_main.h"
#include "engine/session.h"
#include "m_Do/m_Do_controller_pad.h"

namespace cup_tape {

Run run(const std::vector<Frame>& tape, int seat, const Spot* at) {
  Run out;
  for (int& s : out.style) s = -1;
  tww_engine::Init init;
  init.pos.set(at ? at->x : 0.0f, at ? at->y : 0.0f, at ? at->z : 0.0f);
  init.shape_angle_y = static_cast<s16>(tape[0].facing);
  init.travel_angle_y = static_cast<s16>(tape[0].facing);
  init.normal_speed = 0.0f;
  init.speed_f = 0.0f;
  init.proc = daPy_lk_c::daPyProc_WAIT_e;
  tww_engine::RunOptions opts;
  opts.ground = tww_engine::RunOptions::Ground::Supplied;
  opts.floor_y = init.pos.y;
  opts.camera = true;
  const s16 seat_yaw = static_cast<s16>(seat >= 0 ? seat : tape[0].facing);
  opts.camera_yaw = seat_yaw;
  tww_engine::Session session(init, NULL, opts);
  session.bind();
  cam_stub_lockon = false;
  if (at) {
    /* The camera's centre is read off Link's model, which only a frame of his poses. */
    tww_engine::Pad still;
    still.stick_angle_raw = static_cast<s16>(tape[0].facing);
    still.target_angle = static_cast<s16>(tape[0].facing);
    session.step(still);
    session.seatCamera(seat_yaw);
    session.bind();
  }
  out.yaw.reserve(tape.size());
  for (const Frame& f : tape) {
    session.lk.shape_angle.y = static_cast<s16>(f.facing);
    session.lk.current.angle.y = static_cast<s16>(f.facing);
    g_mDoCPd_cpadInfo[PAD_1].mMainStickPosX = f.stick_x;
    g_mDoCPd_cpadInfo[PAD_1].mCStickPosY = f.cstick_y;
    g_mDoCPd_cpadInfo[PAD_1].mCStickValue = f.cstick_y < 0.0f ? -f.cstick_y : f.cstick_y;
    /* L is both the analog trigger and `dAttention_c::Lockon()`. */
    g_mDoCPd_cpadInfo[PAD_1].mTriggerLeft = f.l ? 1.0f : 0.0f;
    cam_stub_lockon = f.l;
    if (f.in_view) {
      session.lk.stub_player_status0 |= daPyStts0_SUBJECT_e;
    } else {
      session.lk.stub_player_status0 &= ~daPyStts0_SUBJECT_e;
    }
    session.runCamera(1);
    const tww_engine::Session::CameraFacts facts = session.cameraFacts();
    out.yaw.push_back(facts.yaw & 0xFFFF);
    if (facts.mode >= 0 && facts.mode < 16) out.style[facts.mode] = facts.style;
  }
  cam_stub_lockon = false;
  g_mDoCPd_cpadInfo[PAD_1].mMainStickPosX = 0.0f;
  g_mDoCPd_cpadInfo[PAD_1].mCStickPosY = 0.0f;
  g_mDoCPd_cpadInfo[PAD_1].mCStickValue = 0.0f;
  g_mDoCPd_cpadInfo[PAD_1].mTriggerLeft = 0.0f;
  return out;
}

std::vector<Frame> tape_of(int end, int dir, int steps, int wait, Exit exit, int held, int out,
                           int* exit_at) {
  const int start = (end - dir * 655 * steps) & 0xFFFF;
  std::vector<Frame> t;
  for (int i = 0; i < 11; ++i) t.push_back({start, 0.0f, false, 0.0f});
  // The view's status comes up on the C up press's fourth frame.
  const int entry = steps > 0 ? 14 : wait;
  for (int i = 0; i < entry; ++i) t.push_back({start, 0.0f, i >= 3, 0.0f});
  const int turn_at = static_cast<int>(t.size());
  const float stick = dir > 0 ? -1.0f : 1.0f;
  for (int i = 0; i < steps; ++i) t.push_back({start, stick, true, 0.0f});
  for (int i = 0; i < (steps > 0 ? wait : 0); ++i) t.push_back({start, 0.0f, true, 0.0f});
  *exit_at = static_cast<int>(t.size());
  // The view's status stays up for B's first two frames and C-down's first four.
  if (exit == Exit::B) {
    t.push_back({start, 0.0f, true, 0.0f});
    for (int i = 0; i < out; ++i) t.push_back({start, 0.0f, i == 0, 0.0f});
  } else {
    for (int i = 0; i < held; ++i) t.push_back({start, 0.0f, i < 4, -1.0f});
    for (int i = 0; i < out; ++i) t.push_back({start, 0.0f, false, 0.0f});
  }
  // As on console: the facing lags the stick two frames and steps 655 a frame.
  for (size_t i = static_cast<size_t>(turn_at) + 2; i < t.size(); ++i) {
    const int k = std::min(static_cast<int>(i) - (turn_at + 2) + 1, steps);
    t[i].facing = (start + dir * 655 * k) & 0xFFFF;
  }
  return t;
}

std::vector<int> exit_series(int end, int dir, int steps, int wait, Exit exit, int held, int out,
                             const Spot* at) {
  int from = 0;
  const std::vector<int> y = run(tape_of(end, dir, steps, wait, exit, held, out, &from), -1, at).yaw;
  return std::vector<int>(y.begin() + from, y.end());
}

void add_taps(std::vector<Frame>* t, size_t at, int facing, int n, int gap, int held) {
  t->resize(at);
  for (int k = 0; k < n; ++k) {
    for (int i = 0; i < gap; ++i) t->push_back({facing, 0.0f, false, -1.0f, false});
    for (int i = 0; i < held; ++i) t->push_back({facing, 0.0f, false, -1.0f, true});
  }
  t->push_back({facing, 0.0f, false, -1.0f, false});
  for (int i = 0; i < kChainTail; ++i) t->push_back({facing, 0.0f, false, 0.0f, false});
}

}  // namespace cup_tape
