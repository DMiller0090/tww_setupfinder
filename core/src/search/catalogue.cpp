#include "catalogue.h"
#include "cup_exit.h"
#include "l_chain.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <set>

namespace search {
namespace {

/* Each hold is an interior point of a window measured at both ends, never a frame-perfect value. */

const uint8_t kA = daPy_lk_c::BTN_A;
const uint8_t kB = daPy_lk_c::BTN_B;
const uint8_t kL = daPy_lk_c::BTN_L;
const uint8_t kR = daPy_lk_c::BTN_R;

const int kHold = 3;

/** R down as ANM_ROLLF passes ROLL_EARLY; 15 to 18 work. */
const int kRollRHold = 16;

/** R held, neutral, until procCrouch's entry morf has counted out. */
const int kCrouchSettle = 10;
/** The push stops before ANM_LIE plays out; the hold outlasts it. */
const int kCrawlPush = 4, kCrawlHold = 14;
/** R re-pressed during the stand-up ends CRAWL_END early. */
const int kCrawlRHold = 8;

/** R as ANM_CUTA passes CUT_EARLY. */
const int kSlashRHold = 10;

/** Within the 8-12 press window and the 1-6 hold. */
const int kGap = 9, kPress = 2;

/** Frames between chained basic cuts: the buffered press is taken at the cut's early poll. */
const int kChainedCut = 11;

/** B for the charged spin: dispatch, cut, wind-up, then standstill CUT_TURN_MOVE frames. */
const int kChargeHold = 28;

/** Without the release tail the exit frame reads as a rest a delivery then walks out of. */
const int kTurnHold = 3;
const int kReleaseTail = kInputDelay;

/** HIO: `mBackJump` field_0x14/0x18, `mCut.mCutJump` field_0x18/0x1C. */
const double kBackJumpVy = 19.0, kBackJumpGrav = -3.0;
const double kJumpCutVy = 27.0, kJumpCutGrav = -3.0;

/** The middle of the rotation accumulator's acceptance band (1748 to 27379). */
const int kSpinStep = (1748 + 27379) / 2;

const int kFineTurnStep = 655;
const int kFineTurnFrames = 46;  // FINE_TURN_BASE 45 + one frame per step, at one step.

/** `GroundCheck` snaps on a strict `ground > pos.y`, so an arc returning to its launch plane spends
 *  that frame airborne too. */
int air_frames(double vy, double gravity) {
  return static_cast<int>((2.0 * vy) / (-gravity) - 1.0) + 1;
}

int wrap16(int a) { return a & 0xFFFF; }

/** `cLib_distanceAngleS`. */
int dist_angle_s(int a, int b) {
  const int d = static_cast<int16_t>(static_cast<uint16_t>(a - b));
  return d < 0 ? -d : d;
}

void push(std::vector<Press>& out, int count, uint8_t buttons, bool lock, int bearing,
          float distance) {
  Press p;
  p.buttons = buttons;
  p.lock = lock;
  p.bearing = wrap16(bearing);
  p.distance = distance;
  for (int i = 0; i < count; ++i) out.push_back(p);
}

/** L + A + full stick at `facing + delta`: 0x8000 backflip, +-0x4000 sidehops. */
std::vector<Press> ballistic(int facing, int delta) {
  std::vector<Press> out;
  push(out, kHold, kL | kA, true, facing + delta, 1.0f);
  return out;
}

/** The dry roll's press; the drawn sword makes it procJumpCut in `setDoStatusBasic`. */
std::vector<Press> jumpslash(int facing) {
  std::vector<Press> out;
  push(out, kHold, kL | kA, true, facing, 0.0f);
  return out;
}

/** `setDoStatusBasic` buckets to JUMP only for a pushed, non-forward stick. */
std::vector<Press> dry_roll(int facing) { return jumpslash(facing); }

/** Past ROLL_EARLY the roll polls `checkNextMode(1)`; R ends it 2 frames early in ATN_MOVE. */
std::vector<Press> dry_roll_r(int facing) {
  std::vector<Press> out = dry_roll(facing);
  push(out, kRollRHold, kL | kR, true, facing, 0.0f);
  return out;
}

/** L released before the exit, so `checkNextMode(l_held)` routes to plain MOVE. */
std::vector<Press> dry_roll_r_free(int facing) {
  std::vector<Press> out = dry_roll(facing);
  push(out, kRollRHold, kR, false, facing, 0.0f);
  return out;
}

/** L + B neutral -> `changeCutProc` -> CUT_A. */
std::vector<Press> target_slash(int facing) {
  std::vector<Press> out;
  push(out, kHold, kL | kB, true, facing, 0.0f);
  return out;
}

/** R must end on the exit frame; one frame more and the standstill crouches. */
int slash_r_hold() { return kPress + kSlashRHold - kHold; }

std::vector<Press> target_slash_r(int facing) {
  std::vector<Press> out = target_slash(facing);
  push(out, slash_r_hold(), kL | kR, true, facing, 0.0f);
  return out;
}

/** B neutral, no L -> CUT_L. */
std::vector<Press> neutral_slash(int facing) {
  std::vector<Press> out;
  push(out, kHold, kB, false, facing, 0.0f);
  return out;
}

std::vector<Press> neutral_slash_r(int facing) {
  std::vector<Press> out = neutral_slash(facing);
  push(out, slash_r_hold(), kR, false, facing, 0.0f);
  return out;
}

/** B held: the cut's exit finds `daPyFlg0_UNK4` set and winds the spin up. */
std::vector<Press> target_slash_qs(int facing) {
  std::vector<Press> out;
  push(out, kChargeHold, kL | kB, true, facing, 0.0f);
  return out;
}

/** The landing polls `abs(m3578) > 0xF800` once; rotating through the flight plus two frames
 *  brackets it, and stopping before the spin's exit keeps a held L from walking out. */
std::vector<Press> spin(int facing, std::vector<Press> launch, double vy, double gravity) {
  const int turns = air_frames(vy, gravity) + 2;
  int bearing = facing;
  for (int i = 0; i < turns; ++i) {
    bearing = wrap16(bearing + kSpinStep);
    push(launch, 1, kL, true, bearing, 1.0f);
  }
  return launch;
}

/** R -> CROUCH, a push -> CRAWL_START, R released -> CRAWL_END; ANM_LIE root motion moves Link. */
std::vector<Press> crawl(int facing) {
  std::vector<Press> out;
  push(out, kCrouchSettle, kR, false, facing, 0.0f);
  push(out, kCrawlPush, kR, false, facing, 1.0f);
  push(out, kCrawlHold, kR, false, facing, 0.0f);
  return out;
}

/** R re-pressed during the stand-up; the released frame makes it a rising edge. */
std::vector<Press> crawl_r(int facing) {
  std::vector<Press> out = crawl(facing);
  push(out, 1, 0, false, facing, 0.0f);
  push(out, kCrawlRHold, kR, false, facing, 0.0f);
  return out;
}

/** procWaitTurn onto `world`, the camera cardinal from `turn_bearing`, not `facing + 0x8000`. */
std::vector<Press> turnaround(int world) {
  std::vector<Press> out;
  push(out, kTurnHold, 0, false, world, 1.0f);
  push(out, kReleaseTail, 0, false, world, 0.0f);
  return out;
}

/* Sword combos: `d_a_player_sword.inc:404`. */

enum CutProc { CUT_A, CUT_F, CUT_L, CUT_R, CUT_EA, CUT_EB };
enum Dir { DIR_FORWARD, DIR_BACKWARD, DIR_LEFT, DIR_RIGHT, DIR_NONE };

const int kComboMax = 4;

/** Notation -> (stick octant, L held); `t` is L held. The order is fixed. */
struct Symbol {
  const char* letter;
  char stick;
  bool lock;
};
const Symbol kAlphabet[] = {{"N", 'N', false},  {"Nt", 'N', true},  {"U", 'U', false},
                            {"L", 'L', false},  {"R", 'R', false},  {"D", 'D', false},
                            {"Ut", 'U', true},  {"Lt", 'L', true},  {"Rt", 'R', true},
                            {"Dt", 'D', true}};
const int kAlphabetSize = 10;

/** `+ QS + R` is frame-identical to `+ QS`, so it is left out. */
const char* kTails[] = {"", "R", "QS"};
const int kTailCount = 3;

int dir_of(char stick) {
  switch (stick) {
    case 'U': return DIR_FORWARD;
    case 'L': return DIR_LEFT;
    case 'R': return DIR_RIGHT;
    case 'D': return DIR_BACKWARD;
    default: return DIR_NONE;
  }
}

int aim_centre(char stick) {
  switch (stick) {
    case 'U': return 0x0000;
    case 'L': return 0x4000;
    case 'R': return wrap16(-0x4000);
    case 'D': return 0x8000;
    default: return 0x0000;
  }
}

/** `aim_is_stick`: the latched `m34D4` is the stick's bearing, not the facing; only this turns Link. */
struct Slot {
  int proc = CUT_A;
  bool has_aim = false;   // an ender never chases one
  bool aim_is_stick = false;
  bool pushed = false;
  char stick = 'N';
  bool lock = false;

  bool turns() const { return has_aim && aim_is_stick; }
};

/** `changeCutProc` (`d_a_player_sword.inc:404`) for 1-based `count`, in the decomp's branch order:
 *  the ender is tested before `bVar1 < 4` and ignores the lock. */
Slot change_cut_proc(int count, const Symbol& sym) {
  Slot s;
  s.stick = sym.stick;
  s.lock = sym.lock;
  const int d = dir_of(sym.stick);
  s.pushed = d != DIR_NONE;
  if (count == kComboMax) {
    s.proc = (d == DIR_RIGHT || d == DIR_FORWARD) ? CUT_EA : CUT_EB;
    s.has_aim = false;
    return s;
  }
  s.has_aim = true;
  s.aim_is_stick = !(sym.lock || !s.pushed);
  if (d == DIR_FORWARD) {
    s.proc = CUT_F;
  } else if (d == DIR_RIGHT) {
    s.proc = CUT_R;
  } else if (d == DIR_NONE) {
    s.proc = sym.lock ? CUT_A : CUT_L;
  } else {
    s.proc = CUT_L;  // LEFT and BACKWARD alike
  }
  return s;
}

struct Combo {
  std::vector<int> letters;  // indices into kAlphabet
  std::string tail;
  std::vector<Slot> slots;

  void resolve() {
    slots.clear();
    for (size_t i = 0; i < letters.size(); ++i) {
      slots.push_back(change_cut_proc(static_cast<int>(i) + 1, kAlphabet[letters[i]]));
    }
  }
  std::string name() const {
    std::string out;
    for (size_t i = 0; i < letters.size(); ++i) out += kAlphabet[letters[i]].letter;
    if (!tail.empty()) out += "+" + tail;
    return out;
  }
  bool ends_on_ender() const {
    return slots.back().proc == CUT_EA || slots.back().proc == CUT_EB;
  }
  /** Only R decides this, never the stick. */
  bool exits_early() const { return tail == "R" && !ends_on_ender(); }
  std::string key() const {
    std::string k;
    for (size_t i = 0; i < slots.size(); ++i) {
      /* The stick letter is part of the key: different latched bearings are different moves. */
      char buf[24];
      std::snprintf(buf, sizeof(buf), "%d/%s;", slots[i].proc,
                    slots[i].has_aim ? (slots[i].aim_is_stick
                                            ? (std::string("stick:") + slots[i].stick).c_str()
                                            : "facing")
                                     : "-");
      k += buf;
    }
    k += exits_early() ? "|early" : "|ride";
    k += (tail == "QS") ? "|qs" : "|-";
    return k;
  }
  /** Any head but a neutral one needs stick and B on one poll, which is frame-perfect. */
  bool block_safe() const {
    const char* h = kAlphabet[letters[0]].letter;
    return h[0] == 'N';
  }
  bool any_turn() const {
    for (size_t i = 0; i < slots.size(); ++i) {
      if (slots[i].turns()) return true;
    }
    return false;
  }
  std::string group() const {
    bool all = true, none = true;
    for (size_t i = 0; i < letters.size(); ++i) {
      if (kAlphabet[letters[i]].lock) {
        none = false;
      } else {
        all = false;
      }
    }
    if (all) return "combo";
    if (none) return "combo_none";
    return "combo_rest";
  }
  /** The lower rank's spelling is kept, so an all-locked chain is not filed as mixed. */
  int rank() const {
    const std::string g = group();
    return g == "combo" ? 0 : g == "combo_none" ? 1 : 2;
  }
};

/** Slot k's direction is read at slot k-1's exit, when Link is exactly on k-1's latched bearing,
 *  so each slot is aimed off the facing the previous one left. */
std::vector<Press> combo_macro(int facing, const Combo& c) {
  std::vector<Press> rows;
  for (size_t k = 0; k < c.slots.size(); ++k) {
    const Slot& slot = c.slots[k];
    const bool last = k + 1 == c.slots.size();
    int bearing = facing;
    float distance = 0.0f;
    if (slot.pushed) {
      bearing = wrap16(facing + aim_centre(slot.stick));
      distance = 1.0f;
    }
    const uint8_t held = slot.lock ? kL : 0;
    /* A lone head with a tail runs only until its cut is live. */
    const int run = (last && !c.tail.empty() && k == 0) ? kPress : kGap;
    for (int i = 0; i < run; ++i) {
      const uint8_t b = (i < kPress || (last && c.tail == "QS")) ? kB : 0;
      push(rows, 1, static_cast<uint8_t>(b | held), slot.lock, bearing, distance);
    }
    if (slot.turns()) facing = bearing;
    if (!last) continue;
    /* Release the stick: held into the last cut's early poll it defeats the r29 bail. */
    if (c.tail == "R") {
      /* R ends on the last cut's exit frame; a later slot goes live `kChainedCut` per slot. */
      const int live = static_cast<int>(k) * kChainedCut;
      const int hold = live + kPress + kSlashRHold - static_cast<int>(rows.size());
      push(rows, hold, static_cast<uint8_t>(kR | held), slot.lock, facing, 0.0f);
    } else if (c.tail == "QS") {
      /* `kChargeHold` counts from the press, which the slot's rows already hold. */
      push(rows, kChargeHold - kPress, static_cast<uint8_t>(kB | held), slot.lock, facing, 0.0f);
    } else if (slot.pushed) {
      push(rows, 1, held, slot.lock, facing, 0.0f);
    }
  }
  return rows;
}

/** Every distinct chain of 1..4 slots with every tail, from a neutral head; must match `moves.ts`. */
std::vector<Combo> combo_catalog() {
  std::vector<Combo> out;
  std::map<std::string, size_t> seen;
  std::vector<std::vector<int> > level;
  for (int i = 0; i < kAlphabetSize; ++i) {
    if (kAlphabet[i].stick == 'N') level.push_back(std::vector<int>(1, i));
  }
  for (int depth = 0; depth < kComboMax; ++depth) {
    for (size_t i = 0; i < level.size(); ++i) {
      for (int t = 0; t < kTailCount; ++t) {
        Combo c;
        c.letters = level[i];
        c.tail = kTails[t];
        c.resolve();
        const std::map<std::string, size_t>::iterator it = seen.find(c.key());
        if (it == seen.end()) {
          seen[c.key()] = out.size();
          out.push_back(c);
        } else if (c.rank() < out[it->second].rank()) {
          out[it->second] = c;
        }
      }
    }
    std::vector<std::vector<int> > next;
    for (size_t i = 0; i < level.size(); ++i) {
      for (int n = 0; n < kAlphabetSize; ++n) {
        std::vector<int> grown = level[i];
        grown.push_back(n);
        next.push_back(grown);
      }
    }
    level.swap(next);
  }
  return out;
}

Move named(const char* id, const char* type, Sword sword, std::vector<Press> action) {
  Move m;
  m.id = id;
  m.type = type;
  m.sword = sword;
  m.action.swap(action);
  return m;
}

/** The landing quickspins reach CUT_TURN, then the engine aborts; see the string. */
const char* kSpinAborts =
    "tww_engine aborts on the 4th CUT_TURN frame: JUT_ASSERT !isnan(pm_pos->x), "
    "d_bg_s_acch.cpp 0x217. Reached with a centred stick too, so it is not the input.";

}  // namespace

Seat seat_of(const std::string& id) {
  if (id == "fine_turn") return Seat::OnTheFacing;
  if (id == "cup_cdown_turnaround") return Seat::LeavesByCDown;
  if (id == "cup_b_settle_turnaround") return Seat::LeavesByBSettle;
  if (id == "cup_b_early_turnaround") return Seat::LeavesByBEarly;
  if (id == "l_cdown_turnaround") return Seat::LHeldCDown;
  return Seat::Kept;
}

bool leave_the_view(Seat seat, const Pose& from, Pose* to) {
  Pose out = from;
  out.facing = from.facing & 0xFFFF;
  out.has_camera = true;
  out.cup_dir = 0;
  out.taps = 0;
  /* Held L puts the camera behind Link wherever it was, so only the facing matters. */
  if (seat == Seat::LHeldCDown) {
    if (!l_chain::held(out.facing, from.taps, &out.camera, &out.frames)) return false;
    if (!turn_bearing(out.facing, out.camera, &out.facing)) return false;
    *to = out;
    return true;
  }
  /* After a C up turn its direction selects the row; elsewhere the move is its own C up press. */
  const cup_exit::Way way = seat == Seat::LeavesByCDown     ? cup_exit::Way::CDown
                            : seat == Seat::LeavesByBSettle ? cup_exit::Way::Settled
                                                            : cup_exit::Way::Stretch;
  cup_exit::Leave l;
  if (!cup_exit::leave(from.facing, from.cup_dir, way, &l)) return false;
  out.camera = l.csangle & 0xFFFF;
  out.frames = l.frames;
  /* C-down kept held, then taps; frames count from the C-down press, after the wait. */
  if (seat == Seat::LeavesByCDown && from.taps > 0) {
    int frames = 0;
    if (!l_chain::cdown(out.facing, from.cup_dir, from.taps, &out.camera, &frames)) return false;
    if (from.taps == 1 && out.camera == (l.csangle & 0xFFFF)) return false;
    out.frames = l.wait + frames;
  }
  if (!turn_bearing(out.facing, out.camera, &out.facing)) return false;
  *to = out;
  return true;
}

bool model_step(const std::string& id, int turn_per_step, int steps, const Pose& from, Pose* to) {
  return model_step(seat_of(id), turn_per_step, steps, from, to);
}

namespace {

const std::vector<Combo>& combos_once() {
  static const std::vector<Combo> all = combo_catalog();
  return all;
}

const std::map<std::string, size_t>& combo_index() {
  static const std::map<std::string, size_t> index = [] {
    std::map<std::string, size_t> out;
    const std::vector<Combo>& all = combos_once();
    for (size_t i = 0; i < all.size(); ++i) {
      if (all[i].block_safe()) out[all[i].name()] = i;
    }
    return out;
  }();
  return index;
}

Move combo_move(int facing, const Combo& c) {
  Move m;
  m.id = c.name();
  m.type = c.group();
  m.sword = Sword::Out;
  m.presses = static_cast<int>(c.letters.size());
  m.action = combo_macro(facing, c);
  return m;
}

}  // namespace

std::vector<Move> roster(int facing, bool combos, const int* camera) {
  std::vector<Move> out;
  out.push_back(named("jumpslash", "jumpslash", Sword::Out, jumpslash(facing)));
  out.push_back(named("jumpslash_qs", "jumpslash", Sword::Out,
                      spin(facing, jumpslash(facing), kJumpCutVy, kJumpCutGrav)));
  out.back().blocked = kSpinAborts;
  out.push_back(named("backflip", "backflip", Sword::Any, ballistic(facing, 0x8000)));
  out.push_back(named("backflip_qs", "backflip", Sword::Out,
                      spin(facing, ballistic(facing, 0x8000), kBackJumpVy, kBackJumpGrav)));
  out.back().blocked = kSpinAborts;
  /* Sidehops travel sideways and keep the facing; no turn is priced before them. */
  out.push_back(named("sidehop_l", "sidehop", Sword::Any, ballistic(facing, 0x4000)));
  out.push_back(named("sidehop_r", "sidehop", Sword::Any, ballistic(facing, -0x4000)));
  out.push_back(named("dry_roll", "roll", Sword::Away, dry_roll(facing)));
  out.push_back(named("dry_roll_r", "roll", Sword::Away, dry_roll_r(facing)));
  out.push_back(named("dry_roll_r_free", "roll", Sword::Away, dry_roll_r_free(facing)));
  out.push_back(named("target_slash_r", "slash", Sword::Out, target_slash_r(facing)));
  out.push_back(named("target_slash", "slash", Sword::Out, target_slash(facing)));
  out.push_back(named("target_slash_qs", "slash", Sword::Out, target_slash_qs(facing)));
  out.push_back(named("neutral_slash_r", "uslash", Sword::Out, neutral_slash_r(facing)));
  out.push_back(named("neutral_slash", "uslash", Sword::Out, neutral_slash(facing)));
  out.push_back(named("crawl", "crawl", Sword::Away, crawl(facing)));
  out.push_back(named("crawl_r", "crawl", Sword::Away, crawl_r(facing)));
  /* The turnaround rows press the same tap, aimed off the caller's camera; with none they are
     marked unaimed. No plain turnaround: the model cannot know where the camera drifted to. */
  std::vector<Press> tap;
  int world = 0;
  const bool aimed = camera != 0 && turn_bearing(facing, *camera, &world);
  if (aimed) tap = turnaround(world);

  Move fine;
  fine.id = "fine_turn";
  fine.type = "turns";
  fine.sword = Sword::Any;
  /* A hundred 655-unit steps each way; the two directions never meet. */
  fine.stands = 200;
  fine.rotates = kFineTurnStep;
  fine.stated_frames = kFineTurnFrames;
  out.push_back(fine);

  out.push_back(named("cup_cdown_turnaround", "turns", Sword::Any, tap));
  out.back().unaimed = !aimed;
  out.push_back(named("cup_b_settle_turnaround", "turns", Sword::Any, tap));
  out.back().unaimed = !aimed;
  out.push_back(named("cup_b_early_turnaround", "turns", Sword::Any, tap));
  out.back().unaimed = !aimed;
  out.push_back(named("l_cdown_turnaround", "turns", Sword::Any, tap));
  out.back().unaimed = !aimed;

  if (!combos) return out;
  const std::vector<Combo>& generated = combos_once();
  for (size_t i = 0; i < generated.size(); ++i) {
    const Combo& c = generated[i];
    if (!c.block_safe()) continue;
    out.push_back(combo_move(facing, c));
  }
  return out;
}

bool move_of(const std::string& id, int facing, Move* out, const int* camera) {
  const std::vector<Move> named = roster(facing, false, camera);
  for (size_t i = 0; i < named.size(); ++i) {
    if (named[i].id == id) {
      *out = named[i];
      return true;
    }
  }
  const std::map<std::string, size_t>& index = combo_index();
  const std::map<std::string, size_t>::const_iterator it = index.find(id);
  if (it == index.end()) return false;
  *out = combo_move(facing, combos_once()[it->second]);
  return true;
}

namespace {
template <typename T>
bool same_table(const std::vector<T>& a, const std::vector<T>& b) {
  return a.size() == b.size() &&
         (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0);
}
}  // namespace

bool same_room(const tww_engine::RoomDzb& a, const tww_engine::RoomDzb& b) {
  return same_table(a.v_tbl, b.v_tbl) && same_table(a.t_tbl, b.t_tbl) &&
         same_table(a.b_tbl, b.b_tbl) && same_table(a.tree_tbl, b.tree_tbl) &&
         same_table(a.g_tbl, b.g_tbl) && same_table(a.ti_tbl, b.ti_tbl);
}

Drive drive(const Move& move, const tww_engine::Init& init, const tww_engine::RoomDzb* dzb,
            const tww_engine::RunOptions& opts, int coast_max, Watcher* watch, bool force_blocked,
            const int* camera) {
  Drive d;
  d.had_camera = camera != 0;
  if (camera) d.camera_in = d.camera_out = *camera & 0xFFFF;
  d.facing_in = init.shape_angle_y & 0xFFFF;
  d.action_frames = static_cast<int>(move.action.size());
  if (move.is_stated()) {
    d.frames = move.stated_frames;
    d.facing_out = wrap16(d.facing_in + move.rotates);
    return d;
  }
  if (move.unaimed) {
    d.unaimed = true;
    d.facing_out = d.facing_in;
    return d;
  }
  if (!move.blocked.empty() && !force_blocked) {
    d.blocked = true;
    d.facing_out = d.facing_in;
    return d;
  }

  tww_engine::Init seed = init;
  /* Drawing and sheathing cost nothing, so the seed starts in the state the move needs. */
  if (move.sword == Sword::Out) {
    seed.equip_item = daPyItem_SWORD_e;
  } else if (move.sword == Sword::Away) {
    seed.equip_item = daPyItem_NONE_e;
  }
  seed.pad.stick_angle_raw = static_cast<s16>(d.facing_in);
  seed.pad.target_angle = static_cast<s16>(d.facing_in);

  /* The boundary table is process-wide; clear it or the previous move's hits are reported. */
  tww_engine::Session::boundary_reset();

  /* Copied so the caller's options never keep the camera on. */
  tww_engine::RunOptions watched = opts;
  if (camera) {
    watched.camera = true;
    watched.camera_yaw = static_cast<s16>(*camera);
  }
  /* One `tww_engine::Room` a thread, rebuilt when the collision differs byte for byte; an address
     key would reuse a freed room. */
  struct Lent {
    std::unique_ptr<tww_engine::Room> room;
    tww_engine::RoomDzb from;
  };
  thread_local Lent lent;
  const bool can_lend = dzb != nullptr && watched.ground != tww_engine::RunOptions::Ground::Supplied;
  if (can_lend) {
    if (!lent.room || !same_room(lent.from, *dzb)) {
      lent.room.reset();
      lent.room.reset(new tww_engine::Room(*dzb));
      lent.from = *dzb;
    }
  }
  /* Built in place, never as a temporary: the constructor registers `&lk` in a global. */
  const std::unique_ptr<tww_engine::Session> made(
      can_lend ? new tww_engine::Session(seed, *lent.room, watched)
               : new tww_engine::Session(seed, dzb, watched));
  tww_engine::Session& session = *made;
  /* From the seed as handed in, so the pre-frame collision pass's push is part of the net. */
  const double x0 = static_cast<double>(init.pos.x);
  const double y0 = static_cast<double>(init.pos.y);
  const double z0 = static_cast<double>(init.pos.z);

  int last_proc = session.state().mCurProc;
  d.procs.push_back(last_proc);

  uint8_t held = 0;
  int bearing = d.facing_in;
  tww_engine::Pad pad;
  pad.stick_angle_raw = static_cast<s16>(bearing);
  pad.target_angle = static_cast<s16>(bearing);

  bool stopped = false;
  struct Step {
    static void run(tww_engine::Session& s, tww_engine::Pad& pad, Drive& d, int& last_proc,
                    bool& stopped, double x0, double y0, double z0, Watcher* watch, bool action) {
      const tww_engine::StepResult r = s.step(pad);
      ++d.engine_frames;
      if (watch) watch->frame(d.engine_frames, s.state(), action);
      const daPy_lk_c& lk = s.state();
      if (lk.mCurProc != last_proc) {
        last_proc = lk.mCurProc;
        d.procs.push_back(last_proc);
      }
      const double dx = lk.current.pos.x - x0, dz = lk.current.pos.z - z0;
      const double far = std::sqrt(dx * dx + dz * dz);
      if (far > d.reach) d.reach = far;
      d.dx = dx;
      d.dy = lk.current.pos.y - y0;
      d.dz = dz;
      if (r != tww_engine::StepResult::Ok) {
        stopped = true;
        d.stopped_in = lk.mCurProc;
      }
    }
  };

  for (size_t i = 0; i < move.action.size() && !stopped; ++i) {
    const Press& p = move.action[i];
    /* `mItemTrigger` is the rising edge only. */
    pad.item_button = p.buttons;
    pad.item_trigger = static_cast<u8>(p.buttons & ~held);
    held = p.buttons;
    pad.attention_lock = p.lock ? TRUE : FALSE;
    pad.stick_distance = p.distance;
    /* A centred stick keeps the last bearing, so the accumulator reads no turn. */
    if (p.distance > 0.0f) bearing = wrap16(p.bearing);
    pad.stick_angle_raw = static_cast<s16>(bearing);
    pad.target_angle = static_cast<s16>(bearing);
    Step::run(session, pad, d, last_proc, stopped, x0, y0, z0, watch, true);
  }

  /* Checked before stepping, so the last frame counted is the first back at rest. */
  pad.item_button = 0;
  pad.item_trigger = 0;
  pad.attention_lock = FALSE;
  pad.stick_distance = 0.0f;
  for (int i = 0; i < coast_max && !stopped; ++i) {
    const daPy_lk_c& lk = session.state();
    const bool at_rest = (lk.mCurProc == daPy_lk_c::daPyProc_WAIT_e ||
                          lk.mCurProc == daPy_lk_c::daPyProc_FREE_WAIT_e) &&
                         std::fabs(lk.mNormalSpeed) < 1e-6;
    if (at_rest) {
      d.rested = true;
      break;
    }
    Step::run(session, pad, d, last_proc, stopped, x0, y0, z0, watch, false);
  }

  const std::vector<tww_engine::BoundaryHit> report = session.boundary_report();
  for (size_t i = 0; i < report.size(); ++i) {
    Drive::Boundary row;
    row.name = report[i].name;
    row.grade = report[i].grade;
    row.hits = report[i].hits;
    d.boundary.push_back(row);
    if (row.grade == "STUB") d.stubs.push_back(row.name);
  }

  d.dispatch_arm = session.state().next_init;
  d.dispatch_calls = session.state().next_init_calls;

  d.ok = !stopped;
  d.frames = d.engine_frames + kInputDelay;
  d.facing_out = session.state().shape_angle.y & 0xFFFF;
  d.angle_x_out = session.state().current.angle.x;
  d.angle_z_out = session.state().current.angle.z;
  if (camera) d.camera_out = session.cameraYaw() & 0xFFFF;
  return d;
}

}  // namespace search
