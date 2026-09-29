#include "verify.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>

#include "catalogue.h"
#include "engine/session.h"
#include "engine/ps_mtx.h"
#include "m_Do/m_Do_mtx.h"

namespace search {
namespace {

double xz_distance(double x, double z, double tx, double tz) {
  const double dx = x - tx, dz = z - tz;
  return std::sqrt(dx * dx + dz * dz);
}

/** The held item's position, `daDitem_c::set_pos` line for line. */
void item_point(double x, double y, double z, int angle_x, int shape_y, int angle_z, double* ax,
                double* ay, double* az) {
  Mtx turn;
  mDoMtx_ZXYrotS(turn, static_cast<s16>(angle_x), static_cast<s16>(shape_y),
                 static_cast<s16>(angle_z));
  cXyz offset(30.0f, 140.0f, 20.0f);
  PSMTXMultVec(turn, &offset, &offset);
  cXyz pos(static_cast<f32>(x), static_cast<f32>(y), static_cast<f32>(z));
  pos += offset;
  *ax = static_cast<double>(pos.x);
  *ay = static_cast<double>(pos.y);
  *az = static_cast<double>(pos.z);
}

/** Named moves first; combos are built only for other ids. Facing and camera both change a move. */
bool move_at(const std::string& id, int facing, const int* camera, Move* out) {
  const std::vector<Move> named = roster(facing, false, camera);
  for (size_t i = 0; i < named.size(); ++i) {
    if (named[i].id != id) continue;
    *out = named[i];
    return true;
  }
  return move_of(id, facing, out, camera);
}

/** Where one move left Link; a non-empty `why` names the move and is `CouldNotRun`. */
struct Link {
  bool ran = false;
  std::string why;
  double x = 0, y = 0, z = 0;
  int facing = 0;
  /** `current.angle.x` and `.z`, which turn the item's offset. */
  int angle_x = 0, angle_z = 0;
  int frames = 0;
  int engine_frames = 0;
  /** Frames priced by hand; non-zero only on the C up turn. */
  int stated_frames = 0;
  int camera = 0;
  /** A move can seat a camera where the path had none. */
  bool has_camera = false;
  int cup_dir = 0;
};

Link step_through_engine(const BaseMove& row, const Edge& edge, const tww_engine::RoomDzb* room,
                         double x, double y, double z, int facing, int angle_x, int angle_z,
                         const int* camera = 0, int cup_dir = 0) {
  /* Start from the f32 the engine is seeded with, so `start + net` is exact. */
  x = static_cast<double>(static_cast<f32>(x));
  y = static_cast<double>(static_cast<f32>(y));
  z = static_cast<double>(static_cast<f32>(z));
  Link link;
  link.x = x;
  link.y = y;
  link.z = z;
  link.facing = facing & 0xFFFF;
  link.angle_x = angle_x;
  link.angle_z = angle_z;
  link.has_camera = camera != 0;
  if (camera) link.camera = *camera & 0xFFFF;

  /* The C up turn is a rotation at a hand-set price, `(frames - 1) + N` for N steps, stepped by
     `model_step` (the same rule the walk reads); it never reaches the engine. */
  if (turn_steps(row) > 0) {
    const int steps = edge.steps != 0 ? edge.steps : 1;
    const int held = steps < 0 ? -steps : steps;
    link.frames = (row.frames - 1) + held;
    link.stated_frames = link.frames;
    Pose was;
    was.facing = link.facing;
    was.camera = link.camera;
    was.has_camera = link.has_camera;
    Pose now;
    if (!model_step(row.id, row.turn, steps, was, &now)) {
      link.why = row.id + ": cannot be stepped from this state";
      return link;
    }
    link.facing = now.facing;
    link.camera = now.camera;
    link.has_camera = now.has_camera;
    link.cup_dir = now.cup_dir;
    link.ran = true;
    return link;
  }

  /* The camera is the model's: the roster is told it for aiming, and the engine never runs one. A
     way out of the view aims off `model_step`'s camera and adds its wait and exit frames. */
  const Seat seat = seat_of(row.id);
  Pose left;
  if (leaves_the_view(seat)) {
    Pose was;
    was.facing = link.facing;
    was.camera = link.camera;
    was.has_camera = link.has_camera;
    was.cup_dir = cup_dir;
    was.taps = edge.taps;
    if (!model_step(seat, row.turn, 1, was, &left)) {
      link.why = row.id + ": cannot be stepped from this state";
      return link;
    }
  }
  /* An ESS turn aims off its own reseat, and adds its frames. */
  int seated = 0, reseat = 0;
  if (is_ess(seat) && !ess_reseat(link.facing, cup_dir, &seated, &reseat)) {
    link.why = row.id + ": cannot be stepped from this state";
    return link;
  }
  const int* aiming = leaves_the_view(seat) ? &left.camera : is_ess(seat) ? &seated : camera;

  Move move;
  if (!move_at(row.id, link.facing, aiming, &move)) {
    link.why = row.id + ": not in the roster at this facing";
    return link;
  }

  /* A non-finite seed would index `cM_atan2s`'s table out of bounds and crash the process. */
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    link.why = row.id + ": the state reaching this move is not a place";
    return link;
  }

  tww_engine::Init init;
  init.pos.set(static_cast<f32>(x), static_cast<f32>(y), static_cast<f32>(z));
  /* A standstill sets both angles. */
  init.shape_angle_y = static_cast<s16>(link.facing);
  init.travel_angle_y = static_cast<s16>(link.facing);
  init.normal_speed = 0.0f;
  init.speed_f = 0.0f;
  init.proc = daPy_lk_c::daPyProc_WAIT_e;

  /* The last argument would raise `RunOptions::camera`, so it is null. */
  const Drive d = drive(move, init, room, tww_engine::RunOptions(), 96, 0, false, 0);
  link.engine_frames = d.engine_frames;
  link.frames = d.frames + row.surcharge;

  if (d.blocked) {
    link.why = row.id + ": " + move.blocked;
    return link;
  }
  if (d.unaimed) {
    link.why = row.id + ": no camera cardinal reverses this facing";
    return link;
  }
  if (!d.ok) {
    char proc[48];
    std::snprintf(proc, sizeof proc, ": stopped in proc %d", d.stopped_in);
    link.why = row.id + proc;
    return link;
  }
  if (d.dispatch_calls > 0) {
    char arm[64];
    std::snprintf(arm, sizeof arm, ": dispatched into arm 0x%X", d.dispatch_arm);
    link.why = row.id + arm;
    return link;
  }
  if (!d.rested) {
    /* The next move is seeded as a standstill. */
    link.why = row.id + ": did not come back to a standstill";
    return link;
  }

  /* Exact: `Drive` differences f32 positions in double. */
  link.x = x + d.dx;
  link.y = y + d.dy;
  link.z = z + d.dz;
  link.facing = d.facing_out & 0xFFFF;
  link.angle_x = d.angle_x_out;
  link.angle_z = d.angle_z_out;
  if (leaves_the_view(seat)) {
    link.camera = left.camera;
    link.has_camera = true;
    link.frames += left.frames;
  }
  if (is_ess(seat)) {
    link.camera = seated;
    link.has_camera = true;
    link.frames += reseat;
  }
  link.ran = true;
  return link;
}

/* A link is a pure function of its inputs, so drives are memoised per thread. The key must hold
   every input `step_through_engine` reads, or a plan is handed another's landing. A different room
   or `kDrivesHeld` empties it. */
struct DriveKey {
  std::string id;
  int frames, surcharge, turn, steps, taps;
  uint32_t x, y, z;
  int facing, angle_x, angle_z;
  bool has_camera;
  int camera, cup_dir;

  bool operator==(const DriveKey& o) const {
    return x == o.x && y == o.y && z == o.z && facing == o.facing && id == o.id &&
           frames == o.frames && surcharge == o.surcharge && turn == o.turn && steps == o.steps &&
           taps == o.taps && angle_x == o.angle_x && angle_z == o.angle_z &&
           has_camera == o.has_camera && camera == o.camera && cup_dir == o.cup_dir;
  }
};

struct DriveKeyHash {
  size_t operator()(const DriveKey& k) const {
    size_t h = std::hash<std::string>()(k.id);
    const int64_t part[] = {k.frames, k.surcharge, k.turn,    k.steps,   k.taps,
                            k.x,      k.y,         k.z,       k.facing,  k.angle_x,
                            k.angle_z, k.has_camera, k.camera, k.cup_dir};
    for (size_t i = 0; i < sizeof part / sizeof part[0]; ++i) {
      h ^= std::hash<int64_t>()(part[i]) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    }
    return h;
  }
};

uint32_t f32_bits(double v) {
  const f32 f = static_cast<f32>(v);
  uint32_t out;
  std::memcpy(&out, &f, sizeof out);
  return out;
}

const size_t kDrivesHeld = 200000;

struct Drives {
  std::unordered_map<DriveKey, Link, DriveKeyHash> held;
  tww_engine::RoomDzb from;
  bool have = false;
  bool on = true;
  DriveMemo count;
};
thread_local Drives drives;

void remember_in(const tww_engine::RoomDzb* room) {
  if (!drives.on || room == nullptr) return;
  if (!drives.have || !same_room(drives.from, *room)) {
    drives.held.clear();
    drives.from = *room;
    drives.have = true;
  }
}

Link step_remembered(const BaseMove& row, const Edge& edge, const tww_engine::RoomDzb* room,
                     double x, double y, double z, int facing, int angle_x, int angle_z,
                     const int* camera, int cup_dir) {
  if (!drives.on || room == nullptr) {
    return step_through_engine(row, edge, room, x, y, z, facing, angle_x, angle_z, camera,
                               cup_dir);
  }
  if (!drives.have || drives.held.size() >= kDrivesHeld) {
    drives.held.clear();
    remember_in(room);
  }
  DriveKey key;
  key.id = row.id;
  key.frames = row.frames;
  key.surcharge = row.surcharge;
  key.turn = row.turn;
  key.steps = edge.steps;
  key.taps = edge.taps;
  key.x = f32_bits(x);
  key.y = f32_bits(y);
  key.z = f32_bits(z);
  key.facing = facing & 0xFFFF;
  key.angle_x = angle_x;
  key.angle_z = angle_z;
  key.has_camera = camera != 0;
  key.camera = camera ? (*camera & 0xFFFF) : 0;
  key.cup_dir = cup_dir;
  const std::unordered_map<DriveKey, Link, DriveKeyHash>::const_iterator at = drives.held.find(key);
  if (at != drives.held.end()) {
    ++drives.count.hits;
    return at->second;
  }
  ++drives.count.misses;
  const Link link =
      step_through_engine(row, edge, room, x, y, z, facing, angle_x, angle_z, camera, cup_dir);
  drives.held.emplace(key, link);
  return link;
}

Consult consult_path(const std::vector<Edge>& path, const Question& question,
                     const BaseTable& base, const tww_engine::RoomDzb* room) {
  remember_in(room);
  Consult out;
  out.path = path;
  /* Start at the f32 Link can be at, not the typed double, so `start + net` stays exact. */
  double x = static_cast<double>(static_cast<f32>(question.start_x));
  double y = static_cast<double>(static_cast<f32>(question.start_y));
  double z = static_cast<double>(static_cast<f32>(question.start_z));
  int facing = question.start_facing & 0xFFFF;
  int angle_x = 0, angle_z = 0;
  /* Carried move by move: the camera at move N depends on every move before it. */
  int camera = question.start_camera & 0xFFFF;
  bool has_camera = question.has_camera;
  out.had_camera = question.has_camera;
  int cup_dir = 0;

  for (size_t i = 0; i < path.size(); ++i) {
    const size_t at = static_cast<size_t>(path[i].row);
    if (path[i].row < 0 || at >= base.move.size()) {
      out.why = "an edge naming no row of the table it was searched over";
      break;
    }
    const int* carried = has_camera ? &camera : 0;
    const Link link = step_remembered(base.move[at], path[i], room, x, y, z, facing, angle_x,
                                      angle_z, carried, cup_dir);
    out.engine_frames += link.engine_frames;
    if (!link.ran) {
      out.why = link.why;
      break;
    }
    ++out.drives;
    Reached reached;
    reached.x = link.x;
    reached.y = link.y;
    reached.z = link.z;
    reached.facing = link.facing;
    reached.frames = link.frames;
    reached.camera = link.camera;
    out.stop.push_back(reached);
    out.frames += link.frames;
    out.stated_frames += link.stated_frames;
    x = link.x;
    y = link.y;
    z = link.z;
    facing = link.facing;
    angle_x = link.angle_x;
    angle_z = link.angle_z;
    camera = link.camera & 0xFFFF;
    has_camera = link.has_camera;
    cup_dir = link.cup_dir;
  }

  out.x = x;
  out.y = y;
  out.z = z;
  out.facing = facing;
  /* With the item aimed, the distance is measured on the item; `x, y, z` stay Link's. */
  if (question.aim == Aim::Overhead) {
    item_point(x, y, z, angle_x, facing, angle_z, &out.aim_x, &out.aim_y, &out.aim_z);
  } else {
    aim_point(question, x, z, facing, &out.aim_x, &out.aim_z);
    out.aim_y = y;
  }
  out.distance = question.target.from_feet ? question.target.distance(x, z)
                                           : question.target.distance(out.aim_x, out.aim_z);

  /* Decided on the engine's end state only, facing included. */
  if (!out.why.empty()) {
    out.outcome = Outcome::CouldNotRun;
  } else if (reaches(question, out.aim_x, out.aim_y, out.aim_z) && question.end_facing.holds(facing)) {
    out.outcome = Outcome::Confirmed;
  } else {
    out.outcome = Outcome::Refuted;
  }
  return out;
}

}  // namespace

DriveMemo drive_memo() { return drives.count; }

bool remember_drives(bool on) {
  const bool was = drives.on;
  drives.on = on;
  drives.held.clear();
  drives.count = DriveMemo();
  return was;
}

bool reaches(const Question& question, double aim_x, double aim_y, double aim_z) {
  /* Tolerance zero means the same f32, counted in float steps: a double `distance <= 0` fails on
     the very landing asked for. An address is also held to its typed bytes, height included. */
  if (question.target.from_feet) {
    /* The region is Link's feet, so the item is held to the typed box directly. */
    if (!question.target.in_within(aim_x, aim_z, question.tolerance)) return false;
    if (question.tolerance > 0.0) {
      return question.target.hull_distance(aim_x, aim_z) <= question.tolerance &&
             question.target.height_off(aim_y) <= question.tolerance;
    }
    return question.target.holds_bytes(aim_x, aim_y, aim_z);
  }
  if (question.tolerance > 0.0) {
    return question.target.distance(aim_x, aim_z) <= question.tolerance &&
           question.target.height_off(aim_y) <= question.tolerance;
  }
  return question.target.off(aim_x, aim_z) == 0 &&
         question.target.holds_bytes(aim_x, aim_y, aim_z);
}

double consults_per_confirmed(const VerifyCounters& count) {
  if (count.confirmed <= 0) return -1.0;
  return static_cast<double>(count.consults) / static_cast<double>(count.confirmed);
}

Verified verify(const Found& found, const Question& question, const BaseTable& base,
                const tww_engine::RoomDzb* room, Consulting* watch) {
  Verified out;
  out.greedy = question.greedy;
  if (room == nullptr) return out;

  for (size_t i = 0; i < found.candidate.size(); ++i) {
    const Candidate& cand = found.candidate[i];

    /* Greedy drops candidates near only by the model's allowance; here, so the tree's list is
       the same with the switch either way. */
    if (question.greedy && cand.distance > question.tolerance) {
      ++out.count.discarded;
      continue;
    }

    const Consult one = consult_path(cand.path, question, base, room);
    Consult row = one;
    row.predicted = cand.distance;
    row.predicted_frames = cand.frames;
    row.allowance = cand.allowance;
    row.error = cand.distance - one.distance;

    ++out.count.consults;
    out.count.engine_frames += row.engine_frames;
    if (row.outcome == Outcome::Confirmed) {
      ++out.count.confirmed;
    } else if (row.outcome == Outcome::Refuted) {
      ++out.count.refuted;
    } else {
      ++out.count.could_not_run;
    }

    out.consult.push_back(row);
    if (out.consult.back().outcome == Outcome::Confirmed) {
      out.answer.push_back(out.consult.size() - 1);
    }

    if (watch != nullptr && !watch->consulted(out, found.candidate.size())) break;
  }

  /* The closest are driven so their landings are the engine's; they never reach `answer`. */
  for (size_t i = 0; i < found.closest.size(); ++i) {
    const Candidate& cand = found.closest[i];
    const Consult one = consult_path(cand.path, question, base, room);
    Consult row = one;
    row.predicted = cand.distance;
    row.predicted_frames = cand.frames;
    row.allowance = cand.allowance;
    row.error = cand.distance - one.distance;
    ++out.count.closest_consults;
    out.count.engine_frames += row.engine_frames;
    out.closest.push_back(row);
    if (watch != nullptr && !watch->consulted(out, found.candidate.size())) break;
  }
  return out;
}

namespace {

std::string key_of(const std::vector<Edge>& path) {
  const std::vector<Edge> c = canonical(path);
  std::string out;
  char one[32];
  for (size_t i = 0; i < c.size(); ++i) {
    std::snprintf(one, sizeof one, "%d:%d:%d,", c[i].row, c[i].steps, c[i].taps);
    out += one;
  }
  return out;
}

/** A `Stepper` that drives every move through the engine. `sloped_frames` stays zero: the engine is
 *  owed no model allowance. */
struct EngineStep : Stepper {
  const Grid* grid;
  const Selection* selection;
  const tww_engine::RoomDzb* room;
  /** Mutable: the search holds the stepper by const reference. */
  mutable std::set<std::string> refused;
  mutable long long drives = 0;

  Stepped step(const BaseMove& row, double x, double y, double z, int facing) const {
    Stepped out;
    out.x = x;
    out.y = y;
    out.z = z;
    out.facing = facing & 0xFFFF;
    out.frames = row.frames;

    Edge edge;
    const Link link = step_through_engine(row, edge, room, x, y, z, facing, 0, 0);
    ++drives;
    if (!link.ran) {
      refused.insert(row.id);
      return out;
    }
    out.x = link.x;
    out.y = link.y;
    out.z = link.z;
    out.facing = link.facing;
    out.frames = link.frames;
    const Floor under = floor_at(*grid, *selection, out.x, out.z, out.y);
    out.left = !under.inside;
    out.ok = true;
    return out;
  }
};

}  // namespace

Recall recall_audit(const Question& question, const BaseTable& base, const Grid& grid,
                    const Selection& selection, const Calibration& cal, const Verified& verified,
                    const tww_engine::RoomDzb* room) {
  Recall out;
  if (room == nullptr) return out;

  EngineStep engine;
  engine.grid = &grid;
  engine.selection = &selection;
  engine.room = room;

  /* No greedy discard, no model-priced distance bound, no camera field. */
  Question exact = question;
  exact.greedy = false;
  exact.distance_bound = false;
  exact.camera_clear = nullptr;
  const Found truth = search_tree(exact, base, grid, selection, cal, &engine);
  out.complete = truth.exhausted;
  out.drives = engine.drives;
  out.truth_generated = truth.count.generated;
  for (std::set<std::string>::const_iterator it = engine.refused.begin();
       it != engine.refused.end(); ++it) {
    out.skipped.push_back(*it);
  }

  std::set<std::string> admitted;
  for (size_t i = 0; i < verified.answer.size(); ++i) {
    admitted.insert(key_of(verified.consult[verified.answer[i]].path));
  }
  std::set<std::string> real;
  for (size_t i = 0; i < truth.candidate.size(); ++i) {
    const std::vector<Edge>& path = truth.candidate[i].path;
    real.insert(key_of(path));
    out.truth.push_back(canonical(path));
  }

  for (size_t i = 0; i < out.truth.size(); ++i) {
    if (admitted.find(key_of(out.truth[i])) != admitted.end()) continue;
    bool camera = false;
    for (const Edge& e : out.truth[i]) {
      if (e.row >= 0 && static_cast<size_t>(e.row) < base.move.size() &&
          runs_camera(seat_of(base.move[static_cast<size_t>(e.row)].id))) {
        camera = true;
      }
    }
    if (camera && question.camera_clear != nullptr) {
      out.behind_camera.push_back(out.truth[i]);
    } else {
      out.lost.push_back(out.truth[i]);
    }
  }
  for (size_t i = 0; i < verified.answer.size(); ++i) {
    const std::vector<Edge>& path = verified.consult[verified.answer[i]].path;
    if (real.find(key_of(path)) == real.end()) out.gained.push_back(canonical(path));
  }
  return out;
}

std::vector<Consult> one_per_landing(const std::vector<Consult>& consults) {
  typedef std::tuple<double, double, double, double, double, int> Landing;
  std::map<Landing, size_t> seen;
  std::vector<Consult> out;
  for (const Consult& one : consults) {
    const Landing at(one.x, one.y, one.z, one.aim_x, one.aim_z, one.facing & 0xFFFF);
    std::map<Landing, size_t>::iterator held = seen.find(at);
    if (held == seen.end()) {
      seen[at] = out.size();
      out.push_back(one);
    } else if (one.frames < out[held->second].frames) {
      out[held->second] = one;
    }
  }
  return out;
}

}  // namespace search
