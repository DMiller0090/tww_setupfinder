#include "search.h"
#include "cup_exit.h"
#include "l_chain.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <chrono>
#include <set>
#include <thread>
#include <unordered_map>

#include "SSystem/SComponent/c_math.h"

namespace search {

/* The held item's offset in Link's frame, from `daDitem_c::set_pos`. */
namespace {
const double kItemAcross = 30.0, kItemAhead = 20.0, kItemUp = 140.0;
}  // namespace

void aim_point(const Question& q, double x, double z, int facing, double* ax, double* az) {
  if (q.aim != Aim::Overhead) {
    *ax = x;
    *az = z;
    return;
  }
  /* The game's table sine (`JMASSin`), not `std::sin`, to match the engine exactly. */
  const double s = static_cast<double>(cM_ssin(static_cast<s16>(facing & 0xFFFF)));
  const double c = static_cast<double>(cM_scos(static_cast<s16>(facing & 0xFFFF)));
  *ax = x + c * kItemAcross + s * kItemAhead;
  *az = z + c * kItemAhead - s * kItemAcross;
}

double aim_reach(const Question& q) {
  if (q.aim != Aim::Overhead) return 0.0;
  return std::sqrt(kItemAcross * kItemAcross + kItemAhead * kItemAhead);
}

double aim_lift(const Question& q) {
  return q.aim == Aim::Overhead ? kItemUp : 0.0;
}

double beyond_error(double off, double noise) {
  return off > noise ? off - noise : 0.0;
}

int closeness(double distance_a, double gap_a, double distance_b, double gap_b) {
  if (distance_a != distance_b) return distance_a < distance_b ? -1 : 1;
  if (gap_a != gap_b) return gap_a < gap_b ? -1 : 1;
  return 0;
}
namespace {

/** The f32 spacing at a magnitude. */
double ulp32(double magnitude) {
  const float m = static_cast<float>(std::fabs(magnitude));
  if (!(m > 0.0f)) return static_cast<double>(std::numeric_limits<float>::denorm_min());
  /* `std::nextafter` by bit increment; it is on the hot path. */
  uint32_t bits = 0;
  std::memcpy(&bits, &m, sizeof bits);
  ++bits;
  float up = 0.0f;
  std::memcpy(&up, &bits, sizeof up);
  return static_cast<double>(up - m);
}

const double kPi = 3.14159265358979323846;
/** One tree node. The walk is depth-first, so a state's path is the stack below it; no parent links. */
struct Node {
  double x = 0, y = 0, z = 0;
  int facing = 0;
  /** The camera the next move's stick is read against; only `model_step` changes it. */
  int camera = 0;
  bool has_camera = false;
  /** `Pose::cup_dir`. */
  int cup_dir = 0;
  int frames = 0;
  int depth = 0;
  Edge edge;
  /** Sticky along the path once any move is handed off. */
  bool handed_off = false;
  /** Whether the move into this state alone was handed off; splices rebuild `handed_off` from it. */
  bool move_handed = false;
  /** What the move into this state added to `allowance`, and to `consult` beyond that. */
  double move_threshold = 0;
  double move_floor = 0;
  /** Walked off the grid: floor unknown, not expanded. */
  bool left = false;
  /** `Candidate::allowance`; never shrinks along a path, which the bound relies on. */
  double allowance = 0;
  /** `Candidate::consult`; never shrinks along a path. */
  double consult = 0;
  double distance = 0;
  double gap = 0;
  /** For an address target, whether the aim is within model error of the bytes; true otherwise. */
  bool bytes_near = true;
  /** `Target::distance_here`. Orders the walk only; `distance` bounds it. */
  double here = 0;
  /** Unique in the run: walk index in the high bits, creation order below. Zero is the start. */
  uint64_t id = 0;
};

/** What one move adds to the consult radius beyond its measured error: `range` f32 quanta at
 *  `magnitude`. Needed because level-ground moves are owed no threshold, yet the model (double)
 *  and engine (f32) still differ. */
double consult_floor(double magnitude, double range) { return range * ulp32(magnitude); }

/** The dominance signature: the quantised state. */
struct Key {
  long long ix = 0, iy = 0, iz = 0, facing = 0;
  /** Needed: the camera decides which turnaround is available. */
  long long camera = 0;
  bool operator==(const Key& o) const {
    return ix == o.ix && iy == o.iy && iz == o.iz && facing == o.facing && camera == o.camera;
  }
};

/** The dominance table: a fixed, direct-mapped cache sized to stay in CPU cache. Eviction only
 *  costs pruning, never an answer; a hit compares the whole key, so a hash collision cannot delete
 *  a subtree. */
struct Seen {
  static const size_t kSlots = 1u << 14;

  struct Slot {
    Key key;
    int frames;
    bool taken;
    /* The `Node::id` holding the signature, so a tie can ask whether its subtree was walked. */
    uint64_t id;
    Slot() : frames(0), taken(false), id(0) {}
  };
  std::vector<Slot> slot;

  void ready() { slot.assign(kSlots, Slot()); }

  static size_t digest(const Key& k) {
    /* splitmix64 per field: the fields are small integers, so a plain xor would cluster. */
    uint64_t h = 0x9E3779B97F4A7C15ull;
    const long long field[5] = {k.ix, k.iy, k.iz, k.facing, k.camera};
    for (int i = 0; i < 5; ++i) {
      uint64_t v = static_cast<uint64_t>(field[i]) + 0x9E3779B97F4A7C15ull;
      v ^= v >> 30;
      v *= 0xBF58476D1CE4E5B9ull;
      v ^= v >> 27;
      v *= 0x94D049BB133111EBull;
      v ^= v >> 31;
      h ^= v;
      h *= 0x100000001B3ull;
    }
    return static_cast<size_t>(h);
  }

  /** The cheapest frames this signature was reached at, or null. `where` is its slot either way. */
  int* find(const Key& k, size_t* where) {
    const size_t at = digest(k) & (kSlots - 1);
    *where = at;
    if (slot[at].taken && slot[at].key == k) return &slot[at].frames;
    return nullptr;
  }

  void keep(const Key& k, int frames, size_t where, uint64_t id = 0) {
    slot[where].key = k;
    slot[where].frames = frames;
    slot[where].taken = true;
    slot[where].id = id;
  }

  uint64_t& id_at(size_t where) { return slot[where].id; }
};

long long quantise(double v, double cell) {
  return static_cast<long long>(std::floor(v / cell));
}

Key key_of(const Node& n, const Quanta& q) {
  Key k;
  k.ix = quantise(n.x, q.cell);
  k.iy = quantise(n.y, q.cell);
  k.iz = quantise(n.z, q.cell);
  k.facing = (n.facing & 0xFFFF) / q.facing;
  k.camera = n.has_camera ? (n.camera & 0xFFFF) / q.facing : -1;
  /* A state just out of a C up turn has extra moves, so `cup_dir` is part of the key. */
  k.camera = k.camera * 3 + (n.cup_dir + 1);
  return k;
}

/** A path with the signature, frames and band after each move, so a merged order can be spliced
 *  onto a plan through the state it merged into. */
struct Trail {
  std::vector<Edge> path;
  std::vector<Key> key;
  std::vector<int> frames;
  std::vector<double> consult;
  std::vector<double> allowance;
  std::vector<char> handed;
  std::vector<double> threshold;
  std::vector<double> floor;
};

/** Whether a camera move's camera stays clear of the room up to its tap, over the directions from
 *  its start camera to its tap camera through the facing (all of them with no camera seated). */
bool camera_clear(const CamField* field, Seat seat, int taps, const Node& here, const Pose& now) {
  if (field == nullptr || !runs_camera(seat)) return true;
  const int level = camera_level(*field, seat, taps);
  uint64_t need = ~0ULL;
  if (here.has_camera) {
    const int face = here.facing & 0xFFFF;
    const int start = static_cast<int16_t>(static_cast<uint16_t>(here.camera - face));
    const int tap = static_cast<int16_t>(static_cast<uint16_t>(now.camera - face));
    const int lo = std::min(0, std::min(start, tap));
    const int hi = std::max(0, std::max(start, tap));
    need = CamField::bins(face + lo, hi - lo, field->margin());
  }
  return field->clear(level, here.x, here.y, here.z, need);
}

}  // namespace

int camera_level(const CamField& field, Seat seat, int taps) {
  switch (seat) {
    case Seat::LeavesByBSettle:
    case Seat::LeavesByBEarly:
      return CamField::follow_level();
    case Seat::LeavesByCDown:
      return field.view_level(taps);
    case Seat::LHeldCDown:
      return field.follow_start_level(taps);
    default:
      return -1;
  }
}

namespace {

double xz_distance(double x, double z, double tx, double tz) {
  const double dx = x - tx, dz = z - tz;
  return std::sqrt(dx * dx + dz * dz);
}

/** The answer list, collapsed and ranked; shared by partial and final runs so the order matches.
 *  `most` cuts the candidate list short, zero is the whole list. */
struct Shape {
  /** The canonical key computed once, since `canonical` allocates and sorts run under the lock. */
  struct Keyed {
    const Candidate* one;
    std::vector<Edge> key;
  };

  static void keys_of(const std::vector<Candidate>& from, std::vector<Keyed>* into) {
    into->resize(from.size());
    for (size_t i = 0; i < from.size(); ++i) {
      (*into)[i].one = &from[i];
      (*into)[i].key = canonical(from[i].path);
    }
  }

  static void into(Found* found, const std::vector<Candidate>& raw,
                   const std::vector<Candidate>& kept, size_t most) {
    /* The closest are kept apart from the near set, in a deterministic order. */
    {
      std::vector<Keyed> by;
      keys_of(kept, &by);
      std::sort(by.begin(), by.end(), closer);
      found->closest.clear();
      found->closest.reserve(by.size());
      for (size_t i = 0; i < by.size(); ++i) found->closest.push_back(*by[i].one);
    }
    found->count.kept_closest = static_cast<long long>(found->closest.size());

    /* Collapse on the path, not its multiset: two orders can land apart in the engine. */
    std::vector<Keyed> all;
    keys_of(raw, &all);
    std::map<const std::vector<Edge>*, size_t, KeyBefore> by_key;
    long long collapses = 0;
    for (size_t i = 0; i < all.size(); ++i) {
      const std::map<const std::vector<Edge>*, size_t, KeyBefore>::iterator it =
          by_key.find(&all[i].one->path);
      if (it == by_key.end()) {
        by_key[&all[i].one->path] = i;
        continue;
      }
      ++collapses;
      const Candidate& held = *all[it->second].one;
      const Candidate& now = *all[i].one;
      /* A total order, since work items finish in any order. */
      if (now.frames < held.frames ||
          (now.frames == held.frames && now.distance < held.distance) ||
          (now.frames == held.frames && now.distance == held.distance &&
           now.path < held.path)) {
        it->second = i;
      }
    }
    found->count.key_collapses = collapses;

    /* Deterministic, not a considered ranking. */
    std::vector<Keyed> survived;
    survived.reserve(by_key.size());
    for (std::map<const std::vector<Edge>*, size_t, KeyBefore>::const_iterator it = by_key.begin();
         it != by_key.end(); ++it) {
      survived.push_back(all[it->second]);
    }
    std::sort(survived.begin(), survived.end(), ranked);
    if (most > 0 && survived.size() > most) survived.resize(most);
    found->candidate.clear();
    found->candidate.reserve(survived.size());
    for (size_t i = 0; i < survived.size(); ++i) found->candidate.push_back(*survived[i].one);
  }

  /** The closest `most` of a merge of every walk's own closest. */
  static std::vector<Candidate> twenty(const std::vector<Candidate>& all, size_t most) {
    std::vector<Keyed> by;
    keys_of(all, &by);
    std::sort(by.begin(), by.end(), closer);
    if (most > 0 && by.size() > most) by.resize(most);
    std::vector<Candidate> out;
    out.reserve(by.size());
    for (size_t i = 0; i < by.size(); ++i) out.push_back(*by[i].one);
    return out;
  }

  /** Compares path pointers by content, so the collapse copies no path. */
  struct KeyBefore {
    bool operator()(const std::vector<Edge>* a, const std::vector<Edge>* b) const {
      return *a < *b;
    }
  };

  /** A total order, so the result does not move with the thread count. */
  static bool closer(const Keyed& a, const Keyed& b) {
    const int by = closeness(a.one->distance, a.one->gap, b.one->distance, b.one->gap);
    if (by != 0) return by < 0;
    if (a.one->frames != b.one->frames) return a.one->frames < b.one->frames;
    if (a.key != b.key) return a.key < b.key;
    return a.one->path < b.one->path;
  }

  static bool ranked(const Keyed& a, const Keyed& b) {
    if (a.one->frames != b.one->frames) return a.one->frames < b.one->frames;
    if (a.one->distance != b.one->distance) return a.one->distance < b.one->distance;
    if (a.key != b.key) return a.key < b.key;
    return a.one->path < b.one->path;
  }
};

}  // namespace

int turn_steps(const BaseMove& row) {
  if (row.driven || row.turn == 0 || row.frames <= 0 || !row.step.empty()) return 0;
  return row.stands > 0 ? row.stands : 1;
}

std::vector<int> rows_for(const Question& question, const BaseTable& base) {
  std::vector<int> out;
  for (size_t i = 0; i < base.move.size(); ++i) {
    const BaseMove& row = base.move[i];
    const bool steppable = row.driven || turn_steps(row) > 0;
    if (!steppable) continue;
    if (question.moves.empty()) {
      out.push_back(static_cast<int>(i));
      continue;
    }
    for (size_t m = 0; m < question.moves.size(); ++m) {
      if (question.moves[m] == row.id) {
        out.push_back(static_cast<int>(i));
        break;
      }
    }
  }
  return out;
}

std::vector<std::string> unstepped_for(const Question& question, const BaseTable& base) {
  std::vector<std::string> out;
  for (size_t m = 0; m < question.moves.size(); ++m) {
    for (size_t i = 0; i < base.move.size(); ++i) {
      const BaseMove& row = base.move[i];
      if (row.id != question.moves[m]) continue;
      if (!row.driven && turn_steps(row) == 0) out.push_back(row.id);
      break;
    }
  }
  return out;
}

/** Highest less lowest ground over cells that have ground. */
double ground_drop(const Grid& grid) {
  double lowest = 0.0, highest = 0.0;
  bool any = false;
  for (size_t c = 0; c < grid.low.size(); ++c) {
    if ((grid.mark[c] & kNoGround) != 0) continue;
    const double lo = grid.low[c], hi = grid.high[c];
    if (!any) {
      lowest = lo;
      highest = hi;
      any = true;
      continue;
    }
    if (lo < lowest) lowest = lo;
    if (hi > highest) highest = hi;
  }
  return any && highest > lowest ? highest - lowest : 0.0;
}

double ground_steepest(const Grid& grid) {
  double most = 0.0;
  for (size_t at = 0; at + 3 < grid.plane.size(); at += 4) {
    const double ny = grid.plane[at + 1];
    if (ny <= 0.0) continue;
    const double across = std::sqrt(grid.plane[at] * grid.plane[at] +
                                    grid.plane[at + 2] * grid.plane[at + 2]);
    most = std::max(most, across / ny);
  }
  return most;
}

double travel_of(const BaseMove& row, double drop, const MoveCal* mc, const PlaneTable* pt,
                 double steepest) {
  /* Each grounded frame's `speedF` part is scaled somewhere in [least, most] by the ground, and
     each air frame is flown or not; the end's distance is bounded by the furthest that set reaches
     over sampled directions, plus `length * kStep` for what the sampling can miss. The level reach
     is not an upper bound. */
  /* Scale range per frame: the cosine arm `1 + share * (cos - 1)`, cos >= 1 / sqrt(1 + g^2) less a
     table margin, times the uphill arm `up_level - up_slope * grade` over the grades the frame's
     direction allows (both side-axis signs; the whole range just after a launch). */
  const bool scales = mc != nullptr && steepest > 0.0;
  double arm_lo = 1.0, arm_hi = 1.0;
  if (scales) {
    const double cos_least = 1.0 / std::sqrt(1.0 + steepest * steepest) - 1e-3;
    const double arm_a = 1.0 + mc->ground_share * (cos_least - 1.0);
    arm_lo = std::min(1.0, arm_a);
    arm_hi = std::max(1.0, arm_a);
  }
  auto range_of = [&](double lo_grade, double hi_grade, double* least, double* most) {
    const double u_a = mc->up_level - mc->up_slope * lo_grade * steepest;
    const double u_b = mc->up_level - mc->up_slope * hi_grade * steepest;
    const double u_lo = std::min(u_a, u_b), u_hi = std::max(u_a, u_b);
    const double can[6] = {arm_lo,        arm_hi,        arm_lo * u_lo,
                           arm_lo * u_hi, arm_hi * u_lo, arm_hi * u_hi};
    *least = can[0];
    *most = can[0];
    for (double c : can) {
      *least = std::min(*least, c);
      *most = std::max(*most, c);
    }
    *least = std::max(0.0, *least);
    *most = std::max(0.0, *most);
  };
  const double kTurn = 2.0 * 3.14159265358979323846 / 65536.0;
  const int air_from = row.air_from, air_last = row.air_to - 1;
  std::vector<double> part_a, part_s, part_least, part_most;
  double fixed_ahead = 0.0, fixed_side = 0.0, length = 0.0;
  for (size_t f = 0; f < row.step.size(); ++f) {
    const BaseFrame& bf = row.step[f];
    const int at = static_cast<int>(f);
    if (air_from >= 0 && at >= air_from && at <= air_last + 1) {
      part_a.push_back(bf.ahead);
      part_s.push_back(bf.side);
      part_least.push_back(0.0);
      part_most.push_back(1.0);
      length += std::hypot(bf.ahead, bf.side);
      continue;
    }
    fixed_ahead += bf.ahead - bf.speed_ahead;
    fixed_side += bf.side - bf.speed_side;
    length += std::hypot(bf.ahead - bf.speed_ahead, bf.side - bf.speed_side);
    double least = 1.0, most = 1.0;
    if (scales) {
      double lo = -1.0, hi = 1.0;
      const bool after_air = air_from >= 0 && at > air_last + 1;
      const double speed = std::hypot(bf.speed_ahead, bf.speed_side);
      if (!after_air && speed > 0.0) {
        const double turned = f > 0 ? row.step[f - 1].turn * kTurn : 0.0;
        const double way = std::atan2(bf.speed_side, bf.speed_ahead);
        lo = 1.0;
        hi = -1.0;
        for (int sign = -1; sign <= 1; sign += 2) {
          const double rel = way + sign * turned;
          const double across = std::fabs(std::sin(rel)) + 0.01;
          if (std::cos(rel) >= -0.01) {
            lo = std::min(lo, -across);
            hi = std::max(hi, 1.0);
          }
          if (std::cos(rel) <= 0.01) {
            lo = std::min(lo, -1.0);
            hi = std::max(hi, across);
          }
        }
        lo = std::max(lo, -1.0);
        hi = std::min(hi, 1.0);
      }
      range_of(lo, hi, &least, &most);
    }
    part_a.push_back(bf.speed_ahead);
    part_s.push_back(bf.speed_side);
    part_least.push_back(least);
    part_most.push_back(most);
    length += most * std::hypot(bf.speed_ahead, bf.speed_side);
  }
  const int kAngles = 8192;
  const double kStep = 2.0 * 3.14159265358979323846 / kAngles;
  double travel = 0.0;
  for (int k = 0; k < kAngles; ++k) {
    const double ca = std::cos(k * kStep), cs = std::sin(k * kStep);
    double reach = ca * fixed_ahead + cs * fixed_side;
    for (size_t i = 0; i < part_a.size(); ++i) {
      const double along = ca * part_a[i] + cs * part_s[i];
      reach += (along > 0.0 ? part_most[i] : part_least[i]) * along;
    }
    travel = std::max(travel, reach);
  }
  travel += length * kStep;
  travel = std::max(travel, row.reach);

  /* A tabled end is a convex mix of nodes, so no further than its furthest node. */
  if (pt != nullptr) {
    for (size_t i = 0; i < pt->ahead.size() && i < pt->side.size(); ++i) {
      if (i < pt->key.size() && pt->key[i] == 255) continue;
      travel = std::max(travel, std::hypot(static_cast<double>(pt->ahead[i]),
                                           static_cast<double>(pt->side[i])));
    }
  }

  /* Extra air frames over a drop H: `k <= sqrt(2H / |g|)`, at the arc's horizontal speed. */
  if (row.air_from >= 0 && row.air_to > row.air_from && row.gravity < 0.0 && drop > 0.0) {
    double air_speed = 0.0;
    for (int f = row.air_from; f < row.air_to; ++f) {
      const BaseFrame& bf = row.step[static_cast<size_t>(f)];
      const double step = std::sqrt(bf.ahead * bf.ahead + bf.side * bf.side);
      if (step > air_speed) air_speed = step;
    }
    const double extra = std::sqrt(2.0 * drop / -row.gravity);
    travel += air_speed * extra;
  }
  return travel;
}

double fastest_rate(const BaseTable& base, const std::vector<int>& rows, const Grid& grid,
                    const Calibration* cal) {
  const double drop = ground_drop(grid);
  const double steepest = ground_steepest(grid);
  double best = 0.0;
  for (size_t i = 0; i < rows.size(); ++i) {
    const BaseMove& row = base.move[static_cast<size_t>(rows[i])];
    if (!row.driven || row.frames <= 0) continue;
    const MoveCal* mc = cal != nullptr ? cal->of(row.id) : nullptr;
    const PlaneTable* pt = cal != nullptr ? cal->plane_of(row.id) : nullptr;
    const double rate =
        travel_of(row, drop, mc, pt, steepest) / static_cast<double>(row.frames);
    if (rate > best) best = rate;
  }
  return best;
}

bool level_ground(const Grid& grid) {
  for (size_t at = 0; at + 3 < grid.plane.size(); at += 4) {
    if (grid.plane[at + 1] == 0.0) continue;
    /* `cM_atan2s` is monotone, so 0 at both extreme facings is 0 at every facing. */
    const float nx = static_cast<float>(grid.plane[at + 0]);
    const float ny = static_cast<float>(grid.plane[at + 1]);
    const float nz = static_cast<float>(grid.plane[at + 2]);
    const float across = std::sqrt(nx * nx + nz * nz);
    if (cM_atan2s(across, ny) != 0 || cM_atan2s(-across, ny) != 0) return false;
  }
  return true;
}

int fewest_frames(const BaseMove& row, const Calibration& cal, bool level) {
  int least = row.frames;
  /* A launch can land early: `step_move` charges `flew - table_flight` with `flew >= 1`. */
  if (row.air_from >= 0 && row.air_to > row.air_from) {
    least = row.frames - (row.air_to - row.air_from);
  }
  if (!level) {
    const PlaneTable* pt = cal.plane_of(row.id);
    if (pt != nullptr) {
      int shortest = 0;
      for (size_t i = 0; i < pt->extra_frames.size(); ++i) {
        if (pt->extra_frames[i] < shortest) shortest = pt->extra_frames[i];
      }
      least += shortest;
    }
  }
  return least > 1 ? least : 1;
}

double widest_slack(const Question& question, const BaseTable& base, const std::vector<int>& rows,
                    const Calibration& cal, const Grid& grid, double rate) {
  const bool level = level_ground(grid);
  /* Check range share: `consult_floor` at the furthest reachable magnitude, over the fewest frames. */
  int fewest = 0;
  for (size_t i = 0; i < rows.size(); ++i) {
    const BaseMove& row = base.move[static_cast<size_t>(rows[i])];
    if (!row.driven || row.frames <= 0) continue;
    const int least = fewest_frames(row, cal, level);
    if (fewest == 0 || least < fewest) fewest = least;
  }
  if (fewest == 0) return 0.0;
  const double furthest = std::max(std::fabs(question.start_x), std::fabs(question.start_z)) +
                          rate * static_cast<double>(question.frames > 0 ? question.frames : 0);
  const double floor_share =
      consult_floor(furthest, question.check_range) / static_cast<double>(fewest);

  /* Every allowance is exactly 0 on level ground. */
  if (level) return floor_share;
  double most = 0.0;
  for (size_t i = 0; i < rows.size(); ++i) {
    const BaseMove& row = base.move[static_cast<size_t>(rows[i])];
    if (!row.driven || row.frames <= 0) continue;
    /* The most the move can owe (every frame sloped) over the fewest frames it can cost. */
    const double per_frame = cal.threshold(row.id, row.frames) /
                             static_cast<double>(fewest_frames(row, cal, false));
    if (per_frame > most) most = per_frame;
  }
  return floor_share + most;
}

Quanta quanta_for(const Question& question, const BaseTable& base, const std::vector<int>& rows) {
  Quanta q;
  /* Floored at magnitude 1: near the origin the f32 step is subnormal and overflows the cell index. */
  const double magnitude =
      std::max(1.0, std::max(std::fabs(question.start_x), std::fabs(question.start_z)));
  const double floor_cell = ulp32(magnitude);
  q.cell = std::max(question.tolerance / std::sqrt(2.0), floor_cell);

  double reach = 0.0;
  for (size_t i = 0; i < rows.size(); ++i) {
    const BaseMove& row = base.move[static_cast<size_t>(rows[i])];
    if (row.reach > reach) reach = row.reach;
  }
  if (reach <= 0.0 || question.tolerance <= 0.0) {
    q.facing = 1;
    return q;
  }
  const double bucket = question.tolerance * 65536.0 / (2.0 * kPi * reach);
  q.facing = bucket >= 1.0 ? static_cast<int>(bucket) : 1;
  if (q.facing > 65536) q.facing = 65536;
  return q;
}

std::vector<Edge> canonical(const std::vector<Edge>& path) {
  std::vector<Edge> out = path;
  std::sort(out.begin(), out.end());
  return out;
}

/* Depth-first so memory is O(depth); children nearest-first so early results are good. The order
   never changes what a finished run returns. */
/** Halfway from the start to the target's middle; a freed axis stays at the start. */
static cup_tape::Spot camera_spot(const Question& q) {
  const Target& t = q.target;
  const double tx = !t.has_x ? q.start_x : t.ranged ? 0.5 * (t.x0 + t.x1) : t.x;
  const double tz = !t.has_z ? q.start_z : t.ranged ? 0.5 * (t.z0 + t.z1) : t.z;
  const double ty = t.has_y ? 0.5 * (t.y0 + t.y1) : q.start_y;
  cup_tape::Spot s;
  s.x = static_cast<float>(0.5 * (q.start_x + tx));
  s.y = static_cast<float>(0.5 * (q.start_y + ty));
  s.z = static_cast<float>(0.5 * (q.start_z + tz));
  return s;
}

Found search_tree(const Question& question, const BaseTable& base, const Grid& grid,
                  const Selection& selection, const Calibration& cal, const Stepper* instead,
                  Watching* watch) {
  Found found;
  const cup_tape::Spot mid = camera_spot(question);
  const std::vector<int> rows = rows_for(question, base);
  found.unstepped = unstepped_for(question, base);
  found.rate = fastest_rate(base, rows, grid, &cal);
  found.slack = widest_slack(question, base, rows, cal, grid, found.rate);

  /* The steps bound: with N steps left a state ends no nearer than its distance less N times the
     widest `travel_of`, less the item's swing; each step widens the recording test by at most
     `step_charge`. */
  const double drop = ground_drop(grid);
  const double steepest = ground_steepest(grid);
  /* How far the model's aim can be from the engine's, per unit of its error across. */
  const double bytes_scale = 1.0 + steepest;
  double step_travel = 0.0, step_charge = 0.0;
  std::map<int, double> row_travel;
  for (size_t i = 0; i < rows.size(); ++i) {
    const BaseMove& row = base.move[static_cast<size_t>(rows[i])];
    const MoveCal* mc = cal.of(row.id);
    const PlaneTable* pt = cal.plane_of(row.id);
    const double travel = travel_of(row, drop, mc, pt, steepest);
    row_travel[rows[i]] = travel;
    step_travel = std::max(step_travel, travel);
    step_charge = std::max(step_charge, charge_of(mc, pt, 1 << 20, false));
    if (pt != nullptr) step_charge = std::max(step_charge, pt->worst);
  }
  const double step_floor = consult_floor(
      std::max(std::fabs(question.start_x), std::fabs(question.start_z)) +
          found.rate * static_cast<double>(question.frames > 0 ? question.frames : 0),
      question.check_range);
  step_charge += step_floor;
  const double aim_swing = 2.0 * aim_reach(question);
  /* The dives' farthest kept closest; infinite, cutting nothing, until they have run. */
  double seeded = std::numeric_limits<double>::infinity();
  found.quanta = quanta_for(question, base, rows);
  const bool given_away = watch != nullptr && watch->takes_every_candidate();

  /* The admissible bound: frames so far plus the frames the fastest move needs for the distance
     left. A rate of zero prunes nothing. */
  struct Bound {
    static long long of(const Question& q, double rate, double slack, double distance, int frames,
                        double consult) {
      /* Forgive what a recorded candidate may be short by (tolerance, consult so far, and the slack
         its continuation can earn), or the bound would cut recordable states. */
      if (!q.distance_bound) return frames;
      const int frames_left = q.frames > frames ? q.frames - frames : 0;
      const double forgiven =
          q.tolerance + consult + slack * static_cast<double>(frames_left);
      const double left = distance - forgiven;
      if (left <= 0.0 || rate <= 0.0) return frames;
      return frames + static_cast<long long>(std::ceil(left / rate));
    }
  };

  /* Every edge out of a state: one per driven row, one per C up turn step count each way (a
     hundred steps of 655 do not close the circle, so both directions are needed), one per L tap
     count on rows that take taps. Seat and calibration are looked up once here, off the hot path. */
  struct Option {
    int row;
    int steps;
    Seat seat;
    int taps;
    const MoveCal* mc;
    const PlaneTable* pt;
    /* Per-move bounds for the steps bound, tested before stepping. */
    double travel;
    double charge;
  };
  std::vector<Option> option;
  int cheapest_move = 0;
  for (size_t r = 0; r < rows.size(); ++r) {
    const BaseMove& row = base.move[static_cast<size_t>(rows[r])];
    const int steps = turn_steps(row);
    if (steps > 0) {
      const int each_way = steps / 2;
      for (int n = -each_way; n <= each_way; ++n) {
        if (n == 0) continue;
        Option o;
        o.row = rows[r];
        o.steps = n;
        o.seat = seat_of(row.id);
        o.mc = cal.of(row.id);
        o.pt = cal.plane_of(row.id);
        o.travel = 0.0;
        o.charge = 0.0;
        o.taps = 0;
        option.push_back(o);
      }
      const int price = row.frames > 0 ? row.frames : 1;
      if (cheapest_move == 0 || price < cheapest_move) cheapest_move = price;
      continue;
    }
    Option o;
    o.row = rows[r];
    o.steps = 0;
    o.seat = seat_of(row.id);
    o.mc = cal.of(row.id);
    o.pt = cal.plane_of(row.id);
    o.travel = row_travel[rows[r]];
    o.charge = std::max(charge_of(o.mc, o.pt, 1 << 20, false), o.pt != nullptr ? o.pt->worst : 0.0) +
               step_floor;
    o.taps = 0;
    option.push_back(o);
    if (takes_taps(o.seat)) {
      for (int t = 1; t <= l_chain::kTaps; ++t) {
        o.taps = t;
        option.push_back(o);
      }
    }
    if (row.frames > 0 && (cheapest_move == 0 || row.frames < cheapest_move)) {
      cheapest_move = row.frames;
    }
  }

  /* Without a Steps cap, the most moves the frame budget can pay at the cheapest price. */
  const int kDeepest = 64;
  int deepest = question.steps;
  if (deepest <= 0) {
    deepest = cheapest_move > 0 ? question.frames / cheapest_move : 1;
    if (deepest < 1) deepest = 1;
    if (deepest > kDeepest) deepest = kDeepest;
  }

  Node start;
  start.x = question.start_x;
  start.y = question.start_y;
  start.z = question.start_z;
  start.facing = question.start_facing & 0xFFFF;
  start.camera = question.start_camera & 0xFFFF;
  start.has_camera = question.has_camera;

  /* The near set: within `tolerance + consult`, wider than the tolerance so a model miss the
     engine would land inside is not lost. */
  struct Near {
    static bool is(const Question& q, const Node& n, double d, bool* by_allowance) {
      /* A filter, not a prune: the next move can still change the facing. */
      if (!q.end_facing.holds(n.facing)) {
        *by_allowance = false;
        return false;
      }
      *by_allowance = d > q.tolerance;
      return d <= q.tolerance + n.consult && n.bytes_near;
    }
  };

  /* The closest states reached, so a run with no near candidate still has something to show. Each
     work item keeps its own `kKeep`; the run cuts once (`Shape::twenty`). */
  const size_t kKeep = 20;
  struct Closest {
    /** A max-heap on closeness: the farthest kept is at the front. */
    std::vector<Candidate> kept;
    static bool worse(const Candidate& a, const Candidate& b) {
      const int by = closeness(a.distance, a.gap, b.distance, b.gap);
      if (by != 0) return by < 0;
      if (a.frames != b.frames) return a.frames > b.frames;
      return canonical(b.path) < canonical(a.path);
    }
    void offer(const Candidate& one, size_t most) {
      if (most == 0) return;
      if (kept.size() < most) {
        kept.push_back(one);
        std::push_heap(kept.begin(), kept.end(), worse);
        return;
      }
      if (!worse(one, kept.front())) return;
      std::pop_heap(kept.begin(), kept.end(), worse);
      kept.back() = one;
      std::push_heap(kept.begin(), kept.end(), worse);
    }
  };

  /** One stack frame: a node's children, sorted nearest first, and the next one to walk. */
  struct Level {
    std::vector<Node> child;
    size_t next;
    /* The first child's `Node::id`; the rest follow in turn. */
    uint64_t lo = 0;
  };

  /** Nearest first, with a total tie-break so a partial run is deterministic. */
  struct Nearest {
    static bool before(const Node& a, const Node& b) {
      const int by = closeness(a.here, a.gap, b.here, b.gap);
      if (by != 0) return by < 0;
      if (a.distance != b.distance) return a.distance < b.distance;
      if (a.frames != b.frames) return a.frames < b.frames;
      return a.edge < b.edge;
    }
  };

  /* Recorded candidates before the collapse; filling it clears `Found::exhausted`. */
  const size_t kMostCandidates = 200000;
  /* States between watcher calls; the caller also throttles by the clock. */
  const long long kTellEvery = 512;
  /** One work item: a self-contained path and state, walked against its own dominance table, so
   *  its answers depend on nothing else. */
  struct Item {
    Trail trail;
    Node node;
  };

  /** A shortlisted plan and its states, for splicing merged orders onto. */
  struct Listed {
    Trail trail;
    Candidate end;
  };

  /** What one walk reaped, private to it until the run merges. */
  struct Reaped {
    std::vector<Candidate> raw;
    std::vector<Candidate> kept;
    Counters count;
    bool exhausted;
    std::vector<Item> item;
    std::vector<Listed> listed;
    /* Signature digest -> (plan, state index) for every shortlisted plan's states. */
    std::unordered_multimap<size_t, std::pair<size_t, size_t> > listed_at;
    /* Splices already handed over. */
    std::set<std::vector<Edge> > spliced;
    /* `serial` in the high bits keeps ids distinct from states other walks made. */
    uint64_t serial;
    uint64_t next_id;
    /* The split hands subtrees over as work instead of walking them. */
    bool splitting;
    Reaped() : exhausted(true), serial(0), next_id(0), splitting(false) {}
  };

  /* The tied path, then the listed plan after its state `s`, with its band rebuilt from its own
     moves. False where the states differ, frames or steps are exceeded, or it fails the near test. */
  auto splice = [&](const Trail& tie, const Listed& one, size_t s, Listed* twin) -> bool {
    if (!(tie.key.back() == one.trail.key[s])) return false;
    const size_t n = one.trail.path.size();
    const int shift = tie.frames.back() - one.trail.frames[s];
    if (one.end.frames + shift > question.frames) return false;
    if (static_cast<int>(tie.path.size() + (n - 1 - s)) > deepest) return false;
    twin->trail = tie;
    double consult = tie.consult.back();
    double allowance = tie.allowance.back();
    bool handed = false;
    for (size_t j = 0; j < tie.handed.size(); ++j) handed = handed || tie.handed[j] != 0;
    for (size_t j = s + 1; j < n; ++j) {
      consult = consult + one.trail.threshold[j] + one.trail.floor[j];
      allowance = allowance + one.trail.threshold[j];
      handed = handed || one.trail.handed[j] != 0;
      twin->trail.path.push_back(one.trail.path[j]);
      twin->trail.key.push_back(one.trail.key[j]);
      twin->trail.frames.push_back(one.trail.frames[j] + shift);
      twin->trail.consult.push_back(consult);
      twin->trail.allowance.push_back(allowance);
      twin->trail.handed.push_back(one.trail.handed[j]);
      twin->trail.threshold.push_back(one.trail.threshold[j]);
      twin->trail.floor.push_back(one.trail.floor[j]);
    }
    twin->end = one.end;
    twin->end.path = twin->trail.path;
    twin->end.frames = one.end.frames + shift;
    twin->end.consult = consult;
    twin->end.allowance = allowance;
    twin->end.handed_off = handed;
    /* The end facing already passed on the listed plan. */
    return twin->end.distance <= question.tolerance + consult;
  };

  /* Shortlist bytes across every walk; `bytes_of` deliberately overestimates. */
  std::atomic<long long> held_bytes(0);
  std::atomic<bool> memory_full(false);
  auto bytes_of = [](const Listed& one) -> long long {
    const long long n = static_cast<long long>(one.trail.path.size());
    const long long entry = static_cast<long long>(sizeof(std::pair<size_t, size_t>)) + 48;
    return static_cast<long long>(sizeof(Listed)) + 96 +
           n * static_cast<long long>(2 * sizeof(Edge) + sizeof(Key) + sizeof(int) +
                                      4 * sizeof(double) + sizeof(char)) + n * entry +
           n * static_cast<long long>(sizeof(Edge)) + 64;
  };

  /* Keep a shortlisted plan indexed by its states. Over `Question::memory` it is dropped (it was
     still handed over) and the run is no longer exhaustive. */
  auto list = [&](Reaped* r, const Listed& one) {
    const long long cost = bytes_of(one);
    if (question.memory > 0 && held_bytes.load() + cost > question.memory) {
      r->exhausted = false;
      memory_full.store(true);
      return;
    }
    held_bytes.fetch_add(cost);
    const size_t at = r->listed.size();
    r->listed.push_back(one);
    for (size_t s = 0; s < one.trail.key.size(); ++s) {
      r->listed_at.emplace(Seen::digest(one.trail.key[s]), std::make_pair(at, s));
    }
  };

  /* Hand a splice over exactly as a walked shortlisted plan. */
  auto give = [&](Reaped* r, Counters* count, const Listed& twin,
                  const std::function<void(const Candidate&)>* drove) {
    ++count->merged_orders;
    if (twin.end.distance > question.tolerance) ++count->allowed;
    if (drove != nullptr) (*drove)(twin.end);
    if (!given_away && r->raw.size() < kMostCandidates) r->raw.push_back(twin.end);
    list(r, twin);
  };

  /* `root` then the current child of the first `levels` levels. Built only where it is kept. */
  auto trail_to = [&](const Trail& root, const std::vector<Level>& stack, size_t levels) {
    Trail t = root;
    t.path.reserve(root.path.size() + levels);
    t.key.reserve(root.key.size() + levels);
    t.frames.reserve(root.frames.size() + levels);
    t.consult.reserve(root.consult.size() + levels);
    t.allowance.reserve(root.allowance.size() + levels);
    t.handed.reserve(root.handed.size() + levels);
    t.threshold.reserve(root.threshold.size() + levels);
    t.floor.reserve(root.floor.size() + levels);
    for (size_t i = 0; i < levels; ++i) {
      const Node& n = stack[i].child[stack[i].next - 1];
      t.path.push_back(n.edge);
      t.key.push_back(key_of(n, found.quanta));
      t.frames.push_back(n.frames);
      t.consult.push_back(n.consult);
      t.allowance.push_back(n.allowance);
      t.handed.push_back(n.move_handed ? 1 : 0);
      t.threshold.push_back(n.move_threshold);
      t.floor.push_back(n.move_floor);
    }
    return t;
  };

  /* Whether a state's subtree is not yet walked by this walk (or was made by another walk), so a
     tie with it has nothing to splice onto and must not be deleted. Always true for the split. */
  auto still_open = [](uint64_t id, const std::vector<Level>& stack, const Reaped* reap) -> bool {
    if (reap->splitting) return true;
    if ((id >> 40) != reap->serial) return true;
    for (size_t l = stack.size(); l-- > 0;) {
      const Level& level = stack[l];
      if (id < level.lo) continue;
      if (id >= level.lo + level.child.size()) return false;
      for (size_t j = level.next > 0 ? level.next - 1 : 0; j < level.child.size(); ++j) {
        if (level.child[j].id == id) return true;
      }
      return false;
    }
    return false;
  };

  /* Fill `into` with `here`'s surviving children, sorted nearest first. `here`'s trail is `root`
     plus every level but the last, which is `into`. */
  auto expand = [&](Seen& best, Counters* count, const Node& here, Level* into, const Trail& root,
                    const std::vector<Level>& stack, Reaped* reap,
                    const std::function<void(const Candidate&)>* drove, double beaten) {
    into->next = 0;
    into->lo = reap->next_id;
    into->child.clear();
    into->child.reserve(option.size());
    for (size_t oi = 0; oi < option.size(); ++oi) {
      const Option o = option[oi];
      const BaseMove& row = base.move[static_cast<size_t>(o.row)];

      Node child;
      if (o.steps != 0) {
        /* Turn collapse: a turn never follows a turn, since one edge reaches any net cheaper. */
        if (here.edge.row >= 0 && turn_steps(base.move[static_cast<size_t>(here.edge.row)]) > 0) {
          continue;
        }
        ++count->generated;
        child = here;
        child.move_handed = false;
        child.move_threshold = 0;
        child.move_floor = 0;
        child.edge.row = o.row;
        child.edge.steps = o.steps;
        /* Overwrite: the copied edge may carry an L chain's taps. */
        child.edge.taps = o.taps;
        child.depth = here.depth + 1;
        const int steps = o.steps < 0 ? -o.steps : o.steps;
        child.frames = here.frames + (row.frames - 1) + steps;
        Pose was;
        was.facing = here.facing;
        was.camera = here.camera;
        was.has_camera = here.has_camera;
        was.cup_dir = here.cup_dir;
        Pose now;
        if (!model_step(o.seat, row.turn, o.steps, was, &now, &mid)) continue;
        child.facing = now.facing;
        child.camera = now.camera;
        child.has_camera = now.has_camera;
        child.cup_dir = now.cup_dir;
      } else {
        /* The steps bound before stepping, with this first step priced as itself. */
        if (question.distance_bound && beaten < std::numeric_limits<double>::infinity()) {
          StepsLeft after;
          after.distance = here.distance - o.travel;
          after.steps = deepest - (here.depth + 1);
          after.travel = step_travel;
          after.swing = aim_swing;
          after.tolerance = question.tolerance;
          after.consult = here.consult + o.charge;
          after.charge = step_charge;
          after.farthest_closest = beaten;
          if (after.cannot_help()) {
            ++count->steps_pruned;
            continue;
          }
        }
        /* The facing comes from `model_step`, not the stepper: a turnaround's rotation depends on
           the camera. Asked first so a refused camera move costs no step. */
        Pose was;
        was.facing = here.facing;
        was.camera = here.camera;
        was.has_camera = here.has_camera;
        was.cup_dir = here.cup_dir;
        was.taps = o.taps;
        Pose now;
        if (!model_step(o.seat, row.turn, 1, was, &now, &mid)) {
          ++count->generated;
          continue;
        }
        if (!camera_clear(question.camera_clear, o.seat, o.taps, here, now)) {
          ++count->generated;
          ++count->camera_met;
          continue;
        }
        const Stepped st =
            instead != nullptr
                ? instead->step(row, here.x, here.y, here.z, here.facing)
                : step_move(row, grid, selection, o.mc, o.pt, here.x, here.y, here.z,
                            here.facing, question.collision);
        ++count->generated;
        /* Defensive: `rows_for` already excludes unsteppable rows. */
        if (!st.ok) continue;
        child.x = st.x;
        child.y = st.y;
        child.z = st.z;
        child.facing = now.facing;
        child.camera = now.camera;
        child.has_camera = now.has_camera;
        child.cup_dir = now.cup_dir;
        /* `now.frames`: the wait and exit before a view turnaround, else 0. */
        child.frames = here.frames + st.frames + now.frames;
        child.depth = here.depth + 1;
        child.edge.row = o.row;
        child.edge.steps = 0;
        child.edge.taps = o.taps;
        child.handed_off = here.handed_off || st.handed_off;
        child.move_handed = st.handed_off;
        const double threshold = charge_of(o.mc, o.pt, st.sloped_frames, st.tabled);
        child.allowance = here.allowance + threshold;
        const double least = consult_floor(std::max(std::fabs(st.x), std::fabs(st.z)),
                                           question.check_range);
        child.consult = here.consult + threshold + least;
        child.move_threshold = threshold;
        child.move_floor = least;
        child.left = st.left;
        if (st.left) ++count->left_corridor;
      }

      if (child.frames > question.frames) {
        ++count->bound_pruned;
        continue;
      }
      if (!question.bounds.holds(child.x, child.z)) {
        ++count->outside_bounds;
        continue;
      }
      {
        double ax = 0, az = 0;
        aim_point(question, child.x, child.z, child.facing, &ax, &az);
        const bool feet = question.target.from_feet;
        const double mx = feet ? child.x : ax, mz = feet ? child.z : az;
        /* Cheap lower bound first (`bound <= distance`); the exact distance only for survivors. */
        child.distance = question.target.bound(mx, mz);
        if (Bound::of(question, found.rate, found.slack, child.distance, child.frames,
                      child.consult) > question.frames) {
          ++count->bound_pruned;
          continue;
        }
        child.distance = question.target.distance(mx, mz);
        /* Address bytes, only as far as the model's error; a height errs by up to the xz error
           times the steepest grade. */
        if (question.target.mask.any()) {
          const double off = question.target.bytes_off(ax, child.y + aim_lift(question), az);
          const double noise = child.consult * bytes_scale;
          child.gap = beyond_error(off, noise);
          child.bytes_near = off <= question.tolerance + noise;
        }
        child.here = question.target.ground.has_y
                         ? question.target.distance_here(mx, child.y, mz)
                         : child.distance;
      }
      const long long f = Bound::of(question, found.rate, found.slack, child.distance,
                                    child.frames, child.consult);
      if (f > question.frames) {
        ++count->bound_pruned;
        continue;
      }

      /* The steps bound. Must run before the dominance table, or a cut state could still claim a
         cell and delete a shallower state that had more steps left. */
      if (question.distance_bound && beaten < std::numeric_limits<double>::infinity()) {
        StepsLeft now;
        now.distance = child.distance;
        now.steps = child.left ? 0 : std::max(0, deepest - child.depth);
        now.travel = step_travel;
        now.swing = aim_swing;
        now.tolerance = question.tolerance;
        now.consult = child.consult;
        now.charge = step_charge;
        now.farthest_closest = beaten;
        if (now.cannot_help()) {
          ++count->steps_pruned;
          continue;
        }
      }

      const Key k = key_of(child, found.quanta);
      size_t where = 0;
      int* held = best.find(k, &where);
      /* A tie is an order the model cannot tell apart but the engine can. If the held state's
         subtree is walked, splice its listed plans onto this path and drop it; otherwise walk
         this one as its own state. */
      const uint64_t id = reap->next_id;
      if (held != nullptr) {
        if (*held <= child.frames) {
          const bool tie = *held == child.frames;
          if (!(tie && still_open(best.id_at(where), stack, reap))) {
            ++count->dominance_kills;
            if (tie) {
              std::vector<std::pair<size_t, size_t> > through;
              const size_t d = Seen::digest(k);
              for (std::unordered_multimap<size_t, std::pair<size_t, size_t> >::const_iterator it =
                       reap->listed_at.find(d);
                   it != reap->listed_at.end() && it->first == d; ++it) {
                through.push_back(it->second);
              }
              if (!through.empty()) {
                Trail t = trail_to(root, stack, stack.size() - 1);
                t.path.push_back(child.edge);
                t.key.push_back(k);
                t.frames.push_back(child.frames);
                t.consult.push_back(child.consult);
                t.allowance.push_back(child.allowance);
                t.handed.push_back(child.move_handed ? 1 : 0);
                t.threshold.push_back(child.move_threshold);
                t.floor.push_back(child.move_floor);
                for (size_t i = 0; i < through.size(); ++i) {
                  const Listed one = reap->listed[through[i].first];
                  Listed twin;
                  if (!splice(t, one, through[i].second, &twin)) continue;
                  if (!reap->spliced.insert(twin.trail.path).second) continue;
                  give(reap, count, twin, drove);
                }
              }
            }
            continue;
          }
        } else {
          *held = child.frames;
          best.id_at(where) = id;
        }
      } else {
        best.keep(k, child.frames, where, id);
      }

      child.id = id;
      ++reap->next_id;
      into->child.push_back(child);
    }
    std::sort(into->child.begin(), into->child.end(), Nearest::before);
  };

  /* One subtree, depth-first on one thread. At `stop_depth` a child becomes an `Item` instead of
     being expanded, so the split and the work are the same walk. `root` itself is not recorded
     here; its producer already did. `tick` returning false stops the walk. */
  auto walk = [&](const Node& root, const Trail& root_trail, Seen& best,
                  int stop_depth, size_t cap,
                  const std::function<bool(const Reaped&, int)>* tick,
                  const std::function<void(const Candidate&)>* drove, Reaped* out) {
    const std::vector<Edge>& root_path = root_trail.path;
    std::vector<Level> stack;
    stack.reserve(static_cast<size_t>(deepest) + 2);
    Closest closest;
    long long since_told = kTellEvery;
    bool stopped = false;

    stack.push_back(Level());
    expand(best, &out->count, root, &stack.back(), root_trail, stack, out, drove, seeded);
    ++out->count.expanded;

    while (!stack.empty() && !stopped) {
      if (tick != nullptr && ++since_told >= kTellEvery) {
        since_told = 0;
        /* Finished share of each level's children, weighted by the levels above. */
        double frac = 0.0, weight = 1.0;
        for (size_t i = 0; i < stack.size(); ++i) {
          const double wide = static_cast<double>(stack[i].child.size());
          if (!(wide > 0.0)) break;
          double done = static_cast<double>(stack[i].next);
          if (i + 1 < stack.size() && done > 0.0) done -= 1.0;
          frac += weight * done / wide;
          weight /= wide;
        }
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;
        out->kept = closest.kept;
        if (!(*tick)(*out, static_cast<int>(frac * 1000.0))) {
          out->exhausted = false;
          stopped = true;
          break;
        }
      }

      const size_t at = stack.size() - 1;
      if (stack[at].next >= stack[at].child.size()) {
        stack.pop_back();
        continue;
      }
      const Node child = stack[at].child[stack[at].next++];

      /* Build the path only if something keeps it. The closest test is `<=`, not `<`, because
         `Closest::worse` can still prefer an equally distant state. */
      bool by_allowance = false;
      const bool near = Near::is(question, child, child.distance, &by_allowance);
      const bool could_be_closest =
          !near && (closest.kept.size() < kKeep || child.distance <= closest.kept.front().distance);
      if (near || could_be_closest) {
        Candidate c;
        c.x = child.x;
        c.y = child.y;
        c.z = child.z;
        c.facing = child.facing;
        c.frames = child.frames;
        c.handed_off = child.handed_off;
        c.allowance = child.allowance;
        c.consult = child.consult;
        c.distance = child.distance;
        c.gap = child.gap;
        c.path = root_path;
        c.path.reserve(root_path.size() + stack.size());
        for (size_t i = 0; i < stack.size(); ++i) {
          c.path.push_back(stack[i].child[stack[i].next - 1].edge);
        }

        if (near) {
          if (by_allowance) ++out->count.allowed;
          /* Its states, for splicing ties onto (see `expand`). */
          if (out->listed.size() < cap) {
            Listed one;
            one.trail = trail_to(root_trail, stack, stack.size());
            one.end = c;
            list(out, one);
          }
          if (drove != nullptr) (*drove)(c);
          if (given_away) {
          } else if (out->raw.size() < cap) {
            out->raw.push_back(c);
          } else {
            out->exhausted = false;
            stopped = true;
            break;
          }
        } else {
          closest.offer(c, kKeep);
        }
      }

      /* Recorded above, but not expanded. */
      if (child.left || child.depth >= deepest) continue;

      if (child.depth >= stop_depth) {
        Item one;
        one.trail = trail_to(root_trail, stack, stack.size());
        one.node = child;
        out->item.push_back(one);
        continue;
      }

      stack.push_back(Level());
      const double own = closest.kept.size() >= kKeep ? closest.kept.front().distance
                                                     : std::numeric_limits<double>::infinity();
      expand(best, &out->count, child, &stack.back(), root_trail, stack, out, drove,
             std::min(own, seeded));
      ++out->count.expanded;
    }
    out->kept = closest.kept;
  };

  struct Add {
    static void into(Counters* a, const Counters& b) {
      a->generated += b.generated;
      a->expanded += b.expanded;
      a->bound_pruned += b.bound_pruned;
      a->steps_pruned += b.steps_pruned;
      a->dived += b.dived;
      a->dominance_kills += b.dominance_kills;
      a->left_corridor += b.left_corridor;
      a->outside_bounds += b.outside_bounds;
      a->camera_met += b.camera_met;
      a->allowed += b.allowed;
      a->merged_orders += b.merged_orders;
      /* `key_collapses` and `kept_closest` are run-level, set by `Shape::into`. */
    }
  };

  /* The split depth must not read `Question::cores`, or the answer would vary with thread count:
     the shallowest depth giving `kItemsWanted` items, at most `kSplitCap`. */
  const size_t kItemsWanted = 64;
  const int kSplitCap = 3;
  const int kPollMs = 20;

  std::atomic<bool> pressed(false);

  /* Candidates are driven on the finding thread with no lock (`tww_engine` is per-thread). The
     split records candidates too, so this is set up before it. */
  std::mutex gate;
  const std::function<void(const Candidate&)> drove = [&](const Candidate& one) {
    watch->found(one);
  };
  const std::function<void(const Candidate&)>* handing = watch != nullptr ? &drove : nullptr;

  Seen seed;
  Reaped prefix;
  int split = 1;
  for (;;) {
    seed.ready();
    prefix = Reaped();
    {
      size_t where = 0;
      const Key root_key = key_of(start, found.quanta);
      seed.find(root_key, &where);
      seed.keep(root_key, 0, where);
    }

    /* The start is a zero-move candidate when it is already there; never one of the closest. */
    {
      Candidate here;
      here.x = start.x;
      here.y = start.y;
      here.z = start.z;
      here.facing = start.facing;
      {
        double ax = 0, az = 0;
        aim_point(question, start.x, start.z, question.start_facing, &ax, &az);
        here.distance = question.target.from_feet ? question.target.distance(start.x, start.z)
                                                  : question.target.distance(ax, az);
      }
      bool by_allowance = false;
      if (Near::is(question, start, here.distance, &by_allowance)) {
        if (by_allowance) ++prefix.count.allowed;
        if (handing != nullptr) (*handing)(here);
        prefix.raw.push_back(here);
      }
    }

    /* The split reports no progress. */
    prefix.serial = 1;
    prefix.next_id = uint64_t(1) << 40;
    prefix.splitting = true;
    walk(start, Trail(), seed, split, kMostCandidates, nullptr, handing, &prefix);

    if (!prefix.exhausted) break;
    if (prefix.item.size() >= kItemsWanted) break;
    /* A split at `deepest` would yield no items. */
    if (split + 1 >= deepest || split >= kSplitCap) break;
    ++split;
  }

  std::vector<Candidate> raw;
  raw.swap(prefix.raw);
  std::vector<Candidate> kept = prefix.kept;
  found.count = prefix.count;
  found.exhausted = prefix.exhausted;

  /* The items are merged order-free, so any core count answers the same list. */
  if (!prefix.item.empty()) {
    const std::vector<Item>& item = prefix.item;
    /* The whole cap, not a share: the run-level ceiling is enforced on the shared list. */
    const size_t cap_each = kMostCandidates;

    std::atomic<size_t> next(0);
    size_t finished = 0;
    /* Progress shown, held monotone across threads. */
    int shown = 0;

    int hands = question.cores;
    if (hands < 1) hands = 1;
    if (static_cast<size_t>(hands) > item.size()) hands = static_cast<int>(item.size());

    /* Dives: walk the first `kDive` states of the `kDiveItems` nearest items, each on a fresh copy
       of the seed table and handing nothing over, to seed `seeded` for the steps bound. Same on
       any core count. Skipped for the recall audit. */
    if (question.distance_bound && instead == nullptr) {
      const long long kDive = 1024;
      const size_t kDiveItems = 64;
      std::vector<size_t> dived(item.size());
      for (size_t i = 0; i < dived.size(); ++i) dived[i] = i;
      std::sort(dived.begin(), dived.end(), [&](size_t a, size_t b) {
        if (item[a].node.distance != item[b].node.distance) {
          return item[a].node.distance < item[b].node.distance;
        }
        return a < b;
      });
      if (dived.size() > kDiveItems) dived.resize(kDiveItems);
      std::vector<std::vector<Candidate> > dove(dived.size());
      std::vector<long long> dove_states(dived.size(), 0);
      std::atomic<size_t> dive_next(0);
      auto diver = [&]() {
        Seen mine;
        const std::function<bool(const Reaped&, int)> enough =
            [&](const Reaped& r, int) -> bool { return r.count.generated < kDive; };
        for (;;) {
          const size_t d = dive_next.fetch_add(1);
          if (d >= dived.size()) return;
          const size_t i = dived[d];
          mine = seed;
          Reaped got;
          got.serial = static_cast<uint64_t>(i) + 2;
          got.next_id = got.serial << 40;
          walk(item[i].node, item[i].trail, mine, deepest + 1, kMostCandidates, &enough, nullptr,
               &got);
          dove[d] = got.kept;
          dove_states[d] = got.count.generated;
        }
      };
      /* The dives' shortlist is discarded, so its bytes are restored. */
      const long long held_before = held_bytes.load();
      const bool full_before = memory_full.load();
      std::vector<std::thread> diving;
      for (int h = 0; h < hands; ++h) diving.push_back(std::thread(diver));
      for (size_t h = 0; h < diving.size(); ++h) diving[h].join();
      held_bytes.store(held_before);
      memory_full.store(full_before);
      std::vector<Candidate> all = kept;
      for (size_t i = 0; i < dove.size(); ++i) all.insert(all.end(), dove[i].begin(), dove[i].end());
      const std::vector<Candidate> best = Shape::twenty(all, kKeep);
      if (best.size() >= kKeep) seeded = best.back().distance;
      for (size_t i = 0; i < dove_states.size(); ++i) found.count.dived += dove_states[i];
    }

    /* Per-thread in-flight counts, so a report is the finished work plus every slot. */
    std::vector<Counters> flight(static_cast<size_t>(hands));
    std::vector<std::vector<Candidate> > flight_kept(static_cast<size_t>(hands));

    /* Report the whole run; called with the gate held. False means stop. */
    auto tell = [&]() -> bool {
      if (watch == nullptr) return true;
      /* A stopped run reports nothing more. */
      if (pressed.load()) return false;
      Counters live = found.count;
      for (size_t h = 0; h < flight.size(); ++h) Add::into(&live, flight[h]);
      live.held_bytes = held_bytes.load();
      live.memory_full = memory_full.load();
      if (!watch->walked(live, shown, raw.size())) {
        pressed.store(true);
        return false;
      }
      /* Only the closest; candidates already went to `Watching::found`. */
      if (watch->wants_so_far()) {
        std::vector<Candidate> near_miss = kept;
        for (size_t h = 0; h < flight_kept.size(); ++h) {
          near_miss.insert(near_miss.end(), flight_kept[h].begin(), flight_kept[h].end());
        }
        Found sofar;
        sofar.count = live;
        sofar.quanta = found.quanta;
        sofar.unstepped = found.unstepped;
        sofar.rate = found.rate;
        sofar.slack = found.slack;
        sofar.exhausted = false;
        sofar.closest = Shape::twenty(near_miss, kKeep);
        sofar.count.kept_closest = static_cast<long long>(sofar.closest.size());
        watch->so_far(sofar);
      }
      return true;
    };

    auto worker = [&](size_t me) {
      Seen mine;
      /* How much of this walk's raw list is already on the run's. */
      size_t published = 0;

      /* Publishes this walk's progress; the calling thread does the reporting. */
      const std::function<bool(const Reaped&, int)> beat =
          [&](const Reaped& walking, int through) -> bool {
        std::lock_guard<std::mutex> lock(gate);
        if (pressed.load()) return false;
        /* Clamped: `finished` can move while this walk is still counted. */
        long long part = (static_cast<long long>(finished) * 1000 +
                          (through < 0 ? 0 : (through > 1000 ? 1000 : through))) /
                         static_cast<long long>(item.size());
        if (part > 1000) part = 1000;
        if (part > shown) shown = static_cast<int>(part);
        flight[me] = walking.count;
        flight_kept[me] = walking.kept;
        raw.insert(raw.end(), walking.raw.begin() + static_cast<long>(published),
                   walking.raw.end());
        published = walking.raw.size();
        /* The run-level candidate ceiling. */
        if (raw.size() >= kMostCandidates) {
          found.exhausted = false;
          pressed.store(true);
          return false;
        }
        return true;
      };

      /* No watcher, no lock. */
      const std::function<bool(const Reaped&, int)>* tick = watch != nullptr ? &beat : nullptr;

      for (;;) {
        if (pressed.load()) return;
        const size_t i = next.fetch_add(1);
        if (i >= item.size()) return;
        /* Each item's own dominance table, seeded from the split's. */
        mine = seed;
        Reaped got;
        got.serial = static_cast<uint64_t>(i) + 2;
        got.next_id = got.serial << 40;
        published = 0;
        walk(item[i].node, item[i].trail, mine, deepest + 1, cap_each, tick, handing, &got);

        std::lock_guard<std::mutex> lock(gate);
        Add::into(&found.count, got.count);
        raw.insert(raw.end(), got.raw.begin() + static_cast<long>(published), got.raw.end());
        kept.insert(kept.end(), got.kept.begin(), got.kept.end());
        if (!got.exhausted) found.exhausted = false;
        /* Cleared under the same lock as the fold, so nothing is missed or doubled. */
        flight[me] = Counters();
        flight_kept[me].clear();
        ++finished;

      }
    };

    /* `tww_engine` keeps `dComIfG_player` and `dComIfG_Bgsp` per thread, so each walker drives its
       own finds. The calling thread only reports. */
    std::vector<std::thread> walking;
    walking.reserve(static_cast<size_t>(hands));
    for (int h = 0; h < hands; ++h) walking.push_back(std::thread(worker, static_cast<size_t>(h)));

    if (watch != nullptr) {
      for (;;) {
        {
          std::lock_guard<std::mutex> lock(gate);
          if (pressed.load() || finished >= item.size()) break;
          tell();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
      }
    }
    for (size_t h = 0; h < walking.size(); ++h) walking[h].join();
  }

  if (pressed.load()) found.exhausted = false;
  found.count.held_bytes = held_bytes.load();
  found.count.memory_full = memory_full.load();
  Shape::into(&found, raw, Shape::twenty(kept, kKeep), 0);
  return found;
}

}  // namespace search
