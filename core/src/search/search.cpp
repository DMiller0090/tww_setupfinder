#include "search.h"
#include "cup_exit.h"
#include "l_chain.h"
#include "checkpoint.h"
#include "profile.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <mutex>
#include <chrono>
#include <set>
#include <thread>
#include <type_traits>
#include <unordered_map>

#include "SSystem/SComponent/c_math.h"
#include "engine/ps_mtx.h"
#include "engine/session.h"
#include "m_Do/m_Do_mtx.h"

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

namespace {

/** Whether some f32 `l` makes the f32 sum `l + o` exactly `t`. That `l` is the float nearest
 *  `t - o` or a step or two from it, so a few each side are tried. */
bool sums_to(float t, float o) {
  float l = static_cast<float>(static_cast<double>(t) - static_cast<double>(o));
  for (int k = 0; k < 4; ++k) l = std::nextafter(l, -std::numeric_limits<float>::infinity());
  for (int k = 0; k <= 8; ++k) {
    const float sum = l + o;
    if (std::memcmp(&sum, &t, sizeof sum) == 0) return true;
    l = std::nextafter(l, std::numeric_limits<float>::infinity());
  }
  return false;
}

}  // namespace

void item_point(double x, double y, double z, int angle_x, int shape_y, int angle_z, double* ax,
                double* ay, double* az) {
  Mtx turn;
  mDoMtx_ZXYrotS(turn, static_cast<s16>(angle_x), static_cast<s16>(shape_y),
                 static_cast<s16>(angle_z));
  Vec offset = {static_cast<f32>(kItemAcross), static_cast<f32>(kItemUp),
                static_cast<f32>(kItemAhead)};
  PSMTXMultVec(turn, &offset, &offset);
  *ax = static_cast<double>(static_cast<f32>(x) + offset.x);
  *ay = static_cast<double>(static_cast<f32>(y) + offset.y);
  *az = static_cast<double>(static_cast<f32>(z) + offset.z);
}

/* The item's coordinate is f32(Link's + offset's). Each is a multiple of its own float step, so
   the sum lands on the finer of the two grids, and a target with a low bit set is out of reach
   wherever both are coarse. Angle x and z are 0: only demos and moving floors write them (the
   item gate holds that). */
std::vector<uint8_t> item_facings(const Question& q) {
  const Target& t = q.target;
  if (q.aim != Aim::Overhead || q.tolerance != 0.0 || t.ranged || !t.list.empty() ||
      t.mask.any() || (!t.has_x && !t.has_z)) {
    return std::vector<uint8_t>();
  }
  /* `jmaSinTable` has no static initialiser. A session installs it once for all threads, so one
     is built here rather than installing it beside the sessions other threads may be building. */
  static std::once_flag sine_ready;
  std::call_once(sine_ready, [] { tww_engine::Session installs; });
  const float tx = static_cast<float>(t.x), tz = static_cast<float>(t.z);
  std::vector<uint8_t> lands(65536, 0);
  for (int f = 0; f < 65536; ++f) {
    /* At Link's origin the item is its offset alone, since 0 + o is o. */
    double ox = 0, oy = 0, oz = 0;
    item_point(0.0, 0.0, 0.0, 0, f, 0, &ox, &oy, &oz);
    lands[static_cast<size_t>(f)] =
        (!t.has_x || sums_to(tx, static_cast<float>(ox))) &&
                (!t.has_z || sums_to(tz, static_cast<float>(oz)))
            ? 1
            : 0;
  }
  return lands;
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
  /** Whether there is one, and its yaw only where `Quanta::camera` says it is read. */
  long long camera = 0;
  bool operator==(const Key& o) const {
    return ix == o.ix && iy == o.iy && iz == o.iz && facing == o.facing && camera == o.camera;
  }
};

/** The dominance table: a direct-mapped cache of `Quanta::slots`. Eviction only costs pruning,
 *  never an answer; a hit compares the whole key, so a hash collision cannot delete a subtree. */
struct Seen {
  struct Slot {
    Key key;
    int frames;
    bool taken;
    /* The `Node::id` holding the signature, so a tie can ask whether its subtree was walked. */
    uint64_t id;
    Slot() : frames(0), taken(false), id(0) {}
  };
  std::vector<Slot> slot;

  void ready(size_t slots) { slot.assign(slots, Slot()); }

  static long long bytes(size_t slots) {
    return static_cast<long long>(slots * sizeof(Slot));
  }

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
    const size_t at = digest(k) & (slot.size() - 1);
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
  k.camera = !n.has_camera ? -1 : q.camera ? (n.camera & 0xFFFF) / q.facing : 0;
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

/* A kept run is read back by the same build, so its states are written as their bytes. */
static_assert(std::is_trivially_copyable<Node>::value, "a kept walk writes its states as bytes");
static_assert(std::is_trivially_copyable<Key>::value, "and its signatures");

void put(checkpoint::Writer& w, const Candidate& c) {
  w.pods(c.path);
  w.pod(c.x);
  w.pod(c.y);
  w.pod(c.z);
  w.pod(c.facing);
  w.pod(c.frames);
  w.pod(c.distance);
  w.pod(c.gap);
  w.pod(c.allowance);
  w.pod(c.consult);
  w.pod(c.handed_off);
}

void get(checkpoint::Reader& r, Candidate* c) {
  r.pods(&c->path);
  c->x = r.pod<double>();
  c->y = r.pod<double>();
  c->z = r.pod<double>();
  c->facing = r.pod<int>();
  c->frames = r.pod<int>();
  c->distance = r.pod<double>();
  c->gap = r.pod<double>();
  c->allowance = r.pod<double>();
  c->consult = r.pod<double>();
  c->handed_off = r.pod<bool>();
}

void put(checkpoint::Writer& w, const std::vector<Candidate>& all) {
  w.pod<uint64_t>(all.size());
  for (const Candidate& c : all) put(w, c);
}

bool get(checkpoint::Reader& r, std::vector<Candidate>* all) {
  const uint64_t n = r.pod<uint64_t>();
  if (!r.ok() || n > (uint64_t(1) << 28)) return false;
  all->resize(static_cast<size_t>(n));
  for (Candidate& c : *all) get(r, &c);
  return r.ok();
}

void put(checkpoint::Writer& w, const Trail& t) {
  w.pods(t.path);
  w.pods(t.key);
  w.pods(t.frames);
  w.pods(t.consult);
  w.pods(t.allowance);
  w.pods(t.handed);
  w.pods(t.threshold);
  w.pods(t.floor);
}

void get(checkpoint::Reader& r, Trail* t) {
  r.pods(&t->path);
  r.pods(&t->key);
  r.pods(&t->frames);
  r.pods(&t->consult);
  r.pods(&t->allowance);
  r.pods(&t->handed);
  r.pods(&t->threshold);
  r.pods(&t->floor);
}

const uint32_t kRunMagic = 0x4B434653;   // "SFCK"
const uint32_t kWalkMagic = 0x4B4C4157;  // "WALK"

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
                 double steepest, double* box) {
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

  /* Each part at whichever end of its range moves the end furthest along each axis. */
  double a_lo = fixed_ahead, a_hi = fixed_ahead, s_lo = fixed_side, s_hi = fixed_side;
  for (size_t i = 0; i < part_a.size(); ++i) {
    a_lo += std::min(part_least[i] * part_a[i], part_most[i] * part_a[i]);
    a_hi += std::max(part_least[i] * part_a[i], part_most[i] * part_a[i]);
    s_lo += std::min(part_least[i] * part_s[i], part_most[i] * part_s[i]);
    s_hi += std::max(part_least[i] * part_s[i], part_most[i] * part_s[i]);
  }

  /* A tabled end is a convex mix of nodes, so no further than its furthest node. */
  if (pt != nullptr) {
    for (size_t i = 0; i < pt->ahead.size() && i < pt->side.size(); ++i) {
      if (i < pt->key.size() && pt->key[i] == 255) continue;
      const double a = static_cast<double>(pt->ahead[i]), s = static_cast<double>(pt->side[i]);
      travel = std::max(travel, std::hypot(a, s));
      a_lo = std::min(a_lo, a);
      a_hi = std::max(a_hi, a);
      s_lo = std::min(s_lo, s);
      s_hi = std::max(s_hi, s);
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
    a_lo -= air_speed * extra;
    a_hi += air_speed * extra;
    s_lo -= air_speed * extra;
    s_hi += air_speed * extra;
  }
  if (box != nullptr) {
    /* The model sums its frames in double; this is far past what that can move the end. */
    const double kSlack = 1e-6;
    box[0] = a_lo - kSlack;
    box[1] = a_hi + kSlack;
    box[2] = s_lo - kSlack;
    box[3] = s_hi + kSlack;
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
  q.camera = question.camera_clear != nullptr;
  /* A sixty-fourth of the memory ceiling per table, between 2^14 and 2^22 slots. */
  const long long per_table = question.memory / 64;
  while (q.slots < (size_t(1) << 22) && Seen::bytes(q.slots * 2) <= per_table) q.slots *= 2;
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
cup_tape::Spot camera_spot(const Question& q) {
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

Model model_for(const BaseTable& base, const Grid& grid, const Selection& selection,
                const Calibration& cal) {
  Model m;
  m.base = &base;
  m.grid = &grid;
  m.selection = &selection;
  m.cal_of.resize(base.move.size(), nullptr);
  m.plane_of.resize(base.move.size(), nullptr);
  for (size_t i = 0; i < base.move.size(); ++i) {
    m.cal_of[i] = cal.of(base.move[i].id);
    m.plane_of[i] = cal.plane_of(base.move[i].id);
  }
  return m;
}

bool can_still_land(const Question& question, const Model& model, const std::vector<Edge>& path,
                    size_t from, const Pose& pose, double x, double y, double z) {
  if (model.base == nullptr || model.grid == nullptr || model.selection == nullptr) return true;
  if (question.target.mask.any()) return true;
  const cup_tape::Spot mid = camera_spot(question);
  Pose now = pose;
  double at_x = x, at_y = y, at_z = z;
  /* Each move's own measured error and float floor, as the walk adds them. */
  double owed = 0.0;
  for (size_t k = from; k < path.size(); ++k) {
    if (path[k].row < 0 || static_cast<size_t>(path[k].row) >= model.base->move.size()) {
      return true;
    }
    const size_t r = static_cast<size_t>(path[k].row);
    const BaseMove& row = model.base->move[r];
    const Seat seat = seat_of(row.id);
    Pose was = now;
    Pose next;
    if (turn_steps(row) > 0) {
      if (!model_step(seat, row.turn, path[k].steps, was, &next, &mid)) return true;
      now = next;
      continue;
    }
    was.taps = path[k].taps;
    if (!model_step(seat, row.turn, 1, was, &next, &mid)) return true;
    const Stepped st = step_move(row, *model.grid, *model.selection, model.cal_of[r],
                                 model.plane_of[r], at_x, at_y, at_z, now.facing,
                                 question.collision);
    if (!st.ok || st.handed_off) return true;
    owed += charge_of(model.cal_of[r], model.plane_of[r], st.sloped_frames, st.tabled) +
            consult_floor(std::max(std::fabs(st.x), std::fabs(st.z)), question.check_range);
    at_x = st.x;
    at_y = st.y;
    at_z = st.z;
    now = next;
  }
  double ax = 0, az = 0;
  aim_point(question, at_x, at_z, now.facing, &ax, &az);
  const bool feet = question.target.from_feet;
  return question.target.distance(feet ? at_x : ax, feet ? at_z : az) <=
         question.tolerance + owed;
}

Found search_tree(const Question& question, const BaseTable& base, const Grid& grid,
                  const Selection& selection, const Calibration& cal, const Stepper* instead,
                  Watching* watch, Pausing* pausing) {
  /* The split runs on this thread, then it reports. */
  profile::enter("split and report");
  struct Leaving {
    ~Leaving() { profile::leave(); }
  } leaving;
  Found found;
  const cup_tape::Spot mid = camera_spot(question);
  const std::vector<int> rows = rows_for(question, base);
  found.unstepped = unstepped_for(question, base);
  found.rate = fastest_rate(base, rows, grid, &cal);
  found.slack = widest_slack(question, base, rows, cal, grid, found.rate);
  const std::vector<uint8_t> lands = item_facings(question);

  /* The steps bound: with N steps left a state ends no nearer than its distance less N times the
     widest `travel_of`, less the item's swing; each step widens the recording test by at most
     `step_charge`. */
  const double drop = ground_drop(grid);
  const double steepest = ground_steepest(grid);
  /* How far the model's aim can be from the engine's, per unit of its error across. */
  const double bytes_scale = 1.0 + steepest;
  double step_travel = 0.0, step_charge = 0.0;
  std::map<int, double> row_travel;
  std::map<int, std::vector<double> > row_box;
  for (size_t i = 0; i < rows.size(); ++i) {
    const BaseMove& row = base.move[static_cast<size_t>(rows[i])];
    const MoveCal* mc = cal.of(row.id);
    const PlaneTable* pt = cal.plane_of(row.id);
    std::vector<double> box(4, 0.0);
    const double travel = travel_of(row, drop, mc, pt, steepest, box.data());
    row_travel[rows[i]] = travel;
    row_box[rows[i]] = box;
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
    /* The row group the profile counts it under. */
    profile::Row group;
    /* Every end the model can give the move, about its entry facing (`travel_of`). */
    double box[4];
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
        o.group = profile::kTurn;
        for (double& side : o.box) side = 0.0;
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
    for (int k = 0; k < 4; ++k) o.box[k] = row_box[rows[r]][static_cast<size_t>(k)];
    o.charge = std::max(charge_of(o.mc, o.pt, 1 << 20, false), o.pt != nullptr ? o.pt->worst : 0.0) +
               step_floor;
    o.taps = 0;
    o.group = is_ess(o.seat)            ? profile::kEss
              : o.seat == Seat::Kept ? profile::kMove
                                     : profile::kExit;
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

  /* Without a Steps cap, the most moves the frame budget can pay at the cheapest price. The app
     never asks for one; the tests do. */
  const int kDeepest = 64;
  int deepest = question.steps;
  if (deepest <= 0) {
    deepest = cheapest_move > 0 ? question.frames / cheapest_move : 1;
    if (deepest < 1) deepest = 1;
    if (deepest > kDeepest) deepest = kDeepest;
  }

  /* A child that can have no children of its own and ends at a facing the item cannot land from
     is never an answer, so it is dropped before it is stepped. "No children" is the move cap, or
     too few frames left for the cheapest any move can take (`fewest_frames`, which the bound
     already trusts). Never in the dominance table, it can only spare states, not cut them. */
  std::vector<int> least_of(option.size(), 1);
  int least_next = std::numeric_limits<int>::max();
  for (size_t i = 0; i < option.size(); ++i) {
    const BaseMove& row = base.move[static_cast<size_t>(option[i].row)];
    least_of[i] = std::max(1, fewest_frames(row, cal, false));
    least_next = std::min(least_next, least_of[i]);
  }
  const auto unlandable = [&](int depth, long long frames_at_least, int facing) {
    if (lands.empty() || !question.drop_unlandable || lands[static_cast<size_t>(facing & 0xFFFF)]) {
      return false;
    }
    return depth >= deepest || frames_at_least + least_next > question.frames;
  };

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
    static bool is(const Question& q, const std::vector<uint8_t>& lands, const Node& n, double d,
                   bool* by_allowance) {
      /* A filter, not a prune: the next move can still change the facing. */
      if (!q.end_facing.holds(n.facing) ||
          (!lands.empty() && !lands[static_cast<size_t>(n.facing & 0xFFFF)])) {
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
  /* For progress only: a subtree is taken to shrink by this factor, as a natural log, for each
     frame its root has already spent. Over five recorded searches the bar sat a mean 2-22% off the
     work with it, against 16-27% counting siblings alike. */
  const double kSizePerFrame = 0.02;
  /** Relative sizes of `nodes`, the cheapest one being 1. */
  auto sizes = [&](const std::vector<double>& frames) -> std::vector<double> {
    double least = frames.empty() ? 0.0 : frames[0];
    for (size_t j = 1; j < frames.size(); ++j) least = std::min(least, frames[j]);
    std::vector<double> w(frames.size());
    for (size_t j = 0; j < frames.size(); ++j) w[j] = std::exp(kSizePerFrame * (least - frames[j]));
    return w;
  };
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

  /* A walk's own place, so a stopped one can be kept and carried on (`Pausing`). */
  struct Walking {
    std::vector<Level> stack;
    Closest closest;
    long long since_told = 0;
    bool begun = false;
    /* Stopped by the watcher rather than finished, and its exhausted flag before the stop. */
    bool paused = false;
    bool was_exhausted = true;
  };

  const std::string save_dir = pausing != nullptr ? pausing->save : std::string();
  const std::string resume_dir = pausing != nullptr ? pausing->resume : std::string();
  auto walk_file = [](const std::string& dir, size_t i) {
    return (std::filesystem::path(dir) / ("walk-" + std::to_string(i) + ".bin")).string();
  };
  auto run_file = [](const std::string& dir) {
    return (std::filesystem::path(dir) / "run.bin").string();
  };
  auto put_layout = [](checkpoint::Writer& w) {
    w.pod(static_cast<uint32_t>(sizeof(Node)));
    w.pod(static_cast<uint32_t>(sizeof(Seen::Slot)));
    w.pod(static_cast<uint32_t>(sizeof(Key)));
    w.pod(static_cast<uint32_t>(sizeof(Counters)));
    w.pod(static_cast<uint32_t>(sizeof(Edge)));
  };
  auto same_layout = [](checkpoint::Reader& r) {
    return r.pod<uint32_t>() == sizeof(Node) && r.pod<uint32_t>() == sizeof(Seen::Slot) &&
           r.pod<uint32_t>() == sizeof(Key) && r.pod<uint32_t>() == sizeof(Counters) &&
           r.pod<uint32_t>() == sizeof(Edge);
  };

  /* Each stopped walk is written by its own thread, straight from its tables. */
  auto save_walk = [&](size_t i, const Seen& mine, const Walking& state, const Reaped& got,
                       size_t published_count) -> bool {
    checkpoint::Writer w(walk_file(save_dir, i));
    w.pod(kWalkMagic);
    w.pod(checkpoint::kVersion);
    put_layout(w);
    w.pod(static_cast<uint64_t>(i));
    /* Only the slots holding a signature, with where they sit. */
    std::vector<uint32_t> held_at;
    std::vector<Seen::Slot> held;
    for (size_t k = 0; k < mine.slot.size(); ++k) {
      if (!mine.slot[k].taken) continue;
      held_at.push_back(static_cast<uint32_t>(k));
      held.push_back(mine.slot[k]);
    }
    w.pod(static_cast<uint64_t>(mine.slot.size()));
    w.pods(held_at);
    w.pods(held);
    w.pod(static_cast<uint64_t>(state.stack.size()));
    for (const Level& level : state.stack) {
      w.pods(level.child);
      w.pod(static_cast<uint64_t>(level.next));
      w.pod(level.lo);
    }
    put(w, state.closest.kept);
    w.pod(state.since_told);
    w.pod(state.was_exhausted);
    put(w, got.raw);
    put(w, got.kept);
    w.pod(got.count);
    w.pod(static_cast<uint64_t>(got.listed.size()));
    for (const Listed& one : got.listed) {
      put(w, one.trail);
      put(w, one.end);
    }
    w.pod(static_cast<uint64_t>(got.spliced.size()));
    for (const std::vector<Edge>& path : got.spliced) w.pods(path);
    w.pod(got.serial);
    w.pod(got.next_id);
    w.pod(static_cast<uint64_t>(published_count));
    return w.close();
  };

  auto load_walk = [&](size_t i, Seen* mine, Walking* state, Reaped* got,
                       size_t* published_count) -> bool {
    checkpoint::Reader r(walk_file(resume_dir, i));
    if (r.pod<uint32_t>() != kWalkMagic || r.pod<uint32_t>() != checkpoint::kVersion ||
        !same_layout(r) || r.pod<uint64_t>() != i) {
      return false;
    }
    const uint64_t slots = r.pod<uint64_t>();
    std::vector<uint32_t> held_at;
    std::vector<Seen::Slot> held;
    r.pods(&held_at);
    r.pods(&held);
    if (!r.ok() || slots != found.quanta.slots || held_at.size() != held.size()) return false;
    mine->ready(static_cast<size_t>(slots));
    for (size_t k = 0; k < held.size(); ++k) {
      if (held_at[k] >= slots) return false;
      mine->slot[held_at[k]] = held[k];
    }
    const uint64_t levels = r.pod<uint64_t>();
    if (!r.ok() || levels > 4096) return false;
    state->stack.resize(static_cast<size_t>(levels));
    for (Level& level : state->stack) {
      r.pods(&level.child);
      level.next = static_cast<size_t>(r.pod<uint64_t>());
      level.lo = r.pod<uint64_t>();
    }
    if (!get(r, &state->closest.kept)) return false;
    state->since_told = r.pod<long long>();
    state->was_exhausted = r.pod<bool>();
    if (!get(r, &got->raw) || !get(r, &got->kept)) return false;
    got->count = r.pod<Counters>();
    const uint64_t listed = r.pod<uint64_t>();
    if (!r.ok() || listed > (uint64_t(1) << 32)) return false;
    got->listed.resize(static_cast<size_t>(listed));
    for (size_t at = 0; at < got->listed.size(); ++at) {
      get(r, &got->listed[at].trail);
      get(r, &got->listed[at].end);
      for (size_t k = 0; k < got->listed[at].trail.key.size(); ++k) {
        got->listed_at.emplace(Seen::digest(got->listed[at].trail.key[k]), std::make_pair(at, k));
      }
    }
    const uint64_t spliced = r.pod<uint64_t>();
    if (!r.ok() || spliced > (uint64_t(1) << 32)) return false;
    for (uint64_t k = 0; k < spliced; ++k) {
      std::vector<Edge> one;
      r.pods(&one);
      got->spliced.insert(one);
    }
    got->serial = r.pod<uint64_t>();
    got->next_id = r.pod<uint64_t>();
    *published_count = static_cast<size_t>(r.pod<uint64_t>());
    got->exhausted = state->was_exhausted;
    state->begun = true;
    state->paused = false;
    return r.ok();
  };

  /* The tied path, then the listed plan after its state `s`, with its band rebuilt from its own
     moves. False where the states differ, frames or steps are exceeded, or it fails the near test. */
  auto splice = [&](const Trail& tie, const Listed& one, size_t s, Listed* twin) -> bool {
    if (!(tie.key.back() == one.trail.key[s])) return false;
    const size_t n = one.trail.path.size();
    const int shift = tie.frames.back() - one.trail.frames[s];
    if (one.end.frames + shift > question.frames) return false;
    if (one.end.frames + shift < question.least_frames) return false;
    const int length = static_cast<int>(tie.path.size() + (n - 1 - s));
    if (length > deepest || length < question.fewest) return false;
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
  /* The dominance tables share the ceiling with the shortlist: the split's, and one for each thread
     this machine can run, the most the cores can ask for. Not the cores asked, which would let the
     thread count decide what a full run keeps (`D13`). */
  const long long tables =
      Seen::bytes(found.quanta.slots) *
      (static_cast<long long>(std::max(1u, std::thread::hardware_concurrency())) + 1);
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
    if (question.memory > 0 && tables + held_bytes.load() + cost > question.memory) {
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
    const bool prof = profile::on();
    profile::Timed whole_expansion(profile::kExpand);
    const long long generated_before = count->generated;
    into->next = 0;
    into->lo = reap->next_id;
    into->child.clear();
    into->child.reserve(option.size());
    for (size_t oi = 0; oi < option.size(); ++oi) {
      const Option o = option[oi];
      const BaseMove& row = base.move[static_cast<size_t>(o.row)];
      const auto fated = [&](profile::Fate fate) {
        if (prof) profile::row_fate(o.group, fate);
      };
      profile::Timed child_time;

      Node child;
      if (o.steps != 0) {
        /* Turn collapse: a turn never follows a turn, since one edge reaches any net cheaper. */
        if (here.edge.row >= 0 && turn_steps(base.move[static_cast<size_t>(here.edge.row)]) > 0) {
          continue;
        }
        child_time.start(profile::kTurnChild, false);
        ++count->generated;
        const int steps = o.steps < 0 ? -o.steps : o.steps;
        const int frames = here.frames + (row.frames - 1) + steps;
        Pose was;
        was.facing = here.facing;
        was.camera = here.camera;
        was.has_camera = here.has_camera;
        was.cup_dir = here.cup_dir;
        Pose now;
        if (!model_step(o.seat, row.turn, o.steps, was, &now, &mid)) {
          fated(profile::kCannot);
          continue;
        }
        if (unlandable(here.depth + 1, frames, now.facing)) {
          ++count->unlandable;
          fated(profile::kUnlandable);
          continue;
        }
        /* Most turn children end here, so the state is copied only for the rest. */
        if (frames > question.frames) {
          ++count->bound_pruned;
          fated(profile::kOverFrames);
          continue;
        }
        child = here;
        child.move_handed = false;
        child.move_threshold = 0;
        child.move_floor = 0;
        child.edge.row = o.row;
        child.edge.steps = o.steps;
        /* Overwrite: the copied edge may carry an L chain's taps. */
        child.edge.taps = o.taps;
        child.depth = here.depth + 1;
        child.frames = frames;
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
            fated(profile::kStepsCut);
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
        bool aimed = false;
        {
          profile::Timed timed(profile::kModelStep, false);
          aimed = model_step(o.seat, row.turn, 1, was, &now, &mid);
        }
        if (!aimed) {
          ++count->generated;
          fated(profile::kCannot);
          continue;
        }
        if (!camera_clear(question.camera_clear, o.seat, o.taps, here, now)) {
          ++count->generated;
          ++count->camera_met;
          fated(profile::kCamera);
          continue;
        }
        if (unlandable(here.depth + 1,
                       static_cast<long long>(here.frames) + least_of[oi] + now.frames,
                       now.facing)) {
          ++count->generated;
          ++count->unlandable;
          fated(profile::kUnlandable);
          continue;
        }
        /* A last move whose every end leaves the aim past the near band and past the closest is
           not stepped: the steps bound with none left, along the move's own direction. */
        if (question.distance_bound && beaten < std::numeric_limits<double>::infinity() &&
            !question.target.mask.any() &&
            (here.depth + 1 >= deepest ||
             static_cast<long long>(here.frames) + least_of[oi] + now.frames + least_next >
                 question.frames)) {
          const double sin_f = static_cast<double>(cM_ssin(static_cast<s16>(here.facing)));
          const double cos_f = static_cast<double>(cM_scos(static_cast<s16>(here.facing)));
          const double* b = o.box;
          const double dx_lo = std::min(b[0] * sin_f, b[1] * sin_f) +
                               std::min(b[2] * cos_f, b[3] * cos_f);
          const double dx_hi = std::max(b[0] * sin_f, b[1] * sin_f) +
                               std::max(b[2] * cos_f, b[3] * cos_f);
          const double dz_lo = std::min(b[0] * cos_f, b[1] * cos_f) +
                               std::min(-b[2] * sin_f, -b[3] * sin_f);
          const double dz_hi = std::max(b[0] * cos_f, b[1] * cos_f) +
                               std::max(-b[2] * sin_f, -b[3] * sin_f);
          double ox = 0.0, oz = 0.0;
          if (!question.target.from_feet) aim_point(question, 0.0, 0.0, now.facing, &ox, &oz);
          const double gap =
              question.target.box_gap(here.x + dx_lo + ox, here.x + dx_hi + ox,
                                      here.z + dz_lo + oz, here.z + dz_hi + oz);
          if (gap > question.tolerance + here.consult + o.charge && gap > beaten) {
            ++count->steps_pruned;
            fated(profile::kStepsCut);
            continue;
          }
        }
        Stepped st;
        {
          profile::Timed timed(profile::kStep);
          st = instead != nullptr
                   ? instead->step(row, here.x, here.y, here.z, here.facing)
                   : step_move(row, grid, selection, o.mc, o.pt, here.x, here.y, here.z,
                               here.facing, question.collision);
        }
        ++count->generated;
        if (prof) {
          profile::plane_step(o.row, here.facing, st.plane, st.sloped_frames > 0, st.planes);
        }
        /* Defensive: `rows_for` already excludes unsteppable rows. */
        if (!st.ok) {
          fated(profile::kCannot);
          continue;
        }
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
        fated(profile::kOverFrames);
        continue;
      }
      if (!question.bounds.holds(child.x, child.z)) {
        ++count->outside_bounds;
        fated(profile::kOutside);
        continue;
      }
      {
      profile::Timed tests(profile::kTests, false);
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
          fated(profile::kOverFrames);
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
        fated(profile::kOverFrames);
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
          fated(profile::kStepsCut);
          continue;
        }
      }
      }

      const uint64_t id = reap->next_id;
      {
      profile::Timed dominance(profile::kDominance, false);
      const Key k = key_of(child, found.quanta);
      size_t where = 0;
      int* held = best.find(k, &where);
      /* A tie is an order the model cannot tell apart but the engine can. If the held state's
         subtree is walked, splice its listed plans onto this path and drop it; otherwise walk
         this one as its own state. */
      if (held != nullptr) {
        if (*held <= child.frames) {
          const bool tie = *held == child.frames;
          if (!(tie && still_open(best.id_at(where), stack, reap))) {
            ++count->dominance_kills;
            /* A splice drives the engine; that is timed as the engine's, not here. */
            dominance.stop();
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
            fated(profile::kDominated);
            continue;
          }
        } else {
          *held = child.frames;
          best.id_at(where) = id;
        }
      } else {
        best.keep(k, child.frames, where, id);
      }
      }

      fated(child.left || child.depth >= deepest ||
                    static_cast<long long>(child.frames) + least_next > question.frames
                ? profile::kLeaf
                : profile::kKept);
      child.id = id;
      ++reap->next_id;
      into->child.push_back(child);
    }
    profile::Timed sorting(profile::kSort);
    if (lands.empty()) {
      std::sort(into->child.begin(), into->child.end(), Nearest::before);
    } else {
      /* A child the engine will be asked about goes ahead of one whose facing cannot land the
         item. */
      std::sort(into->child.begin(), into->child.end(), [&](const Node& a, const Node& b) {
        bool unused = false;
        const bool na = Near::is(question, lands, a, a.distance, &unused);
        const bool nb = Near::is(question, lands, b, b.distance, &unused);
        if (na != nb) return na;
        return Nearest::before(a, b);
      });
    }
    if (prof) {
      profile::depth_generated(here.depth + 1,
                               static_cast<uint64_t>(count->generated - generated_before));
    }
  };

  /* One subtree, depth-first on one thread. At `stop_depth` a child becomes an `Item` instead of
     being expanded, so the split and the work are the same walk. `root` itself is not recorded
     here; its producer already did. `tick` returning false stops the walk. */
  auto walk = [&](const Node& root, const Trail& root_trail, Seen& best,
                  int stop_depth, size_t cap,
                  const std::function<bool(const Reaped&, int)>* tick,
                  const std::function<void(const Candidate&)>* drove, Reaped* out,
                  Walking* state) {
    const std::vector<Edge>& root_path = root_trail.path;
    std::vector<Level>& stack = state->stack;
    stack.reserve(static_cast<size_t>(deepest) + 2);
    Closest& closest = state->closest;
    long long& since_told = state->since_told;
    bool stopped = false;

    const bool prof = profile::on();
    unsigned popped = 0;

    if (!state->begun) {
      since_told = kTellEvery;
      stack.push_back(Level());
      expand(best, &out->count, root, &stack.back(), root_trail, stack, out, drove, seeded);
      ++out->count.expanded;
      if (prof) profile::depth_expanded(root.depth);
      state->begun = true;
    }

    while (!stack.empty() && !stopped) {
      if (prof && (++popped & 1023u) == 0) profile::refresh();
      if (tick != nullptr && ++since_told >= kTellEvery) {
        since_told = 0;
        /* Finished share of each level's children by their likely size, scaled by the share of
           the child above that is being walked. */
        double frac = 0.0, weight = 1.0;
        std::vector<double> spent;
        for (size_t i = 0; i < stack.size(); ++i) {
          const std::vector<Node>& kids = stack[i].child;
          if (kids.empty()) break;
          spent.resize(kids.size());
          for (size_t j = 0; j < kids.size(); ++j) spent[j] = kids[j].frames;
          const std::vector<double> w = sizes(spent);
          const bool deeper = i + 1 < stack.size() && stack[i].next > 0;
          const size_t gone = deeper ? stack[i].next - 1 : stack[i].next;
          double total = 0.0, done = 0.0;
          for (size_t j = 0; j < w.size(); ++j) {
            total += w[j];
            if (j < gone) done += w[j];
          }
          frac += weight * done / total;
          if (!deeper) break;
          weight *= w[gone] / total;
        }
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;
        out->kept = closest.kept;
        if (!(*tick)(*out, static_cast<int>(frac * 1e6))) {
          state->paused = true;
          state->was_exhausted = out->exhausted;
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
      const bool near = Near::is(question, lands, child, child.distance, &by_allowance);
      /* Too short to record, but still listed below: a tie can splice it into a longer plan. */
      const bool short_of = child.depth < question.fewest || child.frames < question.least_frames;
      const bool could_be_closest =
          !near && !short_of &&
          (closest.kept.size() < kKeep || child.distance <= closest.kept.front().distance);
      if (near || could_be_closest) {
        profile::Timed recording(profile::kRecord);
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
          if (by_allowance && !short_of) ++out->count.allowed;
          /* Its states, for splicing ties onto (see `expand`). */
          if (out->listed.size() < cap) {
            Listed one;
            one.trail = trail_to(root_trail, stack, stack.size());
            one.end = c;
            list(out, one);
          }
          recording.stop();
          if (drove != nullptr && !short_of) (*drove)(c);
          if (given_away || short_of) {
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
      if (prof) profile::depth_expanded(child.depth);
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
      a->unlandable += b.unlandable;
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

  /* A kept run to carry on: its totals, and which walks it stopped. */
  struct Resumed {
    bool on = false;
    uint64_t items = 0, next = 0, finished = 0;
    double size_done = 0.0, seeded = 0.0;
    int shown = 0;
    Counters count;
    bool exhausted = true, full = false;
    long long held = 0;
    std::vector<Candidate> raw, kept;
    std::vector<uint64_t> paused;
  } resumed;
  if (!resume_dir.empty()) {
    checkpoint::Reader r(run_file(resume_dir));
    const bool head = r.pod<uint32_t>() == kRunMagic && r.pod<uint32_t>() == checkpoint::kVersion &&
                      same_layout(r);
    const std::string identity = head ? r.text() : std::string();
    const uint64_t slots = r.pod<uint64_t>();
    resumed.items = r.pod<uint64_t>();
    resumed.next = r.pod<uint64_t>();
    resumed.finished = r.pod<uint64_t>();
    resumed.size_done = r.pod<double>();
    resumed.shown = r.pod<int>();
    resumed.seeded = r.pod<double>();
    resumed.count = r.pod<Counters>();
    resumed.exhausted = r.pod<bool>();
    resumed.held = r.pod<long long>();
    resumed.full = r.pod<bool>();
    const bool lists = get(r, &resumed.raw) && get(r, &resumed.kept);
    r.pods(&resumed.paused);
    std::string why;
    if (!head || !lists || !r.ok()) {
      why = "the kept run will not read";
    } else if (identity != pausing->identity) {
      why = "the kept run is another question";
    } else if (slots != found.quanta.slots) {
      why = "the kept run's dominance tables are another size";
    }
    if (!why.empty()) {
      pausing->refused = why;
      found.exhausted = false;
      return found;
    }
    resumed.on = true;
  }

  Seen seed;
  Reaped prefix;
  int split = 1;
  for (;;) {
    seed.ready(found.quanta.slots);
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
      if (question.fewest == 0 && question.least_frames <= 0 && Near::is(question, lands, start, here.distance, &by_allowance)) {
        if (by_allowance) ++prefix.count.allowed;
        if (handing != nullptr) (*handing)(here);
        prefix.raw.push_back(here);
      }
    }

    /* The split reports no progress. */
    prefix.serial = 1;
    prefix.next_id = uint64_t(1) << 40;
    prefix.splitting = true;
    Walking splitting;
    walk(start, Trail(), seed, split, kMostCandidates, nullptr, handing, &prefix, &splitting);

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
  if (resumed.on) {
    if (resumed.items != prefix.item.size()) {
      pausing->refused = "the kept run split into other work items";
      found.exhausted = false;
      return found;
    }
    raw = resumed.raw;
    kept = resumed.kept;
    found.count = resumed.count;
    found.exhausted = resumed.exhausted;
    held_bytes.store(resumed.held);
    memory_full.store(resumed.full);
  }

  /* The items are merged order-free, so any core count answers the same list. */
  if (!prefix.item.empty()) {
    const std::vector<Item>& item = prefix.item;
    /* The whole cap, not a share: the run-level ceiling is enforced on the shared list. */
    const size_t cap_each = kMostCandidates;

    std::atomic<size_t> next(resumed.on ? static_cast<size_t>(resumed.next) : 0);
    size_t finished = resumed.on ? static_cast<size_t>(resumed.finished) : 0;
    /* Each item's likely size, so the run's share is the work done rather than the items. */
    std::vector<double> item_size;
    {
      std::vector<double> spent(item.size());
      for (size_t i = 0; i < item.size(); ++i) spent[i] = item[i].node.frames;
      item_size = sizes(spent);
    }
    double size_total = 0.0, size_done = resumed.on ? resumed.size_done : 0.0;
    for (size_t i = 0; i < item_size.size(); ++i) size_total += item_size[i];
    /* Progress shown, held monotone across threads. */
    int shown = resumed.on ? resumed.shown : 0;
    if (resumed.on) seeded = resumed.seeded;

    int hands = question.cores;
    if (hands < 1) hands = 1;
    if (static_cast<size_t>(hands) > item.size()) hands = static_cast<int>(item.size());

    /* Dives: walk the first `kDive` states of the `kDiveItems` nearest items, each on a fresh copy
       of the seed table and handing nothing over, to seed `seeded` for the steps bound. Same on
       any core count. Skipped for the recall audit. */
    if (question.distance_bound && instead == nullptr && !resumed.on) {
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
        profile::enter("dive");
        struct Leaving {
          ~Leaving() { profile::leave(); }
        } leaving;
        Seen mine;
        const std::function<bool(const Reaped&, int)> enough =
            [&](const Reaped& r, int) -> bool { return r.count.generated < kDive; };
        for (;;) {
          const size_t d = dive_next.fetch_add(1);
          if (d >= dived.size()) return;
          const size_t i = dived[d];
          {
            profile::Timed copying(profile::kItemSetup);
            mine = seed;
          }
          Reaped got;
          got.serial = static_cast<uint64_t>(i) + 2;
          got.next_id = got.serial << 40;
          Walking diving;
          walk(item[i].node, item[i].trail, mine, deepest + 1, kMostCandidates, &enough, nullptr,
               &got, &diving);
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
    /* Each slot's item, or -1, and its share of it in millionths. */
    std::vector<long long> flight_item(static_cast<size_t>(hands), -1);
    std::vector<int> flight_through(static_cast<size_t>(hands), 0);

    /* Walks stopped and kept, with what they had found, folded into the report only after the
       run is written so a resume does not count them twice. */
    std::vector<uint64_t> paused_items;
    std::vector<Counters> paused_count;
    std::vector<Candidate> paused_raw, paused_kept;
    bool keep_failed = false;
    std::atomic<size_t> resumed_next(0);

    /* One report, copied under the gate so the watcher, which writes to the window and drives the
       engine, runs without it: a slow reader must not hold up the walkers. */
    struct Report {
      Counters live;
      int shown = 0;
      size_t candidates = 0;
      bool closest_wanted = false;
      std::vector<Candidate> near_miss;
    };
    /* Called with the gate held. */
    auto gather = [&](Report* r) {
      double got = size_done;
      for (size_t h = 0; h < flight_item.size(); ++h) {
        if (flight_item[h] < 0) continue;
        got += item_size[static_cast<size_t>(flight_item[h])] * flight_through[h] / 1e6;
      }
      long long part = size_total > 0.0 ? static_cast<long long>(got / size_total * 1e6) : 0;
      if (part > 1000000) part = 1000000;
      if (part > shown) shown = static_cast<int>(part);
      r->shown = shown;
      r->live = found.count;
      for (size_t h = 0; h < flight.size(); ++h) Add::into(&r->live, flight[h]);
      r->live.held_bytes = held_bytes.load();
      r->live.memory_full = memory_full.load();
      r->candidates = raw.size();
      r->closest_wanted = watch->wants_so_far();
      if (r->closest_wanted) {
        r->near_miss = kept;
        for (size_t h = 0; h < flight_kept.size(); ++h) {
          r->near_miss.insert(r->near_miss.end(), flight_kept[h].begin(), flight_kept[h].end());
        }
      }
    };
    /* Report the whole run, without the gate. False means stop. */
    auto tell = [&](const Report& r) -> bool {
      /* A stopped run reports nothing more. */
      if (pressed.load()) return false;
      if (!watch->walked(r.live, r.shown, r.candidates)) {
        pressed.store(true);
        return false;
      }
      /* Only the closest; candidates already went to `Watching::found`. */
      if (r.closest_wanted) {
        Found sofar;
        sofar.count = r.live;
        sofar.quanta = found.quanta;
        sofar.unstepped = found.unstepped;
        sofar.rate = found.rate;
        sofar.slack = found.slack;
        sofar.exhausted = false;
        sofar.closest = Shape::twenty(r.near_miss, kKeep);
        sofar.count.kept_closest = static_cast<long long>(sofar.closest.size());
        watch->so_far(sofar);
      }
      return true;
    };

    auto worker = [&](size_t me) {
      profile::enter("walk");
      struct Leaving {
        ~Leaving() { profile::leave(); }
      } leaving;
      Seen mine;
      /* How much of this walk's raw list is already on the run's. */
      size_t published = 0;

      /* Publishes this walk's progress; the calling thread does the reporting. */
      const std::function<bool(const Reaped&, int)> beat =
          [&](const Reaped& walking, int through) -> bool {
        const uint64_t asked = profile::on() ? profile::cycles() : 0;
        std::lock_guard<std::mutex> lock(gate);
        if (asked != 0) profile::add(profile::kReportWait, profile::cycles() - asked);
        if (pressed.load()) return false;
        flight_through[me] = through < 0 ? 0 : (through > 1000000 ? 1000000 : through);
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
        Walking state;
        Reaped got;
        size_t i = 0;
        const size_t r = resumed.on ? resumed_next.fetch_add(1) : resumed.paused.size();
        if (r < resumed.paused.size()) {
          i = static_cast<size_t>(resumed.paused[r]);
          profile::Timed copying(profile::kItemSetup);
          if (i >= item.size() || !load_walk(i, &mine, &state, &got, &published)) {
            std::lock_guard<std::mutex> lock(gate);
            if (pausing->refused.empty()) pausing->refused = "a kept walk will not read";
            found.exhausted = false;
            pressed.store(true);
            return;
          }
        } else {
          i = next.fetch_add(1);
          if (i >= item.size()) return;
          /* Each item's own dominance table, seeded from the split's. */
          {
            profile::Timed copying(profile::kItemSetup);
            mine = seed;
          }
          got.serial = static_cast<uint64_t>(i) + 2;
          got.next_id = got.serial << 40;
          published = 0;
        }
        {
          std::lock_guard<std::mutex> lock(gate);
          flight_item[me] = static_cast<long long>(i);
          flight_through[me] = 0;
        }
        walk(item[i].node, item[i].trail, mine, deepest + 1, cap_each, tick, handing, &got, &state);

        if (state.paused && !save_dir.empty() && pressed.load()) {
          const bool wrote = save_walk(i, mine, state, got, published);
          std::lock_guard<std::mutex> lock(gate);
          if (wrote) {
            paused_items.push_back(i);
          } else {
            keep_failed = true;
          }
          paused_count.push_back(got.count);
          paused_raw.insert(paused_raw.end(), got.raw.begin() + static_cast<long>(published),
                            got.raw.end());
          paused_kept.insert(paused_kept.end(), got.kept.begin(), got.kept.end());
          flight[me] = Counters();
          flight_kept[me].clear();
          flight_item[me] = -1;
          flight_through[me] = 0;
          continue;
        }

        std::lock_guard<std::mutex> lock(gate);
        Add::into(&found.count, got.count);
        raw.insert(raw.end(), got.raw.begin() + static_cast<long>(published), got.raw.end());
        kept.insert(kept.end(), got.kept.begin(), got.kept.end());
        if (!got.exhausted) found.exhausted = false;
        /* Cleared under the same lock as the fold, so nothing is missed or doubled. */
        flight[me] = Counters();
        flight_kept[me].clear();
        flight_item[me] = -1;
        flight_through[me] = 0;
        size_done += item_size[i];
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
        Report report;
        {
          std::lock_guard<std::mutex> lock(gate);
          if (pressed.load() || finished >= item.size()) break;
          profile::Timed holding(profile::kReportHold);
          gather(&report);
        }
        tell(report);
        profile::refresh();
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
      }
    }
    for (size_t h = 0; h < walking.size(); ++h) walking[h].join();

    /* The stopped run, written once every walk has kept its own: the totals without the walks in
       flight, which carry their own. */
    if (pressed.load() && !save_dir.empty() && !keep_failed && pausing->refused.empty()) {
      checkpoint::Writer w(run_file(save_dir));
      w.pod(kRunMagic);
      w.pod(checkpoint::kVersion);
      put_layout(w);
      w.text(pausing->identity);
      w.pod(static_cast<uint64_t>(found.quanta.slots));
      w.pod(static_cast<uint64_t>(item.size()));
      w.pod(static_cast<uint64_t>(std::min(next.load(), item.size())));
      w.pod(static_cast<uint64_t>(finished));
      w.pod(size_done);
      w.pod(shown);
      w.pod(seeded);
      w.pod(found.count);
      w.pod(found.exhausted);
      w.pod(held_bytes.load());
      w.pod(memory_full.load());
      put(w, raw);
      put(w, kept);
      std::sort(paused_items.begin(), paused_items.end());
      w.pods(paused_items);
      pausing->saved = w.close();
    }
    for (size_t k = 0; k < paused_count.size(); ++k) Add::into(&found.count, paused_count[k]);
    raw.insert(raw.end(), paused_raw.begin(), paused_raw.end());
    kept.insert(kept.end(), paused_kept.begin(), paused_kept.end());
  }

  if (pressed.load()) found.exhausted = false;
  found.count.held_bytes = held_bytes.load();
  found.count.memory_full = memory_full.load();
  Shape::into(&found, raw, Shape::twenty(kept, kKeep), 0);
  return found;
}

}  // namespace search
