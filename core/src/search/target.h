/* The goal: a set of places (a point, a point with a free axis, or a box), resolved against the
 * room's floor. The distance to a subset of a box is never less than to the box, and the set is
 * the goal, so the A* bound stays admissible. */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace search {

struct Grid;

/** The target resolved against the room's floor: every ground triangle clipped (Sutherland-Hodgman)
 *  to the region, so each kept polygon is coplanar with the floor it came from. */
struct Region {
  /** False until `resolve` has run; until then `Target` answers about its box or point. */
  bool resolved = false;
  /** The box every polygon was clipped to, widened by the slack. */
  double bx0 = 0, bx1 = 0, bz0 = 0, bz1 = 0;
  bool has_y = false;
  double by0 = 0, by1 = 0;

  /** Three doubles a vertex; polygon p holds vertices `at[p] .. at[p + 1])`. */
  std::vector<double> vert;
  std::vector<int> at;
  /** Whether each polygon has area. A zero-area piece (a segment) is measured to by its edges
   *  only; an inside test on it would answer zero from anywhere. */
  std::vector<uint8_t> solid;
  /** Zero means the run is declined. */
  int kept = 0;

  /* A uniform grid over the region, so `distance` tests only nearby polygons. */
  double cell = 0, origin_x = 0, origin_z = 0;
  int nx = 0, nz = 0;
  /** `(nx * nz) + 1` offsets into `in`; cell c holds `in[start[c] .. start[c + 1])`. */
  std::vector<int> start;
  std::vector<int> in;
  /** Per cell, the Chebyshev ring of the nearest occupied cell, and a polygon in it. */
  std::vector<int> ring;
  std::vector<int> ring_poly;

  /** Each polygon's y range, and the ranges merged as lo, hi pairs. */
  std::vector<double> poly_y0, poly_y1;
  std::vector<double> levels;
};

/** The bytes of an address, byte 0 the high byte. With a blank high byte the allowed values are
 *  separated octaves apart, so the mask is the accept test and the box is only the prune. */
struct Mask {
  /** False where the byte was left blank. */
  bool known[3][4] = {};
  uint8_t byte[3][4] = {};

  bool on(int axis) const {
    return known[axis][0] || known[axis][1] || known[axis][2] || known[axis][3];
  }
  bool any() const { return on(0) || on(1) || on(2); }

  /** Whether the typed bytes, read X, Y, Z as twelve bytes, are one unbroken run of at most
   *  `kMostRun`; `first` and `count` say where. */
  static constexpr int kMostRun = 4;
  bool run(int* first, int* count) const {
    int a = -1, b = -1;
    for (int i = 0; i < 12; ++i) {
      if (!known[i / 4][i % 4]) continue;
      if (a < 0) a = i;
      b = i;
    }
    if (a < 0) return false;
    for (int i = a; i <= b; ++i) {
      if (!known[i / 4][i % 4]) return false;
    }
    if (b - a + 1 > kMostRun) return false;
    *first = a;
    *count = b - a + 1;
    return true;
  }
};

/** The goal. The default is the origin as a point. */
struct Target {
  /** A box rather than a point. */
  bool ranged = false;

  /** False on a freed axis, which matches every value of it (points and ranges alike). */
  bool has_x = true;
  bool has_z = true;
  double x = 0;
  double z = 0;

  /* The range; the caller orders them. */
  double x0 = 0, x1 = 0, z0 = 0, z1 = 0;

  /** A list's rows as x, z pairs, each row a point of the goal. `ranged` is set and the range is
   *  their box, which the corridor is laid over; the region is only the rows. */
  std::vector<double> list;

  /* Only an address has a height. */
  bool has_y = false;
  double y0 = 0, y1 = 0;

  /** A range an address is cut to. It narrows the region only, not the corridor. Ordered by the
   *  caller. */
  bool has_within = false;
  double wx0 = 0, wx1 = 0, wz0 = 0, wz1 = 0;

  Mask mask;

  /** The measured thing's offset above and across from Link's feet; zero for Link himself. */
  double aim_lift = 0;
  double aim_reach = 0;
  /** Set by `resolve` for an address aimed at the item: the region is the floor Link stands on,
   *  widened by the reach, and answers are admitted on the item's bytes only. */
  bool from_feet = false;

  /** The values each axis's bytes allow, as sorted lo, hi pairs; empty on an untyped axis. */
  std::vector<double> allowed[3];

  /** Filled by `resolve`; until then distances are to the box. */
  Region ground;

  /** Distance to the nearest point of the goal. Zero inside it. */
  double distance(double px, double pz) const;

  /** Distance to the box the typed bytes span, ignoring the floor. */
  double hull_distance(double px, double pz) const;

  /** `distance` to the goal on the floor at height `py` only, infinity when none. Orders the walk,
   *  never prunes it. */
  double distance_here(double px, double py, double pz) const;

  /** The nearest point of the goal; a freed axis answers the place's own coordinate. */
  void nearest(double px, double pz, double* nx, double* nz) const;

  /** The least `distance` from anywhere in an xz box to the goal; zero where they meet. */
  double box_gap(double x_lo, double x_hi, double z_lo, double z_hi) const {
    double tx_lo = x, tx_hi = x, tz_lo = z, tz_hi = z;
    bool on_x = has_x, on_z = has_z;
    if (ground.resolved) {
      /* Every polygon is inside the index's box. */
      tx_lo = ground.origin_x;
      tx_hi = ground.origin_x + ground.cell * ground.nx;
      tz_lo = ground.origin_z;
      tz_hi = ground.origin_z + ground.cell * ground.nz;
      on_x = on_z = true;
    } else if (ranged) {
      tx_lo = x0;
      tx_hi = x1;
      tz_lo = z0;
      tz_hi = z1;
    }
    const double gx = on_x ? std::max(0.0, std::max(tx_lo - x_hi, x_lo - tx_hi)) : 0.0;
    const double gz = on_z ? std::max(0.0, std::max(tz_lo - z_hi, z_lo - tz_hi)) : 0.0;
    return std::sqrt(gx * gx + gz * gz);
  }

  /** A cheap lower bound on `distance`, for pruning; inline because the walk calls it per state. */
  double bound(double px, double pz) const {
    if (!ground.resolved) {
      return distance(px, pz);
    }
    /* Every polygon is inside the index's box. */
    const double x1 = ground.origin_x + ground.cell * ground.nx;
    const double z1 = ground.origin_z + ground.cell * ground.nz;
    const double dx = px < ground.origin_x ? ground.origin_x - px : (px > x1 ? px - x1 : 0.0);
    const double dz = pz < ground.origin_z ? ground.origin_z - pz : (pz > z1 ? pz - z1 : 0.0);
    return std::sqrt(dx * dx + dz * dz);
  }

  /** How far off the goal a place is in f32 steps, both axes summed. Units cannot answer a
   *  tolerance of zero, since equal f32 values differ as doubles. */
  long long off(double px, double pz) const;

  /** Whether every typed byte equals the same byte of the landing's f32. */
  bool holds_bytes(double px, double py, double pz) const;

  /** Distance from a height to the nearest height the Y bytes allow; zero when none typed. */
  double height_off(double py) const;

  /** For an address whose typed bytes are one run, the gap between the landing's bytes and the
   *  typed ones read as one unsigned number, high byte first; otherwise `off`. Exact. */
  double miss(double px, double py, double pz) const;

  /** Distance in units to the nearest values the typed bytes allow, all axes together. */
  double bytes_off(double px, double py, double pz) const;

  /** The goal's box, a freed axis taking the room's extent. `room` is `geom::Mesh::box`: min x,
   *  y, z then max x, y, z. */
  void extent(const float room[6], double* min_x, double* min_z, double* max_x,
              double* max_z) const;

  /** Whether a place is inside the `within` box widened by `slack`; true when there is none. */
  bool in_within(double px, double pz, double slack) const {
    return !has_within || (px >= wx0 - slack && px <= wx1 + slack &&
                           pz >= wz0 - slack && pz <= wz1 + slack);
  }
};

/** How many f32 values lie between two numbers, capped. Both are narrowed to f32 first. */
long long float_steps(double a, double b);

/** Clip the room's ground triangles (`Selection::ground`, candidates from the grid's buckets) to
 *  the target's region widened by `slack`, and build the region's index. `source` is
 *  `Selection::ground_source`; null means each triangle is its own.
 *
 *  @return false when nothing was kept; the caller declines the run. */
bool resolve(Target* target, const Grid& grid, const std::vector<float>& ground, double slack,
             const std::function<bool()>* stop = nullptr,
             const std::vector<float>* source = nullptr);

/** The facing a plan must end on: an arc from `a` sweeping positive by `span`. A filter, never a
 *  term in the bound. */
struct EndFacing {
  /** No facing asked for. */
  bool any = true;
  int a = 0;
  int span = 0;

  bool holds(int facing) const;
};

}  // namespace search
