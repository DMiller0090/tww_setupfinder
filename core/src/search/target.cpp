#include "target.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>

#include "corridor.h"
#include "grid.h"
#include "SSystem/SComponent/c_m3d.h"

namespace search {
namespace {

/** One axis of a box: zero between the two edges, the overshoot outside. */
double outside(double v, double lo, double hi) {
  if (v < lo) return lo - v;
  if (v > hi) return v - hi;
  return 0.0;
}

double onto(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

constexpr int kTurn = 65536;

/** How far a triangle may sit outside the Y the bytes allow and still count, covering f32 rounding
 *  of the vertices. Not measured; errs toward keeping. */
const double kHeightSlack = 1.0;

/** The least xz area, in square units, a polygon needs to have an inside (`Region::solid`). */
const double kLeastArea = 1e-6;

/* An address axis is a union of intervals, one per pair of high bytes (see `Mask`). */

struct Span {
  double lo, hi;
};

/** Past this many intervals on one axis the question is declined, not widened to the hull. */
const int kMostSpans = 4096;

/** The float four bytes are, high byte first. */
double float_of(const uint8_t b[4]) {
  const uint32_t bits = (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
                        (static_cast<uint32_t>(b[2]) << 8) | static_cast<uint32_t>(b[3]);
  float f = 0.0f;
  std::memcpy(&f, &bits, sizeof f);
  return static_cast<double>(f);
}

/** The intervals one axis's bytes allow inside `lo` to `hi`, widened by `widen`, clamped, sorted
 *  and merged. Blank low bytes are taken at their ends (a hull); blank high bytes never are.
 *
 *  @return false, with `out` empty, past `kMostSpans`. */
bool spans_of(const Mask& mask, int axis, double lo, double hi, double widen,
              std::vector<Span>* out) {
  out->clear();
  const bool* known = mask.known[axis];
  const uint8_t* fixed = mask.byte[axis];
  const int first0 = known[0] ? fixed[0] : 0, last0 = known[0] ? fixed[0] : 255;
  const int first1 = known[1] ? fixed[1] : 0, last1 = known[1] ? fixed[1] : 255;
  uint8_t b[4] = {0, 0, 0, 0};
  for (int v0 = first0; v0 <= last0; ++v0) {
    for (int v1 = first1; v1 <= last1; ++v1) {
      b[0] = static_cast<uint8_t>(v0);
      b[1] = static_cast<uint8_t>(v1);
      double a = std::numeric_limits<double>::infinity();
      double z = -std::numeric_limits<double>::infinity();
      for (int e2 = 0; e2 < 2; ++e2) {
        for (int e3 = 0; e3 < 2; ++e3) {
          b[2] = known[2] ? fixed[2] : static_cast<uint8_t>(e2 ? 255 : 0);
          b[3] = known[3] ? fixed[3] : static_cast<uint8_t>(e3 ? 255 : 0);
          const double v = float_of(b);
          if (!std::isfinite(v)) continue;
          if (v < a) a = v;
          if (v > z) z = v;
        }
      }
      if (a > z) continue;
      a -= widen;
      z += widen;
      if (z < lo || a > hi) continue;
      out->push_back({a < lo ? lo : a, z > hi ? hi : z});
    }
  }
  std::sort(out->begin(), out->end(),
            [](const Span& p, const Span& q) { return p.lo < q.lo; });
  size_t at = 0;
  for (size_t i = 1; i < out->size(); ++i) {
    if ((*out)[i].lo <= (*out)[at].hi) {
      if ((*out)[i].hi > (*out)[at].hi) (*out)[at].hi = (*out)[i].hi;
    } else {
      (*out)[++at] = (*out)[i];
    }
  }
  if (!out->empty()) out->resize(at + 1);
  /* Counted after the merge: touching runs tile into one. */
  if (static_cast<int>(out->size()) > kMostSpans) {
    out->clear();
    return false;
  }
  return true;
}

/** Whether a polygon reaches into an interval on one axis. */
bool reaches(const std::vector<double>& poly, int axis, const Span& s) {
  const size_t n = poly.size() / 3;
  if (n == 0) return false;
  double lo = poly[static_cast<size_t>(axis)], hi = lo;
  for (size_t i = 1; i < n; ++i) {
    const double v = poly[i * 3 + static_cast<size_t>(axis)];
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  return hi >= s.lo && lo <= s.hi;
}

/** The most cells a side of the region's index, capped so a query's ring count is bounded
 *  whatever the room. */
const int kIndexSide = 32;

/** The squared xz distance from a place to a convex polygon, and its nearest point; zero inside.
 *  A polygon that is not `solid` is measured by its edges only. */
double poly_dist2(const double* v, int n, bool solid, double px, double pz, double* cx,
                  double* cz) {
  /* Inside when every cross product agrees in sign; the winding can be either way. */
  if (solid) {
    bool neg = false, pos = false;
    for (int i = 0; i < n; ++i) {
      const int j = (i + 1) % n;
      const double ex = v[j * 3] - v[i * 3], ez = v[j * 3 + 2] - v[i * 3 + 2];
      const double side = ex * (pz - v[i * 3 + 2]) - ez * (px - v[i * 3]);
      if (side > 0.0) pos = true;
      if (side < 0.0) neg = true;
    }
    if (!(pos && neg)) {
      *cx = px;
      *cz = pz;
      return 0.0;
    }
  }

  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double ax = v[i * 3], az = v[i * 3 + 2];
    const double ex = v[j * 3] - ax, ez = v[j * 3 + 2] - az;
    const double len2 = ex * ex + ez * ez;
    double t = 0.0;
    if (len2 > 0.0) {
      t = ((px - ax) * ex + (pz - az) * ez) / len2;
      t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    }
    const double qx = ax + ex * t, qz = az + ez * t;
    const double dx = px - qx, dz = pz - qz;
    const double d = dx * dx + dz * dz;
    if (d < best) {
      best = d;
      *cx = qx;
      *cz = qz;
    }
  }
  return best;
}

/** Sutherland-Hodgman against one axis-aligned half-space: keep where `axis` (0, 1 or 2) is at
 *  least `at`, or at most it. */
void clip_half(std::vector<double>* poly, std::vector<double>* work, int axis, double at,
               bool keep_above) {
  const size_t n = poly->size() / 3;
  if (n == 0) return;
  work->clear();
  const double* v = poly->data();
  for (size_t i = 0; i < n; ++i) {
    const size_t j = (i + 1) % n;
    const double* a = v + i * 3;
    const double* b = v + j * 3;
    const double da = keep_above ? a[axis] - at : at - a[axis];
    const double db = keep_above ? b[axis] - at : at - b[axis];
    const bool in_a = da >= 0.0, in_b = db >= 0.0;
    if (in_a) work->insert(work->end(), a, a + 3);
    if (in_a != in_b) {
      const double t = da / (da - db);
      for (int k = 0; k < 3; ++k) work->push_back(a[k] + (b[k] - a[k]) * t);
      /* Snapped onto the plane, so a zero-thickness region's second clip at the same `at` keeps
         it. */
      (*work)[work->size() - 3 + static_cast<size_t>(axis)] = at;
    }
  }
  poly->swap(*work);
}

/** The signed area in xz, doubled. */
double area2_xz(const std::vector<double>& poly) {
  const size_t n = poly.size() / 3;
  double a = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const size_t j = (i + 1) % n;
    a += poly[i * 3] * poly[j * 3 + 2] - poly[j * 3] * poly[i * 3 + 2];
  }
  return a;
}

}  // namespace

/* The exact f32 places on a triangle whose game height (`cM3dGPla::getCrossY_NonIsZero` on a
   `cM3d_CalcPla` plane) lands on the bytes. Rows along one axis; along a row the height is
   monotonic, so the fitting places are one run found by bisection. The ground check
   (`cM3d_CrossY_Tri_Front`, edge cross products down to -20) reaches past a triangle's edges, and
   the highest passing plane wins. */
namespace {

/** How much wider than the game's -20 the reach is cut, per unit of edge and box size, for f32
 *  rounding; every place in it is then held to the game's own test. */
const double kReachFuzz = 1e-6;

/** A neighbour higher by at most this is surely the floor, from any arrival (the probe sits 60
 *  above the unsnapped feet). */
const double kSurelyAbove = 1.0;

/** The widest overlap, in floats of a row, asked one float at a time rather than from the planes. */
const long long kExactAcross = 4096;

/** A 2D convex polygon in (x, z) pairs, cut to where `ez * (x - ax) - ex * (z - az) + slack` is
 *  not negative. */
void clip_edge(std::vector<double>* poly, std::vector<double>* work, double ax, double az,
               double ex, double ez, double slack) {
  const size_t n = poly->size() / 2;
  if (n == 0) return;
  work->clear();
  const double* v = poly->data();
  const auto g = [&](const double* p) { return ez * (p[0] - ax) - ex * (p[1] - az) + slack; };
  for (size_t i = 0; i < n; ++i) {
    const double* a = v + i * 2;
    const double* b = v + ((i + 1) % n) * 2;
    const double ga = g(a), gb = g(b);
    if (ga >= 0.0) work->insert(work->end(), a, a + 2);
    if ((ga >= 0.0) != (gb >= 0.0)) {
      const double t = ga / (ga - gb);
      work->push_back(a[0] + (b[0] - a[0]) * t);
      work->push_back(a[1] + (b[1] - a[1]) * t);
    }
  }
  poly->swap(*work);
}

/** A triangle's plane in double. */
struct Level {
  double x0, y0, z0, nx, ny, nz;
  double at(double x, double z) const { return y0 - (nx * (x - x0) + nz * (z - z0)) / ny; }
};
Level level_of(const float* f) {
  const double ux = f[3] - f[0], uy = f[4] - f[1], uz = f[5] - f[2];
  const double vx = f[6] - f[0], vy = f[7] - f[1], vz = f[8] - f[2];
  return {f[0], f[1], f[2], uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx};
}

/** Where the ground check reaches a triangle, as a polygon on its plane: its xz box cut by each
 *  edge moved out to -20. A vertical triangle is kept as itself. */
std::vector<double> reach_of(const float* f) {
  const Level lv = level_of(f);
  if (lv.ny == 0.0) return std::vector<double>(f, f + 9);
  double bx0 = f[0], bx1 = f[0], bz0 = f[2], bz1 = f[2];
  for (int i = 1; i < 3; ++i) {
    bx0 = std::min(bx0, static_cast<double>(f[i * 3]));
    bx1 = std::max(bx1, static_cast<double>(f[i * 3]));
    bz0 = std::min(bz0, static_cast<double>(f[i * 3 + 2]));
    bz1 = std::max(bz1, static_cast<double>(f[i * 3 + 2]));
  }
  std::vector<double> flat = {bx0, bz0, bx1, bz0, bx1, bz1, bx0, bz1}, work;
  const double size = (bx1 - bx0) + (bz1 - bz0) + 1.0;
  for (int e = 0; e < 3 && !flat.empty(); ++e) {
    const float* a = f + e * 3;
    const float* b = f + ((e + 1) % 3) * 3;
    const double ex = static_cast<double>(b[0]) - a[0], ez = static_cast<double>(b[2]) - a[2];
    const double slack = 20.0 + kReachFuzz * (std::fabs(ex) + std::fabs(ez)) * size;
    clip_edge(&flat, &work, a[0], a[2], ex, ez, slack);
  }
  std::vector<double> out;
  for (size_t i = 0; i + 1 < flat.size(); i += 2) {
    out.push_back(flat[i]);
    out.push_back(lv.at(flat[i], flat[i + 1]));
    out.push_back(flat[i + 1]);
  }
  return out;
}

/** Where row `r` on axis `ra` crosses a convex polygon, as an interval on axis `sa`. */
bool row_cross(const std::vector<double>& poly, int ra, int sa, double r, double* lo, double* hi) {
  const size_t nv = poly.size() / 3;
  double s_lo = 1e300, s_hi = -1e300;
  for (size_t i = 0; i < nv; ++i) {
    const size_t j = (i + 1) % nv;
    const double r0 = poly[i * 3 + ra], r1 = poly[j * 3 + ra];
    const double s0 = poly[i * 3 + sa], s1 = poly[j * 3 + sa];
    if ((r < std::min(r0, r1)) || (r > std::max(r0, r1))) continue;
    if (r0 == r1) {
      s_lo = std::min(s_lo, std::min(s0, s1));
      s_hi = std::max(s_hi, std::max(s0, s1));
      continue;
    }
    const double t = (r - r0) / (r1 - r0);
    const double sv = s0 + (s1 - s0) * t;
    s_lo = std::min(s_lo, sv);
    s_hi = std::max(s_hi, sv);
  }
  *lo = s_lo;
  *hi = s_hi;
  return s_lo <= s_hi;
}

/** The game's f32 feet height on a plane. */
float height_on(const Vec& n, float d, float x, float z) {
  return (-n.x * x - n.z * z - d) / n.y;
}

/** A float's place in the order of all floats, and back. */
long long rank_of(float f) {
  uint32_t b;
  std::memcpy(&b, &f, sizeof b);
  return (b & 0x80000000u) ? -static_cast<long long>(b & 0x7FFFFFFFu) : static_cast<long long>(b);
}
float float_at(long long r) {
  const uint32_t b = r < 0 ? (static_cast<uint32_t>(-r) | 0x80000000u) : static_cast<uint32_t>(r);
  float f;
  std::memcpy(&f, &b, sizeof f);
  return f;
}

float up_to_float(double v) {
  float f = static_cast<float>(v);
  if (static_cast<double>(f) < v) f = std::nextafter(f, std::numeric_limits<float>::infinity());
  return f;
}
float down_to_float(double v) {
  float f = static_cast<float>(v);
  if (static_cast<double>(f) > v) f = std::nextafter(f, -std::numeric_limits<float>::infinity());
  return f;
}

/** A ground triangle whose reach overlaps the one being walked. */
struct Beside {
  Vec p0, p1, p2, n;
  f32 d;
  Level level;
  const std::vector<double>* reach;
};

Beside beside_of(const float* f, const std::vector<double>* reach) {
  Beside b;
  b.p0 = {f[0], f[1], f[2]};
  b.p1 = {f[3], f[4], f[5]};
  b.p2 = {f[6], f[7], f[8]};
  cM3d_CalcPla(&b.p0, &b.p1, &b.p2, &b.n, &b.d);
  b.level = level_of(f);
  b.reach = reach;
  return b;
}

}  // namespace

void exact_places(const Target& target, const float* f, const std::vector<double>& piece,
                  const std::vector<Beside>& beside, double aim_lo, double aim_hi,
                  const std::function<void(const std::vector<double>&)>& take,
                  const std::function<bool()>* stop) {
  const Vec p0 = {f[0], f[1], f[2]}, p1 = {f[3], f[4], f[5]}, p2 = {f[6], f[7], f[8]};
  Vec n;
  f32 d = 0.0f;
  cM3d_CalcPla(&p0, &p1, &p2, &n, &d);
  if (n.y == 0.0f || piece.size() < 9) return;
  const Level own = level_of(f);
  /* The game stands Link on this triangle here: its test passes and no passing neighbour is
     higher by up to `kSurelyAbove`. */
  const auto stands = [&](float x, float z) {
    const Vec pos = {x, 0.0f, z};
    if (!cM3d_CrossY_Tri_Front(p0, p1, p2, &pos)) return false;
    const float h = height_on(n, d, x, z);
    for (const Beside& b : beside) {
      if (!cM3d_CrossY_Tri_Front(b.p0, b.p1, b.p2, &pos)) continue;
      const float hb = height_on(b.n, b.d, x, z);
      if (hb > h && static_cast<double>(hb) - h <= kSurelyAbove) return false;
    }
    return true;
  };
  const float lift = static_cast<float>(target.aim_lift);
  const auto aim_at = [&](float x, float z) { return height_on(n, d, x, z) + lift; };
  /* A blank low byte beside a typed one makes the span a comb, so the span alone is not the test. */
  const auto bytes_hold = [&](float x, float z) {
    const float aim = aim_at(x, z);
    if (target.from_feet) {
      uint32_t bits = 0;
      std::memcpy(&bits, &aim, sizeof bits);
      for (int i = 0; i < 4; ++i) {
        if (target.mask.known[1][i] && ((bits >> (24 - 8 * i)) & 0xFFu) != target.mask.byte[1][i]) {
          return false;
        }
      }
      return true;
    }
    return target.holds_bytes(x, aim, z);
  };

  double ax0 = 1e300, ax1 = -1e300, az0 = 1e300, az1 = -1e300;
  for (size_t i = 0; i < piece.size(); i += 3) {
    ax0 = std::min(ax0, piece[i]); ax1 = std::max(ax1, piece[i]);
    az0 = std::min(az0, piece[i + 2]); az1 = std::max(az1, piece[i + 2]);
  }

  /* A horizontal axis with its high byte blank and a low one typed is a comb; it becomes the rows,
     visiting only matching floats. Not when aimed at the item. */
  int comb = -1;
  uint32_t comb_mask = 0, comb_bits = 0;
  if (!target.from_feet) {
    for (int a = 0; a < 3; a += 2) {
      if (!target.mask.on(a) || target.mask.known[a][0]) continue;
      comb = a;
      for (int i = 0; i < 4; ++i) {
        if (!target.mask.known[a][i]) continue;
        comb_mask |= 0xFFu << (24 - 8 * i);
        comb_bits |= static_cast<uint32_t>(target.mask.byte[a][i]) << (24 - 8 * i);
      }
    }
  }
  /* Negative floats rank by falling magnitude, so there the match is searched downward. */
  const uint64_t M = comb_mask, P = comb_bits, F = 0xFFFFFFFFull & ~M;
  /* The smallest pattern at or above `v` that holds the bytes, or 2^32 when there is none. */
  const auto up_match = [&](uint64_t v) -> uint64_t {
    const uint64_t t = (v & F) | P;
    if (t >= v) return t;
    uint64_t h = 1ull << 31;
    while (!((v ^ t) & h)) h >>= 1;
    const uint64_t low = (h << 1) - 1;
    uint64_t inc = 0;
    for (uint64_t bit = h << 1; bit <= (1ull << 31); bit <<= 1) {
      if (F & bit) { inc = bit; break; }
    }
    if (inc == 0) return 1ull << 32;
    const uint64_t above = (((v & F & ~low) | (M & ~low)) + inc) & F & ~low;
    if (above >> 32) return 1ull << 32;
    return above | P;
  };
  /* The largest pattern at or below `v` that holds the bytes, or 2^32 when there is none. */
  const auto down_match = [&](uint64_t v) -> uint64_t {
    const uint64_t t = (v & F) | P;
    if (t <= v) return t;
    uint64_t h = 1ull << 31;
    while (!((v ^ t) & h)) h >>= 1;
    const uint64_t low = (h << 1) - 1;
    uint64_t dec = 0;
    for (uint64_t bit = h << 1; bit <= (1ull << 31); bit <<= 1) {
      if (F & bit) { dec = bit; break; }
    }
    const uint64_t above = v & F & ~low;
    if (dec == 0 || above < dec) return 1ull << 32;
    return (((above - dec) & F & ~low) | (F & low) | P);
  };
  /* The first rank at or after `rr` whose float holds the comb's bytes. */
  const auto next_row = [&](long long rr) -> long long {
    if (comb < 0) return rr;
    if (rr < 0) {
      const uint64_t c = down_match(static_cast<uint64_t>(-rr));
      if (c < (1ull << 31)) return -static_cast<long long>(c);
      rr = 0;
    }
    const uint64_t c = up_match(static_cast<uint64_t>(rr));
    return c >= (1ull << 31) ? (1LL << 40) : static_cast<long long>(c);
  };

  /* A window many height steps wide fits everywhere in its band, so the piece is cut flat at the
     two heights. */
  const double big = std::max({std::fabs(n.x * ax0), std::fabs(n.x * ax1), std::fabs(n.z * az0),
                               std::fabs(n.z * az1), std::fabs(static_cast<double>(d))});
  const double step = std::ldexp(1.0, std::ilogb(big > 0.0 ? big : 1.0) - 23) /
                      std::fabs(static_cast<double>(n.y));
  /* The f32 height is off the double plane by up to a step, so the band is cut a few steps wider;
     the engine still holds every plan to the bytes. */
  const double band_slack = 4.0 * step;
  const auto band_of = [&]() {
    std::vector<double> band = piece, work;
    if (std::isfinite(aim_lo)) clip_half(&band, &work, 1, aim_lo - lift - band_slack, true);
    if (std::isfinite(aim_hi)) clip_half(&band, &work, 1, aim_hi - lift + band_slack, false);
    if (band.size() >= 6) take(band);
  };
  if (comb < 0 && aim_hi - aim_lo > 8.0 * step) {
    band_of();
    return;
  }

  /* A level triangle with no comb is one height, so it fits everywhere or nowhere. */
  if (n.x == 0.0f && n.z == 0.0f && comb < 0) {
    const float cx = static_cast<float>((ax0 + ax1) / 2), cz = static_cast<float>((az0 + az1) / 2);
    const double h = aim_at(cx, cz);
    if (h >= aim_lo && h <= aim_hi && bytes_hold(cx, cz)) take(piece);
    return;
  }

  /* Rows on the comb, else on the axis with coarser floats here. */
  const bool rows_on_z = comb >= 0 ? comb == 2
                                   : std::max(std::fabs(az0), std::fabs(az1)) >=
                                         std::max(std::fabs(ax0), std::fabs(ax1));
  const int ra = rows_on_z ? 2 : 0, sa = rows_on_z ? 0 : 2;

  std::vector<double> run;
  const long long r_first = rank_of(up_to_float(rows_on_z ? az0 : ax0));
  const long long r_last = rank_of(down_to_float(rows_on_z ? az1 : ax1));
  long long visited = 0;
  /* Too many rows to walk falls back to the band, never less. */
  {
    const long long span = r_last - r_first;
    const long long rows = comb >= 0 ? span / (static_cast<long long>(comb_mask) + 1) + 1 : span;
    if (rows > 4000000) {
      band_of();
      return;
    }
  }
  for (long long rr = next_row(r_first); rr <= r_last; rr = next_row(rr + 1), ++visited) {
    if (stop != nullptr && (visited & 4095) == 0 && (*stop)()) return;
    const float r = float_at(rr);
    double s_lo = 0.0, s_hi = 0.0;
    if (!row_cross(piece, ra, sa, r, &s_lo, &s_hi)) continue;
    const long long a = rank_of(up_to_float(s_lo)), b = rank_of(down_to_float(s_hi));
    if (a > b) continue;
    const auto at = [&](long long k) {
      const float sv = float_at(k);
      return rows_on_z ? aim_at(sv, r) : aim_at(r, sv);
    };
    const bool rising = at(b) >= at(a);
    const auto below = [&](long long k) { return static_cast<double>(at(k)) < aim_lo; };
    const auto above = [&](long long k) { return static_cast<double>(at(k)) > aim_hi; };
    const auto first_not = [&](const std::function<bool(long long)>& before) {
      long long lo = a, hi = b + 1;
      while (lo < hi) {
        const long long m = lo + (hi - lo) / 2;
        if (before(m)) lo = m + 1; else hi = m;
      }
      return lo;
    };
    long long from, to;
    if (rising) {
      from = first_not(below);
      to = first_not([&](long long k) { return !above(k); }) - 1;
    } else {
      from = first_not(above);
      to = first_not([&](long long k) { return !below(k); }) - 1;
    }
    if (from > to) continue;
    const auto emit = [&](long long k0, long long k1) {
      const float s0 = float_at(k0), s1 = float_at(k1);
      const float x0 = rows_on_z ? s0 : r, z0 = rows_on_z ? r : s0;
      const float x1 = rows_on_z ? s1 : r, z1 = rows_on_z ? r : s1;
      run.assign({x0, static_cast<double>(height_on(n, d, x0, z0)), z0,
                  x1, static_cast<double>(height_on(n, d, x1, z1)), z1});
      take(run);
    };
    const float fs = float_at(from), ts = float_at(to);
    const bool ends_hold = rows_on_z ? (bytes_hold(fs, r) && bytes_hold(ts, r))
                                     : (bytes_hold(r, fs) && bytes_hold(r, ts));
    /* A short run is held to the game one float at a time. */
    if ((to - from) < 64) {
      /* A separate flag: every rank is a valid value, so none can mean "none". */
      bool is_open = false;
      long long open = 0;
      for (long long k = from; k <= to; ++k) {
        const float sv = float_at(k);
        const float x = rows_on_z ? sv : r, z = rows_on_z ? r : sv;
        const bool ok = bytes_hold(x, z) && stands(x, z);
        if (ok && !is_open) { open = k; is_open = true; }
        if (!ok && is_open) { emit(open, k - 1); is_open = false; }
      }
      if (is_open) emit(open, to);
      continue;
    }
    if (!ends_hold) continue;
    /* A long run loses where a neighbour is surely the floor, from the planes. Within a few f32
       steps the choice is noise and the place is kept. */
    const double s_from = float_at(from), s_to = float_at(to);
    const auto f32_step = [&](const Vec& pn, float pd) {
      const double big = std::fabs(static_cast<double>(pd)) +
                         std::fabs(pn.x) * std::max(std::fabs(ax0), std::fabs(ax1)) +
                         std::fabs(pn.z) * std::max(std::fabs(az0), std::fabs(az1));
      return std::ldexp(1.0, std::ilogb(big > 0.0 ? big : 1.0) - 23) /
             std::fabs(static_cast<double>(pn.y));
    };
    const double own_step = f32_step(n, d);
    std::vector<std::pair<long long, long long> > lost;
    for (const Beside& bs : beside) {
      double blo = 0.0, bhi = 0.0;
      if (!row_cross(*bs.reach, ra, sa, r, &blo, &bhi)) continue;
      blo = std::max(blo, s_from);
      bhi = std::min(bhi, s_to);
      if (blo > bhi) continue;
      /* A narrow overlap (a crease) is asked float by float with the game's own test. */
      {
        const long long e0 = std::max(from, rank_of(down_to_float(blo)) - 8);
        const long long e1 = std::min(to, rank_of(up_to_float(bhi)) + 8);
        if (e1 - e0 <= kExactAcross) {
          bool in = false;
          long long start = 0;
          for (long long e = e0; e <= e1 + 1; ++e) {
            bool wins = false;
            if (e <= e1) {
              const float sv = float_at(e);
              const float x = rows_on_z ? sv : r, z = rows_on_z ? r : sv;
              const Vec pos = {x, 0.0f, z};
              if (cM3d_CrossY_Tri_Front(bs.p0, bs.p1, bs.p2, &pos)) {
                const float h = height_on(n, d, x, z), hb = height_on(bs.n, bs.d, x, z);
                wins = hb > h && static_cast<double>(hb) - h <= kSurelyAbove;
              }
            }
            if (wins && !in) { start = e; in = true; }
            if (!wins && in) { lost.push_back({start, e - 1}); in = false; }
          }
          continue;
        }
      }
      const auto rise = [&](double s) {
        const double x = rows_on_z ? s : r, z = rows_on_z ? r : s;
        return bs.level.at(x, z) - own.at(x, z);
      };
      const double noise = 4.0 * (own_step + f32_step(bs.n, bs.d));
      const double lo_rise = noise, hi_rise = kSurelyAbove - noise;
      if (lo_rise >= hi_rise) continue;
      const double r0 = rise(blo), r1 = rise(bhi);
      double t0 = 0.0, t1 = 1.0;
      if (r0 == r1) {
        if (r0 <= lo_rise || r0 > hi_rise) continue;
      } else {
        /* The part of [blo, bhi] where the rise is in (lo_rise, hi_rise]. */
        double ta = (lo_rise - r0) / (r1 - r0), tb = (hi_rise - r0) / (r1 - r0);
        if (ta > tb) std::swap(ta, tb);
        t0 = std::max(0.0, ta);
        t1 = std::min(1.0, tb);
        if (t0 > t1) continue;
      }
      /* Drawn in a few floats each end against rounding. */
      const long long k0 = rank_of(up_to_float(blo + (bhi - blo) * t0)) + 8;
      const long long k1 = rank_of(down_to_float(blo + (bhi - blo) * t1)) - 8;
      if (k0 <= k1) lost.push_back({k0, k1});
    }
    /* Each remaining piece is trimmed to the game's test by bisection from each end; the reach is
       convex, so the test passes on one stretch. */
    const auto front_at = [&](long long k) {
      const float sv = float_at(k);
      const Vec pos = {rows_on_z ? sv : r, 0.0f, rows_on_z ? r : sv};
      return cM3d_CrossY_Tri_Front(p0, p1, p2, &pos);
    };
    const auto emit_held = [&](long long k0, long long k1) {
      const long long mid = k0 + (k1 - k0) / 2;
      if (!front_at(mid)) return;
      long long lo = k0, hi = mid;
      while (lo < hi) {
        const long long m = lo + (hi - lo) / 2;
        if (front_at(m)) hi = m; else lo = m + 1;
      }
      const long long first = lo;
      lo = mid;
      hi = k1;
      while (lo < hi) {
        const long long m = lo + (hi - lo + 1) / 2;
        if (front_at(m)) lo = m; else hi = m - 1;
      }
      emit(first, lo);
    };
    std::sort(lost.begin(), lost.end());
    long long k = from;
    for (const auto& gap : lost) {
      if (gap.second < k) continue;
      if (gap.first > k) emit_held(k, std::min(to, gap.first - 1));
      k = gap.second + 1;
      if (k > to) break;
    }
    if (k <= to) emit_held(k, to);
  }
}

bool resolve(Target* target, const Grid& grid, const std::vector<float>& ground, double slack,
             const std::function<bool()>* stop, const std::vector<float>* source) {
  Region& kept = target->ground;
  kept = Region();
  if (grid.nx <= 0 || grid.nz <= 0 || grid.cell <= 0.0) return false;

  /* The grid stands in for the room: nothing outside it is reachable. */
  const float room[6] = {
      static_cast<float>(grid.origin_x), 0.0f, static_cast<float>(grid.origin_z),
      static_cast<float>(grid.origin_x + grid.cell * grid.nx), 0.0f,
      static_cast<float>(grid.origin_z + grid.cell * grid.nz)};
  double x0 = 0, z0 = 0, x1 = 0, z1 = 0;
  target->extent(room, &x0, &z0, &x1, &z1);
  /* Cut to the `within` box. An address wholly outside it takes the box itself, and the byte cut
     below keeps only what is within tolerance. */
  if (target->has_within) {
    const auto cut =[](double* lo, double* hi, double r0, double r1) {
      if (*hi < r0 || *lo > r1) { *lo = r0; *hi = r1; return; }
      *lo = std::max(*lo, r0);
      *hi = std::min(*hi, r1);
    };
    cut(&x0, &x1, target->wx0, target->wx1);
    cut(&z0, &z1, target->wz0, target->wz1);
  }
  /* Aimed at the item, the region is the floor Link can stand on, widened by the item's reach. */
  target->from_feet = target->mask.any() && target->aim_reach > 0.0;
  for (int a = 0; a < 3; ++a) {
    target->allowed[a].clear();
    if (!target->mask.on(a)) continue;
    const double most = std::numeric_limits<double>::max();
    std::vector<Span> each;
    if (!spans_of(target->mask, a, -most, most, 0.0, &each)) continue;
    for (const Span& one : each) {
      target->allowed[a].push_back(one.lo);
      target->allowed[a].push_back(one.hi);
    }
  }
  const double across = target->from_feet ? target->aim_reach : 0.0;
  x0 -= slack + across; x1 += slack + across; z0 -= slack + across; z1 += slack + across;

  kept.bx0 = x0; kept.bx1 = x1; kept.bz0 = z0; kept.bz1 = z1;
  kept.has_y = target->has_y;
  kept.by0 = target->y0 - kHeightSlack;
  kept.by1 = target->y1 + kHeightSlack;

  /* Clamped before the int cast: a box spanning the float range overflows an int. */
  const auto grid_column = [](double v, double origin, double cell, int n) {
    const double c = std::floor((v - origin) / cell);
    return static_cast<int>(std::max(-1.0, std::min(static_cast<double>(n), c)));
  };
  int ix0 = grid_column(x0, grid.origin_x, grid.cell, grid.nx);
  int ix1 = grid_column(x1, grid.origin_x, grid.cell, grid.nx);
  int iz0 = grid_column(z0, grid.origin_z, grid.cell, grid.nz);
  int iz1 = grid_column(z1, grid.origin_z, grid.cell, grid.nz);
  if (ix0 < 0) ix0 = 0;
  if (iz0 < 0) iz0 = 0;
  if (ix1 > grid.nx - 1) ix1 = grid.nx - 1;
  if (iz1 > grid.nz - 1) iz1 = grid.nz - 1;
  if (ix1 < ix0 || iz1 < iz0) return false;

  const int tri_count = static_cast<int>(ground.size() / 9);
  std::vector<uint8_t> seen(static_cast<size_t>(tri_count < 0 ? 0 : tri_count), 0);
  std::vector<double> tri, cut_x, cut_z, poly, work;
  double px0 = 0, px1 = 0, pz0 = 0, pz1 = 0;  // the polygons' own extent, which the index covers
  bool any = false;

  /* Each axis is a union of slabs (`Mask`); an untyped axis is one slab. */
  std::vector<Span> xs, zs, ys;
  /* At a tolerance of zero the bytes are asked exactly (`exact_places`), and a comb too fine for
     spans is cut to its hull instead of declined. */
  const bool at_zero = target->mask.any() && slack == 0.0;
  bool combed = false;
  if (target->mask.on(0)) {
    if (!spans_of(target->mask, 0, x0, x1, slack + across, &xs)) {
      if (!at_zero) { kept = Region(); return false; }
      xs.assign(1, {x0, x1});
      combed = true;
    }
  } else {
    xs.assign(1, {x0, x1});
  }
  if (target->mask.on(2)) {
    if (!spans_of(target->mask, 2, z0, z1, slack + across, &zs)) {
      if (!at_zero) { kept = Region(); return false; }
      zs.assign(1, {z0, z1});
      combed = true;
    }
  } else {
    zs.assign(1, {z0, z1});
  }
  const bool exact = at_zero && (target->mask.on(1) || combed);

  /* Cut whenever a Y byte is typed, even when the bytes are not a number and no hull came. */
  const bool cut_y = target->has_y || target->mask.on(1);

  if (cut_y) {
    /* The bytes are the aim's height; the floor sits `aim_lift` below it. */
    const double lift = target->aim_lift;
    double lo = kept.by0, hi = kept.by1;
    if (!target->has_y) {
      bool any_y = false;
      for (size_t i = 1; i < ground.size(); i += 3) {
        const double v = static_cast<double>(ground[i]);
        if (!any_y) { lo = hi = v; any_y = true; }
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
      if (!any_y) { kept = Region(); return false; }
      /* Two slacks: one for each span's widening and one so the exact walk's window, the span
         less a slack, still reaches below the lowest vertex. */
      lo += lift - 2.0 * kHeightSlack;
      hi += lift + 2.0 * kHeightSlack;
    }
    if (target->mask.on(1)) {
      if (!spans_of(target->mask, 1, lo, hi, kHeightSlack, &ys)) { kept = Region(); return false; }
    } else {
      ys.assign(1, {lo, hi});
    }
    for (Span& sy : ys) {
      sy.lo -= lift;
      sy.hi -= lift;
    }
    kept.has_y = true;
    kept.by0 = lo - lift;
    kept.by1 = hi - lift;
  }

  const auto take = [&](const std::vector<double>& piece) {
    if (piece.size() < 6) return;
    const size_t verts = piece.size() / 3;
    kept.solid.push_back(std::fabs(area2_xz(piece)) >= kLeastArea * 2.0 ? 1u : 0u);
    kept.at.push_back(static_cast<int>(kept.vert.size() / 3));
    kept.vert.insert(kept.vert.end(), piece.begin(), piece.end());
    double y0 = piece[1], y1 = piece[1];
    for (size_t i = 1; i < verts; ++i) {
      y0 = std::min(y0, piece[i * 3 + 1]);
      y1 = std::max(y1, piece[i * 3 + 1]);
    }
    kept.poly_y0.push_back(y0);
    kept.poly_y1.push_back(y1);
    ++kept.kept;
    for (size_t i = 0; i < verts; ++i) {
      const double vx = piece[i * 3], vz = piece[i * 3 + 2];
      if (!any) { px0 = px1 = vx; pz0 = pz1 = vz; any = true; }
      if (vx < px0) px0 = vx;
      if (vx > px1) px1 = vx;
      if (vz < pz0) pz0 = vz;
      if (vz > pz1) pz1 = vz;
    }
  };

  /* Reaches and neighbours, only in the exact case. A neighbour is a ground triangle whose xz box
     meets this one's; the game's test never reaches outside a triangle's own box. */
  /* The game's own triangle: the piece itself unless the sea cut it, in which case the source is
     cut to the piece's lowest vertex. */
  const auto src_of = [&](int k) -> const float* {
    const size_t at = static_cast<size_t>(k) * 9;
    return source != nullptr && source->size() >= at + 9 ? &(*source)[at] : &ground[at];
  };
  const auto was_cut = [&](int k) {
    return std::memcmp(src_of(k), &ground[static_cast<size_t>(k) * 9], 9 * sizeof(float)) != 0;
  };
  std::vector<std::vector<double> > reach_cache(exact ? static_cast<size_t>(tri_count) : 0);
  std::vector<uint8_t> reach_known(reach_cache.size(), 0);
  const auto reach_of_tri = [&](int k) -> const std::vector<double>& {
    if (!reach_known[static_cast<size_t>(k)]) {
      std::vector<double>& r = reach_cache[static_cast<size_t>(k)];
      r = reach_of(src_of(k));
      if (was_cut(k)) {
        const float* g = &ground[static_cast<size_t>(k) * 9];
        std::vector<double> scratch;
        clip_half(&r, &scratch, 1, std::min({g[1], g[4], g[7]}), true);
      }
      reach_known[static_cast<size_t>(k)] = 1;
    }
    return reach_cache[static_cast<size_t>(k)];
  };
  std::vector<std::array<float, 9> > walked_sources;
  std::vector<Beside> near;
  std::vector<int> near_stamp(reach_cache.size(), -1);
  const auto box_xz = [&](int k, float* bx) {
    const float* v = src_of(k);
    bx[0] = std::min({v[0], v[3], v[6]}); bx[1] = std::max({v[0], v[3], v[6]});
    bx[2] = std::min({v[2], v[5], v[8]}); bx[3] = std::max({v[2], v[5], v[8]});
  };
  const auto near_of = [&](int t) {
    near.clear();
    float own[4];
    box_xz(t, own);
    const auto col = [](double v, double origin, double cell, int n) {
      const int i = static_cast<int>(std::floor((v - origin) / cell));
      return std::max(0, std::min(n - 1, i));
    };
    const int cx0 = col(own[0], grid.origin_x, grid.cell, grid.nx);
    const int cx1 = col(own[1], grid.origin_x, grid.cell, grid.nx);
    const int cz0 = col(own[2], grid.origin_z, grid.cell, grid.nz);
    const int cz1 = col(own[3], grid.origin_z, grid.cell, grid.nz);
    for (int cz = cz0; cz <= cz1; ++cz) {
      for (int cx = cx0; cx <= cx1; ++cx) {
        const size_t c = static_cast<size_t>(grid.at(cx, cz));
        for (int k = grid.start[c]; k < grid.start[c + 1]; ++k) {
          const int o = grid.tri[static_cast<size_t>(k)];
          if (o < 0 || o >= tri_count || o == t || near_stamp[static_cast<size_t>(o)] == t) continue;
          near_stamp[static_cast<size_t>(o)] = t;
          float other[4];
          box_xz(o, other);
          if (other[1] < own[0] || other[0] > own[1] || other[3] < own[2] || other[2] > own[3]) {
            continue;
          }
          near.push_back(beside_of(src_of(o), &reach_of_tri(o)));
        }
      }
    }
  };

  if (!target->list.empty()) {
    /* Each row is its own point, clipped as a point target is; the box around them all is never
       a place. A triangle two rows reach is kept twice, which changes no distance. */
    const double w = slack + across;
    std::vector<int> stamp(seen.size(), -1);
    const auto clamp = [](int c, int n) { return std::max(0, std::min(n - 1, c)); };
    for (size_t r = 0; r + 1 < target->list.size(); r += 2) {
      if (stop != nullptr && (*stop)()) { kept = Region(); return false; }
      const Span sx{target->list[r] - w, target->list[r] + w};
      const Span sz{target->list[r + 1] - w, target->list[r + 1] + w};
      const int cx0 = clamp(grid_column(sx.lo, grid.origin_x, grid.cell, grid.nx), grid.nx);
      const int cx1 = clamp(grid_column(sx.hi, grid.origin_x, grid.cell, grid.nx), grid.nx);
      const int cz0 = clamp(grid_column(sz.lo, grid.origin_z, grid.cell, grid.nz), grid.nz);
      const int cz1 = clamp(grid_column(sz.hi, grid.origin_z, grid.cell, grid.nz), grid.nz);
      for (int iz = cz0; iz <= cz1; ++iz) {
        for (int ix = cx0; ix <= cx1; ++ix) {
          const size_t c = static_cast<size_t>(grid.at(ix, iz));
          if ((grid.mark[c] & kNoGround) != 0) continue;
          for (int k = grid.start[c]; k < grid.start[c + 1]; ++k) {
            const int t = grid.tri[static_cast<size_t>(k)];
            if (t < 0 || t >= tri_count || stamp[static_cast<size_t>(t)] == static_cast<int>(r)) {
              continue;
            }
            stamp[static_cast<size_t>(t)] = static_cast<int>(r);
            tri.assign(&ground[static_cast<size_t>(t) * 9], &ground[static_cast<size_t>(t) * 9] + 9);
            if (!reaches(tri, 0, sx)) continue;
            cut_x = tri;
            clip_half(&cut_x, &work, 0, sx.lo, true);
            clip_half(&cut_x, &work, 0, sx.hi, false);
            if (cut_x.size() < 6 || !reaches(cut_x, 2, sz)) continue;
            cut_z = cut_x;
            clip_half(&cut_z, &work, 2, sz.lo, true);
            clip_half(&cut_z, &work, 2, sz.hi, false);
            if (cut_z.size() >= 6) take(cut_z);
          }
        }
      }
    }
  } else {
    for (int iz = iz0; iz <= iz1; ++iz) {
      for (int ix = ix0; ix <= ix1; ++ix) {
        const size_t c = static_cast<size_t>(grid.at(ix, iz));
        if ((grid.mark[c] & kNoGround) != 0) continue;
        for (int k = grid.start[c]; k < grid.start[c + 1]; ++k) {
          const int t = grid.tri[static_cast<size_t>(k)];
          if (t < 0 || t >= tri_count || seen[static_cast<size_t>(t)]) continue;
          if (stop != nullptr && (*stop)()) { kept = Region(); return false; }
          seen[static_cast<size_t>(t)] = 1;

          const float* f = &ground[static_cast<size_t>(t) * 9];
          /* In the exact case the game's own triangle is cut as far as the ground check reaches. */
          if (exact) {
            if (was_cut(t)) {
              std::array<float, 9> key;
              std::memcpy(key.data(), src_of(t), sizeof(float) * 9);
              if (std::find(walked_sources.begin(), walked_sources.end(), key) != walked_sources.end()) {
                continue;
              }
              walked_sources.push_back(key);
            }
            f = src_of(t);
            tri = reach_of_tri(t);
          } else {
            tri.assign(f, f + 9);
          }
          bool near_known = false;

          for (const Span& sx : xs) {
            if (!reaches(tri, 0, sx)) continue;
            cut_x = tri;
            clip_half(&cut_x, &work, 0, sx.lo, true);
            clip_half(&cut_x, &work, 0, sx.hi, false);
            if (cut_x.size() < 6) continue;
            for (const Span& sz : zs) {
              if (!reaches(cut_x, 2, sz)) continue;
              cut_z = cut_x;
              clip_half(&cut_z, &work, 2, sz.lo, true);
              clip_half(&cut_z, &work, 2, sz.hi, false);
              /* Two vertices (a segment) is kept; only an empty clip is dropped. */
              if (cut_z.size() < 6) continue;
              if (exact && !near_known) {
                near_of(t);
                near_known = true;
              }
              if (exact && !cut_y) {
                exact_places(*target, f, cut_z, near, -std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(), take, stop);
              } else if (exact) {
                for (const Span& sy : ys) {
                  if (!reaches(cut_z, 1, sy)) continue;
                  poly = cut_z;
                  clip_half(&poly, &work, 1, sy.lo, true);
                  clip_half(&poly, &work, 1, sy.hi, false);
                  exact_places(*target, f, poly, near, sy.lo + kHeightSlack + target->aim_lift,
                               sy.hi - kHeightSlack + target->aim_lift, take, stop);
                }
              } else if (cut_y) {
                for (const Span& sy : ys) {
                  if (!reaches(cut_z, 1, sy)) continue;
                  poly = cut_z;
                  clip_half(&poly, &work, 1, sy.lo, true);
                  clip_half(&poly, &work, 1, sy.hi, false);
                  take(poly);
                }
              } else {
                take(cut_z);
              }
            }
          }
        }
      }
    }
  }

  if (kept.kept == 0) {
    kept = Region();
    return false;
  }
  kept.at.push_back(static_cast<int>(kept.vert.size() / 3));

  {
    std::vector<std::pair<double, double> > ys_of;
    for (size_t p = 0; p < kept.poly_y0.size(); ++p) ys_of.push_back({kept.poly_y0[p], kept.poly_y1[p]});
    std::sort(ys_of.begin(), ys_of.end());
    for (const auto& y : ys_of) {
      const size_t n = kept.levels.size();
      if (n > 0 && y.first <= kept.levels[n - 1]) {
        kept.levels[n - 1] = std::max(kept.levels[n - 1], y.second);
      } else {
        kept.levels.push_back(y.first);
        kept.levels.push_back(y.second);
      }
    }
  }

  /* Square cells, so Chebyshev rings bound distance. */
  const double w = px1 - px0, h = pz1 - pz0;
  const double span = w > h ? w : h;
  kept.cell = span > 0.0 ? span / kIndexSide : 1.0;
  kept.origin_x = px0;
  kept.origin_z = pz0;
  kept.nx = 1 + static_cast<int>(w / kept.cell);
  kept.nz = 1 + static_cast<int>(h / kept.cell);
  if (kept.nx > kIndexSide + 1) kept.nx = kIndexSide + 1;
  if (kept.nz > kIndexSide + 1) kept.nz = kIndexSide + 1;

  const size_t cells = static_cast<size_t>(kept.nx) * static_cast<size_t>(kept.nz);
  std::vector<int> count(cells + 1, 0);
  const auto column = [&kept](double v, double origin, int n) {
    int i = static_cast<int>(std::floor((v - origin) / kept.cell));
    if (i < 0) i = 0;
    if (i > n - 1) i = n - 1;
    return i;
  };
  /* A polygon goes in every cell its xz box touches, a superset of what it covers. */
  const auto box_of = [&](int p, int* cx0, int* cz0, int* cx1, int* cz1) {
    double ax = 0, az = 0, bx = 0, bz = 0;
    for (int i = kept.at[static_cast<size_t>(p)]; i < kept.at[static_cast<size_t>(p) + 1]; ++i) {
      const double vx = kept.vert[static_cast<size_t>(i) * 3];
      const double vz = kept.vert[static_cast<size_t>(i) * 3 + 2];
      if (i == kept.at[static_cast<size_t>(p)]) { ax = bx = vx; az = bz = vz; }
      if (vx < ax) ax = vx;
      if (vx > bx) bx = vx;
      if (vz < az) az = vz;
      if (vz > bz) bz = vz;
    }
    *cx0 = column(ax, kept.origin_x, kept.nx);
    *cx1 = column(bx, kept.origin_x, kept.nx);
    *cz0 = column(az, kept.origin_z, kept.nz);
    *cz1 = column(bz, kept.origin_z, kept.nz);
  };

  for (int p = 0; p < kept.kept; ++p) {
    int cx0, cz0, cx1, cz1;
    box_of(p, &cx0, &cz0, &cx1, &cz1);
    for (int cz = cz0; cz <= cz1; ++cz) {
      for (int cx = cx0; cx <= cx1; ++cx) ++count[static_cast<size_t>(cz * kept.nx + cx) + 1];
    }
  }
  kept.start.assign(cells + 1, 0);
  for (size_t i = 0; i < cells; ++i) kept.start[i + 1] = kept.start[i] + count[i + 1];
  kept.in.assign(static_cast<size_t>(kept.start[cells]), 0);
  std::vector<int> fill(kept.start.begin(), kept.start.end() - 1);
  for (int p = 0; p < kept.kept; ++p) {
    int cx0, cz0, cx1, cz1;
    box_of(p, &cx0, &cz0, &cx1, &cz1);
    for (int cz = cz0; cz <= cz1; ++cz) {
      for (int cx = cx0; cx <= cx1; ++cx) {
        kept.in[static_cast<size_t>(fill[static_cast<size_t>(cz * kept.nx + cx)]++)] = p;
      }
    }
  }

  /* Flood outward from the occupied cells, eight neighbours per Chebyshev step. */
  kept.ring.assign(cells, -1);
  kept.ring_poly.assign(cells, -1);
  std::vector<int> wave, next;
  for (size_t c = 0; c < cells; ++c) {
    if (kept.start[c + 1] > kept.start[c]) {
      kept.ring[c] = 0;
      kept.ring_poly[c] = kept.in[static_cast<size_t>(kept.start[c])];
      wave.push_back(static_cast<int>(c));
    }
  }
  for (int r = 1; !wave.empty(); ++r) {
    next.clear();
    for (const int c : wave) {
      const int cx = c % kept.nx, cz = c / kept.nx;
      for (int dz = -1; dz <= 1; ++dz) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nxx = cx + dx, nzz = cz + dz;
          if (nxx < 0 || nzz < 0 || nxx >= kept.nx || nzz >= kept.nz) continue;
          const size_t n = static_cast<size_t>(nzz * kept.nx + nxx);
          if (kept.ring[n] >= 0) continue;
          kept.ring[n] = r;
          kept.ring_poly[n] = kept.ring_poly[static_cast<size_t>(c)];
          next.push_back(static_cast<int>(n));
        }
      }
    }
    wave.swap(next);
  }

  kept.resolved = true;
  return true;
}

namespace {

/** The nearest point of the region, by rings outward through its index; `band`, when given, keeps
 *  only polygons in that y range. After the rings below `r` are walked, nothing untested is nearer
 *  than the outside of the walked box, which is the bound that stops the walk. */
double closest(const Region& g, double px, double pz, double* bx, double* bz,
               const double* band = nullptr) {
  int ix = static_cast<int>(std::floor((px - g.origin_x) / g.cell));
  int iz = static_cast<int>(std::floor((pz - g.origin_z) / g.cell));
  if (ix < 0) ix = 0;
  if (iz < 0) iz = 0;
  if (ix > g.nx - 1) ix = g.nx - 1;
  if (iz > g.nz - 1) iz = g.nz - 1;
  const size_t here = static_cast<size_t>(iz * g.nx + ix);

  /* The ring bound is measured from the place clamped into the index, plus `out2`, the squared
     distance from the place to the clamp: `|place - q|^2 >= out2 + |clamped - q|^2` for any q in
     the index. Without it the bound never bites for a place outside the region. */
  double qx = px, qz = pz;
  {
    const double gx1 = g.origin_x + g.cell * g.nx, gz1 = g.origin_z + g.cell * g.nz;
    if (qx < g.origin_x) qx = g.origin_x;
    if (qx > gx1) qx = gx1;
    if (qz < g.origin_z) qz = g.origin_z;
    if (qz > gz1) qz = gz1;
  }
  const double out2 = (px - qx) * (px - qx) + (pz - qz) * (pz - qz);

  double best = std::numeric_limits<double>::infinity();
  *bx = px;
  *bz = pz;
  const auto test = [&](int p) {
    if (band != nullptr && (g.poly_y1[static_cast<size_t>(p)] < band[0] ||
                            g.poly_y0[static_cast<size_t>(p)] > band[1])) {
      return;
    }
    const int from = g.at[static_cast<size_t>(p)];
    const int n = g.at[static_cast<size_t>(p) + 1] - from;
    double cx = 0, cz = 0;
    const double d = poly_dist2(&g.vert[static_cast<size_t>(from) * 3], n,
                                g.solid[static_cast<size_t>(p)] != 0, px, pz, &cx, &cz);
    if (d < best) { best = d; *bx = cx; *bz = cz; }
  };

  /* A real distance in hand before the first ring, so the bound works from the start. */
  if (g.ring_poly[here] >= 0) test(g.ring_poly[here]);

  const int last = std::max(std::max(ix + 1, g.nx - ix), std::max(iz + 1, g.nz - iz));
  const int from_ring = g.ring[here] < 0 ? 0 : g.ring[here];

  for (int r = from_ring; r <= last; ++r) {
    if (r > 0) {
      const double bx0 = g.origin_x + g.cell * (ix - r + 1);
      const double bx1 = g.origin_x + g.cell * (ix + r);
      const double bz0 = g.origin_z + g.cell * (iz - r + 1);
      const double bz1 = g.origin_z + g.cell * (iz + r);
      /* A point outside the walked box bounds nothing yet. */
      const double lb = (qx >= bx0 && qx <= bx1 && qz >= bz0 && qz <= bz1)
                            ? std::min(std::min(qx - bx0, bx1 - qx), std::min(qz - bz0, bz1 - qz))
                            : 0.0;
      if (out2 + lb * lb >= best) break;
    }
    const int cx0 = ix - r, cx1 = ix + r, cz0 = iz - r, cz1 = iz + r;
    for (int cz = cz0; cz <= cz1; ++cz) {
      if (cz < 0 || cz >= g.nz) continue;
      /* The ring only: whole rows at the two ends, two cells elsewhere. */
      const int step = (cz == cz0 || cz == cz1) ? 1 : (cx1 - cx0 == 0 ? 1 : cx1 - cx0);
      for (int cx = cx0; cx <= cx1; cx += step) {
        if (cx < 0 || cx >= g.nx) continue;
        const size_t c = static_cast<size_t>(cz * g.nx + cx);
        for (int k = g.start[c]; k < g.start[c + 1]; ++k) test(g.in[static_cast<size_t>(k)]);
      }
    }
  }
  return best;
}

}  // namespace

double Target::distance_here(double px, double py, double pz) const {
  if (!ground.resolved || !ground.has_y) return distance(px, pz);
  const double h = kHeightSlack;
  const double band[2] = {py - h, py + h};
  bool near = false;
  for (size_t i = 0; i + 1 < ground.levels.size() && !near; i += 2) {
    near = ground.levels[i + 1] >= band[0] && ground.levels[i] <= band[1];
  }
  if (!near) return std::numeric_limits<double>::infinity();
  double bx = 0, bz = 0;
  const double best = closest(ground, px, pz, &bx, &bz, band);
  return std::isinf(best) ? best : std::sqrt(best);
}

double Target::hull_distance(double px, double pz) const {
  const double dx = has_x ? outside(px, x0, x1) : 0.0;
  const double dz = has_z ? outside(pz, z0, z1) : 0.0;
  return std::sqrt(dx * dx + dz * dz);
}

double Target::distance(double px, double pz) const {
  if (ground.resolved) {
    double bx = 0, bz = 0;
    const double best = closest(ground, px, pz, &bx, &bz);
    /* Falls back to the box rather than answering infinity. */
    if (!std::isinf(best)) return std::sqrt(best);
  }
  if (!list.empty()) {
    double nx = 0, nz = 0;
    nearest(px, pz, &nx, &nz);
    return std::sqrt((px - nx) * (px - nx) + (pz - nz) * (pz - nz));
  }
  if (ranged) {
    const double dx = has_x ? outside(px, x0, x1) : 0.0;
    const double dz = has_z ? outside(pz, z0, z1) : 0.0;
    return std::sqrt(dx * dx + dz * dz);
  }
  const double dx = has_x ? px - x : 0.0;
  const double dz = has_z ? pz - z : 0.0;
  return std::sqrt(dx * dx + dz * dz);
}

/** A float's place in the order of all floats; -0 and +0 are the same place. */
static long long float_rank(float f) {
  uint32_t bits;
  std::memcpy(&bits, &f, sizeof bits);
  if (bits & 0x80000000u) return -static_cast<long long>(bits & 0x7FFFFFFFu);
  return static_cast<long long>(bits);
}

long long float_steps(double a, double b) {
  const float from = static_cast<float>(a), to = static_cast<float>(b);
  if (from == to) return 0;
  const long long kMost = 1000000;
  const long long apart = std::llabs(float_rank(to) - float_rank(from));
  return apart < kMost ? apart : kMost;
}

long long Target::off(double px, double pz) const {
  double nx = 0, nz = 0;
  nearest(px, pz, &nx, &nz);
  return float_steps(px, nx) + float_steps(pz, nz);
}

bool Target::holds_bytes(double px, double py, double pz) const {
  const double at[3] = {px, py, pz};
  for (int a = 0; a < 3; ++a) {
    if (!mask.on(a)) continue;
    const float f = static_cast<float>(at[a]);
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof bits);
    for (int i = 0; i < 4; ++i) {
      if (!mask.known[a][i]) continue;
      if (((bits >> (24 - 8 * i)) & 0xFFu) != mask.byte[a][i]) return false;
    }
  }
  return true;
}

double Target::miss(double px, double py, double pz) const {
  int start = 0, count = 0;
  if (!mask.run(&start, &count)) return static_cast<double>(off(px, pz));
  const double at[3] = {px, py, pz};
  uint8_t mem[12];
  for (int a = 0; a < 3; ++a) {
    const float f = static_cast<float>(at[a]);
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof bits);
    for (int i = 0; i < 4; ++i) mem[a * 4 + i] = static_cast<uint8_t>(bits >> (24 - 8 * i));
  }
  uint8_t got[12], want[12];
  for (int i = 0; i < count; ++i) {
    got[i] = mem[start + i];
    want[i] = mask.byte[(start + i) / 4][(start + i) % 4];
  }
  /* The larger less the smaller, bytewise with a borrow, so the gap is exact. */
  const bool ahead = std::memcmp(got, want, static_cast<size_t>(count)) >= 0;
  const uint8_t* hi = ahead ? got : want;
  const uint8_t* lo = ahead ? want : got;
  uint8_t gap[12];
  int borrow = 0;
  for (int i = count - 1; i >= 0; --i) {
    int d = static_cast<int>(hi[i]) - static_cast<int>(lo[i]) - borrow;
    borrow = d < 0 ? 1 : 0;
    gap[i] = static_cast<uint8_t>(d + (borrow ? 256 : 0));
  }
  double out = 0.0;
  for (int i = 0; i < count; ++i) out = out * 256.0 + static_cast<double>(gap[i]);
  return out;
}

double Target::bytes_off(double px, double py, double pz) const {
  const double at[3] = {px, py, pz};
  double sum = 0.0;
  for (int a = 0; a < 3; ++a) {
    const std::vector<double>& v = allowed[a];
    if (v.empty()) continue;
    size_t lo = 0, hi = v.size() / 2;
    while (lo < hi) {
      const size_t mid = (lo + hi) / 2;
      if (v[mid * 2 + 1] < at[a]) lo = mid + 1; else hi = mid;
    }
    double best = std::numeric_limits<double>::infinity();
    if (lo < v.size() / 2) best = std::min(best, outside(at[a], v[lo * 2], v[lo * 2 + 1]));
    if (lo > 0) best = std::min(best, outside(at[a], v[lo * 2 - 2], v[lo * 2 - 1]));
    sum += best * best;
  }
  return std::sqrt(sum);
}

double Target::height_off(double py) const {
  if (!mask.on(1)) return 0.0;
  const double inf = std::numeric_limits<double>::infinity();
  const double most = std::numeric_limits<double>::max();
  std::vector<Span> ys;
  if (!spans_of(mask, 1, -most, most, 0.0, &ys)) return inf;
  double best = inf;
  for (const Span& s : ys) {
    const double d = outside(py, s.lo, s.hi);
    if (d < best) best = d;
  }
  return best;
}

void Target::nearest(double px, double pz, double* nx, double* nz) const {
  if (ground.resolved) {
    double bx = 0, bz = 0;
    if (!std::isinf(closest(ground, px, pz, &bx, &bz))) {
      *nx = bx;
      *nz = bz;
      return;
    }
  }
  if (!list.empty()) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < list.size(); i += 2) {
      const double dx = px - list[i], dz = pz - list[i + 1];
      if (dx * dx + dz * dz < best) {
        best = dx * dx + dz * dz;
        *nx = list[i];
        *nz = list[i + 1];
      }
    }
    return;
  }
  if (ranged) {
    *nx = has_x ? onto(px, x0, x1) : px;
    *nz = has_z ? onto(pz, z0, z1) : pz;
    return;
  }
  *nx = has_x ? x : px;
  *nz = has_z ? z : pz;
}

void Target::extent(const float room[6], double* min_x, double* min_z, double* max_x,
                    double* max_z) const {
  if (ranged) {
    /* A typed edge is not clamped here: `room` may be the grid, and `corridor` bounds it. */
    *min_x = has_x ? x0 : static_cast<double>(room[0]);
    *max_x = has_x ? x1 : static_cast<double>(room[3]);
    *min_z = has_z ? z0 : static_cast<double>(room[2]);
    *max_z = has_z ? z1 : static_cast<double>(room[5]);
    return;
  }
  *min_x = has_x ? x : static_cast<double>(room[0]);
  *max_x = has_x ? x : static_cast<double>(room[3]);
  *min_z = has_z ? z : static_cast<double>(room[2]);
  *max_z = has_z ? z : static_cast<double>(room[5]);
}

bool EndFacing::holds(int facing) const {
  if (any) return true;
  /* Positive, as the room draws the fan. */
  const int from = ((facing - a) % kTurn + kTurn) % kTurn;
  return from <= span;
}

}  // namespace search
