#include "grid.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace search {
namespace {

/** A degenerate triangle answers flat up; its neighbours mark the cell anyway. */
struct Plane {
  double nx = 0, ny = 1, nz = 0, d = 0;
  double at(double x, double z) const { return ny == 0.0 ? 0.0 : (d - nx * x - nz * z) / ny; }
};

Plane plane_of(const float* t) {
  const double ax = t[3] - t[0], ay = t[4] - t[1], az = t[5] - t[2];
  const double bx = t[6] - t[0], by = t[7] - t[1], bz = t[8] - t[2];
  double nx = ay * bz - az * by;
  double ny = az * bx - ax * bz;
  double nz = ax * by - ay * bx;
  const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
  Plane p;
  if (len < 1e-9) return p;
  nx /= len;
  ny /= len;
  nz /= len;
  p.nx = nx;
  p.ny = ny;
  p.nz = nz;
  p.d = nx * t[0] + ny * t[1] + nz * t[2];
  return p;
}

/** Exact xz triangle-cell overlap by separating axes. */
bool overlaps(const double* tx, const double* tz, double lo_x, double lo_z, double hi_x,
              double hi_z) {
  const double t_lo_x = std::min({tx[0], tx[1], tx[2]});
  const double t_hi_x = std::max({tx[0], tx[1], tx[2]});
  if (t_hi_x < lo_x || t_lo_x > hi_x) return false;
  const double t_lo_z = std::min({tz[0], tz[1], tz[2]});
  const double t_hi_z = std::max({tz[0], tz[1], tz[2]});
  if (t_hi_z < lo_z || t_lo_z > hi_z) return false;

  const double cx[4] = {lo_x, hi_x, hi_x, lo_x};
  const double cz[4] = {lo_z, lo_z, hi_z, hi_z};
  for (int i = 0; i < 3; ++i) {
    const int j = (i + 1) % 3;
    /* Outside is the side the third vertex is not on, so no winding is assumed. */
    const double ex = tz[j] - tz[i], ez = -(tx[j] - tx[i]);
    const int k = (i + 2) % 3;
    const double here = ex * (tx[k] - tx[i]) + ez * (tz[k] - tz[i]);
    if (here == 0.0) continue;
    const double sign = here > 0.0 ? -1.0 : 1.0;
    bool all_outside = true;
    for (int c = 0; c < 4; ++c) {
      const double side = sign * (ex * (cx[c] - tx[i]) + ez * (cz[c] - tz[i]));
      if (side <= 0.0) {
        all_outside = false;
        break;
      }
    }
    if (all_outside) return false;
  }
  return true;
}

double apart(const Plane& a, const Plane& b) {
  return 1.0 - (a.nx * b.nx + a.ny * b.ny + a.nz * b.nz);
}

}  // namespace

Grid build(const Selection& selection, const Corridor& corridor, const Limits& limits) {
  Grid g;
  const int count = static_cast<int>(selection.ground.size() / 9);

  /* Widen rather than crop: a coarser cell costs speed, a cropped corridor costs plans. */
  const double span_x = std::max(corridor.max_x - corridor.min_x, 1.0);
  const double span_z = std::max(corridor.max_z - corridor.min_z, 1.0);
  double cell = limits.cell > 0.0 ? limits.cell : 1.0;
  for (;;) {
    const double wide = std::ceil(span_x / cell), deep = std::ceil(span_z / cell);
    if (wide * deep <= static_cast<double>(limits.max_cells)) break;
    cell *= 2.0;
  }
  g.cell = cell;
  g.origin_x = corridor.min_x;
  g.origin_z = corridor.min_z;
  g.nx = std::max(1, static_cast<int>(std::ceil(span_x / cell)));
  g.nz = std::max(1, static_cast<int>(std::ceil(span_z / cell)));
  g.cells = g.nx * g.nz;

  /* Count, then fill. */
  std::vector<int> per(static_cast<size_t>(g.cells), 0);
  const auto cover = [&](int t, const auto& visit) {
    const float* v = &selection.ground[static_cast<size_t>(t) * 9];
    const double tx[3] = {v[0], v[3], v[6]};
    const double tz[3] = {v[2], v[5], v[8]};
    const double lo_x = std::min({tx[0], tx[1], tx[2]}), hi_x = std::max({tx[0], tx[1], tx[2]});
    const double lo_z = std::min({tz[0], tz[1], tz[2]}), hi_z = std::max({tz[0], tz[1], tz[2]});
    const int from_x = std::max(0, static_cast<int>(std::floor((lo_x - g.origin_x) / g.cell)));
    const int to_x = std::min(g.nx - 1, static_cast<int>(std::floor((hi_x - g.origin_x) / g.cell)));
    const int from_z = std::max(0, static_cast<int>(std::floor((lo_z - g.origin_z) / g.cell)));
    const int to_z = std::min(g.nz - 1, static_cast<int>(std::floor((hi_z - g.origin_z) / g.cell)));
    for (int iz = from_z; iz <= to_z; ++iz) {
      for (int ix = from_x; ix <= to_x; ++ix) {
        const double cell_x = g.origin_x + ix * g.cell, cell_z = g.origin_z + iz * g.cell;
        if (!overlaps(tx, tz, cell_x, cell_z, cell_x + g.cell, cell_z + g.cell)) continue;
        visit(g.at(ix, iz));
      }
    }
  };

  for (int t = 0; t < count; ++t) cover(t, [&](int c) { ++per[static_cast<size_t>(c)]; });

  g.start.assign(static_cast<size_t>(g.cells) + 1, 0);
  for (int c = 0; c < g.cells; ++c) {
    const int here = per[static_cast<size_t>(c)];
    g.start[static_cast<size_t>(c) + 1] = g.start[static_cast<size_t>(c)] + here;
    g.worst_cell = std::max(g.worst_cell, here);
  }
  g.tri.assign(static_cast<size_t>(g.start.back()), 0);
  g.per_cell = g.cells > 0 ? static_cast<double>(g.tri.size()) / g.cells : 0.0;

  std::vector<int> filled(static_cast<size_t>(g.cells), 0);
  for (int t = 0; t < count; ++t) {
    cover(t, [&](int c) {
      const size_t at = static_cast<size_t>(g.start[static_cast<size_t>(c)] + filled[static_cast<size_t>(c)]);
      g.tri[at] = t;
      ++filled[static_cast<size_t>(c)];
    });
  }

  /* Walls: every cell the xz box grown by `reach` touches. */
  g.reach = limits.reach > 0.0 ? limits.reach : 0.0;
  g.body_height = limits.body_height;
  const int walls = static_cast<int>(selection.wall.size() / 9);
  const auto cover_wall = [&](int t, const auto& visit) {
    const float* v = &selection.wall[static_cast<size_t>(t) * 9];
    const double tx[3] = {v[0], v[3], v[6]};
    const double tz[3] = {v[2], v[5], v[8]};
    const double lo_x = std::min({tx[0], tx[1], tx[2]}) - g.reach;
    const double hi_x = std::max({tx[0], tx[1], tx[2]}) + g.reach;
    const double lo_z = std::min({tz[0], tz[1], tz[2]}) - g.reach;
    const double hi_z = std::max({tz[0], tz[1], tz[2]}) + g.reach;
    const int from_x = std::max(0, static_cast<int>(std::floor((lo_x - g.origin_x) / g.cell)));
    const int to_x = std::min(g.nx - 1, static_cast<int>(std::floor((hi_x - g.origin_x) / g.cell)));
    const int from_z = std::max(0, static_cast<int>(std::floor((lo_z - g.origin_z) / g.cell)));
    const int to_z = std::min(g.nz - 1, static_cast<int>(std::floor((hi_z - g.origin_z) / g.cell)));
    for (int iz = from_z; iz <= to_z; ++iz) {
      for (int ix = from_x; ix <= to_x; ++ix) visit(g.at(ix, iz));
    }
  };

  std::vector<int> wall_per(static_cast<size_t>(g.cells), 0);
  for (int t = 0; t < walls; ++t) cover_wall(t, [&](int c) { ++wall_per[static_cast<size_t>(c)]; });
  g.wall_start.assign(static_cast<size_t>(g.cells) + 1, 0);
  for (int c = 0; c < g.cells; ++c) {
    const int here = wall_per[static_cast<size_t>(c)];
    g.wall_start[static_cast<size_t>(c) + 1] = g.wall_start[static_cast<size_t>(c)] + here;
    if (here > 0) ++g.wall_cells;
    g.worst_wall_cell = std::max(g.worst_wall_cell, here);
  }
  g.wall_tri.assign(static_cast<size_t>(g.wall_start.back()), 0);
  std::vector<int> wall_filled(static_cast<size_t>(g.cells), 0);
  for (int t = 0; t < walls; ++t) {
    cover_wall(t, [&](int c) {
      const size_t at = static_cast<size_t>(g.wall_start[static_cast<size_t>(c)] +
                                            wall_filled[static_cast<size_t>(c)]);
      g.wall_tri[at] = t;
      ++wall_filled[static_cast<size_t>(c)];
    });
  }

  std::vector<Plane> plane(static_cast<size_t>(count));
  for (int t = 0; t < count; ++t) {
    plane[static_cast<size_t>(t)] = plane_of(&selection.ground[static_cast<size_t>(t) * 9]);
  }

  g.plane.resize(static_cast<size_t>(count) * 4);
  for (int t = 0; t < count; ++t) {
    const Plane& p = plane[static_cast<size_t>(t)];
    const size_t at = static_cast<size_t>(t) * 4;
    g.plane[at + 0] = p.nx;
    g.plane[at + 1] = p.ny;
    g.plane[at + 2] = p.nz;
    g.plane[at + 3] = p.d;
  }

  g.mark.assign(static_cast<size_t>(g.cells), 0);
  g.low.assign(static_cast<size_t>(g.cells), 0.0f);
  g.high.assign(static_cast<size_t>(g.cells), 0.0f);

  for (int iz = 0; iz < g.nz; ++iz) {
    for (int ix = 0; ix < g.nx; ++ix) {
      const size_t c = static_cast<size_t>(g.at(ix, iz));
      const int from = g.start[c], to = g.start[c + 1];
      if (from == to) {
        g.mark[c] |= kNoGround;
        continue;
      }
      const double cell_x = g.origin_x + ix * g.cell, cell_z = g.origin_z + iz * g.cell;
      const double corner_x[4] = {cell_x, cell_x + g.cell, cell_x + g.cell, cell_x};
      const double corner_z[4] = {cell_z, cell_z, cell_z + g.cell, cell_z + g.cell};

      double low = std::numeric_limits<double>::max();
      double high = -std::numeric_limits<double>::max();
      bool steep = false;
      for (int i = from; i < to; ++i) {
        const Plane& p = plane[static_cast<size_t>(g.tri[static_cast<size_t>(i)])];
        if (p.ny < limits.slope_normal_y) steep = true;
        for (int k = 0; k < 4; ++k) {
          const double y = p.at(corner_x[k], corner_z[k]);
          low = std::min(low, y);
          high = std::max(high, y);
        }
      }
      g.low[c] = static_cast<float>(low);
      g.high[c] = static_cast<float>(high);
      if (high - low > limits.height_step) g.mark[c] |= kHeight;
      if (steep) g.mark[c] |= kSlope;
    }
  }

  /* Every triangle in the 3x3 neighbourhood against the cell's first; one normal a cell would miss
     a flat-plus-slope cell. A neighbour with no ground is an unbounded step: `kHeight`. */
  for (int iz = 0; iz < g.nz; ++iz) {
    for (int ix = 0; ix < g.nx; ++ix) {
      const size_t c = static_cast<size_t>(g.at(ix, iz));
      if (g.mark[c] & kNoGround) continue;
      const Plane& mine = plane[static_cast<size_t>(g.tri[static_cast<size_t>(g.start[c])])];
      for (int dz = -1; dz <= 1; ++dz) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!g.inside(ix + dx, iz + dz)) continue;
          const size_t n = static_cast<size_t>(g.at(ix + dx, iz + dz));
          if (g.mark[n] & kNoGround) {
            g.mark[c] |= kHeight;
            continue;
          }
          for (int i = g.start[n]; i < g.start[n + 1]; ++i) {
            if (apart(mine, plane[static_cast<size_t>(g.tri[static_cast<size_t>(i)])]) >
                limits.normal_apart) {
              g.mark[c] |= kNormal;
              break;
            }
          }
          if (dx == 0 && dz == 0) continue;
          const double low_apart = std::abs(static_cast<double>(g.low[c]) - g.low[n]);
          const double high_apart = std::abs(static_cast<double>(g.high[c]) - g.high[n]);
          if (low_apart > limits.height_step || high_apart > limits.height_step) {
            g.mark[c] |= kHeight;
          }
        }
      }
    }
  }

  for (int c = 0; c < g.cells; ++c) {
    const uint8_t m = g.mark[static_cast<size_t>(c)];
    if (m == 0) continue;
    ++g.marked;
    if (m & kNoGround) ++g.marked_no_ground;
    if (m & kNormal) ++g.marked_normal;
    if (m & kHeight) ++g.marked_height;
    if (m & kSlope) ++g.marked_slope;
  }
  return g;
}

}  // namespace search
