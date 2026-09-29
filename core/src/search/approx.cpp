#include "approx.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "SSystem/SComponent/c_math.h"

namespace search {
namespace {

const double kPi = 3.14159265358979323846;

/** An s16 facing as radians; facing 0 looks down +z, `x = sin`, `z = cos`. */
double radians(int facing) { return static_cast<double>(static_cast<int16_t>(facing)) * (2.0 * kPi / 65536.0); }

/** A world step as `ahead` / `side` about a facing, and back. `side` is facing + 0x4000. */
void to_local(double dx, double dz, double sin_f, double cos_f, double* ahead, double* side) {
  *ahead = dx * sin_f + dz * cos_f;
  *side = dx * cos_f - dz * sin_f;
}

void to_world(double ahead, double side, double sin_f, double cos_f, double* dx, double* dz) {
  *dx = ahead * sin_f + side * cos_f;
  *dz = ahead * cos_f - side * sin_f;
}

/** Is (x, z) inside a triangle's xz projection, either winding. */
bool inside_xz(const float* t, double x, double z) {
  const double x0 = t[0], z0 = t[2], x1 = t[3], z1 = t[5], x2 = t[6], z2 = t[8];
  const double a = (x1 - x0) * (z - z0) - (z1 - z0) * (x - x0);
  const double b = (x2 - x1) * (z - z1) - (z2 - z1) * (x - x1);
  const double c = (x0 - x2) * (z - z2) - (z0 - z2) * (x - x2);
  const bool any_neg = a < 0.0 || b < 0.0 || c < 0.0;
  const bool any_pos = a > 0.0 || b > 0.0 || c > 0.0;
  return !(any_neg && any_pos);
}

/** Every cell a segment crosses. `fn` gets a cell index, or -1 for any part off the grid. */
template <typename Fn>
void walk_cells(const Grid& g, double x0, double z0, double x1, double z1, Fn fn) {
  if (g.cells <= 0 || g.cell <= 0.0) {
    fn(-1);
    return;
  }
  const double inv = 1.0 / g.cell;
  double ix = std::floor((x0 - g.origin_x) * inv);
  double iz = std::floor((z0 - g.origin_z) * inv);
  const double end_ix = std::floor((x1 - g.origin_x) * inv);
  const double end_iz = std::floor((z1 - g.origin_z) * inv);

  const double dx = x1 - x0, dz = z1 - z0;
  const int sx = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
  const int sz = dz > 0.0 ? 1 : (dz < 0.0 ? -1 : 0);
  const double big = std::numeric_limits<double>::infinity();
  const double t_dx = sx == 0 ? big : std::fabs(g.cell / dx);
  const double t_dz = sz == 0 ? big : std::fabs(g.cell / dz);
  double t_x = big, t_z = big;
  if (sx != 0) {
    const double edge = g.origin_x + (ix + (sx > 0 ? 1.0 : 0.0)) * g.cell;
    t_x = (edge - x0) / dx;
  }
  if (sz != 0) {
    const double edge = g.origin_z + (iz + (sz > 0 ? 1.0 : 0.0)) * g.cell;
    t_z = (edge - z0) / dz;
  }

  for (int guard = 0; guard < 1 << 20; ++guard) {
    const int cx = static_cast<int>(ix), cz = static_cast<int>(iz);
    fn(g.inside(cx, cz) ? g.at(cx, cz) : -1);
    if (ix == end_ix && iz == end_iz) return;
    if (t_x < t_z) {
      ix += sx;
      t_x += t_dx;
    } else {
      iz += sz;
      t_z += t_dz;
    }
    if (t_x > 1.0 && t_z > 1.0 && (ix != end_ix || iz != end_iz)) {
      /* Rounding stepped past a segment ending exactly on a boundary. */
      const int ex = static_cast<int>(end_ix), ez = static_cast<int>(end_iz);
      fn(g.inside(ex, ez) ? g.at(ex, ez) : -1);
      return;
    }
  }
  fn(-1);
}

/** Records each frame of a drive. */
struct Trail : Watcher {
  struct Row {
    double x, y, z;
    int facing;
    /** `speedF` and its travel angle. */
    double speed;
    int travel;
  };
  std::vector<Row> row;
  void frame(int, const daPy_lk_c& lk, bool) override {
    Row r;
    r.x = lk.current.pos.x;
    r.y = lk.current.pos.y;
    r.z = lk.current.pos.z;
    r.facing = lk.shape_angle.y & 0xFFFF;
    r.speed = lk.speedF;
    r.travel = lk.current.angle.y & 0xFFFF;
    row.push_back(r);
  }
};


/** Squared distance between two segments in xz; zero when they cross. */
double seg_seg_2d(double ax, double az, double bx, double bz, double cx, double cz, double dx,
                  double dz) {
  const double ux = bx - ax, uz = bz - az;
  const double vx = dx - cx, vz = dz - cz;
  const double wx = ax - cx, wz = az - cz;
  const double a = ux * ux + uz * uz;
  const double b = ux * vx + uz * vz;
  const double c = vx * vx + vz * vz;
  const double d = ux * wx + uz * wz;
  const double e = vx * wx + vz * wz;
  const double det = a * c - b * b;
  double s, t;
  if (det < 1e-12) {
    s = 0.0;
    t = c > 1e-12 ? e / c : 0.0;
  } else {
    s = (b * e - c * d) / det;
    t = (a * e - b * d) / det;
  }
  /* Each parameter is re-solved against its clamped partner; one clamp alone overstates the
     distance near an end. */
  if (s < 0.0) s = 0.0;
  if (s > 1.0) s = 1.0;
  t = c > 1e-12 ? (b * s + e) / c : 0.0;
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  s = a > 1e-12 ? (b * t - d) / a : 0.0;
  if (s < 0.0) s = 0.0;
  if (s > 1.0) s = 1.0;

  const double sx2 = ax + s * ux - (cx + t * vx);
  const double sz2 = az + s * uz - (cz + t * vz);
  return sx2 * sx2 + sz2 * sz2;
}

}  // namespace

bool wall_in_reach(const Grid& grid, const Selection& selection, double x0, double z0, double x1,
                   double z1, double y, double radius) {
  if (grid.wall_tri.empty() || grid.reach <= 0.0) return false;
  /* The wall buckets were built expanded by `Grid::reach`, so a wider radius cannot be answered. */
  const double want = radius < 0.0 ? grid.reach : radius;
  if (want > grid.reach) return true;
  const double radius2 = want * want;
  const double body_lo = y, body_hi = y + grid.body_height;

  bool near = false;
  walk_cells(grid, x0, z0, x1, z1, [&](int cell) {
    if (cell < 0 || near) return;
    const int from = grid.wall_start[static_cast<size_t>(cell)];
    const int to = grid.wall_start[static_cast<size_t>(cell) + 1];
    for (int at = from; at < to && !near; ++at) {
      const int t = grid.wall_tri[static_cast<size_t>(at)];
      const float* v = &selection.wall[static_cast<size_t>(t) * 9];

      const double lo_y = std::min(std::min(v[1], v[4]), v[7]);
      const double hi_y = std::max(std::max(v[1], v[4]), v[7]);
      if (hi_y < body_lo || lo_y > body_hi) continue;

      const double tx[3] = {v[0], v[3], v[6]};
      const double tz[3] = {v[2], v[5], v[8]};
      if (inside_xz(v, x0, z0) || inside_xz(v, x1, z1)) {
        near = true;
        break;
      }
      for (int e = 0; e < 3 && !near; ++e) {
        const int f = (e + 1) % 3;
        if (seg_seg_2d(x0, z0, x1, z1, tx[e], tz[e], tx[f], tz[f]) <= radius2) near = true;
      }
    }
  });
  return near;
}

namespace {

/** An s16 difference, signed and shortest way round. */
int turn_of(int from, int to) {
  return static_cast<int16_t>(static_cast<uint16_t>(to) - static_cast<uint16_t>(from));
}

}  // namespace

const BaseMove* BaseTable::of(const std::string& id) const {
  for (size_t i = 0; i < move.size(); ++i) {
    if (move[i].id == id) return &move[i];
  }
  return nullptr;
}

BaseTable priced(const BaseTable& measured, const std::map<std::string, int>& costs) {
  BaseTable out = measured;
  for (size_t i = 0; i < out.move.size(); ++i) {
    BaseMove& row = out.move[i];
    const std::map<std::string, int>::const_iterator it = costs.find(row.id);
    if (it == costs.end() || it->second < 1) continue;
    /* An ESS price is per gap; a person's price replaces the median `moves.ts` shows and moves
       every gap by the same amount, down to a frame at the least. */
    const Seat seat = seat_of(row.id);
    if (is_ess(seat)) {
      const int shifted = std::max(1, row.frames + it->second - ess_median(seat));
      row.surcharge = shifted - row.frames;
      row.frames = shifted;
      continue;
    }
    row.surcharge = it->second - row.frames;
    row.frames = it->second;
  }
  return out;
}

BaseTable base_table(bool combos) {
  const tww_engine::RoomDzb flat = tww_engine::flat_floor_dzb(0.0f);
  tww_engine::Init init;
  init.pos.set(0.0f, 0.0f, 0.0f);
  init.proc = daPy_lk_c::daPyProc_WAIT_e;

  BaseTable table;
  /* `roster()` needs a camera to aim the turnaround; the table only measures displacement and
     frames, which the aim does not change. */
  const int flat_camera = 0;
  const std::vector<Move> all = roster(0, combos, &flat_camera);
  table.move.resize(all.size());

  for (size_t i = 0; i < all.size(); ++i) {
    const Move& m = all[i];
    BaseMove& row = table.move[i];
    row.id = m.id;
    row.stands = m.stands;

    if (m.is_stated()) {
      row.frames = m.stated_frames;
      row.turn = m.rotates;
      row.why = "priced by hand";
      ++table.not_driven;
      continue;
    }
    if (!m.blocked.empty()) {
      row.why = m.blocked;
      ++table.not_driven;
      continue;
    }

    /* An ESS turn's frames do depend on the aim: it is measured one unit short of its target, the
       fastest it can be, and `model_step` adds the rest. */
    Move aimed = m;
    const Seat seat = seat_of(m.id);
    if (is_ess(seat)) {
      const int camera = (1 - ess_offset(seat)) & 0xFFFF;
      move_of(m.id, 0, &aimed, &camera);
    }

    Trail trail;
    const Drive d = drive(aimed, init, &flat, tww_engine::RunOptions(), 96, &trail);
    row.frames = d.frames;
    row.engine_frames = d.engine_frames;

    /* A dispatch into a recorded-only arm still reports `ok` and `rested` with zero displacement. */
    if (!d.ok) {
      char why[64];
      std::snprintf(why, sizeof why, "stopped in proc %d", d.stopped_in);
      row.why = why;
      ++table.not_driven;
      continue;
    }
    if (d.dispatch_calls > 0) {
      char why[64];
      std::snprintf(why, sizeof why, "dispatched into arm 0x%X", d.dispatch_arm);
      row.why = why;
      ++table.not_driven;
      continue;
    }
    if (!d.rested) {
      row.why = "did not come back to a standstill";
      ++table.not_driven;
      continue;
    }

    row.driven = true;
    row.reach = d.reach;
    ++table.driven;

    const double sin_f = std::sin(radians(d.facing_in)), cos_f = std::cos(radians(d.facing_in));
    double px = 0.0, py = 0.0, pz = 0.0;
    row.step.resize(trail.row.size());
    for (size_t f = 0; f < trail.row.size(); ++f) {
      BaseFrame& bf = row.step[f];
      to_local(trail.row[f].x - px, trail.row[f].z - pz, sin_f, cos_f, &bf.ahead, &bf.side);
      /* The game's trig table, which is what carried `speedF`. */
      const s16 along = static_cast<s16>(trail.row[f].travel);
      to_local(trail.row[f].speed * static_cast<double>(cM_ssin(along)),
               trail.row[f].speed * static_cast<double>(cM_scos(along)), sin_f, cos_f,
               &bf.speed_ahead, &bf.speed_side);
      bf.rise = trail.row[f].y - py;
      bf.above = trail.row[f].y;  // the floor is at y = 0
      bf.turn = turn_of(d.facing_in, trail.row[f].facing);
      px = trail.row[f].x;
      py = trail.row[f].y;
      pz = trail.row[f].z;
      row.ahead += bf.ahead;
      row.side += bf.side;
    }
    row.turn = turn_of(d.facing_in, d.facing_out);

    for (size_t f = 0; f < row.step.size(); ++f) {
      if (row.step[f].above <= 0.0) continue;
      if (row.air_from < 0) row.air_from = static_cast<int>(f);
      row.air_to = static_cast<int>(f) + 1;
    }
    if (row.air_from >= 0 && row.air_to - row.air_from >= 2) {
      const BaseFrame& last = row.step[static_cast<size_t>(row.air_to) - 1];
      const BaseFrame& before = row.step[static_cast<size_t>(row.air_to) - 2];
      row.gravity = last.rise - before.rise;
    }

    const double net = std::sqrt(row.ahead * row.ahead + row.side * row.side);
    if (net > table.widest) table.widest = net;
  }
  return table;
}

Floor floor_at(const Grid& grid, const Selection& selection, double x, double z, double probe_y) {
  Floor f;
  if (grid.cells <= 0 || grid.cell <= 0.0) return f;
  const int ix = static_cast<int>(std::floor((x - grid.origin_x) / grid.cell));
  const int iz = static_cast<int>(std::floor((z - grid.origin_z) / grid.cell));
  if (!grid.inside(ix, iz)) return f;

  f.inside = true;
  f.cell = grid.at(ix, iz);
  f.mark = grid.mark[static_cast<size_t>(f.cell)];

  const double ceiling = probe_y + kGroundProbe;
  double best = -std::numeric_limits<double>::infinity();
  const int from = grid.start[static_cast<size_t>(f.cell)];
  const int to = grid.start[static_cast<size_t>(f.cell) + 1];
  for (int at = from; at < to; ++at) {
    const int t = grid.tri[static_cast<size_t>(at)];
    const float* nine = &selection.ground[static_cast<size_t>(t) * 9];
    if (!inside_xz(nine, x, z)) continue;
    const double* p = &grid.plane[static_cast<size_t>(t) * 4];
    if (p[1] == 0.0) continue;
    const double y = (p[3] - p[0] * x - p[2] * z) / p[1];
    if (y > ceiling || y <= best) continue;
    best = y;
    f.found = true;
    f.tri = t;
    f.y = y;
    f.nx = p[0];
    f.ny = p[1];
    f.nz = p[2];
  }
  if (f.found && static_cast<size_t>(f.tri) < selection.ground_code.size()) {
    f.code = selection.ground_code[static_cast<size_t>(f.tri)];
  }
  return f;
}

Stepped step_move(const BaseMove& move, const Grid& grid, const Selection& selection,
                  const Calibration& cal, double x, double y, double z, int facing,
                  Collision collision) {
  return step_move(move, grid, selection, cal.of(move.id), cal.plane_of(move.id), x, y, z, facing,
                   collision);
}

Stepped step_move(const BaseMove& move, const Grid& grid, const Selection& selection,
                  const MoveCal* mc, const PlaneTable* pt, double x, double y, double z,
                  int facing, Collision collision) {
  Stepped s;
  s.x = x;
  s.y = y;
  s.z = z;
  s.facing = facing & 0xFFFF;
  s.frames = move.frames;
  if (!move.driven) {
    /* Not approximated: `ok` stays false. Only the hand-priced turn still turns. */
    if (move.turn != 0 && move.step.empty() && move.frames > 0) {
      s.facing = (facing + move.turn) & 0xFFFF;
    }
    return s;
  }

  /* The game's trig (`JMASSin`), not `std::sin`: the two differ by about a thousandth of a radian,
     a quarter unit at a move's reach. */
  const double sin_f = static_cast<double>(cM_ssin(static_cast<s16>(facing)));
  const double cos_f = static_cast<double>(cM_scos(static_cast<s16>(facing)));

  /* Summed frame by frame in the loop's order, not as a net: the result seeds a 0-ULP engine. */
  if (collision == Collision::None) {
    double at_x = x, at_z = z;
    for (size_t f = 0; f < move.step.size(); ++f) {
      double dx = 0.0, dz = 0.0;
      to_world(move.step[f].ahead, move.step[f].side, sin_f, cos_f, &dx, &dz);
      at_x += dx;
      at_z += dz;
    }
    s.x = at_x;
    s.z = at_z;
    s.y = y;
    s.facing = move.step.empty() ? s.facing : (facing + move.step.back().turn) & 0xFFFF;
    s.frames = move.frames;
    s.ok = true;
    return s;
  }

  /* One frame's horizontal move: slope correction, cell marks and wall reach. */
  struct Walk {
    /** `had` is the previous frame's floor read at this place, or null. It is only passed when
     *  Link ended the frame no higher, since a lower probe cannot change the highest plane found. */
    static void go(Stepped& s, const Grid& grid, const Selection& selection, const MoveCal* mc,
                   double dx, double dz, double speed_ahead, double speed_side, double sin_f,
                   double cos_f, bool grounded, bool walls, const Floor* had, int facing) {
      const Floor under = had != nullptr ? *had : floor_at(grid, selection, s.x, s.z, s.y);
      if (!under.inside) s.left = true;
      s.why |= under.mark;
      if (under.inside && !under.found) s.why |= kNoGround;
      /* Two triangles in one plane count as one plane. */
      if (grounded && under.found && s.planes < 2 && under.tri != s.plane_tri) {
        const double* p = &grid.plane[static_cast<size_t>(under.tri) * 4];
        if (s.planes == 0) {
          for (int k = 0; k < 4; ++k) s.plane[k] = p[k];
          s.planes = 1;
        } else if (std::fabs(p[0] - s.plane[0]) > 1e-7 || std::fabs(p[1] - s.plane[1]) > 1e-7 ||
                   std::fabs(p[2] - s.plane[2]) > 1e-7 || std::fabs(p[3] - s.plane[3]) > 1e-3) {
          s.planes = 2;
        }
        s.plane_tri = under.tri;
      }

      /* Only the `speedF` part is scaled, but every moving frame is counted as sloped, root or not,
         and with or without a calibration row. */
      const bool moved = dx != 0.0 || dz != 0.0;
      if (grounded && under.found && moved && under.ny != 0.0) {
        /* The game's own quantised s16 (see `MoveCal`), by facing rather than travel direction. A
           plain sign test would charge the uphill arm on level floors, whose grade is ~1e-9. */
        const float nx = static_cast<float>(under.nx);
        const float ny = static_cast<float>(under.ny);
        const float nz = static_cast<float>(under.nz);
        const float swung =
            cM_scos(static_cast<s16>(cM_atan2s(nx, nz) - static_cast<s16>(facing)));
        const s16 ground = cM_atan2s(std::sqrt(nx * nx + nz * nz) * swung, ny);

        /* `posMoveFromFootPos` zeroes the angle on ground code 8; still counted as sloped. */
        const bool stairs = under.code == geom::kGroundStairs;
        if (ground != 0) ++s.sloped_frames;
        if (ground != 0 && stairs) ++s.stairs_frames;
        if (ground < 0 && !stairs) ++s.uphill_frames;
        const double speed2 = speed_ahead * speed_ahead + speed_side * speed_side;
        if (mc != nullptr && ground != 0 && !stairs && speed2 > 0.0) {
          double speed_dx = 0.0, speed_dz = 0.0;
          to_world(speed_ahead, speed_side, sin_f, cos_f, &speed_dx, &speed_dz);
          const double flat = std::sqrt(speed2);
          double scale =
              1.0 + mc->ground_share * (static_cast<double>(cM_scos(ground)) - 1.0);
          if (ground < 0) {
            const double grade =
                -(under.nx * (speed_dx / flat) + under.nz * (speed_dz / flat)) / under.ny;
            scale *= mc->up_level - mc->up_slope * grade;
          }
          if (scale < 0.0) scale = 0.0;
          dx += speed_dx * (scale - 1.0);
          dz += speed_dz * (scale - 1.0);
        }
      }

      const double nx = s.x + dx, nz = s.z + dz;
      walk_cells(grid, s.x, s.z, nx, nz, [&](int cell) {
        if (cell < 0) {
          s.left = true;
          return;
        }
        s.why |= grid.mark[static_cast<size_t>(cell)];
      });
      if (walls && !s.wall_near && wall_in_reach(grid, selection, s.x, s.z, nx, nz, s.y)) {
        s.wall_near = true;
      }
      s.x = nx;
      s.z = nz;
    }
  };

  const bool walls = collision == Collision::Solid;

  const int air_from = move.air_from, air_to = move.air_to;
  s.frames = move.frames;

  /* The previous frame's floor, for `Walk::go`'s `had`. */
  Floor held;
  bool carried = false;

  for (size_t f = 0; f < move.step.size(); ++f) {
    if (air_from >= 0 && static_cast<int>(f) == air_from) {
      /* The arc, re-integrated against the floor: each pure-air frame's rise differs from the last
         by `gravity`, continuing past the table's flight while the floor is out of reach. The
         table's landing frame is snapped, so it is not replayed. */
      const int air_last = air_to - 1;
      /* The airborne frames plus the landing one. */
      const int table_flight = air_last - air_from + 2;

      double rise = 0.0;
      int flew = 0;
      const int max_air = 96;
      /* The arc is extended only while there is floor under it; with none it would fly on to the
         cap and be priced (and pruned) far above its real cost. It is handed off instead. */
      bool lost_the_floor = false;
      for (; flew < max_air; ++flew) {
        const int at = air_from + flew;
        const bool from_table = at <= air_last;
        const BaseFrame& src = move.step[static_cast<size_t>(from_table ? at : air_last)];
        double dx = 0.0, dz = 0.0;
        to_world(src.ahead, src.side, sin_f, cos_f, &dx, &dz);
        Walk::go(s, grid, selection, mc, dx, dz, 0.0, 0.0, sin_f, cos_f, false, walls, nullptr,
                 s.facing);
        rise = from_table ? src.rise : rise + move.gravity;
        s.y += rise;
        s.facing = (facing + src.turn) & 0xFFFF;

        /* The arc ends on the frame it reaches the floor, not the one after, despite
           `GroundCheck`'s strict `ground > pos.y`: the game's frame has a mid-frame gravity dip
           this model does not carry. */
        const Floor landing = floor_at(grid, selection, s.x, s.z, s.y);
        if (!landing.found) {
          lost_the_floor = true;
          if (flew + 1 >= table_flight) {
            ++flew;
            break;
          }
        }
        if (landing.found && landing.y >= s.y) {
          s.y = landing.y;
          /* `ny` is exactly 1 on flat ground (`grid.cpp` normalises in double). */
          if (landing.ny < 1.0) s.air_landing = true;
          ++flew;
          break;
        }
      }
      if (flew >= max_air || lost_the_floor) s.why |= kNoGround;
      s.frames += flew - table_flight;
      carried = false;
      /* Resume after the table's landing frame. */
      f = static_cast<size_t>(air_last) + 1;
      continue;
    }

    const BaseFrame& bf = move.step[f];
    double dx = 0.0, dz = 0.0;
    to_world(bf.ahead, bf.side, sin_f, cos_f, &dx, &dz);
    /* The facing the frame began at, which the game prices it by. */
    Walk::go(s, grid, selection, mc, dx, dz, bf.speed_ahead, bf.speed_side, sin_f, cos_f, true,
             walls, carried ? &held : nullptr, s.facing);

    const double probe = s.y;
    const Floor landed = floor_at(grid, selection, s.x, s.z, s.y);
    s.y = landed.found ? landed.y : s.y + bf.rise;
    s.facing = (facing + bf.turn) & 0xFFFF;
    held = landed;
    carried = landed.found && landed.y <= probe;
  }

  /* A grounded move that stood on one sloped plane takes its end from the plane table, where the
     table's measured error beats the scale model's. The frames were still walked for marks and
     walls. */
  if (mc != nullptr && s.sloped_frames > 0 && s.stairs_frames == 0 && s.planes == 1 &&
      air_from < 0 && s.plane[1] > 0.0) {
    if (pt != nullptr && pt->worst < mc->worst) {
      const double gx = -s.plane[0] / s.plane[1], gz = -s.plane[2] / s.plane[1];
      const double ga = gx * sin_f + gz * cos_f;
      const double gc = gx * cos_f - gz * sin_f;
      const float nx = static_cast<float>(s.plane[0]);
      const float ny = static_cast<float>(s.plane[1]);
      const float nz = static_cast<float>(s.plane[2]);
      const float swung = cM_scos(static_cast<s16>(cM_atan2s(nx, nz) - static_cast<s16>(facing)));
      const s16 ground = cM_atan2s(std::sqrt(nx * nx + nz * nz) * swung, ny);
      double ea = 0.0, es = 0.0;
      int extra = 0;
      if (pt->at(ga, gc, ground == 0, ground < 0, &ea, &es, &extra)) {
        double dx = 0.0, dz = 0.0;
        to_world(ea, es, sin_f, cos_f, &dx, &dz);
        s.x = x + dx;
        s.z = z + dz;
        s.y = (s.plane[3] - s.plane[0] * s.x - s.plane[2] * s.z) / s.plane[1];
        s.frames += extra;
        s.tabled = true;
      }
    }
  }

  s.ok = true;
  const bool charged = mc != nullptr && mc->rows > 0 && mc->frames > 0;
  s.unmeasured = !charged && s.sloped_frames > 0 && move.reach > 1.0;
  s.handed_off = s.why != 0 || s.left || s.air_landing || s.wall_near || s.unmeasured;
  return s;
}

bool PlaneTable::at(double ga, double gc, bool level_angle, bool uphill, double* end_ahead,
                    double* end_side, int* frames) const {
  if (rows < 2 || cols < 2 || max_grade <= edge) return false;
  if (std::fabs(ga) > max_grade || std::fabs(gc) > max_grade) return false;
  const double tc = (gc + max_grade) / (2.0 * max_grade) * (cols - 1);
  int c0 = static_cast<int>(std::floor(tc));
  if (c0 < 0) c0 = 0;
  if (c0 > cols - 2) c0 = cols - 2;
  const double fc = tc - c0;

  /* Rows chosen by the game's angle, not the grade: an off-level grade inside `edge` clamps to its
     side's first row. */
  int r0 = rows, r1 = rows;
  double fr = 0.0;
  if (!level_angle) {
    double u = (std::fabs(ga) - edge) / (max_grade - edge) * (rows - 1);
    if (u < 0.0) u = 0.0;
    int k = static_cast<int>(std::floor(u));
    if (k > rows - 2) k = rows - 2;
    fr = u - k;
    if (uphill) {
      r0 = rows + 1 + k;
      r1 = r0 + 1;
    } else {
      r0 = rows - 1 - k;
      r1 = r0 - 1;
    }
  }
  const int n00 = r0 * cols + c0, n01 = r0 * cols + c0 + 1;
  const int n10 = r1 * cols + c0, n11 = r1 * cols + c0 + 1;
  const uint8_t k0 = key[static_cast<size_t>(n00)];
  if (k0 == 255 || key[static_cast<size_t>(n01)] != k0 || key[static_cast<size_t>(n10)] != k0 ||
      key[static_cast<size_t>(n11)] != k0) {
    return false;
  }
  const double w00 = (1.0 - fr) * (1.0 - fc), w01 = (1.0 - fr) * fc;
  const double w10 = fr * (1.0 - fc), w11 = fr * fc;
  *end_ahead = w00 * ahead[static_cast<size_t>(n00)] + w01 * ahead[static_cast<size_t>(n01)] +
               w10 * ahead[static_cast<size_t>(n10)] + w11 * ahead[static_cast<size_t>(n11)];
  *end_side = w00 * side[static_cast<size_t>(n00)] + w01 * side[static_cast<size_t>(n01)] +
              w10 * side[static_cast<size_t>(n10)] + w11 * side[static_cast<size_t>(n11)];
  /* One key has one frame count. */
  *frames = extra_frames[static_cast<size_t>(n00)];
  return true;
}

const MoveCal* Calibration::of(const std::string& id) const {
  for (size_t i = 0; i < move.size(); ++i) {
    if (move[i].id == id) return &move[i];
  }
  return nullptr;
}

double Calibration::threshold(const std::string& id, int sloped_frames) const {
  /* Before `of`, which is a linear string scan on a hot path. */
  if (sloped_frames <= 0) return 0.0;
  return charge_of(of(id), nullptr, sloped_frames, false);
}

double Calibration::threshold(const std::string& id, int sloped_frames, bool tabled) const {
  if (!tabled) return threshold(id, sloped_frames);
  return charge_of(of(id), plane_of(id), sloped_frames, true);
}

double charge_of(const MoveCal* mc, const PlaneTable* pt, int sloped_frames, bool tabled) {
  if (tabled && pt != nullptr) return pt->worst;
  if (sloped_frames <= 0) return 0.0;
  if (mc == nullptr || mc->rows == 0 || mc->frames <= 0) return 0.0;

  /* `worst` was measured with every frame sloped, so it is scaled by the sloped share. */
  const double share = static_cast<double>(sloped_frames) / static_cast<double>(mc->frames);
  return mc->worst * (share > 1.0 ? 1.0 : share);
}

const PlaneTable* Calibration::plane_of(const std::string& id) const {
  for (size_t i = 0; i < plane.size(); ++i) {
    if (plane[i].id == id) return &plane[i];
  }
  return nullptr;
}


namespace {

/** One generated row. */
void cal_row(Calibration& c, const char* id, double ground_share, double up_level,
             double up_slope, double worst,
             double mean, int rows, int frames) {
  MoveCal row;
  row.id = id;
  row.ground_share = ground_share;
  row.up_level = up_level;
  row.up_slope = up_slope;
  row.worst = worst;
  row.mean = mean;
  row.rows = rows;
  row.frames = frames;
  c.move.push_back(row);
}

/** One generated plane table. */
void plane_row(Calibration& c, const char* id, double max_grade, double edge, int rows, int cols,
               double worst, double mean, int cells, int answered, const float* ahead,
               const float* side, const uint8_t* key, const int8_t* extra) {
  PlaneTable t;
  t.id = id;
  t.max_grade = max_grade;
  t.edge = edge;
  t.rows = rows;
  t.cols = cols;
  t.worst = worst;
  t.mean = mean;
  t.cells = cells;
  t.answered = answered;
  const size_t n = static_cast<size_t>(2 * rows + 1) * static_cast<size_t>(cols);
  t.ahead.assign(ahead, ahead + n);
  t.side.assign(side, side + n);
  t.key.assign(key, key + n);
  t.extra_frames.assign(extra, extra + n);
  c.plane.push_back(t);
}

/* Generated by `bench/plane_table.cpp`. */
void plane_tables(Calibration& c) {
#include "plane_table.inc"
}

}  // namespace

/* Generated by `bench/calibrate.cpp`. */
Calibration Calibration::measured() {
  Calibration c;
#include "calibration.inc"
  plane_tables(c);
  return c;
}

}  // namespace search
