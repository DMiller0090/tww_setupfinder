#include "corridor.h"

#include <algorithm>
#include <cmath>

namespace search {
namespace {

double point_segment_sq(double px, double pz, double ax, double az, double bx, double bz) {
  const double dx = bx - ax, dz = bz - az;
  const double len_sq = dx * dx + dz * dz;
  double t = 0.0;
  if (len_sq > 0.0) {
    t = ((px - ax) * dx + (pz - az) * dz) / len_sq;
    t = std::max(0.0, std::min(1.0, t));
  }
  const double qx = ax + t * dx, qz = az + t * dz;
  const double ex = px - qx, ez = pz - qz;
  return ex * ex + ez * ez;
}

double segment_segment_sq(double ax, double az, double bx, double bz, double cx, double cz,
                          double dx, double dz) {
  const double r_x = bx - ax, r_z = bz - az;
  const double s_x = dx - cx, s_z = dz - cz;
  const double denom = r_x * s_z - r_z * s_x;
  if (denom != 0.0) {
    const double t = ((cx - ax) * s_z - (cz - az) * s_x) / denom;
    const double u = ((cx - ax) * r_z - (cz - az) * r_x) / denom;
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) return 0.0;
  }
  double best = point_segment_sq(ax, az, cx, cz, dx, dz);
  best = std::min(best, point_segment_sq(bx, bz, cx, cz, dx, dz));
  best = std::min(best, point_segment_sq(cx, cz, ax, az, bx, bz));
  best = std::min(best, point_segment_sq(dx, dz, ax, az, bx, bz));
  return best;
}

/** Edges included. */
bool inside(double px, double pz, const double* tx, const double* tz) {
  bool neg = false, pos = false;
  for (int i = 0; i < 3; ++i) {
    const int j = (i + 1) % 3;
    const double cross =
        (px - tx[i]) * (tz[j] - tz[i]) - (pz - tz[i]) * (tx[j] - tx[i]);
    if (cross < 0.0) neg = true;
    if (cross > 0.0) pos = true;
  }
  return !(neg && pos);
}

double segment_triangle_sq(double ax, double az, double bx, double bz, const double* tx,
                           const double* tz) {
  if (inside(ax, az, tx, tz) || inside(bx, bz, tx, tz)) return 0.0;
  double best = segment_segment_sq(ax, az, bx, bz, tx[0], tz[0], tx[1], tz[1]);
  best = std::min(best, segment_segment_sq(ax, az, bx, bz, tx[1], tz[1], tx[2], tz[2]));
  best = std::min(best, segment_segment_sq(ax, az, bx, bz, tx[2], tz[2], tx[0], tz[0]));
  return best;
}

/** Returns how many were held. Codes and sources travel with their triangles. */
int hold(const std::vector<float>& from, const Corridor& c, std::vector<float>* into,
         const std::vector<uint8_t>* from_code = nullptr, std::vector<uint8_t>* codes = nullptr,
         const std::vector<float>* from_source = nullptr, std::vector<float>* sources = nullptr) {
  const double reach_sq = c.half_width * c.half_width;
  int held = 0;
  for (size_t at = 0; at + 9 <= from.size(); at += 9) {
    const double tx[3] = {from[at + 0], from[at + 3], from[at + 6]};
    const double tz[3] = {from[at + 2], from[at + 5], from[at + 8]};

    const double lo_x = std::min({tx[0], tx[1], tx[2]});
    const double hi_x = std::max({tx[0], tx[1], tx[2]});
    const double lo_z = std::min({tz[0], tz[1], tz[2]});
    const double hi_z = std::max({tz[0], tz[1], tz[2]});
    if (hi_x < c.min_x || lo_x > c.max_x || hi_z < c.min_z || lo_z > c.max_z) continue;

    if (segment_triangle_sq(c.ax, c.az, c.bx, c.bz, tx, tz) > reach_sq) {
      /* Box against grown box: over-holds at corners, which is the safe direction. */
      if (!c.boxed) continue;
      if (hi_x < c.gx0 - c.half_width || lo_x > c.gx1 + c.half_width ||
          hi_z < c.gz0 - c.half_width || lo_z > c.gz1 + c.half_width) {
        continue;
      }
    }

    into->insert(into->end(), from.begin() + static_cast<long>(at),
                 from.begin() + static_cast<long>(at) + 9);
    if (codes != nullptr) {
      const size_t tri = at / 9;
      codes->push_back(from_code != nullptr && tri < from_code->size() ? (*from_code)[tri] : 0);
    }
    if (sources != nullptr) {
      const std::vector<float>& src =
          from_source != nullptr && at + 9 <= from_source->size() ? *from_source : from;
      sources->insert(sources->end(), src.begin() + static_cast<long>(at),
                      src.begin() + static_cast<long>(at) + 9);
    }
    ++held;
  }
  return held;
}

}  // namespace

double Corridor::from_spine(double x, double z) const {
  const double to_spine = std::sqrt(point_segment_sq(x, z, ax, az, bx, bz));
  if (!boxed) return to_spine;
  const double dx = x < gx0 ? gx0 - x : (x > gx1 ? x - gx1 : 0.0);
  const double dz = z < gz0 ? gz0 - z : (z > gz1 ? z - gz1 : 0.0);
  return std::min(to_spine, std::sqrt(dx * dx + dz * dz));
}

Corridor corridor(double start_x, double start_z, double target_x, double target_z,
                  double widest_move) {
  Corridor c;
  c.ax = start_x;
  c.az = start_z;
  c.bx = target_x;
  c.bz = target_z;
  c.half_width = widest_move > 0.0 ? widest_move : 0.0;
  c.min_x = std::min(start_x, target_x) - c.half_width;
  c.max_x = std::max(start_x, target_x) + c.half_width;
  c.min_z = std::min(start_z, target_z) - c.half_width;
  c.max_z = std::max(start_z, target_z) + c.half_width;
  return c;
}

Corridor corridor(double start_x, double start_z, const Target& target, const float room[6],
                  double widest_move) {
  double min_x = 0, min_z = 0, max_x = 0, max_z = 0;
  target.extent(room, &min_x, &min_z, &max_x, &max_z);
  /* Clamp to the room: a typed address range can be ~1e38 wide. An all-zero box is unread. */
  const bool bounded = room[3] > room[0] && room[5] > room[2];
  if (bounded) {
    const auto within = [](double v, float lo, float hi) {
      const double a = static_cast<double>(lo), b = static_cast<double>(hi);
      return v < a ? a : (v > b ? b : v);
    };
    min_x = within(min_x, room[0], room[3]);
    max_x = within(max_x, room[0], room[3]);
    min_z = within(min_z, room[2], room[5]);
    max_z = within(max_z, room[2], room[5]);
  }
  const bool one_place = !target.ranged && target.has_x && target.has_z;
  if (one_place) return corridor(start_x, start_z, target.x, target.z, widest_move);

  Corridor c = corridor(start_x, start_z, (min_x + max_x) / 2.0, (min_z + max_z) / 2.0,
                        widest_move);
  c.boxed = true;
  c.gx0 = min_x;
  c.gx1 = max_x;
  c.gz0 = min_z;
  c.gz1 = max_z;
  c.min_x = std::min(c.min_x, min_x - c.half_width);
  c.max_x = std::max(c.max_x, max_x + c.half_width);
  c.min_z = std::min(c.min_z, min_z - c.half_width);
  c.max_z = std::max(c.max_z, max_z + c.half_width);
  return c;
}

Selection select(const geom::Mesh& mesh, const Corridor& c) {
  Selection s;
  s.ground_total = static_cast<int>(mesh.ground.size() / 9);
  s.wall_total = static_cast<int>(mesh.wall.size() / 9);
  s.ground_held = hold(mesh.ground, c, &s.ground, &mesh.ground_code, &s.ground_code,
                       &mesh.ground_source, &s.ground_source);
  s.wall_held = hold(mesh.wall, c, &s.wall);
  return s;
}

}  // namespace search
