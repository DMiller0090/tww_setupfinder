/* Writes `src/search/plane_table.inc`: each grounded move's engine end over a grid of planes.
 *   setupcore_plane_table [max grade] [rows each side] [columns] [threads] [out path|-] [move]
 */
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "../src/disc/link.h"
#include "../src/search/approx.h"
#include "../src/search/catalogue.h"
#include "../src/search/corridor.h"
#include "../src/search/dzb.h"
#include "../src/search/grid.h"
#include "SSystem/SComponent/c_math.h"
#include "engine/session.h"

namespace {

/** Planted foot (`m34BC`) only on frames where `m3598 > 0`; elsewhere it moves nothing. */
struct Feet : search::Watcher {
  std::string planted;
  /** Arms from the engine's own `m34E2`, not a recomputed angle: they can differ by one entry. */
  bool level = false, up = false, down = false;
  void frame(int, const daPy_lk_c& lk, bool) override {
    if (lk.m3598 > 0.0f) planted += static_cast<char>('0' + lk.m34BC);
    if (lk.m34E2 == 0) level = true;
    if (lk.m34E2 < 0) up = true;
    if (lk.m34E2 > 0) down = true;
  }
  std::string arms() const {
    return std::string(level ? "L" : "") + (up ? "U" : "") + (down ? "D" : "");
  }
};

/** One engine run over the plane (ga ahead, gc across) at facing 0. */
struct Node {
  bool ok = false;
  double ahead = 0, side = 0;
  int frames = 0;
  std::string key;
};

Node drive_on(const std::string& id, double ga, double gc) {
  Node n;
  search::Move m;
  if (!search::move_of(id, 0, &m)) return n;
  /* At facing 0, ahead is +z and side is +x. */
  const search::Slab world = search::slab(0.0, gc, ga, 8192.0);
  tww_engine::Init init;
  init.pos.set(0.0f, static_cast<f32>(world.at(0.0, 0.0)), 0.0f);
  init.proc = daPy_lk_c::daPyProc_WAIT_e;
  Feet feet;
  const search::Drive d = search::drive(m, init, &world.room, tww_engine::RunOptions(), 96, &feet);
  if (!d.ok || !d.rested || d.dispatch_calls > 0) return n;
  n.ok = true;
  n.ahead = d.dz;
  n.side = d.dx;
  n.frames = d.frames;
  char head[16];
  std::snprintf(head, sizeof head, "%d:", d.frames);
  n.key = feet.arms() + head + feet.planted;
  return n;
}

s16 ground_angle(double ga, double gc);

/** The game's s16 ground angle for a plane along facing 0, as `step_move` computes it. */
s16 ground_angle(double ga, double gc) {
  const search::Slab w = search::slab(0.0, gc, ga, 8192.0);
  const float nx = static_cast<float>(w.nx), ny = static_cast<float>(w.ny);
  const float nz = static_cast<float>(w.nz);
  const float swung = cM_scos(static_cast<s16>(cM_atan2s(nx, nz)));
  return cM_atan2s(std::sqrt(nx * nx + nz * nz) * swung, ny);
}

/** Run `work(i)` for i in [0, n) on `threads` threads. Each drives flat ground first: a first
 *  drive over a slope crashes the process. */
template <typename F>
void each(int n, int threads, F work) {
  std::atomic<int> next(0);
  std::vector<std::thread> pool;
  for (int t = 0; t < threads; ++t) {
    pool.emplace_back([&]() {
      {
        search::Move m;
        search::move_of("dry_roll", 0, &m);
        const tww_engine::RoomDzb flat = tww_engine::flat_floor_dzb(0.0f);
        tww_engine::Init init;
        init.proc = daPy_lk_c::daPyProc_WAIT_e;
        search::drive(m, init, &flat);
      }
      for (int i = next++; i < n; i = next++) work(i);
    });
  }
  for (size_t t = 0; t < pool.size(); ++t) pool[t].join();
}

}  // namespace

int main(int argc, char** argv) {
  {
    std::string why;
    if (!disc::install_link_from_settings(&why)) std::fprintf(stderr, "link: %s\n", why.c_str());
  }
  const double max_grade = argc > 1 ? std::atof(argv[1]) : 0.30;
  const int rows = argc > 2 ? std::atoi(argv[2]) : 13;
  const int cols = argc > 3 ? std::atoi(argv[3]) : 25;
  int threads = argc > 4 ? std::atoi(argv[4]) : 6;
  if (threads < 1) threads = 1;
  if (threads > 6) threads = 6;
  const std::string out_path = argc > 5 ? argv[5] : "src/search/plane_table.inc";
  const std::string only = argc > 6 ? argv[6] : "";
  std::setvbuf(stdout, nullptr, _IONBF, 0);

  /* `cM_atan2s` quantises at 1/1024, so grades just past it can still read as level. */
  const double edge = 1.5 / 1024.0;
  const search::BaseTable base = search::base_table(true);
  search::Calibration cal = search::Calibration::measured();
  cal.plane.clear();

  auto ga_of = [&](int r) {
    if (r == rows) return 0.0;
    const double step = (max_grade - edge) / (rows - 1);
    return r > rows ? edge + (r - rows - 1) * step : -(edge + (rows - 1 - r) * step);
  };
  auto gc_of = [&](int c) { return -max_grade + c * (2.0 * max_grade) / (cols - 1); };
  const int n_nodes = (2 * rows + 1) * cols;

  std::printf("plane tables: grade to %.2f, %d rows each side and a band, %d columns, %d threads\n",
              max_grade, rows, cols, threads);
  std::printf("%-18s %6s %6s %9s %9s %9s %11s\n", "move", "cells", "answer", "worst", "mean",
              "model", "wrong frames");

  for (size_t i = 0; i < base.move.size(); ++i) {
    const search::BaseMove& row = base.move[i];
    if (!row.driven || row.step.empty() || row.air_from >= 0) continue;
    if (!only.empty() && row.id != only) continue;
    const search::MoveCal* mc = cal.of(row.id);
    if (mc == nullptr) continue;

    std::vector<Node> node(static_cast<size_t>(n_nodes));
    each(n_nodes, threads, [&](int k) {
      node[static_cast<size_t>(k)] = drive_on(row.id, ga_of(k / cols), gc_of(k % cols));
    });

    search::PlaneTable t;
    t.id = row.id;
    t.max_grade = max_grade;
    t.edge = edge;
    t.rows = rows;
    t.cols = cols;
    std::map<std::string, int> keys;
    for (int k = 0; k < n_nodes; ++k) {
      const Node& n = node[static_cast<size_t>(k)];
      t.ahead.push_back(static_cast<float>(n.ahead));
      t.side.push_back(static_cast<float>(n.side));
      int id = 255;
      if (n.ok) {
        std::map<std::string, int>::iterator it = keys.find(n.key);
        if (it == keys.end()) {
          const int fresh = static_cast<int>(keys.size());
          it = keys.insert(std::make_pair(n.key, fresh < 255 ? fresh : 255)).first;
        }
        id = it->second;
      }
      t.key.push_back(static_cast<uint8_t>(id));
      int extra = n.frames - row.frames;
      if (extra < -127) extra = -127;
      if (extra > 127) extra = 127;
      t.extra_frames.push_back(static_cast<int8_t>(n.ok ? extra : 0));
    }

    struct Probe {
      double ga, gc;
    };
    std::vector<Probe> probe;
    for (int c = 0; c + 1 < cols; ++c) {
      const double gc = 0.5 * (gc_of(c) + gc_of(c + 1));
      for (int r = 0; r + 1 < rows; ++r) probe.push_back({0.5 * (ga_of(r) + ga_of(r + 1)), gc});
      for (int r = rows + 1; r + 1 <= 2 * rows; ++r) {
        probe.push_back({0.5 * (ga_of(r) + ga_of(r + 1)), gc});
      }
      probe.push_back({0.4 / 1024.0, gc});
      probe.push_back({-0.4 / 1024.0, gc});
    }
    std::vector<Node> truth(probe.size());
    each(static_cast<int>(probe.size()), threads, [&](int k) {
      truth[static_cast<size_t>(k)] = drive_on(row.id, probe[static_cast<size_t>(k)].ga,
                                               probe[static_cast<size_t>(k)].gc);
    });
    double worst = 0, sum = 0;
    int answered = 0, wrong_frames = 0;
    size_t at_worst = 0;
    double worst_ea = 0, worst_es = 0;
    for (size_t k = 0; k < probe.size(); ++k) {
      if (!truth[k].ok) continue;
      const s16 g = ground_angle(probe[k].ga, probe[k].gc);
      double ea = 0, es = 0;
      int extra = 0;
      if (!t.at(probe[k].ga, probe[k].gc, g == 0, g < 0, &ea, &es, &extra)) continue;
      ++answered;
      if (extra != truth[k].frames - row.frames) ++wrong_frames;
      const double e = std::hypot(ea - truth[k].ahead, es - truth[k].side);
      sum += e;
      if (e > worst) {
        worst = e;
        at_worst = k;
        worst_ea = ea;
        worst_es = es;
      }
    }
    t.cells = static_cast<int>(probe.size());
    t.answered = answered;
    t.worst = worst;
    t.mean = answered ? sum / answered : 0.0;
    /* A wrong frame count means a wrong price, which the walk prunes on: reject the table. */
    if (wrong_frames > 0 || answered == 0) t.worst = 1e9;
    std::printf("%-18s %6d %6d %9.4f %9.4f %9.4f %11d\n", row.id.c_str(), t.cells, answered,
                t.worst, t.mean, mc->worst, wrong_frames);
    if (answered > 0) {
      const Node& w = truth[at_worst];
      std::printf("  worst at plane (%.5f ahead, %.5f across): table (%.4f, %.4f), engine (%.4f, "
                  "%.4f) %s\n",
                  probe[at_worst].ga, probe[at_worst].gc, worst_ea, worst_es, w.ahead, w.side,
                  w.key.c_str());
    }
    cal.plane.push_back(t);
  }

  std::printf("\nend to end: step_move with the tables against the engine, off the tables' facing\n");
  std::printf("%-18s %6s %7s %9s %9s %9s\n", "move", "rows", "tabled", "worst", "mean", "untabled");
  struct Tally {
    int rows = 0, tabled = 0;
    double worst = 0, sum = 0, worst_untabled = 0;
  };
  std::map<std::string, Tally> tally;
  const int facings = 12;
  for (int s = -6; s <= 6; ++s) {
    const double slope = max_grade * s / 6.0 * 0.97;
    const search::Slab world = search::slab(0.0, 0.0, slope, 8192.0);
    const search::Corridor corridor = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
    const search::Selection selection = search::select(world.mesh, corridor);
    search::Limits limits;
    limits.slope_normal_y = 0.0;
    limits.height_step = 1e9;
    limits.normal_apart = 1e9;
    const search::Grid grid = search::build(selection, corridor, limits);
    struct Job {
      std::string id;
      int facing;
      double err = 0;
      bool tabled = false, ok = false;
    };
    std::vector<Job> job;
    for (int f = 0; f < facings; ++f) {
      const int facing = (f * 5461 + 1777) & 0xFFFF;
      for (size_t i = 0; i < cal.plane.size(); ++i) job.push_back({cal.plane[i].id, facing});
    }
    each(static_cast<int>(job.size()), threads, [&](int k) {
      Job& j = job[static_cast<size_t>(k)];
      search::Move m;
      if (!search::move_of(j.id, j.facing, &m)) return;
      tww_engine::Init init;
      init.pos.set(0.0f, static_cast<f32>(world.at(0.0, 0.0)), 0.0f);
      init.shape_angle_y = static_cast<s16>(j.facing);
      init.travel_angle_y = static_cast<s16>(j.facing);
      init.proc = daPy_lk_c::daPyProc_WAIT_e;
      const search::Drive d = search::drive(m, init, &world.room);
      if (!d.ok || !d.rested) return;
      const search::BaseMove* row = base.of(j.id);
      const search::Stepped st = search::step_move(*row, grid, selection, cal, init.pos.x,
                                                   init.pos.y, init.pos.z, j.facing);
      j.ok = st.ok;
      j.tabled = st.tabled;
      j.err = std::hypot(st.x - init.pos.x - d.dx, st.z - init.pos.z - d.dz);
    });
    for (size_t k = 0; k < job.size(); ++k) {
      if (!job[k].ok) continue;
      Tally& t = tally[job[k].id];
      ++t.rows;
      if (job[k].tabled) {
        ++t.tabled;
        t.sum += job[k].err;
        if (job[k].err > t.worst) t.worst = job[k].err;
      } else if (job[k].err > t.worst_untabled) {
        t.worst_untabled = job[k].err;
      }
    }
  }
  for (size_t i = 0; i < cal.plane.size(); ++i) {
    const Tally& t = tally[cal.plane[i].id];
    std::printf("%-18s %6d %7d %9.4f %9.4f %9.4f\n", cal.plane[i].id.c_str(), t.rows, t.tabled,
                t.worst, t.tabled ? t.sum / t.tabled : 0.0, t.worst_untabled);
  }
  /* Other facings carry the game trig table's rounding, so the stored worst includes the ladder. */
  std::printf("\nstored worst, the larger of the cell centres and the ladder\n");
  for (size_t i = 0; i < cal.plane.size(); ++i) {
    search::PlaneTable& p = cal.plane[i];
    const Tally& t = tally[p.id];
    if (t.worst > p.worst) p.worst = t.worst;
    const search::MoveCal* mc = cal.of(p.id);
    std::printf("%-18s %9.4f %s\n", p.id.c_str(), p.worst,
                mc != nullptr && p.worst < mc->worst ? "used" : "not used, the scale model is better");
  }

  if (out_path == "-") {
    std::printf("\nnothing written\n");
    return 0;
  }
  FILE* out = std::fopen(out_path.c_str(), "w");
  if (out == nullptr) {
    std::printf("\ncould not write %s\n", out_path.c_str());
    return 1;
  }
  std::fprintf(out,
               "/* GENERATED by `setupcore_plane_table`. Do not edit by hand - re-run the "
               "instrument.\n *\n"
               " * One table a grounded move: the engine's end over a grid of planes in the move's\n"
               " * own frame, grade to %.2f with %d rows each side of the level band and %d\n"
               " * columns, then the error measured at every cell's centre. `PlaneTable` in\n"
               " * `approx.h` says what each array is.\n */\n",
               max_grade, rows, cols);
  for (size_t i = 0; i < cal.plane.size(); ++i) {
    const search::PlaneTable& t = cal.plane[i];
    const char* names[4] = {"ahead", "side", "key", "extra"};
    for (int a = 0; a < 4; ++a) {
      std::fprintf(out, "static const %s kPlane%d_%s[] = {",
                   a < 2 ? "float" : (a == 2 ? "uint8_t" : "int8_t"), static_cast<int>(i),
                   names[a]);
      for (size_t k = 0; k < t.key.size(); ++k) {
        if (k % 8 == 0) std::fprintf(out, "\n   ");
        if (a == 0) std::fprintf(out, " %.6ff,", t.ahead[k]);
        if (a == 1) std::fprintf(out, " %.6ff,", t.side[k]);
        if (a == 2) std::fprintf(out, " %d,", t.key[k]);
        if (a == 3) std::fprintf(out, " %d,", t.extra_frames[k]);
      }
      std::fprintf(out, "\n};\n");
    }
    std::fprintf(out,
                 "plane_row(c, \"%s\", %.6f, %.9f, %d, %d, %.4f, %.4f, %d, %d, kPlane%d_ahead, "
                 "kPlane%d_side, kPlane%d_key, kPlane%d_extra);\n",
                 t.id.c_str(), t.max_grade, t.edge, t.rows, t.cols, t.worst, t.mean, t.cells,
                 t.answered, static_cast<int>(i), static_cast<int>(i), static_cast<int>(i),
                 static_cast<int>(i));
  }
  std::fclose(out);
  std::printf("\nwrote %s, %d tables\n", out_path.c_str(), static_cast<int>(cal.plane.size()));
  return 0;
}
