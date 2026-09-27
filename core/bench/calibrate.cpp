/* Fits the slope correction against the engine on a slab ladder and writes
 * `src/search/calibration.inc` with each move's coefficients and residual error.
 *   setupcore_calibrate [max grade] [steps] [facings] [out path|-]
 *     [-fine] [-charge N] [-threads N] [-all] [-batch k n] [-join n] [-only a,b] [-patch rows facings]
 */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <string>
#include <thread>
#include <utility>
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

const char* built_by() {
#if defined(_MSC_VER)
  static char buf[32];
  std::snprintf(buf, sizeof buf, "MSVC %d.%d", _MSC_VER / 100, _MSC_VER % 100);
  return buf;
#elif defined(__clang__)
  return "clang " __clang_version__;
#elif defined(__GNUC__)
  return "g++";
#else
  return "an unknown compiler";
#endif
}

/** One move, one slope, one facing: the engine's end and the model's. */
struct Row {
  std::string id;
  int table_frames = 0;
  int engine_frames = 0;
  int model_frames = 0;
  int facing = 0;
  double slope = 0;
  /** Along the direction travelled, positive uphill. */
  double grade = 0;
  double true_x = 0, true_z = 0;
  double model_x = 0, model_z = 0;
  /** Travel along the heading, model and engine. */
  double flat_along = 0, true_along = 0;
  /** The part of `flat_along` carried through `speedF`, the only part the correction scales. */
  double speed_along = 0;
  bool airborne = false;
  bool locks = false;
  double true_y = 0, model_y = 0;
  bool handed_off = false;
};

double hypot2(double x, double z) { return std::sqrt(x * x + z * z); }

double miss(const Row& r) { return hypot2(r.true_x - r.model_x, r.true_z - r.model_z); }

/** Every row, locked ones included: with no target a lock holds the facing (`setAtnList`). */
bool usable(const Row&) { return true; }

/** `cM_atan2s` reads `atntable[(int)(ratio * 1024)]`: a gentler grade is level to the game. */
const double kGroundStep = 1.0 / 1024.0;

/** The share of `speedF` travel the slope left, with the root's part held at its flat value. */
double speed_ratio(const Row& r) {
  return (r.true_along - (r.flat_along - r.speed_along)) / r.speed_along;
}

/** Below this much `speedF` travel the fit cannot read the slope above the root's noise. */
const double kSpeedFloor = 1.0;

/** Moves not calibrated; the engine takes them wherever they cross a slope. */
std::set<std::string> left_out;

void collect_slope(const search::BaseTable& base, double slope, int facings,
                   const search::Calibration& cal, std::vector<Row>* out) {
  std::vector<Row>& rows = *out;
  {
    const search::Slab world = search::slab(0.0, 0.0, slope, 8192.0);

    /* Wide enough that no move leaves the grid. */
    const search::Corridor corridor = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
    const search::Selection selection = search::select(world.mesh, corridor);
    search::Limits limits;
    /* Off, so no cell is marked and the model answers everywhere. */
    limits.slope_normal_y = 0.0;
    limits.height_step = 1e9;
    limits.normal_apart = 1e9;
    const search::Grid grid = search::build(selection, corridor, limits);

    for (int f = 0; f < facings; ++f) {
      const int facing = static_cast<int>(65536.0 * f / facings) & 0xFFFF;

      tww_engine::Init init;
      init.pos.set(0.0f, static_cast<f32>(world.at(0.0, 0.0)), 0.0f);
      init.shape_angle_y = static_cast<s16>(facing);
      init.travel_angle_y = static_cast<s16>(facing);
      init.proc = daPy_lk_c::daPyProc_WAIT_e;

      for (size_t i = 0; i < base.move.size(); ++i) {
        const search::BaseMove& row = base.move[i];
        if (!row.driven || left_out.count(row.id) != 0) continue;
        search::Move m;
        if (!search::move_of(row.id, facing, &m)) continue;

        const search::Drive truth = search::drive(m, init, &world.room);
        if (!truth.ok || !truth.rested || truth.dispatch_calls > 0) continue;

        const search::Stepped guess = search::step_move(row, grid, selection, cal, init.pos.x,
                                                        init.pos.y, init.pos.z, facing);

        Row r;
        r.id = row.id;
        r.table_frames = row.frames;
        r.engine_frames = truth.frames;
        r.model_frames = guess.frames;
        r.facing = facing;
        r.slope = slope;
        r.true_x = truth.dx;
        r.true_z = truth.dz;
        r.model_x = guess.x - init.pos.x;
        r.model_z = guess.z - init.pos.z;
        r.true_y = truth.dy;
        r.model_y = guess.y - init.pos.y;
        r.airborne = row.air_from >= 0;
        r.handed_off = guess.handed_off;
        for (size_t k = 0; k < m.action.size(); ++k) {
          if (m.action[k].lock) r.locks = true;
        }

        const double len = hypot2(r.model_x, r.model_z);
        if (len > 1e-9) {
          const double ux = r.model_x / len, uz = r.model_z / len;
          r.grade = -(world.nx * ux + world.nz * uz) / world.ny;
          r.flat_along = len;
          r.true_along = r.true_x * ux + r.true_z * uz;
          double sa = 0, ss = 0;
          for (size_t k = 0; k < row.step.size(); ++k) {
            sa += row.step[k].speed_ahead;
            ss += row.step[k].speed_side;
          }
          /* The game's trig, as `step_move` uses. */
          const double fs = static_cast<double>(cM_ssin(static_cast<s16>(facing)));
          const double fc = static_cast<double>(cM_scos(static_cast<s16>(facing)));
          r.speed_along = (sa * fs + ss * fc) * ux + (sa * fc - ss * fs) * uz;
        }
        rows.push_back(r);
      }
    }
  }
}

int threads = 6;

/** Drive everything once with the model on `cal`. Rows come back in slope order, so the output
 *  is the same bytes at any thread count. */
std::vector<Row> collect(const search::BaseTable& base, const std::vector<double>& slopes,
                         int facings, const search::Calibration& cal) {
  std::vector<std::vector<Row>> per(slopes.size());
  std::atomic<size_t> next(0);
  auto work = [&]() {
    for (size_t s = next++; s < slopes.size(); s = next++) {
      collect_slope(base, slopes[s], facings, cal, &per[s]);
    }
  };
  std::vector<std::thread> pool;
  for (int t = 1; t < threads; ++t) pool.push_back(std::thread(work));
  work();
  for (size_t t = 0; t < pool.size(); ++t) pool[t].join();
  std::vector<Row> rows;
  for (size_t s = 0; s < per.size(); ++s) rows.insert(rows.end(), per[s].begin(), per[s].end());
  return rows;
}

/** Distinct move ids, in first-seen order. */
std::vector<std::string> ids_of(const std::vector<Row>& rows) {
  std::vector<std::string> out;
  for (size_t i = 0; i < rows.size(); ++i) {
    bool seen = false;
    for (size_t j = 0; j < out.size(); ++j) {
      if (out[j] == rows[i].id) seen = true;
    }
    if (!seen && usable(rows[i])) out.push_back(rows[i].id);
  }
  return out;
}

std::string part_path(const std::string& out_path, int k);
int write_table(const std::string& out_path, const std::string& rows_text, double max_grade,
                int steps, int facings, int charge_steps, int left,
                const std::vector<std::pair<int, int>>& patched =
                    std::vector<std::pair<int, int>>());

}  // namespace

int main(int argc, char** argv) {
  {
    std::string why;
    if (!disc::install_link_from_settings(&why)) std::fprintf(stderr, "link: %s\n", why.c_str());
  }
  const double max_grade = argc > 1 ? std::atof(argv[1]) : 0.30;
  const int steps = argc > 2 ? std::atoi(argv[2]) : 6;
  const int facings = argc > 3 ? std::atoi(argv[3]) : 8;
  const std::string out_path = argc > 4 ? argv[4] : "src/search/calibration.inc";
  bool fine = false;
  bool every_group = false;
  int charge_steps = steps;
  /* Each move's row depends on its own rows only, so `-batch` slices join to the same bytes. */
  int batch_k = 0, batch_n = 0, join_n = 0;
  /* `-join N -patch <rows> <facings>` replaces rows by move; a later patch wins. */
  std::string only;
  std::vector<std::pair<std::string, int>> patches;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "-fine") fine = true;
    if (std::string(argv[i]) == "-charge" && i + 1 < argc) charge_steps = std::atoi(argv[i + 1]);
    if (std::string(argv[i]) == "-threads" && i + 1 < argc) threads = std::atoi(argv[i + 1]);
    if (std::string(argv[i]) == "-all") every_group = true;
    if (std::string(argv[i]) == "-batch" && i + 2 < argc) {
      batch_k = std::atoi(argv[i + 1]);
      batch_n = std::atoi(argv[i + 2]);
    }
    if (std::string(argv[i]) == "-join" && i + 1 < argc) join_n = std::atoi(argv[i + 1]);
    if (std::string(argv[i]) == "-only" && i + 1 < argc) only = argv[i + 1];
    if (std::string(argv[i]) == "-patch" && i + 2 < argc) {
      patches.push_back(std::make_pair(std::string(argv[i + 1]), std::atoi(argv[i + 2])));
    }
  }
  if (batch_n > 0 && (batch_k < 0 || batch_k >= batch_n)) {
    std::printf("refused: batch %d of %d\n", batch_k, batch_n);
    return 1;
  }
  threads = std::max(1, std::min(threads, 6));
  /* A charge ladder coarser than the fit's would under-charge slopes. */
  if (steps < 1 || charge_steps < steps || facings < 1 || !(max_grade > 0.0)) {
    std::printf("refused: steps %d, charge %d, facings %d, grade %g\n", steps, charge_steps,
                facings, max_grade);
    return 1;
  }
  if (!every_group) {
    const std::vector<search::Move> all = search::roster(0, true);
    for (size_t i = 0; i < all.size(); ++i) {
      if (all[i].type == "combo_rest") left_out.insert(all[i].id);
    }
  }
  if (join_n > 0) {
    std::string rows_text;
    for (int k = 0; k < join_n; ++k) {
      const std::string part = part_path(out_path, k);
      FILE* p = std::fopen(part.c_str(), "rb");
      if (p == nullptr) {
        std::printf("refused: %s is missing, so the table would be short\n", part.c_str());
        return 1;
      }
      char buf[4096];
      size_t got = 0;
      while ((got = std::fread(buf, 1, sizeof buf, p)) > 0) rows_text.append(buf, got);
      std::fclose(p);
    }
    /* Facings each row was last measured on; 0 is the batches' own. */
    std::vector<int> row_facings;
    for (size_t n = 0; n < patches.size(); ++n) {
      const std::string& patch_path = patches[n].first;
      std::string patch;
      FILE* p = std::fopen(patch_path.c_str(), "rb");
      if (p == nullptr) {
        std::printf("refused: %s is missing\n", patch_path.c_str());
        return 1;
      }
      char buf[4096];
      size_t got = 0;
      while ((got = std::fread(buf, 1, sizeof buf, p)) > 0) patch.append(buf, got);
      std::fclose(p);
      /* One row a line, named by its first quoted word. */
      struct Rows {
        static std::vector<std::string> split(const std::string& text) {
          std::vector<std::string> out;
          size_t at = 0;
          while (at < text.size()) {
            const size_t end = text.find('\n', at);
            const size_t stop = end == std::string::npos ? text.size() : end + 1;
            out.push_back(text.substr(at, stop - at));
            at = stop;
          }
          return out;
        }
        static std::string id(const std::string& line) {
          const size_t a = line.find('"');
          const size_t b = a == std::string::npos ? a : line.find('"', a + 1);
          return b == std::string::npos ? std::string() : line.substr(a + 1, b - a - 1);
        }
      };
      std::vector<std::string> lines = Rows::split(rows_text);
      row_facings.resize(lines.size(), 0);
      const std::vector<std::string> fresh = Rows::split(patch);
      for (size_t f = 0; f < fresh.size(); ++f) {
        const std::string id = Rows::id(fresh[f]);
        bool found = false;
        for (size_t i = 0; i < lines.size(); ++i) {
          if (!id.empty() && Rows::id(lines[i]) == id) {
            lines[i] = fresh[f];
            row_facings[i] = patches[n].second;
            found = true;
          }
        }
        if (!found) {
          std::printf("refused: %s is in the patch and not in the table\n", id.c_str());
          return 1;
        }
      }
      rows_text.clear();
      for (size_t i = 0; i < lines.size(); ++i) rows_text += lines[i];
    }
    std::vector<std::pair<int, int>> patched;
    for (size_t n = 0; n < patches.size(); ++n) {
      int count = 0;
      for (size_t i = 0; i < row_facings.size(); ++i) {
        if (row_facings[i] == patches[n].second) ++count;
      }
      bool seen = false;
      for (size_t j = 0; j < patched.size(); ++j) {
        if (patched[j].second == patches[n].second) seen = true;
      }
      if (count > 0 && !seen) patched.push_back(std::make_pair(count, patches[n].second));
    }
    return write_table(out_path, rows_text, max_grade, steps, facings, charge_steps,
                       static_cast<int>(left_out.size()), patched);
  }
  /* Unbuffered: `JUT_ASSERT` calls `abort()`. */
  std::setvbuf(stdout, nullptr, _IONBF, 0);

  std::printf("calibrate: the slope correction against tww_engine, on one machine\n");
  std::printf("  built by %s, %s\n", built_by(),
#ifdef NDEBUG
              "release"
#else
              "debug"
#endif
  );

  const std::chrono::steady_clock::time_point began = std::chrono::steady_clock::now();
  const search::BaseTable base = search::base_table(true);
  const double built_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
  std::printf("  base table: %d moves driven, %d not, widest net %.4f units, built in %.0f ms\n",
              base.driven, base.not_driven, base.widest, built_ms);
  std::printf("  %d left out by group, %d threads\n", static_cast<int>(left_out.size()), threads);

  if (batch_n > 0) {
    std::vector<std::string> in;
    for (size_t i = 0; i < base.move.size(); ++i) {
      if (base.move[i].driven && left_out.count(base.move[i].id) == 0) {
        in.push_back(base.move[i].id);
      }
    }
    const size_t from = in.size() * static_cast<size_t>(batch_k) / static_cast<size_t>(batch_n);
    const size_t to = in.size() * static_cast<size_t>(batch_k + 1) / static_cast<size_t>(batch_n);
    for (size_t i = 0; i < in.size(); ++i) {
      if (i < from || i >= to) left_out.insert(in[i]);
    }
    std::printf("  batch %d of %d: moves %d to %d of %d\n", batch_k, batch_n,
                static_cast<int>(from), static_cast<int>(to) - 1, static_cast<int>(in.size()));
  }
  if (!only.empty()) {
    std::set<std::string> named;
    size_t at = 0;
    while (at <= only.size()) {
      const size_t comma = only.find(',', at);
      const size_t stop = comma == std::string::npos ? only.size() : comma;
      if (stop > at) named.insert(only.substr(at, stop - at));
      at = stop + 1;
    }
    int kept = 0;
    for (size_t i = 0; i < base.move.size(); ++i) {
      if (named.count(base.move[i].id) == 0) {
        left_out.insert(base.move[i].id);
      } else if (base.move[i].driven) {
        ++kept;
      }
    }
    std::printf("  only %d named, %d of them driven\n", static_cast<int>(named.size()), kept);
    if (kept != static_cast<int>(named.size())) {
      std::printf("refused: a named move is not in the table or is not driven\n");
      return 1;
    }
  }

  /* Slopes in z only; the ring of facings covers every direction of travel. */
  std::vector<double> slopes;
  slopes.push_back(0.0);
  for (int i = 1; i <= steps; ++i) {
    slopes.push_back(max_grade * i / steps);
    slopes.push_back(-max_grade * i / steps);
  }

  const std::vector<Row> before = collect(base, slopes, facings, search::Calibration());
  const std::vector<std::string> ids = ids_of(before);

  /* The base table is measured at facing 0; check other facings on level ground. */
  double worst_rotation = 0.0;
  int rotation_rows = 0, camera_rows = 0;
  for (size_t i = 0; i < before.size(); ++i) {
    const Row& r = before[i];
    if (r.facing == 0 || r.slope != 0.0) continue;
    if (!usable(r)) {
      ++camera_rows;
      continue;
    }
    ++rotation_rows;
    if (miss(r) > worst_rotation) worst_rotation = miss(r);
  }
  std::printf("\nrotation on level ground: %d rows off facing 0, worst miss %.6f units\n",
              rotation_rows, worst_rotation);
  std::printf("  %d rows left out: the move holds the lock, and this port has no camera\n",
              camera_rows);

  /* Rows whose frame count changed on a slope, and how many of those the model predicted. */
  int moved = 0, predicted = 0, worst_gap = 0;
  for (size_t i = 0; i < before.size(); ++i) {
    const Row& r = before[i];
    if (!usable(r)) continue;
    const int gap = r.engine_frames - r.table_frames;
    if (gap == 0) continue;
    ++moved;
    if (r.model_frames == r.engine_frames) ++predicted;
    if (std::abs(gap) > std::abs(worst_gap)) worst_gap = gap;
  }
  std::printf("\nframe cost on a slope: %d rows cost something other than the table's count "
              "(worst %+d)\n", moved, worst_gap);
  std::printf("  the model predicted the engine's count on %d of those %d\n", predicted, moved);

  /* The raw ratio per rung, to check the uphill fit's assumed step at zero. */
  if (fine) {
    std::printf("\nthe ratio the engine travelled, rung by rung, both signs\n");
    std::printf("%-18s", "move");
    std::vector<double> rung;
    for (size_t i = 0; i < before.size(); ++i) {
      if (std::fabs(before[i].slope) <= 1e-12 || !usable(before[i])) continue;
      bool seen = false;
      for (size_t j = 0; j < rung.size(); ++j) {
        if (std::fabs(rung[j] - before[i].slope) < 1e-15) seen = true;
      }
      if (!seen) rung.push_back(before[i].slope);
    }
    std::sort(rung.begin(), rung.end());
    for (size_t j = 0; j < rung.size(); ++j) std::printf(" %10.3e", rung[j]);
    std::printf("\n");
    for (size_t k = 0; k < ids.size(); ++k) {
      std::printf("%-18s", ids[k].c_str());
      for (size_t j = 0; j < rung.size(); ++j) {
        double sum = 0;
        int n = 0;
        for (size_t i = 0; i < before.size(); ++i) {
          const Row& r = before[i];
          if (r.id != ids[k] || !usable(r) || r.airborne) continue;
          if (std::fabs(r.slope - rung[j]) > 1e-15 || r.flat_along <= 1e-9) continue;
          if (std::fabs(r.grade) <= 1e-12) continue;
          sum += r.true_along / r.flat_along;
          ++n;
        }
        if (n > 0) {
          std::printf(" %10.5f", sum / n);
        } else {
          std::printf(" %10s", "-");
        }
      }
      std::printf("\n");
    }
  }
  /* Per move: the cosine arm from downhill rows first, then the uphill arm on what is left, so
   * neither absorbs the other. Rows the game reads as level and airborne moves are not fitted. */
  search::Calibration cal;
  for (size_t k = 0; k < ids.size(); ++k) {
    search::MoveCal mc;
    mc.id = ids[k];
    bool airborne = false;

    /* Least squares through 1.0 on `cos - 1`. */
    double sxx = 0, sxy = 0;
    for (size_t i = 0; i < before.size(); ++i) {
      const Row& r = before[i];
      if (r.id != ids[k] || !usable(r)) continue;
      mc.frames = r.table_frames;
      if (r.airborne) airborne = true;
      if (r.grade >= -kGroundStep || r.speed_along < kSpeedFloor) continue;
      const double cosm1 = 1.0 / std::sqrt(1.0 + r.grade * r.grade) - 1.0;
      sxx += cosm1 * cosm1;
      sxy += cosm1 * (speed_ratio(r) - 1.0);
    }
    if (!airborne && sxx > 1e-18) mc.ground_share = sxy / sxx;

    double ux = 0, uy = 0, uxx = 0, uxy = 0;
    int n = 0;
    for (size_t i = 0; i < before.size(); ++i) {
      const Row& r = before[i];
      if (r.id != ids[k] || !usable(r)) continue;
      if (r.grade <= kGroundStep || r.speed_along < kSpeedFloor) continue;
      const double cosm1 = 1.0 / std::sqrt(1.0 + r.grade * r.grade) - 1.0;
      const double taken = 1.0 + mc.ground_share * cosm1;
      if (std::fabs(taken) < 1e-9) continue;
      const double ratio = speed_ratio(r) / taken;
      ux += r.grade;
      uy += ratio;
      uxx += r.grade * r.grade;
      uxy += r.grade * ratio;
      ++n;
    }
    if (!airborne && n >= 2) {
      const double den = n * uxx - ux * ux;
      if (std::fabs(den) > 1e-12) {
        mc.up_slope = -(n * uxy - ux * uy) / den;
        mc.up_level = (uy + mc.up_slope * ux) / n;
      } else if (n > 0) {
        mc.up_level = uy / n;
      }
    }
    cal.move.push_back(mc);
  }
  /* Base-table order, so a batched table and a whole one match. */
  {
    std::vector<search::MoveCal> ordered;
    for (size_t i = 0; i < base.move.size(); ++i) {
      for (size_t k = 0; k < cal.move.size(); ++k) {
        if (cal.move[k].id == base.move[i].id) ordered.push_back(cal.move[k]);
      }
    }
    cal.move.swap(ordered);
  }

  std::vector<double> charge_slopes;
  charge_slopes.push_back(0.0);
  for (int i = 1; i <= charge_steps; ++i) {
    charge_slopes.push_back(max_grade * i / charge_steps);
    charge_slopes.push_back(-max_grade * i / charge_steps);
  }
  const std::vector<Row> after = collect(base, charge_slopes, facings, cal);
  struct Score {
    static void of(const std::vector<Row>& rows, double* rms, double* worst) {
      double sum = 0;
      int n = 0;
      *worst = 0;
      for (size_t i = 0; i < rows.size(); ++i) {
        if (!usable(rows[i])) continue;
        const double e = miss(rows[i]);
        sum += e * e;
        if (e > *worst) *worst = e;
        ++n;
      }
      *rms = n ? std::sqrt(sum / n) : 0.0;
    }
  };
  double rms_before = 0, worst_before = 0, rms_after = 0, worst_after = 0;
  Score::of(before, &rms_before, &worst_before);
  Score::of(after, &rms_after, &worst_after);
  std::printf("\nfitted on %d rows over %d slopes, charged on %d rows over %d, %d facings\n",
              static_cast<int>(before.size()), static_cast<int>(slopes.size()),
              static_cast<int>(after.size()), static_cast<int>(charge_slopes.size()), facings);
  std::printf("  %-28s rms %8.4f   worst %8.4f\n", "downhill and arc only", rms_before,
              worst_before);
  std::printf("  %-28s rms %8.4f   worst %8.4f\n", "with the uphill fit", rms_after, worst_after);

  /* The residual per move, which the error threshold is built from. */
  std::vector<const Row*> worst_row(cal.move.size(), nullptr);
  for (size_t k = 0; k < cal.move.size(); ++k) {
    search::MoveCal& mc = cal.move[k];
    double sum = 0;
    for (size_t i = 0; i < after.size(); ++i) {
      if (after[i].id != mc.id || !usable(after[i])) continue;
      const double e = miss(after[i]);
      if (e > mc.worst) {
        mc.worst = e;
        worst_row[k] = &after[i];
      }
      sum += e;
      ++mc.rows;
    }
    mc.mean = mc.rows ? sum / mc.rows : 0.0;
  }

  std::printf("\n%-18s %6s %6s %9s %9s %9s %10s %10s %10s %9s %6s\n", "move", "frames", "rows",
              "cos share", "up level", "up slope", "worst", "mean", "handed off", "at slope",
              "facing");
  for (size_t k = 0; k < cal.move.size(); ++k) {
    const search::MoveCal& mc = cal.move[k];
    int handed = 0;
    for (size_t i = 0; i < after.size(); ++i) {
      if (after[i].id == mc.id && usable(after[i]) && after[i].handed_off) ++handed;
    }
    std::printf("%-18s %6d %6d %9.5f %9.5f %9.5f %10.4f %10.4f %10d", mc.id.c_str(), mc.frames,
                mc.rows, mc.ground_share, mc.up_level, mc.up_slope, mc.worst, mc.mean, handed);
    if (worst_row[k] != nullptr) {
      std::printf(" %+9.6f %6d\n", worst_row[k]->slope, worst_row[k]->facing);
    } else {
      std::printf(" %9s %6s\n", "-", "-");
    }
  }

  std::string rows_text;
  for (size_t k = 0; k < cal.move.size(); ++k) {
    const search::MoveCal& mc = cal.move[k];
    char line[256];
    std::snprintf(line, sizeof line, "cal_row(c, \"%s\", %.6f, %.6f, %.6f, %.4f, %.4f, %d, %d);\n",
                  mc.id.c_str(), mc.ground_share, mc.up_level, mc.up_slope,
                  std::ceil(mc.worst * 1e4) / 1e4, mc.mean, mc.rows, mc.frames);
    rows_text += line;
  }
  if (out_path == "-") {
    std::printf("\nnothing written\n");
    return 0;
  }
  if (batch_n > 0 || !only.empty()) {
    const std::string part = batch_n > 0 ? part_path(out_path, batch_k) : out_path;
    FILE* p = std::fopen(part.c_str(), "wb");
    if (p == nullptr) {
      std::printf("\ncould not write %s\n", part.c_str());
      return 1;
    }
    std::fputs(rows_text.c_str(), p);
    std::fclose(p);
    std::printf("\nwrote %s, %d rows\n", part.c_str(), static_cast<int>(cal.move.size()));
    return 0;
  }
  return write_table(out_path, rows_text, max_grade, steps, facings, charge_steps,
                     static_cast<int>(left_out.size()));
}

namespace {

std::string part_path(const std::string& out_path, int k) {
  return out_path + ".part" + std::to_string(k);
}

int write_table(const std::string& out_path, const std::string& rows_text, double max_grade,
                int steps, int facings, int charge_steps, int left,
                const std::vector<std::pair<int, int>>& patched) {
  FILE* out = std::fopen(out_path.c_str(), "w");
  if (out == nullptr) {
    std::printf("\ncould not write %s\n", out_path.c_str());
    return 1;
  }
  std::fprintf(out,
               "/* GENERATED by `setupcore_calibrate`. Do not edit by hand - re-run the "
               "instrument.\n *\n"
               " * One row a move: how much of the cosine arm the move takes, then the uphill\n"
               " * arm's step and its decline, then what the whole\n"
               " * model was still wrong by with those coefficients over the charge ladder -\n"
               " * the worst row and the mean, in world units - then how many rows that was and\n"
               " * the move's own frame count. `Calibration::threshold` is built out of the\n"
               " * last three.\n *\n"
               " * A move that is not in this table is stepped by the flat table with no\n"
               " * correction on level ground, and handed to the engine wherever it crosses a\n"
               " * slope: nobody measured it there.\n *\n"
               " * Conditions: %s, %s, a slab ladder to grade %.4f in %d steps, %d facings,\n"
               " * and the worst and the mean measured again at %d steps. The worst is rounded\n"
               " * up, so the stored charge is never below what was measured. %d moves were\n"
               " * left out by group, and the engine takes each of them over a slope.\n",
               built_by(),
#ifdef NDEBUG
               "release",
#else
               "debug",
#endif
               max_grade, steps, facings, charge_steps, left);
  if (!patched.empty()) {
    std::fputs(" *\n * Measured again, fit and charge both, because at facings off the ring above\n"
               " * their misses were past their charge:\n", out);
    for (size_t j = 0; j < patched.size(); ++j) {
      std::fprintf(out, " *   %d rows on %d facings\n", patched[j].first, patched[j].second);
    }
  }
  std::fputs(" */\n", out);
  std::fputs(rows_text.c_str(), out);
  std::fprintf(out, "c.ladder = %.6f;\n", max_grade);
  std::fprintf(out, "c.charge_steps = %d;\n", charge_steps);
  std::fclose(out);
  std::printf("\nwrote %s\n", out_path.c_str());
  return 0;
}

}  // namespace
