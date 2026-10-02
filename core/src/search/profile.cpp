#include "profile.h"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "../machine/machine.h"

namespace profile {
namespace {

const char* const kStageName[kStages] = {
    "expand", "turn child", "model step", "step", "tests", "dominance", "sort",
    "record", "engine", "exit find", "exit wait", "exit row", "report wait", "report hold",
    "plans wait", "write", "memo", "drive", "session", "same room", "item setup", "re-aim",
    "re-aim cut"};
const char* const kRowName[kRows] = {"move", "turn", "exit", "ess"};
const char* const kFateName[kFates] = {"cannot",  "unlandable", "over frames",
                                       "steps cut", "camera",   "outside",
                                       "dominated", "kept",     "leaf"};

uint64_t now_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}

uint64_t load(const std::atomic<uint64_t>& a) { return a.load(std::memory_order_relaxed); }

struct Run {
  std::mutex lock;
  std::vector<std::unique_ptr<Tally>> tallies;
  std::string path;
  uint64_t started_ns = 0;
  uint64_t started_cycles = 0;
  /** Bumped by `begin`, so a thread's tally from an earlier run is never reused. */
  std::atomic<uint64_t> serial{0};
  std::thread writer;
  std::condition_variable wake;
  bool stopping = false;
  std::atomic<uint64_t> exit_rows{0};
  /** For the last second's busy cores. */
  uint64_t was_ns = 0;
  uint64_t was_cpu_ns = 0;
};

Run& the_run() {
  static Run r;
  return r;
}

thread_local Tally* t_tally = nullptr;
thread_local uint64_t t_serial = 0;

uint64_t mix(uint64_t v) {
  v += 0x9E3779B97F4A7C15ull;
  v = (v ^ (v >> 30)) * 0xBF58476D1CE4E5B9ull;
  v = (v ^ (v >> 27)) * 0x94D049BB133111EBull;
  return v ^ (v >> 31);
}

/** HyperLogLog over 4096 registers. */
double distinct(const uint8_t* reg) {
  const double m = 4096.0;
  double sum = 0.0;
  int zeros = 0;
  for (int j = 0; j < 4096; ++j) {
    sum += std::ldexp(1.0, -static_cast<int>(reg[j]));
    if (reg[j] == 0) ++zeros;
  }
  const double alpha = 0.7213 / (1.0 + 1.079 / m);
  double e = alpha * m * m / sum;
  if (e <= 2.5 * m && zeros > 0) e = m * std::log(m / zeros);
  return e;
}

/** A float's place among all floats, so two places are a count of floats apart. */
long long rank(float f) {
  uint32_t bits = 0;
  std::memcpy(&bits, &f, sizeof bits);
  if (bits & 0x80000000u) return -static_cast<long long>(bits & 0x7FFFFFFFu);
  return static_cast<long long>(bits);
}

int bucket(double steps) {
  if (!(steps >= 1.0)) return 0;
  const int b = 1 + static_cast<int>(std::floor(std::log2(steps)));
  return b < kBuckets ? b : kBuckets - 1;
}

std::string histogram(const uint64_t* counts) {
  std::string out = "{";
  bool first = true;
  for (int b = 0; b < kBuckets; ++b) {
    if (counts[b] == 0) continue;
    const double from = b == 0 ? 0.0 : std::ldexp(1.0, b - 1);
    char key[32];
    std::snprintf(key, sizeof key, "%.0f", from);
    out += std::string(first ? "" : ", ") + "\"" + key + "\": " +
           std::to_string(static_cast<unsigned long long>(counts[b]));
    first = false;
  }
  return out + "}";
}

std::string number(double v) {
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.6g", v);
  return buf;
}

std::string whole(uint64_t v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%llu", static_cast<unsigned long long>(v));
  return buf;
}

std::string text_of(Run& r, bool finished) {
  const uint64_t ns = now_ns();
  const double seconds = static_cast<double>(ns - r.started_ns) * 1e-9;
  const uint64_t cycle_span = cycles() - r.started_cycles;
  const double cycle_rate = ns > r.started_ns
                               ? static_cast<double>(cycle_span) /
                                     (static_cast<double>(ns - r.started_ns) * 1e-9)
                               : 1.0;

  uint64_t calls[kStages] = {}, timed[kStages] = {}, spent[kStages] = {};
  uint64_t fate[kRows][kFates] = {};
  uint64_t generated[kDepths] = {}, expanded[kDepths] = {};
  uint64_t level = 0, one_plane = 0, crossing = 0;
  uint8_t keys[4096] = {};
  uint64_t missed[2][2][kBuckets] = {}, band[2][2][kBuckets] = {};
  uint64_t cpu_ns = 0;
  double walk_wall = 0.0;
  std::string threads = "[";
  for (size_t i = 0; i < r.tallies.size(); ++i) {
    const Tally& t = *r.tallies[i];
    for (int s = 0; s < kStages; ++s) {
      calls[s] += load(t.calls[s]);
      timed[s] += load(t.timed[s]);
      spent[s] += load(t.spent[s]);
    }
    for (int w = 0; w < kRows; ++w) {
      for (int f = 0; f < kFates; ++f) fate[w][f] += load(t.fate[w][f]);
    }
    for (int d = 0; d < kDepths; ++d) {
      generated[d] += load(t.generated[d]);
      expanded[d] += load(t.expanded[d]);
    }
    for (int a = 0; a < 2; ++a) {
      for (int s = 0; s < 2; ++s) {
        for (int b = 0; b < kBuckets; ++b) {
          missed[a][s][b] += load(t.missed[a][s][b]);
          band[a][s][b] += load(t.band[a][s][b]);
        }
      }
    }
    level += load(t.level_steps);
    one_plane += load(t.one_plane_steps);
    crossing += load(t.crossing_steps);
    for (int j = 0; j < 4096; ++j) {
      const uint8_t k = t.keys[j].load(std::memory_order_relaxed);
      if (k > keys[j]) keys[j] = k;
    }
    const uint64_t cpu = load(t.cpu_ns);
    cpu_ns += cpu;
    const uint64_t left = load(t.left_at_ns);
    const uint64_t until = left != 0 ? left : ns;
    const double wall =
        until > t.entered_ns ? static_cast<double>(until - t.entered_ns) * 1e-9 : 0.0;
    if (t.role == "walk") walk_wall += wall;
    if (i > 0) threads += ", ";
    threads += "{\"role\": \"" + t.role + "\", \"cpu\": " +
               number(static_cast<double>(cpu) * 1e-9) +
               ", \"wall\": " + number(wall) +
               ", \"busy\": " + number(wall > 0.0 ? static_cast<double>(cpu) * 1e-9 / wall : 0.0) +
               (left != 0 ? ", \"done\": true" : "") + "}";
  }
  threads += "]";

  double recent = 0.0;
  if (ns > r.was_ns && r.was_ns != 0) {
    recent = static_cast<double>(cpu_ns - std::min(cpu_ns, r.was_cpu_ns)) /
             static_cast<double>(ns - r.was_ns);
  }
  r.was_ns = ns;
  r.was_cpu_ns = cpu_ns;

  std::string out = "{\n";
  out += "  \"running\": " + std::string(finished ? "false" : "true") + ",\n";
  out += "  \"seconds\": " + number(seconds) + ",\n";
  out += "  \"cores busy, last second\": " + number(recent) + ",\n";
  out += "  \"cores busy, whole run\": " +
         number(seconds > 0.0 ? static_cast<double>(cpu_ns) * 1e-9 / seconds : 0.0) + ",\n";
  out += "  \"walk thread seconds\": " + number(walk_wall) + ",\n";
  out += "  \"threads\": " + threads + ",\n";

  out += "  \"stages\": {";
  for (int s = 0; s < kStages; ++s) {
    const double est = timed[s] > 0 ? static_cast<double>(spent[s]) *
                                          (static_cast<double>(calls[s]) /
                                           static_cast<double>(timed[s]))
                                    : 0.0;
    const double secs = cycle_rate > 0.0 ? est / cycle_rate : 0.0;
    out += std::string(s ? ",\n" : "\n") + "    \"" + kStageName[s] + "\": {\"calls\": " +
           whole(calls[s]) + ", \"seconds\": " + number(secs) + ", \"share of walk\": " +
           number(walk_wall > 0.0 ? secs / walk_wall : 0.0) + "}";
  }
  out += "\n  },\n";

  out += "  \"children\": {";
  for (int w = 0; w < kRows; ++w) {
    out += std::string(w ? ",\n" : "\n") + "    \"" + kRowName[w] + "\": {";
    for (int f = 0; f < kFates; ++f) {
      out += std::string(f ? ", " : "") + "\"" + kFateName[f] + "\": " + whole(fate[w][f]);
    }
    out += "}";
  }
  out += "\n  },\n";

  out += "  \"by depth\": [";
  int deepest = 0;
  for (int d = 0; d < kDepths; ++d) {
    if (generated[d] != 0 || expanded[d] != 0) deepest = d;
  }
  for (int d = 0; d <= deepest; ++d) {
    out += std::string(d ? ", " : "") + "{\"depth\": " + whole(static_cast<uint64_t>(d)) +
           ", \"generated\": " + whole(generated[d]) +
           ", \"expanded\": " + whole(expanded[d]) + "}";
  }
  out += "],\n";

  const char* const axis_name[2] = {"x", "z"};
  const char* const ground_name[2] = {"level", "sloped"};
  for (int which = 0; which < 2; ++which) {
    out += std::string(which == 0 ? "  \"engine from model, float steps\": {"
                                  : "  \"check band, float steps\": {");
    bool first = true;
    for (int a = 0; a < 2; ++a) {
      for (int s = 0; s < 2; ++s) {
        const uint64_t* counts = which == 0 ? missed[a][s] : band[a][s];
        uint64_t any = 0;
        for (int b = 0; b < kBuckets; ++b) any += counts[b];
        if (any == 0) continue;
        out += std::string(first ? "" : ", ") + "\"" + axis_name[a] + " " + ground_name[s] +
               "\": " + histogram(counts);
        first = false;
      }
    }
    out += "},\n";
  }
  out += "  \"exit rows held\": " + whole(r.exit_rows.load()) + ",\n";
  out += "  \"model steps\": {\"level\": " + whole(level) + ", \"one sloped plane\": " +
         whole(one_plane) + ", \"crossing planes\": " + whole(crossing) +
         ", \"distinct move, facing, plane\": " + number(std::floor(distinct(keys))) + "}\n";
  out += "}\n";
  return out;
}

void write_now(Run& r, bool finished) {
  std::string text;
  {
    std::lock_guard<std::mutex> hold(r.lock);
    if (r.path.empty()) return;
    text = text_of(r, finished);
  }
  const std::filesystem::path file(r.path);
  const std::filesystem::path beside = file.string() + ".new";
  {
    std::ofstream out(beside, std::ios::binary | std::ios::trunc);
    if (!out) return;
    out << text;
  }
  std::error_code ec;
  std::filesystem::rename(beside, file, ec);
  if (ec) {
    std::filesystem::copy_file(beside, file, std::filesystem::copy_options::overwrite_existing,
                               ec);
    std::error_code ignored;
    std::filesystem::remove(beside, ignored);
  }
}

}  // namespace

Tally::Tally() {
  for (int s = 0; s < kStages; ++s) {
    calls[s].store(0);
    timed[s].store(0);
    spent[s].store(0);
  }
  for (int w = 0; w < kRows; ++w) {
    for (int f = 0; f < kFates; ++f) fate[w][f].store(0);
  }
  for (int d = 0; d < kDepths; ++d) {
    generated[d].store(0);
    expanded[d].store(0);
  }
  for (int a = 0; a < 2; ++a) {
    for (int s = 0; s < 2; ++s) {
      for (int b = 0; b < kBuckets; ++b) {
        missed[a][s][b].store(0);
        band[a][s][b].store(0);
      }
    }
  }
  level_steps.store(0);
  one_plane_steps.store(0);
  crossing_steps.store(0);
  cpu_ns.store(0);
  left_at_ns.store(0);
  for (int j = 0; j < 4096; ++j) keys[j].store(0);
}

Tally& mine() {
  Run& r = the_run();
  const uint64_t serial = r.serial.load(std::memory_order_acquire);
  if (t_tally == nullptr || t_serial != serial) {
    std::lock_guard<std::mutex> hold(r.lock);
    r.tallies.push_back(std::unique_ptr<Tally>(new Tally()));
    t_tally = r.tallies.back().get();
    t_tally->role = "other";
    t_tally->entered_ns = now_ns();
    t_serial = serial;
  }
  return *t_tally;
}

void begin(const std::string& path) {
  end();
  Run& r = the_run();
  {
    std::lock_guard<std::mutex> hold(r.lock);
    r.tallies.clear();
    r.path = path;
    r.started_ns = now_ns();
    r.started_cycles = cycles();
    r.was_ns = 0;
    r.was_cpu_ns = 0;
    r.stopping = false;
    r.exit_rows.store(0);
    r.serial.fetch_add(1, std::memory_order_release);
  }
  running().store(true);
  r.writer = std::thread([&r]() {
    std::unique_lock<std::mutex> hold(r.lock);
    while (!r.stopping) {
      r.wake.wait_for(hold, std::chrono::seconds(1));
      if (r.stopping) break;
      hold.unlock();
      write_now(r, false);
      hold.lock();
    }
  });
}

bool begin_if_asked(const std::string& folder) {
  if (folder.empty()) return false;
  std::error_code ec;
  const std::filesystem::path dir(folder);
  if (!std::filesystem::exists(dir / "profile.on", ec) || ec) return false;
  begin((dir / "profile.json").string());
  return true;
}

void end() {
  Run& r = the_run();
  if (!r.writer.joinable()) return;
  running().store(false);
  {
    std::lock_guard<std::mutex> hold(r.lock);
    r.stopping = true;
  }
  r.wake.notify_all();
  r.writer.join();
  write_now(r, true);
}

void enter(const char* role) {
  if (!on()) return;
  Tally& t = mine();
  {
    /* The writer reads both under this lock. */
    std::lock_guard<std::mutex> hold(the_run().lock);
    t.role = role;
    t.entered_ns = now_ns();
  }
  t.cpu_ns.store(machine::thread_cpu_ns(), std::memory_order_relaxed);
  t.left_at_ns.store(0, std::memory_order_relaxed);
}

void refresh() {
  if (!on()) return;
  mine().cpu_ns.store(machine::thread_cpu_ns(), std::memory_order_relaxed);
}

void leave() {
  if (!on()) return;
  Tally& t = mine();
  t.cpu_ns.store(machine::thread_cpu_ns(), std::memory_order_relaxed);
  t.left_at_ns.store(now_ns(), std::memory_order_relaxed);
}

void row_fate(Row row, Fate fate) {
  if (!on()) return;
  bump(mine().fate[row][fate]);
}

void depth_generated(int depth, uint64_t states) {
  if (!on()) return;
  bump(mine().generated[depth < 0 ? 0 : depth >= kDepths ? kDepths - 1 : depth], states);
}

void depth_expanded(int depth) {
  if (!on()) return;
  bump(mine().expanded[depth < 0 ? 0 : depth >= kDepths ? kDepths - 1 : depth]);
}

void exit_rows_held(size_t rows) {
  if (!on()) return;
  the_run().exit_rows.store(rows, std::memory_order_relaxed);
}

void engine_miss(int axis, double model, double engine, double band_width, bool sloped) {
  if (!on() || axis < 0 || axis > 1) return;
  Tally& t = mine();
  const float e = static_cast<float>(engine);
  const long long apart = rank(static_cast<float>(model)) - rank(e);
  bump(t.missed[axis][sloped ? 1 : 0][bucket(static_cast<double>(apart < 0 ? -apart : apart))]);
  const float magnitude = std::fabs(e);
  const float up = std::nextafter(magnitude, std::numeric_limits<float>::infinity());
  const double step = static_cast<double>(up - magnitude);
  bump(t.band[axis][sloped ? 1 : 0][bucket(step > 0.0 ? band_width / step : 0.0)]);
}

void plane_step(int row, int facing, const double plane[4], bool sloped, int planes) {
  if (!on()) return;
  Tally& t = mine();
  if (!sloped) {
    bump(t.level_steps);
    return;
  }
  if (planes != 1) {
    bump(t.crossing_steps);
    return;
  }
  bump(t.one_plane_steps);
  uint64_t h = mix(static_cast<uint64_t>(row) * 65536u + static_cast<uint64_t>(facing & 0xFFFF));
  for (int k = 0; k < 4; ++k) {
    uint64_t bits = 0;
    std::memcpy(&bits, &plane[k], sizeof bits);
    h = mix(h ^ bits);
  }
  const size_t at = static_cast<size_t>(h >> 52);
  const uint64_t rest = (h << 12) | (uint64_t(1) << 11);
  uint8_t rank = 1;
  for (uint64_t probe = uint64_t(1) << 63; (rest & probe) == 0 && rank < 52; probe >>= 1) ++rank;
  if (rank > t.keys[at].load(std::memory_order_relaxed)) {
    t.keys[at].store(rank, std::memory_order_relaxed);
  }
}

}  // namespace profile
