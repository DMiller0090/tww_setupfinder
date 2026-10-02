/* Where a search spends its time, read while it runs. A search is profiled only when the settings
 * folder holds `profile.on`; it then rewrites `profile.json` beside it every second and once at the
 * end. With no such file nothing is timed and nothing is written. Never shown on a surface.
 */
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#elif (defined(__GNUC__) || defined(__clang__)) && (defined(__x86_64__) || defined(__i386__))
#include <x86intrin.h>
#endif

namespace profile {

/** Timed places. The cheap ones are timed one call in `kSample` and scaled up. */
enum Stage {
  kExpand,      // one whole expansion
  kTurnChild,   // one C up turn child, built and tested (sampled)
  kModelStep,   // the camera rule for any other row (sampled)
  kStep,        // `step_move`
  kTests,       // the frame bound, the distance and the steps bound (sampled)
  kDominance,   // the dominance probe (sampled)
  kSort,        // ordering a state's children
  kRecord,      // keeping a near or closest state
  kEngine,      // plans driven through the engine
  kExitFind,    // the exit cache's lookup, its lock held (sampled)
  kExitWait,    // waiting for the exit cache's lock
  kExitRow,     // an exit row run through the engine
  kReportWait,  // a walker waiting for the progress lock
  kReportHold,  // the progress thread holding it
  kPlansWait,   // waiting for the seam's list of driven plans
  kWrite,       // writing a line to the window
  kMemo,        // a move of an engine check looked up among the moves already driven
  kDrive,       // a move driven through the engine
  kSession,     // building the engine's session for one move
  kSameRoom,    // comparing a room's collision with the one held
  kItemSetup,   // a work item's own dominance table, copied from the split's
  kReaim,       // aiming a plan's remaining moves from where the engine put Link
  kReaimCut,    // a plan whose remaining moves could no longer land, not driven further
  kStages
};

/** The row a generated child came from. */
enum Row { kMove, kTurn, kExit, kEss, kRows };

/** What became of it. */
enum Fate {
  kCannot,      // the row cannot be taken from this state
  kUnlandable,  // childless at a facing the item cannot land from
  kOverFrames,  // the frame budget or the frame bound
  kStepsCut,    // the steps bound
  kCamera,      // the room's camera check
  kOutside,     // the Bounds
  kDominated,
  kKept,        // walked on
  kLeaf,        // kept, with no children of its own
  kFates
};

const int kSample = 32;
const int kDepths = 16;
/** Float-step buckets: 0, then 1, 2-3, 4-7 and on up. */
const int kBuckets = 34;

#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
inline uint64_t cycles() { return __rdtsc(); }
#elif (defined(__GNUC__) || defined(__clang__)) && (defined(__x86_64__) || defined(__i386__))
inline uint64_t cycles() { return __rdtsc(); }
#else
inline uint64_t cycles() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}
#endif

/** Written by its own thread only and read by the writer, so plain relaxed loads and stores. */
struct Tally {
  std::atomic<uint64_t> calls[kStages];
  std::atomic<uint64_t> timed[kStages];
  std::atomic<uint64_t> spent[kStages];
  std::atomic<uint64_t> fate[kRows][kFates];
  std::atomic<uint64_t> generated[kDepths];
  std::atomic<uint64_t> expanded[kDepths];
  std::atomic<uint64_t> level_steps, one_plane_steps, crossing_steps;
  /** By axis, then level or sloped plan: how far the engine's aim landed from the model's, and
   *  the check band the plan was consulted within, both in float steps of that axis. */
  std::atomic<uint64_t> missed[2][2][kBuckets];
  std::atomic<uint64_t> band[2][2][kBuckets];
  std::atomic<uint64_t> cpu_ns;
  std::atomic<uint64_t> left_at_ns;
  uint64_t entered_ns = 0;
  /** A HyperLogLog of (row, facing, plane) over sloped steps on one plane. */
  std::atomic<uint8_t> keys[4096];
  std::string role;
  Tally();
};

inline void bump(std::atomic<uint64_t>& a, uint64_t by = 1) {
  a.store(a.load(std::memory_order_relaxed) + by, std::memory_order_relaxed);
}

/** Whether this search is profiled. */
inline std::atomic<bool>& running() {
  static std::atomic<bool> on(false);
  return on;
}
inline bool on() { return running().load(std::memory_order_relaxed); }

/** Starts profiling when `folder` holds `profile.on`, writing `profile.json` beside it. */
bool begin_if_asked(const std::string& folder);

/** Starts profiling into `path`, whatever any folder holds. */
void begin(const std::string& path);

/** The last write, marked finished, and the writer stopped. */
void end();

/** This thread's tally for the run, made on first use. */
Tally& mine();

/** Names this thread in the file and starts its clock. */
void enter(const char* role);

/** Records this thread's processor time; the walk calls it every so often. */
void refresh();

/** Stops this thread's clock. */
void leave();

void row_fate(Row row, Fate fate);
void depth_generated(int depth, uint64_t states);
void depth_expanded(int depth);
void exit_rows_held(size_t rows);

/** One engine check on one axis: the model's aim, the engine's, and the check band. */
void engine_miss(int axis, double model, double engine, double band, bool sloped);

/** A step the model took: on level ground, on one sloped plane, or across more than one. */
void plane_step(int row, int facing, const double plane[4], bool sloped, int planes);

/** Times a stage; `every` false times one call in `kSample`. Off, it does nothing. */
class Timed {
 public:
  Timed() {}
  explicit Timed(Stage stage, bool every = true) { start(stage, every); }
  /** For a span that starts after the object is made. */
  void start(Stage stage, bool every = true) {
    if (!on()) return;
    Tally& t = mine();
    const uint64_t n = t.calls[stage].load(std::memory_order_relaxed);
    bump(t.calls[stage]);
    if (!every && (n % kSample) != 0) return;
    tally_ = &t;
    stage_ = stage;
    from_ = cycles();
  }
  ~Timed() { stop(); }
  /** Ends the span before the scope does. */
  void stop() {
    if (tally_ == nullptr) return;
    bump(tally_->timed[stage_]);
    bump(tally_->spent[stage_], cycles() - from_);
    tally_ = nullptr;
  }
  Timed(const Timed&) = delete;
  Timed& operator=(const Timed&) = delete;

 private:
  Tally* tally_ = nullptr;
  Stage stage_ = kExpand;
  uint64_t from_ = 0;
};

/** Adds a span already measured, such as a lock wait. */
inline void add(Stage stage, uint64_t spent) {
  if (!on()) return;
  Tally& t = mine();
  bump(t.calls[stage]);
  bump(t.timed[stage]);
  bump(t.spent[stage], spent);
}

}  // namespace profile
