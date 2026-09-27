/* Whether the camera a turnaround reads can meet the room before the tap. The camera tables hold
 * only while `dCamera_c::bumpCheck`'s centre-to-eye line meets nothing, so each place stores, per
 * height layer and camera, the first ring each of 64 directions meets a camera polygon in.
 * Every approximation errs toward refusing. */
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "../disc/disc.h"
#include "corridor.h"
#include "engine/room.h"

namespace search {

/** One camera's extent before the tap, relative to Link. */
struct CamBounds {
  double reach = 0;
  /** Degrees. */
  double rise_lo = 0, rise_hi = 0;
  /** Over Link's feet. */
  double centre_lo = 0, centre_hi = 0;
  double centre_off = 0;
};

/** The C-down press and up to eight taps. */
const int kCamSteps = 9;
const int kCamStepFrames = 15;

/** `margin` is on the 65536 circle and widens both ends of a block's direction span. */
struct CamRoom {
  bool ok = false;
  /** For the Logs. */
  std::string why;
  /** 0 the follow camera, then one per distinct manual reach, nearest first. */
  std::vector<CamBounds> level;
  /** The level a C-down move reads after `steps` steps, index `steps - 1`, by where the manual
   *  camera starts. */
  int from_view[kCamSteps] = {0};
  int from_follow[kCamSteps] = {0};
  int margin = 1024;
  int stray = 0;
  /** `dCamera_c::types` indices. */
  std::vector<int> types;
  /** No camera names were to hand, so `types` is every type. */
  bool every_type = false;
  bool from_disc = false;
};

/** `names` null when no disc is to hand; `cache` a file path, empty for none. */
CamRoom cam_room(const tww_engine::RoomDzb& dzb, const disc::CameraNames* names,
                 const std::string& cache);

/** A place not built is worked out, with its tile, on first ask and kept. The room must outlive
 *  the field. */
class CamField {
 public:
  static constexpr double kCell = 50.0;
  static constexpr double kLayer = 50.0;
  /** 1024 of the 65536 circle each. */
  static constexpr int kBins = 64;
  static constexpr double kRing = 25.0;
  /** Meets nothing out to the camera's farthest reach. */
  static constexpr uint8_t kOpen = 255;
  /** Follow camera's rings, then the manual camera's. */
  static constexpr size_t kRowBytes = 2 * kBins;

  /** Eye directions for csangles `lo` .. `lo + span`, widened by `margin`; the eye is the csangle
   *  plus half a circle. */
  static uint64_t bins(int lo, int span, int margin);

  /** False for a level not held or a field never built. */
  bool clear(int level, double x, double y, double z, uint64_t need) const {
    if (levels_ == 0 || level < 0 || level >= levels_) return false;
    const uint64_t k = key_at(x, y, z);
    if (!key_.empty()) {
      size_t i = static_cast<size_t>(mix(k)) & mask_;
      for (;;) {
        const uint64_t held = key_[i];
        if (held == k) return open(&ring_[i * kRowBytes], level, need);
        if (held == 0) break;
        i = (i + 1) & mask_;
      }
    }
    return later(k, level, need);
  }

  /** The level a B move reads. */
  static int follow_level() { return 0; }
  /** A C-down move out of the C up view, after `taps` L taps. */
  int view_level(int taps) const { return steps_level(from_view_, taps); }
  /** Held L, or the plain C down turnaround. */
  int follow_start_level(int taps) const { return steps_level(from_follow_, taps); }

  int margin() const { return margin_; }
  int levels() const { return levels_; }
  size_t size() const { return size_; }
  size_t later_places() const;

  struct Built {
    size_t places = 0;
    size_t from_disc = 0;
    size_t computed = 0;
    double seconds = 0;
    bool saved = false;
    std::string why;
  };

  static uint64_t key_of(long long ix, long long iz, long long iy);
  static uint64_t key_at(double x, double y, double z);
  static uint64_t mix(uint64_t k) {
    k ^= k >> 33;
    k *= 0xff51afd7ed558ccdULL;
    k ^= k >> 33;
    k *= 0xc4ceb53fe1a85ec9ULL;
    k ^= k >> 33;
    return k;
  }

  /** `rings` holds `kRowBytes` per key. */
  void fill(const std::vector<uint64_t>& keys, const std::vector<uint8_t>& rings,
            const CamRoom& room);

  struct Builder;
  struct Later;

 private:
  friend CamField cam_field(const tww_engine::RoomDzb&, const CamRoom&,
                            const std::vector<uint64_t>&, const std::string&, int, Built*,
                            const std::function<bool()>*);
  friend bool cam_keep(const CamField&, const std::string&);

  bool open(const uint8_t* row, int level, uint64_t need) const {
    const uint8_t* ring = row + (level == 0 ? 0 : kBins);
    const int least = need_[static_cast<size_t>(level)];
    for (uint64_t left = need; left != 0; left &= left - 1) {
      if (ring[lowest(left)] < least) return false;
    }
    return true;
  }
  /** Works out a place the table does not hold, with its tile, under a lock. */
  bool later(uint64_t k, int level, uint64_t need) const;

  static int lowest(uint64_t v) {
    int n = 0;
    while ((v & 1ULL) == 0) {
      v >>= 1;
      ++n;
    }
    return n;
  }

  static int steps_level(const std::vector<int>& by, int taps) {
    if (taps < 0 || taps + 1 > static_cast<int>(by.size())) return -1;
    return by[static_cast<size_t>(taps)];
  }

  std::vector<uint64_t> key_;
  std::vector<uint8_t> ring_;
  /** Per level, the first ring a direction may meet the room in and still be clear. */
  std::vector<int> need_;
  std::vector<int> from_view_, from_follow_;
  int levels_ = 0;
  size_t mask_ = 0;
  size_t size_ = 0;
  int margin_ = 1024;
  CamRoom room_;
  std::shared_ptr<Builder> builder_;
  std::shared_ptr<Later> later_;
};

/** Every cell and layer the selected ground covers, or the start's layer over the corridor when
 *  the run reads no floors. */
std::vector<uint64_t> cam_places(const Selection& selection, const Corridor& corridor,
                                 bool reads_floors, double start_y);

/** Takes what `cache` holds, computes the rest a tile at a time, and writes the whole back.
 *  `stop` is asked between tiles. */
CamField cam_field(const tww_engine::RoomDzb& dzb, const CamRoom& room,
                   const std::vector<uint64_t>& places, const std::string& cache, int threads,
                   CamField::Built* built, const std::function<bool()>* stop = nullptr);

/** Writes everything held, including places worked out during the walk. False when nothing was
 *  written. */
bool cam_keep(const CamField& field, const std::string& cache);

std::string cam_cache_path(const std::string& dir, const tww_engine::RoomDzb& dzb);

}  // namespace search
