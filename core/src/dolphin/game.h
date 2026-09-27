/* The disc, place and start read out of a running game. Addresses are TWW-JP (the decomp is US). */
#pragma once

#include <string>

#include "dolphin.h"

namespace dolphin {

namespace addr {
constexpr uint32_t kGameId = 0x80000000;  // char[6], e.g. `GZLJ01`
/* The title is not in MEM1 (0x80000020 is the boot magic), so names come from `name_of`.
   The Link position triple is a set of debug globals: a tool that writes them stops them tracking
   Link, who is really at `[0x803AD860]+0x120`. */
constexpr uint32_t kLinkX = 0x803D78FC;     // f32
constexpr uint32_t kLinkY = 0x803D7900;     // f32
constexpr uint32_t kLinkZ = 0x803D7904;     // f32
constexpr uint32_t kFacing = 0x803EA3D2;    // u16, 0x10000 = 360 degrees
constexpr uint32_t kCameraBase = 0x803AD380;  // cSAngle, chain +0x34 +0x2B0, u16
constexpr uint32_t kFrame = 0x803E9D34;     // s32 frame counter
constexpr uint32_t kCurStage = 0x803BD23C;  // mCurStage, char[8], null-padded
constexpr uint32_t kStayNo = 0x803E9F48;    // mStayNo, s8, negative before a play scene exists
}  // namespace addr

struct Disc {
  std::string id;    // empty when there is no disc header
  std::string name;
  /** True only for the version the addresses above are for. */
  bool reads = false;
};

Disc disc_of(const Mem& mem);

/** A plain name for a disc id, or the id itself when unknown. */
std::string name_of(const std::string& id);

/** Fields are meaningful only when `ok`. */
struct Start {
  bool ok = false;
  std::string why;
  float x = 0;
  float y = 0;
  float z = 0;
  int facing = 0;
  int camera = 0;  // cSAngle the stick mapping is relative to
};

Start start_of(const Mem& mem);

/** Not `ok` before a play scene exists. */
struct Place {
  bool ok = false;
  std::string stage;
  int room = 0;
};

Place place_of(const Mem& mem);

/** Samples the frame counter twice, `ms` apart. False when it cannot tell. */
bool is_paused(const Mem& mem, int ms);

}  // namespace dolphin
