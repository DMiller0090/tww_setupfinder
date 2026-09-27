#include "game.h"

#include <chrono>
#include <thread>
#include <vector>

namespace dolphin {
namespace {

constexpr char kSupportedId[] = "GZLJ01";

/* Matched on the game code, ignoring region (4th char) and maker. */
struct Named {
  const char* prefix;
  const char* name;
};
constexpr Named kNames[] = {
    {"GZL", "The Wind Waker"},
    {"GZ2", "Twilight Princess"},
    {"G4S", "The Wind Waker"},        // Japanese Zelda Collection disc
};

/** A fixed-length, possibly-unterminated string, trimmed of padding. */
std::string text_at(const Mem& mem, uint32_t gc_addr, size_t max) {
  std::vector<char> buf(max + 1, '\0');
  if (!mem.read(gc_addr, buf.data(), max)) return {};
  std::string out(buf.data());
  while (!out.empty() && (out.back() == ' ' || out.back() == '\0')) out.pop_back();
  return out;
}

}  // namespace

std::string name_of(const std::string& id) {
  for (const Named& n : kNames) {
    if (id.rfind(n.prefix, 0) == 0) return n.name;
  }
  return id;
}

Disc disc_of(const Mem& mem) {
  Disc out;
  uint32_t magic = 0;
  if (!mem.u32(addr::kGameId + 0x1C, &magic) || magic != kDiscMagic) return out;
  out.id = text_at(mem, addr::kGameId, 6);
  if (out.id.empty()) return out;
  out.name = name_of(out.id);
  out.reads = out.id == kSupportedId;
  return out;
}

Start start_of(const Mem& mem) {
  Start out;
  const Disc disc = disc_of(mem);
  if (disc.id.empty()) { out.why = "that Dolphin has no game in it"; return out; }
  if (!disc.reads) {
    out.why = "reads " + std::string(kSupportedId) + ", found " + disc.id;
    return out;
  }

  float x = 0, y = 0, z = 0;
  uint16_t facing = 0, camera = 0;
  uint32_t camera_addr = 0;
  if (!mem.f32(addr::kLinkX, &x) || !mem.f32(addr::kLinkY, &y) ||
      !mem.f32(addr::kLinkZ, &z) || !mem.u16(addr::kFacing, &facing)) {
    out.why = "could not read the game - is it still running?";
    return out;
  }
  // The camera chain is unwritten at a title screen; zero is fine then.
  if (mem.chain(addr::kCameraBase, {0x34, 0x2B0}, &camera_addr)) {
    mem.u16(camera_addr, &camera);
  }

  out.ok = true;
  out.x = x;
  out.y = y;
  out.z = z;
  out.facing = facing;
  out.camera = camera;
  return out;
}

Place place_of(const Mem& mem) {
  Place out;
  const std::string name = text_at(mem, addr::kCurStage, 8);
  if (name.empty()) return out;
  uint8_t room = 0;
  if (!mem.u8(addr::kStayNo, &room)) return out;
  const int8_t signed_room = static_cast<int8_t>(room);
  if (signed_room < 0) return out;
  out.ok = true;
  out.stage = name;
  out.room = signed_room;
  return out;
}

bool is_paused(const Mem& mem, int ms) {
  int32_t before = 0, after = 0;
  if (!mem.s32(addr::kFrame, &before)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  if (!mem.s32(addr::kFrame, &after)) return false;
  return before == after;
}

}  // namespace dolphin
