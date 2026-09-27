/* The platform-independent half of `Mem`. */
#include "dolphin.h"

#include <cstring>

namespace dolphin {
namespace {

uint16_t be16(const uint8_t* b) {
  return static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
}

uint32_t be32(const uint8_t* b) {
  return (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
         (static_cast<uint32_t>(b[2]) << 8) | static_cast<uint32_t>(b[3]);
}

}  // namespace

bool Mem::u8(uint32_t gc_addr, uint8_t* out) const { return read(gc_addr, out, 1); }

bool Mem::u16(uint32_t gc_addr, uint16_t* out) const {
  uint8_t b[2];
  if (!read(gc_addr, b, sizeof b)) return false;
  *out = be16(b);
  return true;
}

bool Mem::u32(uint32_t gc_addr, uint32_t* out) const {
  uint8_t b[4];
  if (!read(gc_addr, b, sizeof b)) return false;
  *out = be32(b);
  return true;
}

bool Mem::s32(uint32_t gc_addr, int32_t* out) const {
  uint32_t v;
  if (!u32(gc_addr, &v)) return false;
  std::memcpy(out, &v, sizeof v);
  return true;
}

bool Mem::f32(uint32_t gc_addr, float* out) const {
  uint32_t v;
  if (!u32(gc_addr, &v)) return false;
  std::memcpy(out, &v, sizeof v);
  return true;
}

bool Mem::chain(uint32_t base, const std::vector<uint32_t>& offsets, uint32_t* out) const {
  uint32_t addr = base;
  for (uint32_t off : offsets) {
    uint32_t ptr;
    if (!u32(addr, &ptr)) return false;
    // Normal at a menu, before the game has written the chain.
    if (ptr < kMem1Start || ptr >= kMem1Start + kMem1Size) return false;
    addr = ptr + off;
  }
  *out = addr;
  return true;
}

}  // namespace dolphin
