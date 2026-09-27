/* Big-endian reads, for game RAM and disc data alike. */
#pragma once

#include <cstdint>
#include <cstring>

namespace geom {

inline uint16_t be16(const uint8_t* b) {
  return static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
}

inline uint32_t be32(const uint8_t* b) {
  return (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
         (static_cast<uint32_t>(b[2]) << 8) | static_cast<uint32_t>(b[3]);
}

inline int32_t be_s32(const uint8_t* b) { return static_cast<int32_t>(be32(b)); }

inline float be_f32(const uint8_t* b) {
  const uint32_t v = be32(b);
  float out;
  std::memcpy(&out, &v, sizeof out);
  return out;
}

}  // namespace geom
