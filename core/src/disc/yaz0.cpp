#include "yaz0.h"

#include "../geom/be.h"

namespace disc {

bool yaz0(const std::vector<uint8_t>& src, std::vector<uint8_t>* out) {
  if (src.size() < 4 || src[0] != 'Y' || src[1] != 'a' || src[2] != 'z' || src[3] != '0') {
    *out = src;
    return true;
  }
  if (src.size() < 16) return false;
  const uint32_t want = geom::be32(src.data() + 4);
  // Caps an allocation sized by the file.
  if (want > 64u * 1024u * 1024u) return false;

  std::vector<uint8_t>& dst = *out;
  dst.clear();
  dst.reserve(want);
  size_t p = 16;
  while (dst.size() < want) {
    if (p >= src.size()) return false;
    const uint8_t code = src[p++];
    for (int i = 0; i < 8 && dst.size() < want; ++i) {
      if (code & (0x80 >> i)) {
        if (p >= src.size()) return false;
        dst.push_back(src[p++]);
        continue;
      }
      if (p + 1 >= src.size()) return false;
      const uint8_t b1 = src[p], b2 = src[p + 1];
      p += 2;
      const size_t dist = (static_cast<size_t>(b1 & 0x0F) << 8) | b2;
      size_t count = b1 >> 4;
      if (count == 0) {
        if (p >= src.size()) return false;
        count = static_cast<size_t>(src[p++]) + 0x12;
      } else {
        count += 2;
      }
      if (dist + 1 > dst.size()) return false;
      size_t r = dst.size() - dist - 1;
      // Byte by byte: a run may overlap its own output.
      for (size_t k = 0; k < count; ++k) dst.push_back(dst[r++]);
    }
  }
  return true;
}

}  // namespace disc
