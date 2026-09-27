#pragma once

#include <cstdint>
#include <vector>

namespace disc {

/** Yaz0 decompression. Untagged bytes come back unchanged; false only for truncated Yaz0. */
bool yaz0(const std::vector<uint8_t>& src, std::vector<uint8_t>* out);

}  // namespace disc
