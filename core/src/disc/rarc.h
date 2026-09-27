/* RARC archives, searched flat by file name; the directory tree is not rebuilt. */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace disc {

/** The first file whose name ends with `suffix` (e.g. `".dzb"`), case-insensitive. */
bool from_rarc(const std::vector<uint8_t>& archive, const std::string& suffix,
               std::vector<uint8_t>* out);

/** By whole name, so `"walk.bck"` does not match `"dwalk.bck"`. */
bool named_in_rarc(const std::vector<uint8_t>& archive, const std::string& name,
                   std::vector<uint8_t>* out);

}  // namespace disc
