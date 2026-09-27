#include "rarc.h"

#include "../geom/be.h"
#include "yaz0.h"

namespace disc {
namespace {

/* Table offsets are relative to the info block; the data offset to the end of the header. */
constexpr size_t kHeaderInfo = 0x08;
constexpr size_t kHeaderData = 0x0C;
constexpr size_t kHeaderBytes = 0x20;
constexpr size_t kInfoFileCount = 0x08;
constexpr size_t kInfoFileTable = 0x0C;
constexpr size_t kInfoStrings = 0x14;
constexpr size_t kEntryBytes = 0x14;
constexpr uint16_t kIsDirectory = 0xFFFF;

char lower(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }

bool ends_with(const std::string& name, const std::string& suffix) {
  if (name.size() < suffix.size()) return false;
  for (size_t i = 0; i < suffix.size(); ++i) {
    if (lower(name[name.size() - suffix.size() + i]) != lower(suffix[i])) return false;
  }
  return true;
}

}  // namespace

static bool find_in_rarc(const std::vector<uint8_t>& archive,
                         bool (*wanted)(const std::string&, const std::string&),
                         const std::string& key, std::vector<uint8_t>* out) {
  std::vector<uint8_t> b;
  if (!yaz0(archive, &b)) return false;
  if (b.size() < kHeaderBytes) return false;
  if (b[0] != 'R' || b[1] != 'A' || b[2] != 'R' || b[3] != 'C') return false;

  const size_t info = geom::be32(b.data() + kHeaderInfo);
  const size_t data = geom::be32(b.data() + kHeaderData);
  if (info + kInfoStrings + 4 > b.size()) return false;
  const uint32_t files = geom::be32(b.data() + info + kInfoFileCount);
  const size_t table = info + geom::be32(b.data() + info + kInfoFileTable);
  const size_t strings = info + geom::be32(b.data() + info + kInfoStrings);
  if (files > 0xFFFF) return false;
  if (table + static_cast<size_t>(files) * kEntryBytes > b.size()) return false;

  for (uint32_t i = 0; i < files; ++i) {
    const uint8_t* e = b.data() + table + static_cast<size_t>(i) * kEntryBytes;
    // A subdirectory entry.
    if (geom::be16(e) == kIsDirectory) continue;
    const size_t name_at = strings + (geom::be32(e + 0x04) & 0xFFFFFF);
    if (name_at >= b.size()) continue;
    std::string name;
    for (size_t k = name_at; k < b.size() && b[k] != 0; ++k) name.push_back(static_cast<char>(b[k]));
    if (!wanted(name, key)) continue;

    const size_t at = kHeaderBytes + data + geom::be32(e + 0x08);
    const size_t size = geom::be32(e + 0x0C);
    if (at > b.size() || size > b.size() - at) return false;
    out->assign(b.begin() + static_cast<long long>(at),
                b.begin() + static_cast<long long>(at + size));
    return true;
  }
  return false;
}

static bool same_name(const std::string& name, const std::string& wanted) {
  return name.size() == wanted.size() && ends_with(name, wanted);
}

bool from_rarc(const std::vector<uint8_t>& archive, const std::string& suffix,
               std::vector<uint8_t>* out) {
  return find_in_rarc(archive, ends_with, suffix, out);
}

bool named_in_rarc(const std::vector<uint8_t>& archive, const std::string& name,
                   std::vector<uint8_t>* out) {
  return find_in_rarc(archive, same_name, name, out);
}

}  // namespace disc
