#include "gcm.h"

#if !defined(_WIN32)
#include <sys/types.h>
#endif

#include "../geom/be.h"

namespace disc {
namespace {

constexpr uint64_t kIdAt = 0x000;
constexpr size_t kIdBytes = 6;
constexpr uint64_t kTableAt = 0x424;
constexpr uint64_t kTableSize = 0x428;
constexpr size_t kRecordBytes = 12;

/* Record: type u8, name offset u24, then file (start, length) or directory (parent, end index). */
constexpr uint8_t kDirectory = 1;

char lower(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }

bool same(const std::string& a, const std::string& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (lower(a[i]) != lower(b[i])) return false;
  }
  return true;
}

}  // namespace

Iso::~Iso() {
  if (handle_) std::fclose(handle_);
}

bool Iso::at(uint64_t offset, void* dst, size_t n) const {
  if (!handle_) return false;
#if defined(_WIN32)
  if (::_fseeki64(handle_, static_cast<long long>(offset), SEEK_SET) != 0) return false;
#else
  if (::fseeko(handle_, static_cast<off_t>(offset), SEEK_SET) != 0) return false;
#endif
  return std::fread(dst, 1, n, handle_) == n;
}

std::unique_ptr<Iso> Iso::open(const std::string& path, std::string* why) {
  std::unique_ptr<Iso> iso(new Iso());
#if defined(_MSC_VER)
  if (::fopen_s(&iso->handle_, path.c_str(), "rb") != 0) iso->handle_ = nullptr;
#else
  iso->handle_ = std::fopen(path.c_str(), "rb");
#endif
  if (!iso->handle_) {
    if (why) *why = "could not open " + path;
    return nullptr;
  }

  uint8_t id[kIdBytes] = {0};
  if (!iso->at(kIdAt, id, sizeof id)) {
    if (why) *why = "could not read the start of " + path;
    return nullptr;
  }
  iso->id_.assign(reinterpret_cast<const char*>(id), kIdBytes);

  uint8_t where[4] = {0}, how_long[4] = {0};
  if (!iso->at(kTableAt, where, 4) || !iso->at(kTableSize, how_long, 4)) {
    if (why) *why = "could not read the disc header of " + path;
    return nullptr;
  }
  const uint64_t table_at = geom::be32(where);
  const uint32_t table_size = geom::be32(how_long);
  // Bounds the allocation; real tables are well under a megabyte.
  if (table_size < kRecordBytes || table_size > 16u * 1024u * 1024u) {
    if (why) *why = "that file has no disc table in it";
    return nullptr;
  }
  std::vector<uint8_t> table(table_size);
  if (!iso->at(table_at, table.data(), table.size())) {
    if (why) *why = "could not read the disc table of " + path;
    return nullptr;
  }

  // The root record's count is also where the string table starts.
  const uint32_t records = geom::be32(table.data() + 8);
  const size_t strings = static_cast<size_t>(records) * kRecordBytes;
  if (records == 0 || strings > table.size()) {
    if (why) *why = "that file has no disc table in it";
    return nullptr;
  }

  // Flat walk: a stack of directory end indices rebuilds the paths.
  std::vector<std::string> open_dirs;
  std::vector<uint32_t> ends;
  std::string prefix;
  for (uint32_t i = 1; i < records; ++i) {
    while (!ends.empty() && ends.back() <= i) {
      ends.pop_back();
      open_dirs.pop_back();
      prefix.clear();
      for (const std::string& part : open_dirs) prefix += part + "/";
    }
    const uint8_t* r = table.data() + static_cast<size_t>(i) * kRecordBytes;
    const size_t name_at = strings + (geom::be32(r) & 0xFFFFFF);
    std::string name;
    for (size_t k = name_at; k < table.size() && table[k] != 0; ++k) {
      name.push_back(static_cast<char>(table[k]));
    }
    if (r[0] == kDirectory) {
      open_dirs.push_back(name);
      ends.push_back(geom::be32(r + 8));
      prefix += name + "/";
      continue;
    }
    Entry e;
    e.path = prefix + name;
    e.at = geom::be32(r + 4);
    e.size = geom::be32(r + 8);
    iso->files_.push_back(std::move(e));
  }
  return iso;
}

bool Iso::file(const std::string& path, std::vector<uint8_t>* out) const {
  for (const Entry& e : files_) {
    if (!same(e.path, path)) continue;
    if (e.size > 64u * 1024u * 1024u) return false;
    out->assign(static_cast<size_t>(e.size), 0);
    return at(e.at, out->data(), out->size());
  }
  return false;
}

std::vector<std::string> Iso::under(const std::string& directory) const {
  const std::string prefix = directory + "/";
  std::vector<std::string> found;
  for (const Entry& e : files_) {
    if (e.path.size() <= prefix.size()) continue;
    if (!same(e.path.substr(0, prefix.size()), prefix)) continue;
    found.push_back(e.path);
  }
  return found;
}

}  // namespace disc
