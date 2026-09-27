/* A GameCube disc image: its file system table, read once; file bytes are read on demand. */
#pragma once

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace disc {

class Iso {
 public:
  /** Null with `why` set when the path is not a readable image. */
  static std::unique_ptr<Iso> open(const std::string& path, std::string* why);
  ~Iso();

  Iso(const Iso&) = delete;
  Iso& operator=(const Iso&) = delete;

  /** The game id at offset 0, e.g. `GZLJ01`. */
  const std::string& id() const { return id_; }

  /** By whole path, e.g. `res/Stage/sea/Room41.arc`; case-insensitive. */
  bool file(const std::string& path, std::vector<uint8_t>* out) const;

  /** Whole paths under `directory` (no trailing slash), in table order. */
  std::vector<std::string> under(const std::string& directory) const;

 private:
  Iso() = default;
  bool at(uint64_t offset, void* dst, size_t n) const;

  std::FILE* handle_ = nullptr;
  std::string id_;
  struct Entry {
    std::string path;
    uint64_t at = 0;
    uint64_t size = 0;
  };
  std::vector<Entry> files_;
};

}  // namespace disc
