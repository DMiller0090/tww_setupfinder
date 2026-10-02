/* The files a stopped search is kept in, so a test can carry a long run on from where it stopped.
 * Written only when the settings folder holds `checkpoint.on` or a request names `save`; read only
 * when a request names `resume`. Never shown on a surface.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <type_traits>
#include <vector>

namespace checkpoint {

/** Bumped with any change to what is written, so an old file is refused rather than misread. */
const uint32_t kVersion = 1;

/** Binary out, written beside its name and renamed over it on `close`. */
class Writer {
 public:
  explicit Writer(const std::string& path);
  ~Writer();
  bool ok() const { return file_ != nullptr && good_; }
  void bytes(const void* p, size_t n);
  template <class T>
  void pod(const T& v) {
    static_assert(std::is_trivially_copyable<T>::value, "written as its bytes");
    bytes(&v, sizeof v);
  }
  template <class T>
  void pods(const std::vector<T>& v) {
    static_assert(std::is_trivially_copyable<T>::value, "written as its bytes");
    pod<uint64_t>(v.size());
    if (!v.empty()) bytes(v.data(), v.size() * sizeof(T));
  }
  void text(const std::string& s);
  /** Flushes and puts the file in place; false if anything failed. */
  bool close();

 private:
  std::string path_;
  std::FILE* file_ = nullptr;
  bool good_ = true;
};

class Reader {
 public:
  explicit Reader(const std::string& path);
  ~Reader();
  bool ok() const { return file_ != nullptr && good_; }
  void bytes(void* p, size_t n);
  template <class T>
  T pod() {
    static_assert(std::is_trivially_copyable<T>::value, "read as its bytes");
    T v{};
    bytes(&v, sizeof v);
    return v;
  }
  template <class T>
  void pods(std::vector<T>* v) {
    static_assert(std::is_trivially_copyable<T>::value, "read as its bytes");
    const uint64_t n = pod<uint64_t>();
    if (!ok() || n > (uint64_t(1) << 34) / (sizeof(T) ? sizeof(T) : 1)) {
      good_ = false;
      v->clear();
      return;
    }
    v->resize(static_cast<size_t>(n));
    if (n != 0) bytes(v->data(), static_cast<size_t>(n) * sizeof(T));
  }
  std::string text();

 private:
  std::FILE* file_ = nullptr;
  bool good_ = true;
};

/** A fresh, empty folder at `path` + ".new" for a checkpoint being written; empty on failure. An
 *  older checkpoint at `path` is deleted first, so the disk never holds two, unless `keep_old`
 *  (the run is resuming from it). */
std::string begin_folder(const std::string& path, bool keep_old);

/** Puts a finished `.new` folder in place of `path`, replacing any older one. */
bool finish_folder(const std::string& path);

/** Throws away a `.new` folder that is not wanted (the run was not stopped). */
void drop_folder(const std::string& path);

}  // namespace checkpoint
