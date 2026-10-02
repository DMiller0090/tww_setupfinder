#include "checkpoint.h"

#include <filesystem>
#include <system_error>

namespace checkpoint {
namespace {

std::FILE* open(const std::string& path, const char* mode) {
#if defined(_MSC_VER)
  std::FILE* f = nullptr;
  if (fopen_s(&f, path.c_str(), mode) != 0) return nullptr;
  return f;
#else
  return std::fopen(path.c_str(), mode);
#endif
}

const size_t kBuffer = size_t(1) << 20;

}  // namespace

Writer::Writer(const std::string& path) : path_(path) {
  file_ = open(path + ".part", "wb");
  if (file_ != nullptr) std::setvbuf(file_, nullptr, _IOFBF, kBuffer);
}

Writer::~Writer() {
  if (file_ != nullptr) std::fclose(file_);
}

void Writer::bytes(const void* p, size_t n) {
  if (!ok() || n == 0) return;
  if (std::fwrite(p, 1, n, file_) != n) good_ = false;
}

void Writer::text(const std::string& s) {
  pod<uint64_t>(s.size());
  bytes(s.data(), s.size());
}

bool Writer::close() {
  if (file_ == nullptr) return false;
  if (std::fflush(file_) != 0) good_ = false;
  std::fclose(file_);
  file_ = nullptr;
  if (!good_) return false;
  std::error_code ec;
  std::filesystem::rename(path_ + ".part", path_, ec);
  return !ec;
}

Reader::Reader(const std::string& path) {
  file_ = open(path, "rb");
  if (file_ != nullptr) std::setvbuf(file_, nullptr, _IOFBF, kBuffer);
}

Reader::~Reader() {
  if (file_ != nullptr) std::fclose(file_);
}

void Reader::bytes(void* p, size_t n) {
  if (!ok() || n == 0) return;
  if (std::fread(p, 1, n, file_) != n) good_ = false;
}

std::string Reader::text() {
  const uint64_t n = pod<uint64_t>();
  if (!ok() || n > (uint64_t(1) << 30)) {
    good_ = false;
    return std::string();
  }
  std::string s(static_cast<size_t>(n), '\0');
  if (n != 0) bytes(&s[0], static_cast<size_t>(n));
  return s;
}

std::string begin_folder(const std::string& path, bool keep_old) {
  if (path.empty()) return std::string();
  std::error_code ec;
  const std::filesystem::path fresh(path + ".new");
  if (!keep_old) std::filesystem::remove_all(path, ec);
  std::filesystem::remove_all(fresh, ec);
  std::filesystem::create_directories(fresh, ec);
  if (ec) return std::string();
  return fresh.string();
}

bool finish_folder(const std::string& path) {
  std::error_code ec;
  const std::filesystem::path fresh(path + ".new");
  if (!std::filesystem::exists(fresh, ec)) return false;
  std::filesystem::remove_all(path, ec);
  ec.clear();
  std::filesystem::rename(fresh, path, ec);
  return !ec;
}

void drop_folder(const std::string& path) {
  std::error_code ec;
  std::filesystem::remove_all(std::filesystem::path(path + ".new"), ec);
}

}  // namespace checkpoint
