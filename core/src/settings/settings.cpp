#include "settings.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>
#include <thread>

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

namespace settings {
namespace {

constexpr const char* kFolder = "setup-finder";
constexpr const char* kFile = "settings.json";
constexpr const char* kLogFile = "log.txt";

/** Empty if unset. `_dupenv_s` because MSVC deprecates `getenv`. */
std::string env(const char* name) {
#if defined(_MSC_VER)
  char* value = nullptr;
  size_t len = 0;
  if (::_dupenv_s(&value, &len, name) != 0 || !value) return {};
  std::string out(value);
  std::free(value);
  return out;
#else
  const char* value = std::getenv(name);
  return value ? std::string(value) : std::string();
#endif
}

/* The per-user config directory for this OS. */
std::filesystem::path home() {
#if defined(_WIN32)
  const std::string roaming = env("APPDATA");
  if (!roaming.empty()) return std::filesystem::path(roaming);
  return {};
#elif defined(__APPLE__)
  const std::string h = env("HOME");
  if (h.empty()) return {};
  return std::filesystem::path(h) / "Library" / "Application Support";
#else
  const std::string x = env("XDG_CONFIG_HOME");
  if (!x.empty()) return std::filesystem::path(x);
  const std::string h = env("HOME");
  if (h.empty()) return {};
  return std::filesystem::path(h) / ".config";
#endif
}

}  // namespace

std::string path() {
  const std::filesystem::path base = home();
  if (base.empty()) return {};
  return (base / kFolder / kFile).string();
}

bool read(std::string* json, std::string* why) {
  json->clear();
  const std::string where = path();
  if (where.empty()) {
    *why = "this machine names no place to keep settings";
    return false;
  }
  std::error_code ec;
  // No file yet is not a failure.
  if (!std::filesystem::exists(where, ec) || ec) return true;

  std::ifstream in(where, std::ios::binary);
  if (!in) {
    *why = "the settings file would not open";
    return false;
  }
  std::ostringstream text;
  text << in.rdbuf();
  if (in.bad()) {
    *why = "the settings file would not read";
    return false;
  }
  *json = text.str();
  return true;
}

bool write(const std::string& json, std::string* why) {
  const std::string where = path();
  if (where.empty()) {
    *why = "this machine names no place to keep settings";
    return false;
  }
  const std::filesystem::path file(where);
  std::error_code ec;
  std::filesystem::create_directories(file.parent_path(), ec);
  if (ec) {
    *why = "the settings folder could not be made";
    return false;
  }

  // Write beside, then rename over, so a kill mid-write leaves the old file. Beside is this
  // process's own, so two windows saving at once never write one file.
#if defined(_WIN32)
  const long pid = ::_getpid();
#else
  const long pid = static_cast<long>(::getpid());
#endif
  const std::filesystem::path beside =
      file.parent_path() / (std::string(kFile) + "." + std::to_string(pid) + ".new");
  {
    std::ofstream out(beside, std::ios::binary | std::ios::trunc);
    if (!out) {
      *why = "the settings file could not be written";
      return false;
    }
    out << json;
    out.flush();
    if (!out) {
      *why = "the settings file could not be written";
      return false;
    }
  }
  // A rename fails for an instant while another window replaces the same file.
  for (int tries = 0; tries < 50; ++tries) {
    std::filesystem::rename(beside, file, ec);
    if (!ec) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (ec) {
    // Some Windows filesystems refuse a rename over an existing file.
    std::filesystem::copy_file(beside, file,
                               std::filesystem::copy_options::overwrite_existing, ec);
    std::error_code ignored;
    std::filesystem::remove(beside, ignored);
    if (ec) {
      *why = "the settings file could not be replaced";
      return false;
    }
  }
  return true;
}

std::string folder() {
  const std::filesystem::path base = home();
  if (base.empty()) return {};
  return (base / kFolder).string();
}

std::string cache_dir() {
  const std::filesystem::path base = home();
  if (base.empty()) return {};
  return (base / kFolder / "camera").string();
}

std::string log_path() {
  const std::filesystem::path base = home();
  if (base.empty()) return {};
  return (base / kFolder / kLogFile).string();
}

void keep_log(const std::string& text) {
  const std::string where = log_path();
  if (where.empty()) return;
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(where).parent_path(), ec);
  static bool started = false;
  std::ofstream out(where, started ? (std::ios::binary | std::ios::app)
                                   : (std::ios::binary | std::ios::trunc));
  if (!out) return;
  started = true;
  out << text << "\n";
}

}  // namespace settings
