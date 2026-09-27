/* Linux: `/proc/<pid>/maps` to find MEM1, `process_vm_readv` to read it. */
#include "dolphin.h"

#include <dirent.h>
#include <sys/uio.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

namespace dolphin {
namespace {

std::string lower(std::string s) {
  for (char& c : s) c = static_cast<char>(::tolower(static_cast<unsigned char>(c)));
  return s;
}

std::string read_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string comm(int pid) {
  std::string s = read_file("/proc/" + std::to_string(pid) + "/comm");
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
  return s;
}

bool ptrace_scope_blocks() {
  std::string s = read_file("/proc/sys/kernel/yama/ptrace_scope");
  return !s.empty() && s[0] != '0';
}

bool read_at(int pid, uintptr_t host, void* dst, size_t n) {
  iovec local{dst, n};
  iovec remote{reinterpret_cast<void*>(host), n};
  ssize_t got = ::process_vm_readv(pid, &local, 1, &remote, 1, 0);
  return got >= 0 && static_cast<size_t>(got) == n;
}

bool has_disc_header(int pid, uintptr_t base) {
  uint8_t b[4];
  if (!read_at(pid, base + 0x1C, b, sizeof b)) return false;
  uint32_t magic = (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
                   (static_cast<uint32_t>(b[2]) << 8) | static_cast<uint32_t>(b[3]);
  return magic == kDiscMagic;
}

uintptr_t find_mem1(int pid) {
  std::ifstream maps("/proc/" + std::to_string(pid) + "/maps");
  if (!maps) return 0;
  uintptr_t fallback = 0;
  std::string line;
  while (std::getline(maps, line)) {
    // `start-end perms ...`; MEM1 is shared memory, so the path is not matched on.
    uintptr_t from = 0, to = 0;
    char perms[8] = {0};
    if (std::sscanf(line.c_str(), "%zx-%zx %7s", &from, &to, perms) != 3) continue;
    if (perms[0] != 'r') continue;
    const uintptr_t size = to - from;
    if (size >= kMem1Size && has_disc_header(pid, from)) return from;
    if (size == kMem1Size && !fallback) fallback = from;
  }
  return fallback;
}

}  // namespace

const char* why_not() { return ""; }

std::vector<Found> running() {
  std::vector<Found> out;
  DIR* proc = ::opendir("/proc");
  if (!proc) return out;
  while (dirent* e = ::readdir(proc)) {
    char* end = nullptr;
    long pid = std::strtol(e->d_name, &end, 10);
    if (!end || *end || pid <= 0) continue;
    std::string name = comm(static_cast<int>(pid));
    if (lower(name).find("dolphin") != std::string::npos) {
      out.push_back({static_cast<int>(pid), name});
    }
  }
  ::closedir(proc);
  return out;
}

std::unique_ptr<Mem> Mem::attach(int pid, std::string* why) {
  const uintptr_t base = find_mem1(pid);
  if (!base) {
    if (why) {
      // Yama `ptrace_scope` 1 (the common default) blocks reads of non-child processes.
      if (ptrace_scope_blocks()) {
        *why = "Linux is blocking the read: run "
               "`sudo sysctl -w kernel.yama.ptrace_scope=0` and press Connect again";
      } else {
        *why = "that Dolphin has no game in it";
      }
    }
    return nullptr;
  }
  std::unique_ptr<Mem> mem(new Mem());
  mem->handle_ = nullptr;
  mem->pid_ = pid;
  mem->base_ = base;
  return mem;
}

Mem::~Mem() = default;

bool Mem::read(uint32_t gc_addr, void* dst, size_t n) const {
  if (gc_addr < kMem1Start || n > kMem1Size ||
      gc_addr - kMem1Start > kMem1Size - static_cast<uint32_t>(n)) {
    return false;
  }
  return read_at(pid_, base_ + (gc_addr - kMem1Start), dst, n);
}

}  // namespace dolphin
