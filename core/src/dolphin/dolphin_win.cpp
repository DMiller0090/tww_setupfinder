/* Windows. MEM1 is found by the disc magic; size alone can match an unrelated mapping. */
#include "dolphin.h"

#include <windows.h>
#include <psapi.h>
#include <tlhelp32.h>

#include <cstring>
#include <string>

namespace dolphin {
namespace {

constexpr DWORD kRights = PROCESS_QUERY_INFORMATION | PROCESS_VM_READ;

bool looks_like_dolphin(const wchar_t* exe) {
  // Loose: `Dolphin.exe`, `dolphin-emu.exe`, and forks.
  std::wstring name(exe);
  for (wchar_t& c : name) c = static_cast<wchar_t>(::towlower(c));
  return name.find(L"dolphin") != std::wstring::npos;
}

std::string narrow(const wchar_t* w) {
  int n = ::WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
  if (n <= 1) return {};
  std::string out(static_cast<size_t>(n - 1), '\0');
  ::WideCharToMultiByte(CP_UTF8, 0, w, -1, out.data(), n, nullptr, nullptr);
  return out;
}

bool has_disc_header(HANDLE h, uintptr_t base) {
  uint8_t b[4];
  SIZE_T got = 0;
  if (!::ReadProcessMemory(h, reinterpret_cast<LPCVOID>(base + 0x1C), b, sizeof b, &got) ||
      got != sizeof b) {
    return false;
  }
  uint32_t magic = (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16) |
                   (static_cast<uint32_t>(b[2]) << 8) | static_cast<uint32_t>(b[3]);
  return magic == kDiscMagic;
}

bool region_valid(HANDLE h, uintptr_t base) {
  PSAPI_WORKING_SET_EX_INFORMATION ws{};
  ws.VirtualAddress = reinterpret_cast<PVOID>(base);
  if (!::QueryWorkingSetEx(h, &ws, sizeof ws)) return false;
  return ws.VirtualAttributes.Valid != 0;
}

uintptr_t find_mem1(HANDLE h) {
  MEMORY_BASIC_INFORMATION mbi{};
  uintptr_t addr = 0;
  uintptr_t fallback = 0;
  while (::VirtualQueryEx(h, reinterpret_cast<LPCVOID>(addr), &mbi, sizeof mbi) == sizeof mbi) {
    const uintptr_t base = reinterpret_cast<uintptr_t>(mbi.BaseAddress);
    const uintptr_t size = static_cast<uintptr_t>(mbi.RegionSize);
    // Ascending order, so the first match is MEM1, not a fastmem mirror above it.
    if (size >= kMem1Size && mbi.State == MEM_COMMIT && has_disc_header(h, base)) return base;
    if (size == kMem1Size && mbi.Type == MEM_MAPPED && !fallback && region_valid(h, base)) {
      fallback = base;
    }
    if (size == 0) break;
    addr = base + size;
  }
  // Before the game writes its header.
  return fallback;
}

}  // namespace

const char* why_not() { return ""; }

std::vector<Found> running() {
  std::vector<Found> out;
  HANDLE snap = ::CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (snap == INVALID_HANDLE_VALUE) return out;
  PROCESSENTRY32W e{};
  e.dwSize = sizeof e;
  if (::Process32FirstW(snap, &e)) {
    do {
      if (looks_like_dolphin(e.szExeFile)) {
        out.push_back({static_cast<int>(e.th32ProcessID), narrow(e.szExeFile)});
      }
    } while (::Process32NextW(snap, &e));
  }
  ::CloseHandle(snap);
  return out;
}

std::unique_ptr<Mem> Mem::attach(int pid, std::string* why) {
  HANDLE h = ::OpenProcess(kRights, FALSE, static_cast<DWORD>(pid));
  if (!h) {
    if (why) *why = "could not open that Dolphin - try running the app as the same user";
    return nullptr;
  }
  const uintptr_t base = find_mem1(h);
  if (!base) {
    ::CloseHandle(h);
    if (why) *why = "that Dolphin has no game in it";
    return nullptr;
  }
  std::unique_ptr<Mem> mem(new Mem());
  mem->handle_ = h;
  mem->pid_ = pid;
  mem->base_ = base;
  return mem;
}

Mem::~Mem() {
  if (handle_) ::CloseHandle(static_cast<HANDLE>(handle_));
}

bool Mem::read(uint32_t gc_addr, void* dst, size_t n) const {
  if (gc_addr < kMem1Start || n > kMem1Size ||
      gc_addr - kMem1Start > kMem1Size - static_cast<uint32_t>(n)) {
    return false;
  }
  SIZE_T got = 0;
  const uintptr_t host = base_ + (gc_addr - kMem1Start);
  return ::ReadProcessMemory(static_cast<HANDLE>(handle_), reinterpret_cast<LPCVOID>(host), dst,
                             n, &got) != 0 && got == n;
}

}  // namespace dolphin
