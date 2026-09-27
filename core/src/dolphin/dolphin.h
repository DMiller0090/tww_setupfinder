/* Read-only access to a running Dolphin's emulated RAM, located as Dolphin Memory Engine does: the
 * committed 32 MiB region whose GC `0x8000001C` holds the disc magic. Not possible on macOS (SIP).
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace dolphin {

constexpr uint32_t kMem1Start = 0x80000000u;
constexpr uint32_t kMem1Size = 0x02000000u;  // 32 MiB
/** At GC `0x8000001C`. */
constexpr uint32_t kDiscMagic = 0xC2339F3Du;

#if defined(__APPLE__)
constexpr bool kCanRead = false;
#else
constexpr bool kCanRead = true;
#endif

/** Empty when `kCanRead` is true. */
const char* why_not();

struct Found {
  int pid = 0;
  std::string exe;
};

/** Every Dolphin running for this user. */
std::vector<Found> running();

class Mem {
 public:
  /** Null on failure, with `why` set. */
  static std::unique_ptr<Mem> attach(int pid, std::string* why);
  ~Mem();

  Mem(const Mem&) = delete;
  Mem& operator=(const Mem&) = delete;

  /** False unless all `n` bytes were read. */
  bool read(uint32_t gc_addr, void* dst, size_t n) const;

  /* Big-endian. */
  bool u8(uint32_t gc_addr, uint8_t* out) const;
  bool u16(uint32_t gc_addr, uint16_t* out) const;
  bool u32(uint32_t gc_addr, uint32_t* out) const;
  bool s32(uint32_t gc_addr, int32_t* out) const;
  bool f32(uint32_t gc_addr, float* out) const;

  /** Dereference, then add the offset, once per offset. */
  bool chain(uint32_t base, const std::vector<uint32_t>& offsets, uint32_t* out) const;

 private:
  Mem() = default;

  /* A `HANDLE` on Windows; unused on Linux, which uses `pid_`. */
  void* handle_ = nullptr;
  int pid_ = 0;
  /** MEM1's host address. */
  uintptr_t base_ = 0;
};

}  // namespace dolphin
