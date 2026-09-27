/* macOS: `mach_vm_read` under SIP needs root or a debugger entitlement, so no live read. */
#include "dolphin.h"

namespace dolphin {

const char* why_not() {
  return "macOS will not let one app read another's memory, so reading a running game "
         "is Windows and Linux only";
}

std::vector<Found> running() { return {}; }

std::unique_ptr<Mem> Mem::attach(int, std::string* why) {
  if (why) *why = why_not();
  return nullptr;
}

Mem::~Mem() = default;

bool Mem::read(uint32_t, void*, size_t) const { return false; }

}  // namespace dolphin
