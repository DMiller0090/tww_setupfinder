#include "machine.h"

#include <sys/sysctl.h>
#include <sys/types.h>

namespace machine {

uint64_t physical_memory() {
  uint64_t bytes = 0;
  size_t length = sizeof bytes;
  if (sysctlbyname("hw.memsize", &bytes, &length, nullptr, 0) != 0) return 0;
  return bytes;
}

}  // namespace machine
