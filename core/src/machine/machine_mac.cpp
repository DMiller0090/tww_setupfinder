#include "machine.h"

#include <time.h>
#include <sys/sysctl.h>
#include <sys/types.h>

namespace machine {

uint64_t physical_memory() {
  uint64_t bytes = 0;
  size_t length = sizeof bytes;
  if (sysctlbyname("hw.memsize", &bytes, &length, nullptr, 0) != 0) return 0;
  return bytes;
}

uint64_t thread_cpu_ns() {
  timespec ts;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) return 0;
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000u + static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace machine
