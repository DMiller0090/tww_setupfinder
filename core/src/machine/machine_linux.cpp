#include "machine.h"

#include <time.h>
#include <unistd.h>

namespace machine {

uint64_t physical_memory() {
  const long pages = sysconf(_SC_PHYS_PAGES);
  const long size = sysconf(_SC_PAGE_SIZE);
  if (pages <= 0 || size <= 0) return 0;
  return static_cast<uint64_t>(pages) * static_cast<uint64_t>(size);
}

uint64_t thread_cpu_ns() {
  timespec ts;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) return 0;
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000u + static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace machine
