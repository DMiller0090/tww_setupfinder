#include "machine.h"

#include <unistd.h>

namespace machine {

uint64_t physical_memory() {
  const long pages = sysconf(_SC_PHYS_PAGES);
  const long size = sysconf(_SC_PAGE_SIZE);
  if (pages <= 0 || size <= 0) return 0;
  return static_cast<uint64_t>(pages) * static_cast<uint64_t>(size);
}

}  // namespace machine
