#include "machine.h"

#include <windows.h>

namespace machine {

uint64_t physical_memory() {
  MEMORYSTATUSEX status;
  status.dwLength = sizeof status;
  if (!GlobalMemoryStatusEx(&status)) return 0;
  return static_cast<uint64_t>(status.ullTotalPhys);
}

}  // namespace machine
