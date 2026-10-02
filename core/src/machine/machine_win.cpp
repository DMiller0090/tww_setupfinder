#include "machine.h"

#include <windows.h>

namespace machine {

uint64_t physical_memory() {
  MEMORYSTATUSEX status;
  status.dwLength = sizeof status;
  if (!GlobalMemoryStatusEx(&status)) return 0;
  return static_cast<uint64_t>(status.ullTotalPhys);
}

uint64_t thread_cpu_ns() {
  FILETIME made, ended, kernel, user;
  if (!GetThreadTimes(GetCurrentThread(), &made, &ended, &kernel, &user)) return 0;
  const uint64_t k = (static_cast<uint64_t>(kernel.dwHighDateTime) << 32) | kernel.dwLowDateTime;
  const uint64_t u = (static_cast<uint64_t>(user.dwHighDateTime) << 32) | user.dwLowDateTime;
  return (k + u) * 100;
}

}  // namespace machine
