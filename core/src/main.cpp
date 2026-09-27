/* setupcore: the window's child process. One JSON request per stdin line, e.g.
 *     echo {"ask":"emulators"} | setupcore
 */
#include <chrono>
#include <cstdio>
#include <thread>

#include "seam/seam.h"

#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#include <windows.h>
#include <dbghelp.h>
#include <tlhelp32.h>

namespace {

/* Writes the fault and a stack trace to stderr; the process still dies. */
LONG WINAPI say_the_fault(EXCEPTION_POINTERS* about) {
  const DWORD code = about->ExceptionRecord->ExceptionCode;
  std::fprintf(stderr, "setupcore: FAULT 0x%08lX at %p on thread %lu\n",
               static_cast<unsigned long>(code), about->ExceptionRecord->ExceptionAddress,
               ::GetCurrentThreadId());
  if (code == EXCEPTION_ACCESS_VIOLATION &&
      about->ExceptionRecord->NumberParameters >= 2) {
    std::fprintf(stderr, "setupcore: %s %p\n",
                 about->ExceptionRecord->ExceptionInformation[0] ? "writing" : "reading",
                 reinterpret_cast<void*>(about->ExceptionRecord->ExceptionInformation[1]));
  }
  const HANDLE me = ::GetCurrentProcess();
  ::SymSetOptions(SYMOPT_DEFERRED_LOADS | SYMOPT_UNDNAME | SYMOPT_LOAD_LINES);
  ::SymInitialize(me, NULL, TRUE);
  void* frame[62];
  const USHORT deep = ::CaptureStackBackTrace(0, 62, frame, NULL);
  char room[sizeof(SYMBOL_INFO) + 512];
  SYMBOL_INFO* sym = reinterpret_cast<SYMBOL_INFO*>(room);
  sym->SizeOfStruct = sizeof(SYMBOL_INFO);
  sym->MaxNameLen = 511;
  for (USHORT i = 0; i < deep; ++i) {
    DWORD64 off = 0;
    IMAGEHLP_LINE64 at;
    at.SizeOfStruct = sizeof(at);
    DWORD line_off = 0;
    const bool named = ::SymFromAddr(me, reinterpret_cast<DWORD64>(frame[i]), &off, sym) != FALSE;
    const bool placed =
        ::SymGetLineFromAddr64(me, reinterpret_cast<DWORD64>(frame[i]), &line_off, &at) != FALSE;
    std::fprintf(stderr, "setupcore:  %2u %s%s%s:%lu\n", static_cast<unsigned>(i),
                 named ? sym->Name : "?", placed ? "  " : "",
                 placed ? at.FileName : "", placed ? at.LineNumber : 0);
  }
  std::fflush(stderr);
  return EXCEPTION_EXECUTE_HANDLER;
}

/* Null when there is none, or when the pid was reused (a process younger than this one). */
HANDLE open_parent() {
  const DWORD me = ::GetCurrentProcessId();
  DWORD parent = 0;
  const HANDLE snap = ::CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (snap == INVALID_HANDLE_VALUE) return NULL;
  PROCESSENTRY32W entry;
  entry.dwSize = sizeof(entry);
  for (BOOL more = ::Process32FirstW(snap, &entry); more; more = ::Process32NextW(snap, &entry)) {
    if (entry.th32ProcessID == me) {
      parent = entry.th32ParentProcessID;
      break;
    }
  }
  ::CloseHandle(snap);
  if (parent == 0) return NULL;
  const HANDLE held = ::OpenProcess(SYNCHRONIZE | PROCESS_QUERY_LIMITED_INFORMATION, FALSE, parent);
  if (held == NULL) return NULL;
  FILETIME born_parent, born_me, unused1, unused2, unused3;
  if (!::GetProcessTimes(held, &born_parent, &unused1, &unused2, &unused3) ||
      !::GetProcessTimes(::GetCurrentProcess(), &born_me, &unused1, &unused2, &unused3) ||
      ::CompareFileTime(&born_parent, &born_me) > 0) {
    ::CloseHandle(held);
    return NULL;
  }
  return held;
}

}  // namespace
#else
#include <unistd.h>
#endif

namespace {

/* A parent that dies without closing its pipe still ends the core. */
void watch_the_parent() {
#if defined(_WIN32)
  const HANDLE parent = open_parent();
  if (parent == NULL) return;
  std::thread([parent]() {
    ::WaitForSingleObject(parent, INFINITE);
    seam::closing();
  }).detach();
#else
  // An orphan is re-parented, so a changed ppid means the parent is gone.
  const pid_t parent = ::getppid();
  std::thread([parent]() {
    while (::getppid() == parent) std::this_thread::sleep_for(std::chrono::seconds(1));
    seam::closing();
  }).detach();
#endif
}

}  // namespace

int main() {
#if defined(_WIN32)
  ::SetUnhandledExceptionFilter(say_the_fault);
#endif
#if defined(_WIN32)
  // Binary, so `\n` is not written as `\r\n`.
  ::_setmode(::_fileno(stdout), _O_BINARY);
  ::_setmode(::_fileno(stdin), _O_BINARY);
#endif
  watch_the_parent();
  return seam::run();
}
