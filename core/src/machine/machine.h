#pragma once

#include <cstdint>

namespace machine {

/** In bytes, or zero when unknown. */
uint64_t physical_memory();

/** This thread's processor time, user and kernel, in nanoseconds. */
uint64_t thread_cpu_ns();

}  // namespace machine
