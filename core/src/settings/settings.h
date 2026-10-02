/* The per-user settings file: one JSON blob stored and returned opaque, written atomically. */
#pragma once

#include <string>

namespace settings {

/** Empty if this machine has no per-user config directory. */
std::string path();

/** `false` with `why` for a file that exists and would not read; a missing file is `true` with
 *  an empty `json`. */
bool read(std::string* json, std::string* why);

bool write(const std::string& json, std::string* why);

/** Empty with no per-user config directory. */
std::string log_path();

/** The folder holding the settings file. Empty with no per-user config directory. */
std::string folder();

/** The camera field cache. Empty with no per-user config directory. */
std::string cache_dir();

/** Appends one Logs line. The first call in a process truncates the file. Never reports failure. */
void keep_log(const std::string& text);

}  // namespace settings
