/* The core's side of the seam: one JSON request line on stdin, then `{"line":"data",...}` lines
 * and exactly one `{"line":"done"}` or `{"line":"fail","why":...}` on stdout.
 */
#pragma once

#include <functional>
#include <string>

namespace seam {

/** One line, no newline - the caller adds it and flushes. */
using Out = std::function<void(const std::string&)>;

/** Always writes a `done` or a `fail`, whatever the line contained. */
void answer(const std::string& request, const Out& out);

/** Answer stdin lines in order until stdin closes. `{"signal":"stop"}` is read on its own thread
 *  and ends the running search instead of being answered. */
int run();

/** Stops the run and the queue, and exits within ten seconds. Safe from any thread, repeatedly. */
void closing();

}  // namespace seam
