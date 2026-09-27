/* Link's skeleton (`res/Object/Link.arc` `cl.bdl`) and animations (`res/Object/LkAnm.arc` `.bck`)
 * read off the disc and handed to the engine once per process.
 */
#pragma once

#include <string>

#include "gcm.h"

namespace disc {

/** Thread-safe; only the first successful call installs. */
bool install_link(const Iso& iso, std::string* why);

/** From `SETUPCORE_ISO`, else the image the app's settings name. */
bool install_link_from_settings(std::string* why);

/** That disc's path, or empty. */
std::string the_disc();

}  // namespace disc
