#include "link.h"

#include <cstdlib>
#include <memory>
#include <mutex>
#include <vector>

#include "engine/link_assets.h"
#include "rarc.h"
#include "yaz0.h"
#include "../seam/json.h"
#include "../settings/settings.h"

namespace disc {
namespace {

std::mutex& gate() {
  static std::mutex m;
  return m;
}

/** By exact name, decompressed: each `.bck` in LkAnm.arc is compressed on its own. */
bool member(const std::vector<uint8_t>& archive, const std::string& name,
            std::vector<uint8_t>* out) {
  std::vector<uint8_t> packed;
  if (!named_in_rarc(archive, name, &packed)) return false;
  return yaz0(packed, out);
}

}  // namespace

bool install_link(const Iso& iso, std::string* why) {
  std::lock_guard<std::mutex> hold(gate());
  if (tww_engine::linkAssetsInstalled()) return true;

  tww_engine::LinkAssets assets;
  std::vector<uint8_t> archive;
  if (!iso.file(tww_engine::kLinkModelArchive, &archive) ||
      !member(archive, tww_engine::kLinkModelFile, &assets.model)) {
    *why = std::string("no ") + tww_engine::kLinkModelFile + " in " + tww_engine::kLinkModelArchive;
    return false;
  }
  if (!iso.file(tww_engine::kLinkAnimationArchive, &archive)) {
    *why = std::string("no ") + tww_engine::kLinkAnimationArchive;
    return false;
  }
  for (const std::string& name : tww_engine::linkAnimationFiles()) {
    if (!member(archive, name, &assets.animations[name])) {
      *why = "no " + name + " in " + tww_engine::kLinkAnimationArchive;
      return false;
    }
  }
  return tww_engine::installLinkAssets(assets, why);
}

std::string the_disc() {
  const char* env = std::getenv("SETUPCORE_ISO");
  if (env != nullptr && *env) return env;
  std::string json, why;
  if (!settings::read(&json, &why) || json.empty()) return "";
  const seam::Json j = seam::parse(json);
  return j.ok() && j.has("iso") ? j.at("iso").as_str() : "";
}

bool install_link_from_settings(std::string* why) {
  const std::string path = the_disc();
  if (path.empty()) {
    *why = "no disc: set SETUPCORE_ISO or attach one in the app";
    return false;
  }
  std::unique_ptr<Iso> iso = Iso::open(path, why);
  if (!iso) return false;
  return install_link(*iso, why);
}

}  // namespace disc
