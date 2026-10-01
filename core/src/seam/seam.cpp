#include "seam.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "../disc/disc.h"
#include "../disc/link.h"
#include "../dolphin/collision.h"
#include "../dolphin/dolphin.h"
#include "../dolphin/game.h"
#include "../geom/mesh.h"
#include "../search/approx.h"
#include "../search/cam_clear.h"
#include "../search/corridor.h"
#include "../search/dzb.h"
#include "../search/grid.h"
#include "../search/search.h"
#include "../search/verify.h"
#include "engine/link_assets.h"
#include "../machine/machine.h"
#include "../settings/settings.h"
#include "json.h"

namespace seam {
namespace {

constexpr int kPausedSampleMs = 40;

void data(const Out& out, const char* of, Json body) {
  Json line = Json::obj();
  line.set("line", Json::str("data"));
  line.set("of", Json::str(of));
  line.set("body", std::move(body));
  out(line.dump());
}

void done(const Out& out) {
  Json line = Json::obj();
  line.set("line", Json::str("done"));
  out(line.dump());
}

void fail(const Out& out, const std::string& why) {
  Json line = Json::obj();
  line.set("line", Json::str("fail"));
  line.set("why", Json::str(why));
  out(line.dump());
}

/** A failure the window words itself, keyed on `code`; `why` is only for the Logs. */
void fail(const Out& out, const std::string& why, const char* code) {
  Json line = Json::obj();
  line.set("line", Json::str("fail"));
  line.set("why", Json::str(why));
  line.set("code", Json::str(code));
  out(line.dump());
}

/** `DOLPHIN_PID`, or 0. */
int preferred_pid() {
#if defined(_MSC_VER)
  char* value = nullptr;
  size_t len = 0;
  if (::_dupenv_s(&value, &len, "DOLPHIN_PID") != 0 || !value) return 0;
  const int pid = std::atoi(value);
  std::free(value);
  return pid;
#else
  const char* env = std::getenv("DOLPHIN_PID");
  if (!env || !*env) return 0;
  return std::atoi(env);
#endif
}

void machine(const Out& out) {
  Json body = Json::obj();
  body.set("reads", Json::boolean(dolphin::kCanRead));
  // A ceiling for the thread control; `hardware_concurrency` may answer 0.
  const unsigned told = std::thread::hardware_concurrency();
  body.set("cores", Json::num(static_cast<double>(told > 0 ? told : 1u)));
  data(out, "machine", std::move(body));
  done(out);
}

void emulators(const Out& out) {
  if (!dolphin::kCanRead) {
    fail(out, dolphin::why_not());
    return;
  }
  const int pref = preferred_pid();
  Json list = Json::arr();
  for (const dolphin::Found& found : dolphin::running()) {
    Json e = Json::obj();
    e.set("pid", Json::num(found.pid));
    e.set("game", Json::null_());
    e.set("paused", Json::boolean(false));
    e.set("reads", Json::boolean(false));
    if (pref && found.pid == pref) e.set("pref", Json::boolean(true));

    // A failed attach still lists the Dolphin, with no game.
    std::string why;
    if (std::unique_ptr<dolphin::Mem> mem = dolphin::Mem::attach(found.pid, &why)) {
      const dolphin::Disc disc = dolphin::disc_of(*mem);
      if (!disc.name.empty()) e.set("game", Json::str(disc.name));
      // The frame counter address is only valid on the known release.
      if (disc.reads) e.set("paused", Json::boolean(dolphin::is_paused(*mem, kPausedSampleMs)));
      e.set("reads", Json::boolean(disc.reads));
    }
    list.push(std::move(e));
  }
  data(out, "emulators", std::move(list));
  done(out);
}

/* Attaches to the request's `pid`; the core never picks a Dolphin itself. */
std::unique_ptr<dolphin::Mem> attached(const Out& out, const Json& request) {
  if (!dolphin::kCanRead) {
    fail(out, dolphin::why_not());
    return nullptr;
  }
  const Json& pid = request.at("pid");
  if (pid.type() != Json::Type::Num) {
    fail(out, "which Dolphin? the request carried no pid");
    return nullptr;
  }
  std::string why;
  std::unique_ptr<dolphin::Mem> mem = dolphin::Mem::attach(static_cast<int>(pid.as_num()), &why);
  if (!mem) fail(out, why.empty() ? "could not attach to that Dolphin" : why);
  return mem;
}

/* Checks the release before any `game.h` address is read. */
bool placed(const Out& out, const dolphin::Mem& mem, dolphin::Place* place) {
  const dolphin::Disc disc = dolphin::disc_of(mem);
  if (disc.id.empty()) {
    fail(out, "that Dolphin has no game in it");
    return false;
  }
  if (!disc.reads) {
    fail(out, "found " + disc.id);
    return false;
  }
  *place = dolphin::place_of(mem);
  if (!place->ok) {
    fail(out, "the game is not in a room yet");
    return false;
  }
  return true;
}

/* Opens the request's `iso` and checks it is the known release. */
std::unique_ptr<disc::Iso> opened(const Out& out, const Json& request) {
  const Json& named = request.at("iso");
  if (named.type() != Json::Type::Str || named.as_str().empty()) {
    fail(out, "which disc? the request carried no iso");
    return nullptr;
  }
  std::string why;
  std::unique_ptr<disc::Iso> iso = disc::Iso::open(named.as_str(), &why);
  if (!iso) {
    fail(out, why.empty() ? "could not open that disc" : why);
    return nullptr;
  }
  if (iso->id() != disc::kKnownDisc) {
    fail(out, "found " + iso->id());
    return nullptr;
  }
  return iso;
}

/** Installs Link off the request's disc once per process; needed even when the room comes from RAM. */
bool link_ready(const Out& out, const Json& request) {
  if (tww_engine::linkAssetsInstalled()) return true;
  std::unique_ptr<disc::Iso> iso = opened(out, request);
  if (!iso) return false;
  std::string why;
  if (!disc::install_link(*iso, &why)) {
    std::fprintf(stderr, "link: %s\n", why.c_str());
    fail(out, "could not open that disc");
    return false;
  }
  return true;
}

/** A request coordinate rounded to the nearest f32, as every game position is. */
double coordinate(const Json& field, double fallback = 0.0) {
  return static_cast<double>(static_cast<float>(field.as_num(fallback)));
}

/** Every room read goes through this, so drawing and searching agree on what is floor. */
void seabed_of(const std::string& stage, geom::Mesh* mesh) {
  geom::seabed_of(stage, *mesh);
}

Json geometry(const std::string& stage, int room, const geom::Mesh& mesh) {
  Json body = Json::obj();
  body.set("stage", Json::str(stage));
  body.set("room", Json::num(room));
  body.set("ground", Json::raw(floats(mesh.ground)));
  body.set("wall", Json::raw(floats(mesh.wall)));
  body.set("roof", Json::raw(floats(mesh.roof)));
  // Drawn only; never treated as ground.
  body.set("seabed", Json::raw(floats(mesh.seabed)));
  return body;
}

Json listed(const std::string& stage, int room, const geom::Mesh& mesh) {
  const size_t ground = mesh.ground.size() / 9, wall = mesh.wall.size() / 9,
               roof = mesh.roof.size() / 9;
  Json counts = Json::obj();
  counts.set("ground", Json::num(static_cast<double>(ground)));
  counts.set("wall", Json::num(static_cast<double>(wall)));
  counts.set("roof", Json::num(static_cast<double>(roof)));
  Json box = Json::arr();
  for (float edge : mesh.box) box.push(Json::f32(edge));
  Json it = Json::obj();
  it.set("stage", Json::str(stage));
  it.set("room", Json::num(room));
  it.set("tris", Json::num(static_cast<double>(ground + wall + roof)));
  it.set("box", std::move(box));
  it.set("counts", std::move(counts));
  return it;
}

/* The current room from RAM, moving collision included. */
void room(const Out& out, const Json& request) {
  std::unique_ptr<dolphin::Mem> mem = attached(out, request);
  if (!mem) return;
  dolphin::Place place;
  if (!placed(out, *mem, &place)) return;
  geom::Mesh got = dolphin::room_of(*mem, place.room);
  if (!got.ok) {
    fail(out, got.why);
    return;
  }
  seabed_of(place.stage, &got);
  data(out, "room", geometry(place.stage, place.room, got));
  done(out);
}

void room_on_disc(const Out& out, const Json& request) {
  std::unique_ptr<disc::Iso> iso = opened(out, request);
  if (!iso) return;
  const std::string stage = request.at("stage").as_str();
  const int room = static_cast<int>(request.at("room").as_num(-1));
  geom::Mesh got = disc::room_of(*iso, stage, room);
  if (!got.ok) {
    fail(out, got.why);
    return;
  }
  seabed_of(stage, &got);
  data(out, "room", geometry(stage, room, got));
  done(out);
}

/* Attached, the list is only the room the game is in. */
void rooms(const Out& out, const Json& request) {
  std::unique_ptr<dolphin::Mem> mem = attached(out, request);
  if (!mem) return;
  dolphin::Place place;
  if (!placed(out, *mem, &place)) return;
  const geom::Mesh got = dolphin::room_of(*mem, place.room);
  if (!got.ok) {
    fail(out, got.why);
    return;
  }
  Json list = Json::arr();
  list.push(listed(place.stage, place.room, got));
  data(out, "rooms", std::move(list));
  done(out);
}

void rooms_on_disc(const Out& out, const Json& request) {
  std::unique_ptr<disc::Iso> iso = opened(out, request);
  if (!iso) return;
  Json list = Json::arr();
  for (const disc::Where& where : disc::rooms_on(*iso)) {
    const geom::Mesh got = disc::room_of(*iso, where.stage, where.room);
    if (!got.ok) continue;
    list.push(listed(where.stage, where.room, got));
  }
  data(out, "rooms", std::move(list));
  done(out);
}

void start(const Out& out, const Json& request) {
  std::unique_ptr<dolphin::Mem> mem = attached(out, request);
  if (!mem) return;
  const dolphin::Start got = dolphin::start_of(*mem);
  if (!got.ok) {
    fail(out, got.why);
    return;
  }
  Json body = Json::obj();
  body.set("x", Json::f32(got.x));
  body.set("y", Json::f32(got.y));
  body.set("z", Json::f32(got.z));
  body.set("facing", Json::num(got.facing));
  body.set("camera", Json::num(got.camera));
  data(out, "start", std::move(body));
  done(out);
}

/* The settings blob, opaque to the core. No file yet answers an empty object. */
void settings_read(const Out& out) {
  std::string text, why;
  if (!settings::read(&text, &why)) {
    fail(out, why);
    return;
  }
  if (text.empty()) {
    data(out, "settings", Json::obj());
    done(out);
    return;
  }
  // Parsed so a corrupt file cannot put non-JSON on stdout.
  Json kept = parse(text);
  if (!kept.ok()) {
    fail(out, "the settings file could not be read: " + kept.why());
    return;
  }
  data(out, "settings", std::move(kept));
  done(out);
}

void settings_write(const Out& out, const Json& request) {
  const Json& body = request.at("body");
  if (body.type() != Json::Type::Obj) {
    fail(out, "the request carried nothing to keep");
    return;
  }
  std::string why;
  if (!settings::write(body.dump(), &why)) {
    fail(out, why);
    return;
  }
  done(out);
}

}  // namespace

/** Shared with the search so the reported offset and the tolerance count the same way. */
using search::float_steps;

/** Built on the first search and kept: it depends on the moves, not the room. */
const search::BaseTable& base_table_once() {
  static const search::BaseTable table = search::base_table(true);
  return table;
}

/* The search answers `{"line":"data","of":"plans"}` snapshots of `{plans, pct, searched, rate}`.
 * An attached search steps on the room's own `cBgD_t` only, not moving collision. */

/* The room as engine tables and as a mesh, from one read (`pid`: the current room; else `iso`). */
struct World {
  tww_engine::RoomDzb dzb;
  geom::Mesh mesh;
  /** Without camera names the camera field allows every type. */
  bool has_names = false;
  disc::CameraNames names;
};

bool world_of(const Out& out, const Json& request, World* world) {
  std::string why;
  if (request.at("pid").type() == Json::Type::Num) {
    std::unique_ptr<dolphin::Mem> mem = attached(out, request);
    if (!mem) return false;
    dolphin::Place place;
    if (!placed(out, *mem, &place)) return false;
    const dolphin::Dzb got = dolphin::dzb_of(*mem, place.room);
    if (!got.ok) {
      fail(out, got.why);
      return false;
    }
    if (!search::room_of(got.located(), &world->dzb, &why)) {
      fail(out, why);
      return false;
    }
    world->mesh = got.mesh;
    seabed_of(place.stage, &world->mesh);
    // From RAM, else the disc; a failure is not reported.
    world->has_names = dolphin::camera_names_of(*mem, place.room, &world->names);
    const Json& named = request.at("iso");
    if (!world->has_names && named.type() == Json::Type::Str && !named.as_str().empty()) {
      std::string quiet;
      std::unique_ptr<disc::Iso> iso = disc::Iso::open(named.as_str(), &quiet);
      if (iso && iso->id() == disc::kKnownDisc) {
        world->has_names = disc::camera_names_of(*iso, place.stage, place.room, &world->names,
                                                 &quiet);
      }
    }
    return true;
  }

  std::unique_ptr<disc::Iso> iso = opened(out, request);
  if (!iso) return false;
  const std::string stage = request.at("stage").as_str();
  const int room = static_cast<int>(request.at("room").as_num(-1));
  std::vector<uint8_t> bytes;
  if (!disc::dzb_of(*iso, stage, room, &bytes, &why)) {
    fail(out, why);
    return false;
  }
  if (!search::room_of(bytes, &world->dzb, &why)) {
    fail(out, why);
    return false;
  }
  world->mesh = disc::room_of(*iso, stage, room);
  if (!world->mesh.ok) {
    fail(out, world->mesh.why);
    return false;
  }
  seabed_of(stage, &world->mesh);
  world->has_names = disc::camera_names_of(*iso, stage, room, &world->names, &why);
  return true;
}

/* Signals (`{"signal":...}`) are read on the reader thread and never queued, so a stop reaches a
   running search. A stop records the question count when it arrived, rather than clearing a flag,
   so a stop sent while its search is still queued is not lost. */
std::atomic<long long> g_queued(0);
std::atomic<long long> g_stopped_at(0);
std::atomic<long long> g_answering(0);
/* `targetGround` is re-asked per keystroke; only the newest matters. */
std::atomic<long long> g_ground_answering(0);
/* The `targetGround` being answered when `{"signal":"ground"}` arrived; 0 for none. */
std::atomic<long long> g_ground_left(0);
/* Set when the window is gone; a stop that never clears. */
std::atomic<bool> g_closing(false);
/* At namespace scope so `closing` can wake an idle loop. */
std::mutex g_lock;
std::condition_variable g_woke;

void closing() {
  g_closing.store(true);
  {
    std::lock_guard<std::mutex> held(g_lock);
  }
  g_woke.notify_all();
  static std::atomic<bool> once(false);
  if (once.exchange(true)) return;
  // Forces an exit if shutdown hangs.
  std::thread([]() {
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::fflush(stdout);
    std::_Exit(0);
  }).detach();
}

/** A stop since this run's question was read. Always false when `answer` is called directly. */
bool stop_asked() {
  if (g_closing.load()) return true;
  const long long at = g_stopped_at.load();
  return at > 0 && at >= g_answering.load();
}

bool is_signal(const std::string& line, const char* name) {
  const Json in = parse(line);
  return in.ok() && in.at("signal").as_str() == name;
}

bool is_stop(const std::string& line) {
  const Json in = parse(line);
  return in.ok() && in.at("signal").as_str() == "stop";
}

/** A Logs line, written to the log file. A signal so it lands during a search; never answered. */
bool is_log(const std::string& line) {
  const Json in = parse(line);
  if (!in.ok() || in.at("signal").as_str() != "log") return false;
  settings::keep_log(in.at("text").as_str());
  return true;
}

/** Shared by the search and `target_ground` so the drawn goal and the answer agree. A field that
 *  is not a number (e.g. `null`) frees that axis. */
void target_of(const Json& request, search::Target* into) {
  const Json& target = request.at("target");
  if (target.at("shape").as_str() == "list") {
    // x, z pairs; a pair with a non-number is dropped whole.
    const Json& rows = target.at("rows");
    const std::vector<Json> none;
    const std::vector<Json>& it = rows.type() == Json::Type::Arr ? rows.items() : none;
    for (size_t i = 0; i + 1 < it.size(); i += 2) {
      if (it[i].type() != Json::Type::Num || it[i + 1].type() != Json::Type::Num) continue;
      const double x = coordinate(it[i]), z = coordinate(it[i + 1]);
      if (into->list.empty()) {
        into->x0 = into->x1 = x;
        into->z0 = into->z1 = z;
      }
      into->x0 = std::min(into->x0, x);
      into->x1 = std::max(into->x1, x);
      into->z0 = std::min(into->z0, z);
      into->z1 = std::max(into->z1, z);
      into->list.push_back(x);
      into->list.push_back(z);
    }
    into->ranged = true;
    return;
  }
  if (target.at("shape").as_str() == "range") {
    into->ranged = true;
    // A free axis, not a +-FLT_MAX span, which would make the grid one cell.
    into->has_x = target.at("x0").type() == Json::Type::Num &&
                  target.at("x1").type() == Json::Type::Num;
    into->has_z = target.at("z0").type() == Json::Type::Num &&
                  target.at("z1").type() == Json::Type::Num;
    const double ax = coordinate(target.at("x0")), bx = coordinate(target.at("x1"));
    const double az = coordinate(target.at("z0")), bz = coordinate(target.at("z1"));
    into->x0 = ax < bx ? ax : bx;
    into->x1 = ax < bx ? bx : ax;
    into->z0 = az < bz ? az : bz;
    into->z1 = az < bz ? bz : az;
    // Height: only an address target sends it.
    const Json& ay = target.at("y0");
    const Json& by = target.at("y1");
    if (ay.type() == Json::Type::Num && by.type() == Json::Type::Num) {
      into->has_y = true;
      const double lo = coordinate(ay), hi = coordinate(by);
      into->y0 = lo < hi ? lo : hi;
      into->y1 = lo < hi ? hi : lo;
    }
    // Narrows what counts as arriving, not the corridor (`Target::has_within`).
    const Json& within = target.at("within");
    if (within.type() == Json::Type::Obj) {
      const Json& a = within.at("x0");
      const Json& b = within.at("x1");
      const Json& c = within.at("z0");
      const Json& d = within.at("z1");
      if (a.type() == Json::Type::Num && b.type() == Json::Type::Num &&
          c.type() == Json::Type::Num && d.type() == Json::Type::Num) {
        const double wa = coordinate(a), wb = coordinate(b);
        const double wc = coordinate(c), wd = coordinate(d);
        into->has_within = true;
        into->wx0 = std::min(wa, wb);
        into->wx1 = std::max(wa, wb);
        into->wz0 = std::min(wc, wd);
        into->wz1 = std::max(wc, wd);
      }
    }
    /* The spans only bound the address; a blank high byte makes them exclude nothing, so the
       floor is cut to the mask. Four bytes an axis, high first; a non-number is blank. */
    const Json& mask = target.at("mask");
    if (mask.type() == Json::Type::Obj) {
      const char* axis[3] = {"x", "y", "z"};
      for (int a = 0; a < 3; ++a) {
        const Json& bytes = mask.at(axis[a]);
        if (bytes.type() != Json::Type::Arr) continue;
        const std::vector<Json>& it = bytes.items();
        for (size_t i = 0; i < it.size() && i < 4; ++i) {
          if (it[i].type() != Json::Type::Num) continue;
          const double v = it[i].as_num();
          if (v < 0 || v > 255) continue;
          into->mask.known[a][i] = true;
          into->mask.byte[a][i] = static_cast<uint8_t>(v);
        }
      }
    }
    return;
  }
  into->has_x = target.at("x").type() == Json::Type::Num;
  into->has_z = target.at("z").type() == Json::Type::Num;
  into->x = coordinate(target.at("x"));
  into->z = coordinate(target.at("z"));
}

/* Each side optional. Sides are named by screen position, so a pair is reordered here. */
void bounds_of(const Json& request, search::Question::Box* into) {
  const Json& box = request.at("bounds");
  const Json& x0 = box.at("xmin");
  const Json& x1 = box.at("xmax");
  const Json& z0 = box.at("zmin");
  const Json& z1 = box.at("zmax");
  const bool has_x = x0.type() == Json::Type::Num && x1.type() == Json::Type::Num;
  const bool has_z = z0.type() == Json::Type::Num && z1.type() == Json::Type::Num;
  if (x0.type() == Json::Type::Num) { into->has_xmin = true; into->xmin = coordinate(x0); }
  if (x1.type() == Json::Type::Num) { into->has_xmax = true; into->xmax = coordinate(x1); }
  if (z0.type() == Json::Type::Num) { into->has_zmin = true; into->zmin = coordinate(z0); }
  if (z1.type() == Json::Type::Num) { into->has_zmax = true; into->zmax = coordinate(z1); }
  if (has_x && into->xmin > into->xmax) std::swap(into->xmin, into->xmax);
  if (has_z && into->zmin > into->zmax) std::swap(into->zmin, into->zmax);
}

/* Narrows the corridor to the Bounds grown by one move width, and always keeps the start: a move
   can pass outside the Bounds, and the start may lie outside them. */
void bound_corridor(search::Corridor* c, const search::Question::Box& b, double sx, double sz) {
  const double w = c->half_width;
  if (b.has_xmin) c->min_x = std::max(c->min_x, std::min(b.xmin, sx) - w);
  if (b.has_xmax) c->max_x = std::min(c->max_x, std::max(b.xmax, sx) + w);
  if (b.has_zmin) c->min_z = std::max(c->min_z, std::min(b.zmin, sz) - w);
  if (b.has_zmax) c->max_z = std::min(c->max_z, std::max(b.zmax, sz) + w);
  if (c->max_x < c->min_x) c->max_x = c->min_x;
  if (c->max_z < c->min_z) c->max_z = c->min_z;
}

/* The target's valid ground as floor-flush polygons, through the same corridor, grid and
   `search::resolve` a search uses, so the drawing matches what a search accepts. */
void target_ground(const Out& out, const Json& request) {
  if (!link_ready(out, request)) return;
  World world;
  if (!world_of(out, request, &world)) return;
  const geom::Mesh& mesh = world.mesh;

  search::Question q;
  q.start_x = coordinate(request.at("start").at("x"));
  q.start_z = coordinate(request.at("start").at("z"));
  target_of(request, &q.target);
  q.tolerance = request.at("tol").as_num();
  q.aim = request.at("aim").as_str("player") == "overhead" ? search::Aim::Overhead
                                                          : search::Aim::Player;

  const search::BaseTable& base = base_table_once();
  const search::Corridor corridor =
      search::corridor(q.start_x, q.start_z, q.target, mesh.box, base.widest);
  const search::Selection selection = search::select(mesh, corridor);
  const search::Limits limits;
  const search::Grid grid = search::build(selection, corridor, limits);

  /* Not `tolerance + aim_reach`: `aim_point` already applies the aim offset, so adding it here
     would count it twice and admit landings that far off. */
  q.target.aim_lift = search::aim_lift(q);
  q.target.aim_reach = search::aim_reach(q);
  const std::function<bool()> asked_again = []() {
    const long long left = g_ground_left.load();
    return g_closing.load() || (left > 0 && left >= g_ground_answering.load());
  };
  if (!search::resolve(&q.target, grid, selection.ground, q.tolerance, &asked_again,
                       &selection.ground_source)) {
    if (asked_again()) {
      fail(out, "superseded", "superseded");
      return;
    }
    fail(out, "the target has no ground in it", "noTargetGround");
    return;
  }

  // `verts` is x, y, z per vertex; `sides` is each polygon's vertex count (3 to 9).
  const search::Region& kept = q.target.ground;
  Json body = Json::obj();
  body.set("kept", Json::num(static_cast<double>(kept.kept)));
  Json sides = Json::arr();
  std::vector<float> verts;
  verts.reserve(kept.vert.size());
  for (int p = 0; p < kept.kept; ++p) {
    const int from = kept.at[static_cast<size_t>(p)], to = kept.at[static_cast<size_t>(p) + 1];
    sides.push(Json::num(static_cast<double>(to - from)));
    for (int i = from * 3; i < to * 3; ++i) {
      verts.push_back(static_cast<float>(kept.vert[static_cast<size_t>(i)]));
    }
  }
  body.set("sides", std::move(sides));
  body.set("verts", Json::raw(floats(verts)));
  data(out, "targetGround", std::move(body));
  done(out);
}

void search_run(const Out& out, const Json& request) {
  if (!link_ready(out, request)) return;
  World world;
  if (!world_of(out, request, &world)) return;
  const tww_engine::RoomDzb& dzb = world.dzb;
  const geom::Mesh& mesh = world.mesh;

  // A missing ceiling is zero, not a default.
  search::Question q;
  q.start_x = coordinate(request.at("start").at("x"));
  q.start_z = coordinate(request.at("start").at("z"));
  q.start_facing = static_cast<int>(request.at("start").at("f").as_num());
  // The stick is camera-relative. Missing means no camera, not a camera at zero.
  const Json& cam = request.at("start").at("cam");
  q.has_camera = cam.type() == Json::Type::Num;
  if (q.has_camera) q.start_camera = static_cast<int>(cam.as_num()) & 0xFFFF;
  target_of(request, &q.target);
  // End facing; absent means any.
  const Json& facing = request.at("facing");
  if (facing.at("a").type() == Json::Type::Num) {
    q.end_facing.any = false;
    q.end_facing.a = static_cast<int>(facing.at("a").as_num());
    const Json& to = facing.at("b");
    q.end_facing.span = to.type() == Json::Type::Num
        ? ((static_cast<int>(to.as_num()) - q.end_facing.a) % 65536 + 65536) % 65536
        : 0;
  }
  q.tolerance = request.at("tol").as_num();
  q.frames = static_cast<int>(request.at("frames").as_num());
  q.least_frames = static_cast<int>(request.at("leastFrames").as_num(0));
  q.steps = static_cast<int>(request.at("steps").as_num());
  q.fewest = static_cast<int>(request.at("fewest").as_num(0));
  q.greedy = request.at("greedy").as_bool(false);
  q.check_range = request.at("checkRange").as_num(4.0);
  // Thread count changes run time only, never the results.
  q.cores = static_cast<int>(request.at("cores").as_num(1));
  // Shortlist ceiling: a share of physical memory, 25% unless asked; 0 (unknown) means none.
  {
    double share = request.at("memoryUsage").as_num(25.0);
    if (!(share >= 5.0)) share = 5.0;
    if (share > 90.0) share = 90.0;
    q.memory = static_cast<long long>(static_cast<double>(machine::physical_memory()) * share / 100.0);
  }
  q.aim = request.at("aim").as_str("player") == "overhead" ? search::Aim::Overhead
                                                          : search::Aim::Player;
  bounds_of(request, &q.bounds);
  {
    const std::string said = request.at("collision").as_str("solid");
    q.collision = said == "none" ? search::Collision::None
                : said == "floors" ? search::Collision::Floors
                                   : search::Collision::Solid;
  }
  // Only retimed moves carry a cost; the rest keep the measured one.
  std::map<std::string, int> costs;
  const Json& priced_at = request.at("costs");
  for (const Json& one : request.at("moves").items()) {
    const std::string id = one.as_str();
    if (id.empty()) continue;
    q.moves.push_back(id);
    const Json& cost = priced_at.at(id);
    if (cost.type() == Json::Type::Num) costs[id] = static_cast<int>(cost.as_num());
  }

  const search::BaseTable base = search::priced(base_table_once(), costs);

  search::Corridor corridor =
      search::corridor(q.start_x, q.start_z, q.target, mesh.box, base.widest);
  bound_corridor(&corridor, q.bounds, q.start_x, q.start_z);
  const search::Selection selection = search::select(mesh, corridor);
  const search::Limits limits;
  const search::Grid grid = search::build(selection, corridor, limits);

  // A sent height is the game's own and is kept; otherwise the highest floor under the start.
  const Json& given = request.at("start").at("y");
  if (given.type() == Json::Type::Num) {
    q.start_y = coordinate(given);
  } else {
    const double probe = static_cast<double>(mesh.box[4]);
    const search::Floor under = search::floor_at(grid, selection, q.start_x, q.start_z, probe);
    q.start_y = static_cast<double>(static_cast<float>(under.found ? under.y : probe));
  }

  // Must match `target_ground`. No ground in the target refuses the run.
  q.target.aim_lift = search::aim_lift(q);
  q.target.aim_reach = search::aim_reach(q);
  if (!search::resolve(&q.target, grid, selection.ground, q.tolerance, nullptr,
                       &selection.ground_source)) {
    fail(out, "the target has no ground in it", "noTargetGround");
    return;
  }

  /* The camera-clearance field, cached on disk per room, built only when a chosen move runs a
     camera, and regardless of the collision setting. */
  const std::string camera_cache = search::cam_cache_path(settings::cache_dir(), dzb);
  bool camera_moves = false;
  /* Off by default: with it off, no camera move is refused for the room. */
  if (request.at("cameraChecks").as_bool(false)) {
    for (const std::string& id : q.moves) {
      camera_moves = camera_moves || search::runs_camera(search::seat_of(id));
    }
  }
  search::CamField camera_field;
  if (camera_moves) {
    const search::CamRoom cameras =
        search::cam_room(dzb, world.has_names ? &world.names : nullptr, camera_cache);
    const std::vector<uint64_t> places = search::cam_places(
        selection, corridor, q.collision != search::Collision::None, q.start_y);
    const std::function<bool()> stopping = [] { return stop_asked(); };
    camera_field =
        search::cam_field(dzb, cameras, places, camera_cache, q.cores, nullptr, &stopping);
    q.camera_clear = &camera_field;
  }

  const search::Calibration cal = search::Calibration::measured();
  /* At most `kPlansSent` plans: reaching ones in walk order, then the closest misses. Stops carry
     the engine's positions. Consults the engine could not finish are only counted. */
  const size_t kPlansSent = 20;
  struct Shaped {
    static Json plan(const search::Consult& one, const search::Question& q,
                     const search::BaseTable& base) {
      Json out = Json::obj();
      out.set("frames", Json::num(one.frames));
      // The aim point, not Link's position when aimed overhead.
      out.set("x", Json::f32(one.aim_x));
      out.set("z", Json::f32(one.aim_z));
      out.set("d", Json::num(one.distance));
      out.set("reached", Json::boolean(one.outcome == search::Outcome::Confirmed));
      // Float steps to the nearest allowed place; a freed axis contributes nothing.
      double nx = 0, nz = 0;
      q.target.nearest(one.aim_x, one.aim_z, &nx, &nz);
      const long long sx = float_steps(one.aim_x, nx), sz = float_steps(one.aim_z, nz);
      // An address miss has no per-axis split.
      int run_at = 0, run_len = 0;
      if (q.target.mask.run(&run_at, &run_len)) {
        out.set("off", Json::num(q.target.miss(one.aim_x, one.aim_y, one.aim_z)));
      } else {
        out.set("off", Json::num(static_cast<double>(sx + sz)));
        // Signed per axis: negative falls short.
        out.set("ox", Json::num(static_cast<double>(one.aim_x < nx ? -sx : sx)));
        out.set("oz", Json::num(static_cast<double>(one.aim_z < nz ? -sz : sz)));
      }
      Json stops = Json::arr();
      int total = 0;
      for (size_t s = 0; s < one.stop.size() && s < one.path.size(); ++s) {
        const search::Reached& at = one.stop[s];
        total += at.frames;
        Json stop = Json::obj();
        stop.set("id", Json::str(base.move[static_cast<size_t>(one.path[s].row)].id));
        stop.set("steps", Json::num(one.path[s].steps));
        stop.set("taps", Json::num(one.path[s].taps));
        stop.set("frames", Json::num(at.frames));
        stop.set("total", Json::num(total));
        stop.set("x", Json::f32(at.x));
        stop.set("z", Json::f32(at.z));
        stop.set("y", Json::f32(at.y));
        stop.set("f", Json::num(at.facing));
        // `Reached::camera` is deliberately not sent; nothing reads it.
        stops.push(std::move(stop));
      }
      out.set("stops", std::move(stops));
      return out;
    }

    /* Misses come from refuted consults and from `Verified::closest`, which fills the table when
       nothing got inside the tolerance. */
    static void take(const std::vector<search::Consult>& from, std::vector<size_t>* reach,
                     std::vector<size_t>* miss, size_t most, size_t base_index) {
      for (size_t i = 0; i < from.size(); ++i) {
        const search::Consult& one = from[i];
        if (one.outcome == search::Outcome::CouldNotRun) continue;
        if (one.outcome == search::Outcome::Confirmed) {
          if (reach->size() < most) reach->push_back(base_index + i);
          continue;
        }
        miss->push_back(base_index + i);
      }
    }

    static Json plans(const search::Verified& said, const search::Question& q,
                      const search::BaseTable& base, size_t most) {
      std::vector<search::Consult> all = said.consult;
      all.insert(all.end(), said.closest.begin(), said.closest.end());
      std::vector<size_t> reach, miss;
      take(said.consult, &reach, &miss, most, 0);
      take(said.closest, &reach, &miss, most, said.consult.size());
      /* Ranked by float steps, the number shown, with distance only as a tie-break: the two do
         not rank alike. */
      const size_t room = most > reach.size() ? most - reach.size() : 0;
      const size_t keep = room < miss.size() ? room : miss.size();
      if (keep > 0) {
        struct Closer {
          const std::vector<search::Consult>* all;
          const search::Question* q;
          bool operator()(size_t a, size_t b) const {
            const search::Consult& ca = (*all)[a];
            const search::Consult& cb = (*all)[b];
            const double oa = q->target.miss(ca.aim_x, ca.aim_y, ca.aim_z);
            const double ob = q->target.miss(cb.aim_x, cb.aim_y, cb.aim_z);
            if (oa != ob) return oa < ob;
            return ca.distance != cb.distance ? ca.distance < cb.distance : a < b;
          }
        };
        Closer closer;
        closer.all = &all;
        closer.q = &q;
        std::partial_sort(miss.begin(),
                          miss.begin() + static_cast<std::ptrdiff_t>(keep), miss.end(), closer);
      }
      Json out = Json::arr();
      for (size_t i = 0; i < reach.size(); ++i) out.push(plan(all[reach[i]], q, base));
      for (size_t i = 0; i < keep; ++i) out.push(plan(all[miss[i]], q, base));
      return out;
    }
  };

  /* Progress: the walk fills `pct` to 99, since its candidates are driven as they are found and the
     pass after it has almost nothing left; 100 is the finished line. A line is written at most
     every `kTellMs`. */
  const int kTellMs = 80;
  struct Telling : search::Watching, search::Consulting {
    const Out* out;
    const search::Question* q;
    const search::BaseTable* base;
    const tww_engine::RoomDzb* room;
    size_t most = 20;
    std::chrono::steady_clock::time_point began;
    std::chrono::steady_clock::time_point last;
    std::chrono::steady_clock::time_point last_table;
    int every_ms = 80;
    int table_ms = 2000;
    /* The last table sent; every line is a full snapshot, so bar lines repeat it. */
    std::string table = "[]";

    /* Every driven plan, keyed on path, so the table is the best by the engine's verdict and only
       improves (the model's ranking can disagree). Unfinished consults are not kept. */
    std::map<std::vector<search::Edge>, search::Consult> ever;
    typedef std::tuple<double, double, double, double, double, int> Landing;
    static Landing landing_of(const search::Consult& c) {
      return Landing(c.x, c.y, c.z, c.aim_x, c.aim_z, c.facing & 0xFFFF);
    }
    std::map<Landing, std::vector<search::Edge>> landed;
    size_t keep_ever = 400;

    void remember(const std::vector<search::Consult>& from) {
      for (size_t i = 0; i < from.size(); ++i) {
        if (from[i].outcome == search::Outcome::CouldNotRun) continue;
        const std::vector<search::Edge>& key = from[i].path;
        if (ever.find(key) != ever.end()) continue;
        // One plan per landing: fewer frames wins, then the lesser path.
        const Landing at = landing_of(from[i]);
        std::map<Landing, std::vector<search::Edge>>::iterator held = landed.find(at);
        if (held != landed.end()) {
          const search::Consult& was = ever[held->second];
          if (was.frames < from[i].frames ||
              (was.frames == from[i].frames && held->second < key)) {
            continue;
          }
          ever.erase(held->second);
          held->second = key;
        } else {
          landed[at] = key;
        }
        ever[key] = from[i];
      }
      if (ever.size() <= keep_ever) return;
      // Over the cap: drop the furthest misses, in the same order `plans` ranks them.
      std::vector<std::pair<std::pair<double, double>, const std::vector<search::Edge>*> >
          by_distance;
      for (std::map<std::vector<search::Edge>, search::Consult>::const_iterator it = ever.begin();
           it != ever.end(); ++it) {
        if (it->second.outcome == search::Outcome::Confirmed) continue;
        const double off =
            q->target.miss(it->second.aim_x, it->second.aim_y, it->second.aim_z);
        by_distance.push_back(
            std::make_pair(std::make_pair(off, it->second.distance), &it->first));
      }
      std::sort(by_distance.begin(), by_distance.end());
      size_t over = ever.size() - keep_ever;
      for (size_t i = by_distance.size(); i > 0 && over > 0; --i, --over) {
        landed.erase(landing_of(ever[*by_distance[i - 1].second]));
        ever.erase(*by_distance[i - 1].second);
      }
    }

    /** Takes `books` itself; callers are on the reporting thread while walkers write. */
    std::string driven() {
      std::lock_guard<std::mutex> lock(books);
      search::Verified all;
      // Path order makes ties independent of walker timing.
      std::vector<search::Consult> driven_so_far;
      for (std::map<std::vector<search::Edge>, search::Consult>::const_iterator it = ever.begin();
           it != ever.end(); ++it) {
        driven_so_far.push_back(it->second);
      }
      all.consult = search::one_per_landing(driven_so_far);
      return Shaped::plans(all, *q, *base, most).dump();
    }

    bool due() {
      const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count() < every_ms) {
        return false;
      }
      last = now;
      return true;
    }

    bool table_due() {
      const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_table).count() <
          table_ms) {
        return false;
      }
      last_table = now;
      return true;
    }

    double rate(long long states) const {
      const double seconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
      return seconds > 0.0 ? static_cast<double>(states) / seconds
                           : static_cast<double>(states);
    }

    long long shown_states = 0;
    double shown_rate = 0;

    /** The walk's state total, frozen during verification so `searched` always counts states. */
    long long walked_states = 0;

    /** So a stopped run's last line reports where it got to, not 100. */
    int shown_pct = 0;

    std::atomic<long long> verified{0};
    std::atomic<long long> engine_ns{0};
    void engine_spent(std::chrono::steady_clock::time_point from, long long plans) {
      engine_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::steady_clock::now() - from).count();
      verified += plans;
    }

    void say(Json plans, int pct) {
      shown_pct = pct;
      Json body = Json::obj();
      body.set("plans", std::move(plans));
      body.set("pct", Json::num(pct));
      body.set("searched", Json::num(static_cast<double>(shown_states)));
      body.set("rate", Json::num(shown_rate));
      body.set("verified", Json::num(shown_verified));
      body.set("engine", Json::num(shown_engine));
      // Absent when there is no memory ceiling.
      if (q->memory > 0) body.set("memory", Json::num(shown_memory));
      data(*out, "plans", std::move(body));
    }

    /* Percent of `Question::memory`; 100 means the run is no longer exhaustive. */
    double shown_memory = 0;
    /* Exactly 100 only once a plan was refused, since held bytes never quite reach the ceiling. */
    void held(long long bytes, bool full) {
      shown_memory = q->memory > 0
          ? 100.0 * static_cast<double>(bytes) / static_cast<double>(q->memory) : 0.0;
      if (full) shown_memory = 100.0;
      else if (shown_memory > 99.9) shown_memory = 99.9;
    }

    double shown_verified = 0;
    double shown_engine = 0;

    void figures(long long states) {
      shown_states = states;
      shown_rate = rate(states);
      const double seconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
      const int walkers = q->cores > 0 ? q->cores : 1;
      shown_verified = seconds > 0.0 ? static_cast<double>(verified) / seconds : 0.0;
      shown_engine = seconds > 0.0
          ? 100.0 * static_cast<double>(engine_ns) * 1e-9 / (seconds * walkers) : 0.0;
    }

    /** `through` is in millionths of the tree. */
    bool walked(const search::Counters& count, int through, size_t candidates) {
      (void)candidates;
      held(count.held_bytes, count.memory_full);
      // Before the clock, so a stop does not wait for the next report.
      if (stop_asked()) return false;
      if (!due()) return true;
      const int part = through < 0 ? 0 : (through > 1000000 ? 1000000 : through);
      walked_pct = std::min(99, part / 10000);
      say(Json::raw(table), walked_pct);
      return true;
    }

    /* The table is refreshed during the walk, verified and shaped exactly as the final one. */
    int walked_pct = 0;
    bool wants_so_far() { return table_due(); }

    /* Plans already driven, including unfinished ones `ever` drops, so none is re-driven every
       report. Fixed-size and direct-mapped: a collision only costs a repeat engine run. */
    struct Tried {
      enum : size_t { kSlots = size_t(1) << 18 };
      struct Slot {
        std::vector<search::Edge> path;
        bool taken = false;
      };
      std::vector<Slot> slot = std::vector<Slot>(kSlots);

      static size_t digest(const std::vector<search::Edge>& path) {
        uint64_t h = 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(path.size());
        for (size_t i = 0; i < path.size(); ++i) {
          const long long field[3] = {path[i].row, path[i].steps, path[i].taps};
          for (int f = 0; f < 3; ++f) {
            uint64_t v = static_cast<uint64_t>(field[f]) + 0x9E3779B97F4A7C15ull;
            v ^= v >> 30;
            v *= 0xBF58476D1CE4E5B9ull;
            v ^= v >> 27;
            v *= 0x94D049BB133111EBull;
            v ^= v >> 31;
            h ^= v;
            h *= 0x100000001B3ull;
          }
        }
        return static_cast<size_t>(h);
      }

      /** True when the slot did not hold this plan; it does afterwards. */
      bool insert(const std::vector<search::Edge>& path) {
        Slot& at = slot[digest(path) & (kSlots - 1)];
        if (at.taken && at.path == path) return false;
        at.path = path;
        at.taken = true;
        return true;
      }
    };
    Tried tried;

    /* Guards `tried`, `spent` and `ever` across walkers and the reporting thread. `search::verify`
       runs outside it; the engine keeps one player per thread and needs no lock. */
    std::mutex books;

    /** Order matters: two orders of the same moves can land differently. */
    bool fresh_path(const std::vector<search::Edge>& path) {
      return tried.insert(path);
    }

    /* Candidates are driven in `found`, so the walk need not keep an unbounded list. */
    bool takes_every_candidate() { return true; }

    search::VerifyCounters spent;

    void spend(const search::VerifyCounters& c) {
      spent.consults += c.consults;
      spent.confirmed += c.confirmed;
      spent.refuted += c.refuted;
      spent.could_not_run += c.could_not_run;
      spent.discarded += c.discarded;
      spent.engine_frames += c.engine_frames;
      spent.closest_consults += c.closest_consults;
    }

    void found(const search::Candidate& one) {
      if (stop_asked()) return;
      {
        // Test and claim together, so two walkers cannot both drive it.
        std::lock_guard<std::mutex> lock(books);
        if (!fresh_path(one.path)) return;
      }
      search::Found just;
      just.candidate.push_back(one);
      const std::chrono::steady_clock::time_point from = std::chrono::steady_clock::now();
      const search::Verified said = search::verify(just, *q, *base, room);
      engine_spent(from, 1);
      std::lock_guard<std::mutex> lock(books);
      spend(said.count);
      remember(said.consult);
    }

    /* On the table's clock, drives the walk's current closest set (at most twenty). The shortlist
       is not ranked here, so a report's cost does not grow with the check range. */
    void so_far(const search::Found& run) {
      search::Found fresh;
      {
        std::lock_guard<std::mutex> lock(books);
        for (size_t i = 0; i < run.closest.size(); ++i) {
          if (!fresh_path(run.closest[i].path)) continue;
          fresh.closest.push_back(run.closest[i]);
        }
      }
      if (!fresh.closest.empty()) {
        const std::chrono::steady_clock::time_point from = std::chrono::steady_clock::now();
        const search::Verified said = search::verify(fresh, *q, *base, room);
        engine_spent(from, static_cast<long long>(fresh.closest.size()));
        std::lock_guard<std::mutex> lock(books);
        spend(said.count);
        remember(said.closest);
      }
      table = driven();
      walked_states = run.count.generated;
      held(run.count.held_bytes, run.count.memory_full);
      figures(walked_states);
      say(Json::raw(table), walked_pct);
    }

    bool consulted(const search::Verified& sofar, size_t of) {
      if (stop_asked()) return false;
      const bool last_one = static_cast<size_t>(sofar.count.consults) +
                                static_cast<size_t>(sofar.count.discarded) >= of;
      if (!last_one && !due()) return true;
      {
        std::lock_guard<std::mutex> lock(books);
        remember(sofar.consult);
        remember(sofar.closest);
      }
      table = driven();
      figures(walked_states);
      say(Json::raw(table), walked_pct);
      return true;
    }
  };

  Telling telling;
  telling.out = &out;
  telling.q = &q;
  telling.base = &base;
  telling.room = &dzb;
  telling.began = std::chrono::steady_clock::now();
  telling.last = telling.began;
  telling.last_table = telling.began;
  telling.every_ms = kTellMs;
  telling.most = kPlansSent;

  const search::Found found =
      search::search_tree(q, base, grid, selection, cal, nullptr, &telling);
  // For a walk that ended before its first table report.
  telling.walked_states = found.count.generated;
  // The last pass drives only plans no report has driven yet.
  search::Found rest;
  rest.quanta = found.quanta;
  rest.count = found.count;
  rest.rate = found.rate;
  rest.slack = found.slack;
  rest.exhausted = found.exhausted;
  rest.unstepped = found.unstepped;
  for (size_t i = 0; i < found.candidate.size(); ++i) {
    if (!telling.fresh_path(found.candidate[i].path)) continue;
    rest.candidate.push_back(found.candidate[i]);
  }
  for (size_t i = 0; i < found.closest.size(); ++i) {
    if (!telling.fresh_path(found.closest[i].path)) continue;
    rest.closest.push_back(found.closest[i]);
  }
  const std::chrono::steady_clock::time_point last_from = std::chrono::steady_clock::now();
  const search::Verified said = search::verify(rest, q, base, &dzb, &telling);
  telling.engine_spent(last_from,
                       static_cast<long long>(rest.candidate.size() + rest.closest.size()));
  if (q.camera_clear != nullptr) search::cam_keep(camera_field, camera_cache);
  const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - telling.began).count();

  // The run's accumulated table, not a reshaping of the final walk's closest set.
  telling.spend(said.count);
  telling.remember(said.consult);
  telling.remember(said.closest);
  Json body = Json::obj();
  body.set("plans", Json::raw(telling.driven()));
  body.set("pct", Json::num(stop_asked() ? telling.shown_pct : 100));
  body.set("searched", Json::num(static_cast<double>(found.count.generated)));
  body.set("rate", Json::num(seconds > 0.0
      ? static_cast<double>(found.count.generated) / seconds
      : static_cast<double>(found.count.generated)));
  {
    const int walkers = q.cores > 0 ? q.cores : 1;
    body.set("verified", Json::num(seconds > 0.0
        ? static_cast<double>(telling.verified) / seconds : 0.0));
    body.set("engine", Json::num(seconds > 0.0
        ? 100.0 * static_cast<double>(telling.engine_ns) * 1e-9 / (seconds * walkers) : 0.0));
  }
  telling.held(found.count.held_bytes, found.count.memory_full);
  if (q.memory > 0) body.set("memory", Json::num(telling.shown_memory));

  /* Shown verbatim in the Logs, so every key here is user-visible text and needs review. */
  Json counted = Json::obj();
  counted.set("generated", Json::num(static_cast<double>(found.count.generated)));
  counted.set("expanded", Json::num(static_cast<double>(found.count.expanded)));
  counted.set("pruned", Json::num(static_cast<double>(found.count.bound_pruned)));
  counted.set("dominated", Json::num(static_cast<double>(found.count.dominance_kills)));
  counted.set("collapsed", Json::num(static_cast<double>(found.count.key_collapses)));
  counted.set("leftCorridor", Json::num(static_cast<double>(found.count.left_corridor)));
  // `Counters::outside_bounds` is not reported until its key is approved.
  counted.set("allowed", Json::num(static_cast<double>(found.count.allowed)));
  counted.set("exhausted", Json::boolean(found.exhausted));
  counted.set("consults", Json::num(static_cast<double>(telling.spent.consults)));
  counted.set("confirmed", Json::num(static_cast<double>(telling.spent.confirmed)));
  counted.set("refuted", Json::num(static_cast<double>(telling.spent.refuted)));
  counted.set("couldNotRun", Json::num(static_cast<double>(telling.spent.could_not_run)));
  counted.set("discarded", Json::num(static_cast<double>(telling.spent.discarded)));
  // Frames from hand-set prices (only the C up turn has one).
  long long stated = 0, with_stated = 0;
  for (size_t i = 0; i < said.consult.size(); ++i) {
    if (said.consult[i].stated_frames <= 0) continue;
    stated += said.consult[i].stated_frames;
    ++with_stated;
  }
  counted.set("statedFrames", Json::num(static_cast<double>(stated)));
  counted.set("plansWithAStatedPrice", Json::num(static_cast<double>(with_stated)));
  counted.set("cells", Json::num(static_cast<double>(grid.start.empty() ? 0
                                                     : grid.start.size() - 1)));
  counted.set("triangles", Json::num(static_cast<double>(selection.ground_held)));
  Json unstepped = Json::arr();
  for (size_t i = 0; i < found.unstepped.size(); ++i) {
    unstepped.push(Json::str(found.unstepped[i]));
  }
  counted.set("unstepped", std::move(unstepped));
  body.set("counted", std::move(counted));

  data(out, "plans", std::move(body));
  done(out);
}

void answer(const std::string& request, const Out& out) {
  const Json in = parse(request);
  if (!in.ok()) {
    fail(out, "the core could not read that request: " + in.why());
    return;
  }
  const std::string ask = in.at("ask").as_str();
  if (ask == "machine") {
    machine(out);
  } else if (ask == "emulators") {
    emulators(out);
  } else if (ask == "start") {
    start(out, in);
  } else if (ask == "rooms") {
    if (in.at("pid").type() == Json::Type::Num) {
      rooms(out, in);
    } else {
      rooms_on_disc(out, in);
    }
  } else if (ask == "room") {
    if (in.at("pid").type() == Json::Type::Num) {
      room(out, in);
    } else {
      room_on_disc(out, in);
    }
  } else if (ask == "settings") {
    settings_read(out);
  } else if (ask == "remember") {
    settings_write(out, in);
  } else if (ask == "targetGround") {
    target_ground(out, in);
  } else if (ask == "actors") {
    fail(out, "the core cannot read actors yet");
  } else if (ask == "search") {
    search_run(out, in);
  } else if (ask.empty()) {
    fail(out, "the core could not read that request: it names no question");
  } else {
    fail(out, "the core does not know the question `" + ask + "`");
  }
}

int run() {
  const Out out = [](const std::string& line) {
    std::fwrite(line.data(), 1, line.size(), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);
  };
  std::mutex& lock = g_lock;
  std::condition_variable& woke = g_woke;
  std::deque<std::string> queue;
  bool closed = false;

  std::thread reader([&lock, &woke, &queue, &closed]() {
    std::string line;
    while (std::getline(std::cin, line)) {
      while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
      if (line.empty()) continue;
      if (is_stop(line)) {
        g_stopped_at.store(g_queued.load());
        continue;
      }
      if (is_log(line)) continue;
      if (is_signal(line, "ground")) {
        g_ground_left.store(g_ground_answering.load());
        continue;
      }
      g_queued.fetch_add(1);
      {
        std::lock_guard<std::mutex> held(lock);
        queue.push_back(line);
      }
      woke.notify_one();
    }
    closing();
    {
      std::lock_guard<std::mutex> held(lock);
      closed = true;
    }
    woke.notify_one();
  });

  for (;;) {
    std::string line;
    {
      std::unique_lock<std::mutex> held(lock);
      woke.wait(held, [&queue, &closed]() {
        return !queue.empty() || closed || g_closing.load();
      });
      // Queued questions are still answered after close.
      if (queue.empty()) break;
      line = queue.front();
      queue.pop_front();
    }
    g_answering.fetch_add(1);
    if (line.find("\"targetGround\"") != std::string::npos) g_ground_answering.fetch_add(1);
    answer(line, out);
    // Cleared so a stop does not carry over to questions queued behind.
    g_stopped_at.store(0);
  }
  // The parent may be gone with the pipe still open, so the reader may never return.
  if (g_closing.load()) {
    reader.detach();
  } else {
    reader.join();
  }
  return 0;
}

}  // namespace seam
