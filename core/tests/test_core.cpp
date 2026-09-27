/* Core suite: everything checkable with no game running. */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#include "../src/disc/link.h"
#include "../src/disc/disc.h"
#include "../src/disc/rarc.h"
#include "../src/disc/yaz0.h"
#include "../src/dolphin/collision.h"
#include "../src/dolphin/dolphin.h"
#include "../src/dolphin/game.h"
#include "../src/geom/mesh.h"
#include "../src/seam/json.h"
#include "../src/seam/seam.h"
#include "../src/settings/settings.h"

namespace {

int failed = 0;

void ok(bool cond, const std::string& what) {
  std::printf("%s %s\n", cond ? "ok  " : "FAIL", what.c_str());
  if (!cond) ++failed;
}

/** Every line the seam emits for one request. */
std::vector<std::string> answer(const std::string& request) {
  std::vector<std::string> lines;
  seam::answer(request, [&lines](const std::string& line) { lines.push_back(line); });
  return lines;
}

std::string line_of(const std::string& line) {
  return seam::parse(line).at("line").as_str();
}

void json_tests() {
  using seam::Json;

  ok(seam::parse(R"({"ask":"emulators"})").at("ask").as_str() == "emulators",
     "json: a flat request reads back");

  const seam::Json search = seam::parse(
      R"({"ask":"search","start":{"x":-93.4267578125,"z":522.08447265625,"f":49152},)"
      R"("target":{"x":-42.5,"z":668.75},"steps":3,"frames":900})");
  ok(search.ok() && search.at("start").at("x").as_num() == -93.4267578125 &&
         search.at("target").at("z").as_num() == 668.75 && search.at("steps").as_num() == 3,
     "json: a nested search request reads back, every field");

  const double exact = -93.4267578125;
  const std::string dumped = Json::num(exact).dump();
  ok(dumped == "-93.4267578125", "json: a float dumps shortest and exact (" + dumped + ")");
  ok(seam::parse(dumped).as_num() == exact, "json: and reads back bit-for-bit");

  const seam::Json flat = seam::parse(R"({"ask":"start"})");
  ok(flat.ok() && !flat.at("pid").ok(), "json: a field that is not there is `Bad`, not 0");

  const std::string quoted = Json::str("a \"name\" with\na newline").dump();
  ok(quoted.find('\n') == std::string::npos && seam::parse(quoted).as_str().find('\n') != std::string::npos,
     "json: a string escapes and reads back whole");

  const char* junk[] = {"", "{", "}", "[1,", R"({"ask")", R"({"ask":})", "nul",
                        R"({"a":1} {"b":2})", "\xff\xfe", R"({"ask":"x","n":--1})"};
  bool all_bad = true;
  for (const char* j : junk) {
    if (seam::parse(j).ok()) {
      std::printf("     survived: %s\n", j);
      all_bad = false;
    }
  }
  ok(all_bad, "json: MUTATION - every malformed line comes back `Bad` rather than a value");
}

void envelope_tests() {
  const std::vector<std::string> requests = {
      R"({"ask":"emulators"})", R"({"ask":"start","pid":1})", R"({"ask":"start"})",
      R"({"ask":"rooms"})",     R"({"ask":"room","stage":"Kaisen","room":0})",
      R"({"ask":"actors","stage":"Kaisen","room":0})",
      R"({"ask":"search","start":{"x":0,"z":0,"f":0},"target":{"x":1,"z":1},"steps":1,"frames":1})",
      R"({"ask":"nonsense"})",  R"({"nothing":true})", "{",
  };
  bool every_ends = true, every_line_legal = true;
  for (const std::string& r : requests) {
    const std::vector<std::string> lines = answer(r);
    const bool ends = !lines.empty() &&
                      (line_of(lines.back()) == "done" || line_of(lines.back()) == "fail");
    if (!ends) { std::printf("     no ending: %s\n", r.c_str()); every_ends = false; }
    for (size_t i = 0; i < lines.size(); ++i) {
      const seam::Json line = seam::parse(lines[i]);
      const std::string k = line.at("line").as_str();
      const bool legal = line.ok() && (k == "data" || k == "done" || k == "fail") &&
                         (k != "data" || (line.at("of").type() == seam::Json::Type::Str &&
                                          line.has("body"))) &&
                         (k != "fail" || !line.at("why").as_str().empty()) &&
                         (i + 1 == lines.size() || k == "data");
      if (!legal) {
        std::printf("     illegal line for %s: %s\n", r.c_str(), lines[i].c_str());
        every_line_legal = false;
      }
    }
  }
  ok(every_ends, "envelope: every request ends in exactly one `done` or `fail`");
  ok(every_line_legal, "envelope: every line is `data` with an `of` and a `body`, or the ending");

  const std::vector<std::string> searched = answer(
      R"({"ask":"search","start":{"x":0,"z":0,"f":0},"target":{"x":1,"z":1},"steps":1,"frames":1})");
  ok(searched.size() == 1 && line_of(searched[0]) == "fail",
     "envelope: `search` refuses, in one line, and emits no plans");

  const std::vector<std::string> unknown = answer(R"({"ask":"nonsense"})");
  ok(unknown.size() == 1 && line_of(unknown[0]) == "fail" &&
         seam::parse(unknown[0]).at("why").as_str().find("nonsense") != std::string::npos,
     "envelope: an unknown question comes back named");

  ok(dolphin::kCanRead == (std::string(dolphin::why_not()).empty()),
     "envelope: a build that cannot read has a line for the Logs, and one that can has none");
}

void bounds_tests() {
  ok(dolphin::kMem1Start == 0x80000000u && dolphin::kMem1Size == 0x02000000u,
     "attach: MEM1 is 32 MiB at 0x80000000");
  ok(dolphin::kDiscMagic == 0xC2339F3Du, "attach: the disc magic is the one at GC 0x8000001C");
  const std::vector<dolphin::Found> found = dolphin::running();
  bool shaped = true;
  for (const dolphin::Found& f : found) {
    if (f.pid <= 0 || f.exe.empty()) shaped = false;
  }
  ok(shaped, "attach: enumerating answers with or without a Dolphin (" +
                 std::to_string(found.size()) + " found)");

  ok(dolphin::name_of("GZLJ01") == "The Wind Waker" &&
         dolphin::name_of("GZLE01") == "The Wind Waker",
     "attach: a known disc is offered by its name");
  /* In memory GC 0x80000020 holds the boot magic 0x0D15EA5E, not the disc title. */
  ok(dolphin::name_of("\x0d\x15\xea^") == "\x0d\x15\xea^" && dolphin::name_of("") == "" &&
         dolphin::name_of("GALE01") == "GALE01",
     "attach: MUTATION - an unmet disc comes back as its id, never as a guess");

  const std::vector<std::string> said = answer(R"({"ask":"machine"})");
  const seam::Json body = seam::parse(said.at(0)).at("body");
  ok(said.size() == 2 && body.at("reads").type() == seam::Json::Type::Bool &&
         !body.has("why"),
     "attach: `machine` answers one boolean and carries no reason");

  bool wordless = true;
  for (const char* q : {R"({"ask":"machine"})", R"({"ask":"emulators"})"}) {
    for (const std::string& line : answer(q)) {
      if (seam::parse(line).at("line").as_str() != "data") continue;
      size_t spaces = 0;
      for (char c : line) spaces += (c == ' ');
      if (spaces > 6) {
        std::printf("     prose in a data line: %s\n", line.c_str());
        wordless = false;
      }
    }
  }
  ok(wordless, "attach: MUTATION - no answer the screen shows carries a sentence");
}

void collision_tests() {
  using geom::Face;

  /* Thresholds from cBgW_CheckB*. */
  ok(geom::classify(1.0) == Face::Ground && geom::classify(0.5) == Face::Ground &&
         geom::classify(0.4999) == Face::Wall && geom::classify(0.0) == Face::Wall &&
         geom::classify(-0.8) == Face::Wall && geom::classify(-0.80001) == Face::Roof &&
         geom::classify(-1.0) == Face::Roof,
     "room: a triangle's role is the game's own two numbers, on both sides of each");

  /* The reader passes 0, not NaN, for a degenerate normal. */
  ok(geom::classify(0.0) == Face::Wall, "room: a triangle with no area is a wall, not a crash");

  {
    geom::Mesh m;
    const auto flat = [&m](float y) {
      const float t[9] = {0, y, 0, 100, y, 0, 0, y, 100};
      m.ground.insert(m.ground.end(), t, t + 9);
    };
    flat(-80.0f);
    flat(-100.0f);
    flat(-5000.0f);
    const float beach[9] = {0, -200, 0, 100, 20, 0, 0, 20, 100};
    m.ground.insert(m.ground.end(), beach, beach + 9);
    geom::split_seabed(m);

    const auto area = [](const std::vector<float>& g) {
      double a = 0;
      for (size_t t = 0; t + 8 < g.size(); t += 9) {
        a += std::fabs((g[t + 3] - g[t]) * (g[t + 8] - g[t + 2]) -
                       (g[t + 6] - g[t]) * (g[t + 5] - g[t + 2])) / 2;
      }
      return a;
    };
    const auto lowest = [](const std::vector<float>& g) {
      float y = 1e30f;
      for (size_t i = 1; i < g.size(); i += 3) y = std::min(y, g[i]);
      return y;
    };
    const auto highest = [](const std::vector<float>& g) {
      float y = -1e30f;
      for (size_t i = 1; i < g.size(); i += 3) y = std::max(y, g[i]);
      return y;
    };
    ok(lowest(m.ground) == -90.0f && highest(m.seabed) == -90.0f,
       "seabed: the ground stops at the swim depth and the seabed starts there, exactly");
    ok(std::fabs(area(m.ground) + area(m.seabed) - 20000.0) < 1e-3,
       "seabed: cutting a beach through the swim depth loses none of its area");
    /* 8750: the -80 flat (5000) plus the beach above -90 (3750). */
    ok(std::fabs(area(m.ground) - 8750.0) < 1e-3,
       "RED: ground 80 under the surface is floor, and 100 under is not");
  }

  /* Water, lava and poison groups are not floor: dBgW::ChkGrpThrough on the group's m_info. */
  {
    std::vector<uint8_t> verts(3 * geom::kVertexBytes, 0);
    const float xyz[9] = {-100, 0, -100, 100, 0, -100, -100, 0, 100};
    for (int i = 0; i < 9; ++i) {
      uint32_t bits;
      std::memcpy(&bits, &xyz[i], 4);
      for (int k = 0; k < 4; ++k) {
        verts[static_cast<size_t>(i) * 4 + k] = static_cast<uint8_t>(bits >> (24 - 8 * k));
      }
    }
    std::vector<uint8_t> tris(geom::kTriBytes, 0);
    /* Order 0, 2, 1 so cM3d_VectorProduct gives an upward normal. */
    tris[1] = 0; tris[3] = 2; tris[5] = 1;
    tris[9] = 0;

    const auto with_info = [&](uint32_t info, int parent_of_0, uint32_t parent_info) {
      std::vector<uint8_t> g(static_cast<size_t>(parent_of_0 < 0 ? 1 : 2) * geom::kGrpBytes, 0);
      const auto row = [&g](int at, uint32_t off, uint32_t v, int width) {
        const size_t p = static_cast<size_t>(at) * geom::kGrpBytes + off;
        for (int k = 0; k < width; ++k) {
          g[p + static_cast<size_t>(k)] = static_cast<uint8_t>(v >> (8 * (width - 1 - k)));
        }
      };
      row(0, 0x24, parent_of_0 < 0 ? 0xFFFFu : static_cast<uint32_t>(parent_of_0), 2);
      row(0, 0x30, info, 4);
      if (parent_of_0 >= 0) {
        row(1, 0x24, 0xFFFF, 2);
        row(1, 0x30, parent_info, 4);
      }
      geom::Mesh m;
      geom::Groups groups;
      groups.rows = g.data();
      groups.bytes = g.size();
      groups.count = parent_of_0 < 0 ? 1 : 2;
      geom::append(m, verts.data(), verts.size(), 3, tris.data(), tris.size(), 1, groups);
      return m;
    };

    ok(with_info(0, -1, 0).ground.size() == 9 && with_info(0, -1, 0).not_solid == 0,
       "room: a group that says nothing leaves its floor a floor");
    ok(with_info(geom::kGrpWater, -1, 0).ground.empty() &&
           with_info(geom::kGrpWater, -1, 0).not_solid == 1,
       "room: RED: a sheet of water points straight up and is not floor, which the normal alone "
       "cannot say");
    ok(with_info(geom::kGrpLava, -1, 0).ground.empty() &&
           with_info(geom::kGrpPoison, -1, 0).ground.empty(),
       "room: RED: nor is lava or poison");
    ok(with_info(geom::kGrpLight, -1, 0).ground.size() == 9,
       "room: RED: and light is left alone");
    /* ChkGrpThrough reads the group at depth 2, which can be the leaf's ancestor. */
    ok(with_info(0, 1, geom::kGrpWater).ground.empty(),
       "room: RED: and a leaf whose ancestor is the water group is water too");
    geom::Mesh bare;
    geom::append(bare, verts.data(), verts.size(), 3, tris.data(), tris.size(), 1);
    ok(bare.ground.size() == 9 && bare.not_solid == 0,
       "room: a room with no group table has no group that could be water");

    /* The triangle's fourth u16 names its attribute row; row 1 is stairs, row 0 is not. */
    std::vector<uint8_t> ti(2 * geom::kTiBytes, 0);
    const uint32_t inf1 = static_cast<uint32_t>(geom::kGroundStairs) << 21;
    for (int k = 0; k < 4; ++k) {
      ti[geom::kTiBytes + 4 + static_cast<size_t>(k)] = static_cast<uint8_t>(inf1 >> (24 - 8 * k));
    }
    geom::Attributes attributes;
    attributes.rows = ti.data();
    attributes.bytes = ti.size();
    attributes.count = 2;
    std::vector<uint8_t> on_row_1 = tris;
    on_row_1[7] = 1;
    geom::Mesh stairs;
    geom::append(stairs, verts.data(), verts.size(), 3, on_row_1.data(), on_row_1.size(), 1,
                 geom::Groups(), attributes);
    ok(stairs.ground_code.size() == 1 && stairs.ground_code[0] == geom::kGroundStairs,
       "room: a floor whose attribute row says stairs carries code 8");
    geom::Mesh on_row_0;
    geom::append(on_row_0, verts.data(), verts.size(), 3, tris.data(), tris.size(), 1,
                 geom::Groups(), attributes);
    ok(on_row_0.ground_code.size() == 1 && on_row_0.ground_code[0] == 0 &&
           bare.ground_code.size() == 1 && bare.ground_code[0] == 0,
       "room: RED: one on another row carries 0, and so does one with no attribute table");
  }

  ok(seam::floats({0.1f, -93.4267578125f, 217657.515625f}) == "[0.1,-93.42676,217657.52]",
     "room: a float writes as the shortest text that reads back as the same float");
  ok(seam::floats({}) == "[]", "room: no triangles is an empty array, not a missing field");
  const std::vector<float> awkward = {0.1f, 1e-30f, 3.4028235e38f, -0.0f, 1.0f / 3.0f};
  const seam::Json back = seam::parse(seam::floats(awkward));
  bool exact = back.ok() && back.items().size() == awkward.size();
  for (size_t i = 0; exact && i < awkward.size(); ++i) {
    if (static_cast<float>(back.items()[i].as_num()) != awkward[i]) exact = false;
  }
  ok(exact, "room: MUTATION - every float reads back bit-for-bit through the seam");

  seam::Json body = seam::Json::obj();
  body.set("ground", seam::Json::raw(seam::floats({1.0f, 2.0f})));
  ok(body.dump() == R"({"ground":[1,2]})", "room: a written-out array dumps as itself");

  const std::vector<std::string> asked = answer(R"({"ask":"room","pid":0})");
  ok(asked.size() == 1 && line_of(asked[0]) == "fail",
     "room: a pid that is not a Dolphin is refused in one line");
  const std::vector<std::string> nowhere = answer(R"({"ask":"room","stage":"Kaisen","room":0})");
  ok(nowhere.size() == 1 && line_of(nowhere[0]) == "fail" &&
         seam::parse(nowhere[0]).at("why").as_str().find("iso") != std::string::npos,
     "room: MUTATION - with neither a pid nor an iso it refuses rather than choosing a source");
}

void disc_tests() {
  /* `aaaaaaaa`: one literal, then a back-reference of seven that overlaps its own output. */
  const std::vector<uint8_t> run = {'Y', 'a', 'z', '0', 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0x80, 'a', 0x50, 0x00};
  std::vector<uint8_t> out;
  ok(disc::yaz0(run, &out) && out.size() == 8 &&
         std::string(out.begin(), out.end()) == "aaaaaaaa",
     "disc: a Yaz0 run that overlaps itself comes back whole");

  const std::vector<uint8_t> cut = {'Y', 'a', 'z', '0', 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0x80};
  ok(!disc::yaz0(cut, &out), "disc: MUTATION - a Yaz0 stream that runs out is refused");

  const std::vector<uint8_t> plain = {'R', 'A', 'R', 'C'};
  ok(disc::yaz0(plain, &out) && out == plain, "disc: bytes that are not Yaz0 pass through");

  std::vector<uint8_t> nothing;
  ok(!disc::from_rarc(plain, ".dzb", &nothing),
     "disc: MUTATION - four bytes that say RARC and hold no archive are refused");

  std::string why;
  ok(disc::Iso::open("no-such-disc.iso", &why) == nullptr && !why.empty(),
     "disc: a path that is not a disc refuses with a reason for the Logs");

  const std::vector<std::string> asked = answer(R"({"ask":"rooms"})");
  ok(asked.size() == 1 && line_of(asked[0]) == "fail" &&
         seam::parse(asked[0]).at("why").as_str().find("iso") != std::string::npos,
     "disc: the room list with no disc named refuses rather than going looking");
}

/* Points the per-user config folder at a temp dir first, so a run never touches real settings. */
void settings_tests() {
  const std::filesystem::path sandbox =
      std::filesystem::temp_directory_path() / "setupcore-settings-test";
  std::error_code ec;
  std::filesystem::remove_all(sandbox, ec);
  std::filesystem::create_directories(sandbox, ec);
  const std::string put = sandbox.string();
#if defined(_WIN32)
  ::_putenv_s("APPDATA", put.c_str());
#elif defined(__APPLE__)
  ::setenv("HOME", put.c_str(), 1);
#else
  ::setenv("XDG_CONFIG_HOME", put.c_str(), 1);
#endif
  ok(settings::path().find("setupcore-settings-test") != std::string::npos,
     "settings: the file is under this machine's own per-user folder, never beside the app");

  const std::vector<std::string> fresh = answer(R"({"ask":"settings"})");
  ok(fresh.size() == 2 && line_of(fresh[0]) == "data" &&
         seam::parse(fresh[0]).at("body").type() == seam::Json::Type::Obj,
     "settings: nothing kept yet answers an empty object, not a refusal");

  answer(R"({"ask":"remember","body":{"cores":3,"moves":{"cap":2,"off":["a"]}}})");
  const std::vector<std::string> back = answer(R"({"ask":"settings"})");
  const seam::Json body = seam::parse(back[0]).at("body");
  ok(body.at("cores").as_num(0) == 3 && body.at("moves").at("cap").as_num(0) == 2 &&
         body.at("moves").at("off").items().size() == 1,
     "settings: what was kept comes back whole, including a section the core never reads");

  answer(R"({"ask":"remember","body":{"cores":3}})");
  const std::vector<std::string> cleared = answer(R"({"ask":"settings"})");
  ok(seam::parse(cleared[0]).at("body").at("moves").type() == seam::Json::Type::Bad,
     "settings: a section left out of the write is cleared from the file");

  {
    std::ofstream broken(std::filesystem::path(settings::path()), std::ios::binary | std::ios::trunc);
    broken << "{not json";
  }
  const std::vector<std::string> bad = answer(R"({"ask":"settings"})");
  ok(bad.size() == 1 && line_of(bad[0]) == "fail",
     "settings: MUTATION - a file that will not read is one refusal, not a crash");

  const std::vector<std::string> empty = answer(R"({"ask":"remember"})");
  ok(empty.size() == 1 && line_of(empty[0]) == "fail",
     "settings: MUTATION - a write carrying nothing is refused rather than emptying the file");

  std::filesystem::remove_all(sandbox, ec);
}

}  // namespace

int main() {
  {
    std::string why;
    if (!disc::install_link_from_settings(&why)) {
      if (disc::the_disc().empty()) {
        std::printf("SKIPPED: link: %s\n", why.c_str());
        return 0;
      }
      std::printf("CHECKS FAILED: link: %s\n", why.c_str());
      return 1;
    }
  }
  json_tests();
  envelope_tests();
  bounds_tests();
  collision_tests();
  disc_tests();
  settings_tests();
  std::printf(failed ? "\n%d check(s) failed\n" : "\nall checks pass\n", failed);
  return failed ? 1 : 0;
}
