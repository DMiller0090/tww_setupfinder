/* Search suite: corridor, grid, engine adapter and the search. RED marks a case that must fail. */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <map>
#include <mutex>
#include <cstring>
#include <string>
#include <vector>

#include "../src/disc/link.h"
#include "../src/geom/mesh.h"
#include <set>

#include "../src/dolphin/collision.h"
#include "../src/search/approx.h"
#include "../src/search/cam_clear.h"
#include "SSystem/SComponent/c_math.h"
#include "../src/search/camera_type.h"
#include "../src/disc/disc.h"
#include "../src/disc/rarc.h"
#include "../src/geom/be.h"
#include "../src/seam/json.h"
#include "../src/settings/settings.h"
#include <algorithm>
#include <cstdlib>
#include "../src/search/catalogue.h"
#include "../src/search/cup_exit.h"
#include "../src/search/cup_tape.h"
#include "../src/search/l_chain.h"
#include "../src/search/corridor.h"
#include "../src/search/dzb.h"
#include "../src/search/grid.h"
#include "../src/search/search.h"
#include "../src/search/verify.h"
#include "d/d_camera.h"
#include "engine/session.h"

namespace {

int failed = 0;

void ok(bool cond, const std::string& what) {
  std::printf("%s %s\n", cond ? "ok  " : "FAIL", what.c_str());
  if (!cond) ++failed;
}

void tri(std::vector<float>& into, float x0, float y0, float z0, float x1, float y1, float z1,
         float x2, float y2, float z2) {
  const float nine[9] = {x0, y0, z0, x1, y1, z1, x2, y2, z2};
  into.insert(into.end(), nine, nine + 9);
}

/** A flat quad, wound so its normal points up (ground). */
void quad(std::vector<float>& into, float x0, float z0, float x1, float z1, float y) {
  tri(into, x0, y, z0, x0, y, z1, x1, y, z0);
  tri(into, x1, y, z1, x1, y, z0, x0, y, z1);
}

/** `quad` with its far x edge raised by `rise`. */
void ramp(std::vector<float>& into, float x0, float z0, float x1, float z1, float y, float rise) {
  tri(into, x0, y, z0, x0, y, z1, x1, y + rise, z0);
  tri(into, x1, y + rise, z1, x1, y + rise, z0, x0, y, z1);
}

bool holds_triangle(const std::vector<float>& list, float x, float z) {
  for (size_t at = 0; at + 9 <= list.size(); at += 9) {
    for (int v = 0; v < 3; ++v) {
      if (list[at + v * 3] == x && list[at + v * 3 + 2] == z) return true;
    }
  }
  return false;
}

bool cell_holds(const search::Grid& g, int ix, int iz, int t) {
  const int c = g.at(ix, iz);
  for (int i = g.start[static_cast<size_t>(c)]; i < g.start[static_cast<size_t>(c) + 1]; ++i) {
    if (g.tri[static_cast<size_t>(i)] == t) return true;
  }
  return false;
}

void corridor_tests() {
  const search::Corridor c = search::corridor(0, 0, 100, 0, 10);

  ok(c.from_spine(50, 0) == 0.0, "a point on the spine is no distance from it");
  ok(c.holds(50, 10), "a point exactly the widest move away is inside");
  ok(!c.holds(50, 10.5), "a point further than the widest move is outside");
  ok(c.holds(-9, 0) && !c.holds(-11, 0),
     "the corridor is a stadium: the caps reach behind the start");
  ok(c.min_x == -10 && c.max_x == 110 && c.min_z == -10 && c.max_z == 10,
     "the box is the segment widened by the widest move on every side");

  const search::Corridor point = search::corridor(5, 5, 5, 5, 10);
  ok(point.holds(5, 14) && !point.holds(5, 16),
     "a start that equals its target is a disc, not a degenerate line");

  const search::Corridor none = search::corridor(0, 0, 100, 0, -1);
  ok(none.half_width == 0.0, "a negative widest move is taken as none, never as a reach");

  ok(!none.holds(50, 1), "and a corridor with no width holds nothing beside the spine");
}

void selection_tests() {
  geom::Mesh mesh;
  quad(mesh.ground, 0, -50, 100, 50, 0);
  quad(mesh.ground, 900, 900, 1000, 1000, 0);
  tri(mesh.wall, 50, 0, -5, 50, 100, -5, 50, 0, 5);
  tri(mesh.roof, 0, 100, 0, 0, 100, 10, 10, 100, 0);
  tri(mesh.ground, 50, 0, -100, 50, 0, 100, 60, 0, 100);

  const search::Corridor c = search::corridor(0, 0, 100, 0, 10);
  const search::Selection s = search::select(mesh, c);

  ok(s.ground_total == 5 && s.wall_total == 1, "the totals are what the room held, not what was kept");
  ok(s.ground_held == 3, "three ground triangles reach the corridor and the far quad does not");
  ok(!holds_triangle(s.ground, 900, 900), "a triangle outside the corridor is not selected");
  ok(holds_triangle(s.ground, 50, -100),
     "a triangle crossing the corridor with every vertex outside it is selected");
  ok(s.wall_held == 1 && s.wall.size() == 9, "walls are selected as their own set");
  ok(s.ground.size() == 27, "and roofs are not selected at all: nothing in this plan tests one");

  int by_vertex = 0;
  for (size_t at = 0; at + 9 <= mesh.ground.size(); at += 9) {
    for (int v = 0; v < 3; ++v) {
      if (c.holds(mesh.ground[at + v * 3], mesh.ground[at + v * 3 + 2])) {
        ++by_vertex;
        break;
      }
    }
  }
  ok(by_vertex == 0, "a vertex-only test would have kept none of the three, which is the bug");
}

void grid_tests() {
  geom::Mesh mesh;
  quad(mesh.ground, -200, -200, 200, 200, 0);
  const search::Corridor c = search::corridor(-50, 0, 50, 0, 50);
  search::Limits limits;
  limits.cell = 25.0;
  const search::Selection s = search::select(mesh, c);
  const search::Grid g = search::build(s, c, limits);

  ok(g.nx == 8 && g.nz == 4, "the grid covers the corridor's box at the cell it was given");
  ok(g.cells == g.nx * g.nz && g.mark.size() == static_cast<size_t>(g.cells),
     "one mark a cell, and the cell count is its own answer");
  ok(g.start.size() == static_cast<size_t>(g.cells) + 1 &&
         g.start.back() == static_cast<int>(g.tri.size()),
     "the offsets end where the triangle list does");

  const int middle = g.at(4, 2);
  ok(g.start[static_cast<size_t>(middle) + 1] > g.start[static_cast<size_t>(middle)],
     "a cell over the floor holds the triangle under it");
  ok(g.mark[static_cast<size_t>(middle)] == 0,
     "flat ground reaching past the corridor leaves its inner cells unmarked");
  ok(std::abs(g.low[static_cast<size_t>(middle)]) < 1e-6 &&
         std::abs(g.high[static_cast<size_t>(middle)]) < 1e-6,
     "and the height it reports over that cell is the floor's own");

  geom::Mesh half;
  quad(half.ground, -200, -200, 0, 200, 0);
  const search::Grid stops = search::build(search::select(half, c), c, limits);
  ok(stops.mark[static_cast<size_t>(stops.at(7, 2))] & search::kNoGround,
     "a cell with no ground under it is marked for it");
  int edge = 0, unmarked_edge = 0;
  for (int iz = 0; iz < stops.nz; ++iz) {
    for (int ix = 0; ix < stops.nx; ++ix) {
      const size_t here = static_cast<size_t>(stops.at(ix, iz));
      if (stops.mark[here] & search::kNoGround) continue;
      bool beside_nothing = false;
      for (int dz = -1; dz <= 1; ++dz) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!stops.inside(ix + dx, iz + dz)) continue;
          if (stops.mark[static_cast<size_t>(stops.at(ix + dx, iz + dz))] & search::kNoGround) {
            beside_nothing = true;
          }
        }
      }
      if (!beside_nothing) continue;
      ++edge;
      if (!(stops.mark[here] & search::kHeight)) ++unmarked_edge;
    }
  }
  ok(edge > 0 && unmarked_edge == 0,
     "and every cell beside one is marked too: the floor ending is a step without a bottom");
}

void mark_tests() {
  const search::Corridor c = search::corridor(-50, 0, 50, 0, 50);
  search::Limits limits;
  limits.cell = 25.0;

  geom::Mesh step;
  quad(step.ground, -200, -200, 0, 200, 0);
  quad(step.ground, 0, -200, 200, 200, 100);
  const search::Grid stepped = search::build(search::select(step, c), c, limits);
  ok(stepped.mark[static_cast<size_t>(stepped.at(3, 2))] & search::kHeight,
     "a hundred-unit step marks the cells either side of it");
  ok(stepped.marked_height > 0 && stepped.marked >= stepped.marked_height,
     "and the run counts the reason, with the cell counted once however many reasons it has");

  search::Limits tall = limits;
  tall.height_step = 500.0;
  const search::Grid flat_enough = search::build(search::select(step, c), c, tall);
  ok(!(flat_enough.mark[static_cast<size_t>(flat_enough.at(3, 2))] & search::kHeight),
     "a step smaller than the threshold is not a height mark");

  geom::Mesh sloped;
  quad(sloped.ground, -200, -200, 0, 200, 0);
  ramp(sloped.ground, 0, -200, 200, 200, 0, 60);
  const search::Grid bent = search::build(search::select(sloped, c), c, limits);
  ok(bent.mark[static_cast<size_t>(bent.at(3, 2))] & search::kNormal,
     "ground that changes normal marks the cell on the flat side of the change");
  ok(bent.marked_normal > 0, "and that reason is counted");

  search::Limits loose = limits;
  loose.normal_apart = 2.0;
  const search::Grid ignored = search::build(search::select(sloped, c), c, loose);
  ok(!(ignored.mark[static_cast<size_t>(ignored.at(3, 2))] & search::kNormal),
     "a threshold no two normals can exceed marks nothing, which is what the number decides");

  geom::Mesh steep;
  ramp(steep.ground, -200, -200, 200, 200, 0, 300);
  const search::Grid leaning = search::build(search::select(steep, c), c, limits);
  ok(leaning.marked_slope > 0, "ground steeper than a ported state is marked for its slope");

  search::Limits any_slope = limits;
  any_slope.slope_normal_y = 0.0;
  const search::Grid allowed = search::build(search::select(steep, c), c, any_slope);
  ok(allowed.marked_slope == 0, "and the angle that decides it is a number, not this file's taste");
}

void cover_tests() {
  geom::Mesh mesh;
  tri(mesh.ground, 0, 0, 0, 0, 0, 90, 90, 0, 0);
  const search::Corridor c = search::corridor(0, 0, 100, 100, 0);
  search::Limits limits;
  limits.cell = 50.0;
  const search::Selection s = search::select(mesh, c);
  const search::Grid g = search::build(s, c, limits);

  ok(g.nx == 2 && g.nz == 2, "the box of a corridor with no width is the segment's own");
  ok(cell_holds(g, 0, 0, 0), "the cell the triangle fills holds it");
  ok(cell_holds(g, 1, 0, 0) && cell_holds(g, 0, 1, 0), "so do the two its hypotenuse cuts");
  ok(!cell_holds(g, 1, 1, 0), "and the far cell, which only its box reaches, does not");
  ok(g.worst_cell >= 1 && g.per_cell > 0, "the run counts what a lookup would have to walk");

  ok(g.tri.size() == 3, "a box cover would have put it in all four, and that is the waste avoided");
}

void ceiling_tests() {
  geom::Mesh mesh;
  quad(mesh.ground, 0, 0, 100000, 100000, 0);
  const search::Corridor c = search::corridor(0, 0, 100000, 100000, 500);
  search::Limits limits;
  limits.cell = 50.0;
  limits.max_cells = 10000;
  const search::Grid g = search::build(search::select(mesh, c), c, limits);

  ok(g.cell > limits.cell, "a grid that would not fit the ceiling is built at a wider cell");
  ok(g.cells <= limits.max_cells, "and the grid it built is inside it");
  ok(g.cell == 1600.0, "widened by doubling, so the cell is always a whole number of the first");
}

/** Big-endian, as the disc is. */
void put32(std::vector<uint8_t>& b, size_t at, uint32_t v) {
  b[at + 0] = static_cast<uint8_t>(v >> 24);
  b[at + 1] = static_cast<uint8_t>(v >> 16);
  b[at + 2] = static_cast<uint8_t>(v >> 8);
  b[at + 3] = static_cast<uint8_t>(v);
}

void put16(std::vector<uint8_t>& b, size_t at, uint16_t v) {
  b[at + 0] = static_cast<uint8_t>(v >> 8);
  b[at + 1] = static_cast<uint8_t>(v);
}

void put_f32(std::vector<uint8_t>& b, size_t at, float v) {
  uint32_t bits;
  std::memcpy(&bits, &v, sizeof bits);
  put32(b, at, bits);
}

/** The smallest whole `.dzb`: six counts and offsets, then one row in each table. */
std::vector<uint8_t> a_dzb() {
  const uint32_t verts_at = 0x34;
  const uint32_t tris_at = verts_at + 3 * 12;
  const uint32_t blk_at = tris_at + 10;
  const uint32_t tree_at = blk_at + 2;
  const uint32_t grp_at = tree_at + 20;
  const uint32_t ti_at = grp_at + 0x34;
  std::vector<uint8_t> b(ti_at + 16, 0);

  put32(b, 0x00, 3);  put32(b, 0x04, verts_at);
  put32(b, 0x08, 1);  put32(b, 0x0C, tris_at);
  put32(b, 0x10, 1);  put32(b, 0x14, blk_at);
  put32(b, 0x18, 1);  put32(b, 0x1C, tree_at);
  put32(b, 0x20, 1);  put32(b, 0x24, grp_at);
  put32(b, 0x28, 1);  put32(b, 0x2C, ti_at);

  put_f32(b, verts_at + 0, -100.0f);
  put_f32(b, verts_at + 4, 7.5f);
  put_f32(b, verts_at + 8, -100.0f);
  put_f32(b, verts_at + 12, 100.0f);
  put_f32(b, verts_at + 16, 7.5f);
  put_f32(b, verts_at + 20, -100.0f);
  put_f32(b, verts_at + 24, -100.0f);
  put_f32(b, verts_at + 28, 7.5f);
  put_f32(b, verts_at + 32, 100.0f);

  put16(b, tris_at + 0, 0);
  put16(b, tris_at + 2, 1);
  put16(b, tris_at + 4, 2);
  put16(b, tris_at + 6, 0);  /* id */
  put16(b, tris_at + 8, 0);  /* grp */

  put16(b, blk_at, 0);
  put16(b, tree_at + 0, 1);       /* mFlag: leaf */
  put16(b, tree_at + 2, 0xFFFF);  /* mParent */
  put16(b, tree_at + 4, 0);       /* mChild[0], which a leaf reads as mBlock */

  put_f32(b, grp_at + 0x04, 1.0f);
  put_f32(b, grp_at + 0x08, 1.0f);
  put_f32(b, grp_at + 0x0C, 1.0f);
  put16(b, grp_at + 0x24, 0xFFFF);  /* m_parent */
  put16(b, grp_at + 0x26, 0xFFFF);  /* m_next_sibling */
  put16(b, grp_at + 0x28, 0xFFFF);  /* m_first_child */
  put16(b, grp_at + 0x2E, 0);       /* m_tree_idx */
  /* m_info: the light through-bit, which the search ignores, so the triangle stays floor. */
  put32(b, grp_at + 0x30, 0x80034);

  put32(b, ti_at + 0, 0x11111111);
  put32(b, ti_at + 4, 0x22222222);
  put32(b, ti_at + 8, 0x33333333);
  put32(b, ti_at + 12, 0x44444444);
  return b;
}

void adapter_tests() {
  std::vector<uint8_t> bytes = a_dzb();
  tww_engine::RoomDzb d;
  std::string why;
  ok(search::room_of(bytes, &d, &why), "a whole .dzb reads");

  ok(d.v_tbl.size() == 3 && d.t_tbl.size() == 1, "the vertices and the triangles");
  ok(d.b_tbl.size() == 1 && d.tree_tbl.size() == 1, "the blocks and the tree - the octree itself");
  ok(d.g_tbl.size() == 1 && d.ti_tbl.size() == 1, "the groups and the attribute rows");

  ok(d.v_tbl[1].x == 100.0f && d.v_tbl[0].y == 7.5f,
     "a vertex comes back at its own value, byte-swapped");
  ok(d.t_tbl[0].vtx0 == 0 && d.t_tbl[0].vtx1 == 1 && d.t_tbl[0].vtx2 == 2,
     "a triangle names its three vertices");
  ok(d.tree_tbl[0].mFlag == 1 && d.tree_tbl[0].mBlock == 0,
     "a leaf's block index is mChild[0], because the two are a union");
  ok(d.g_tbl[0].m_info == 0x80034, "the group's info, which ChkGrpThrough tests, is not zeroed");
  ok(d.ti_tbl[0].mPolyInf3 == 0x44444444, "the attribute row GetPolyInf3 reads");

  std::vector<uint8_t> shorter(bytes.begin(), bytes.begin() + 0x20);
  ok(!search::room_of(shorter, &d, &why), "a file too short to hold a header is refused");

  std::vector<uint8_t> off_end = a_dzb();
  put32(off_end, 0x1C, 0xFFFF0000);
  ok(!search::room_of(off_end, &d, &why), "a table offset past the end of the file is refused");

  std::vector<uint8_t> too_many = a_dzb();
  put32(too_many, 0x18, 1000000);
  ok(!search::room_of(too_many, &d, &why), "a count no room has is refused");

  /* RED: cM3d_CalcPla indexes the vertex table unchecked. */
  std::vector<uint8_t> bad_index = a_dzb();
  put16(bad_index, 0x34 + 3 * 12 + 2, 9);
  ok(!search::room_of(bad_index, &d, &why), "a triangle naming a vertex the room does not have");
}

/* A room as loaded in MEM1: the .dzb after cBgS::ConvDzb adds its own address to the six offsets. */
constexpr uint32_t kFakeBgw = 0x80100000u;
constexpr uint32_t kFakeBgd = 0x80101000u;

std::vector<uint8_t> a_loaded_room() {
  std::vector<uint8_t> image(0x2000, 0);  /* from kFakeBgw upward */
  const std::vector<uint8_t> file = a_dzb();
  const uint32_t bgd_at = kFakeBgd - kFakeBgw;

  /* cBgW::pm_bgd */
  put32(image, 0x94, kFakeBgd);
  std::memcpy(image.data() + bgd_at, file.data(), file.size());

  const uint32_t fields[6] = {0x04, 0x0C, 0x14, 0x1C, 0x24, 0x2C};
  for (int i = 0; i < 6; ++i) {
    const uint32_t offset = (static_cast<uint32_t>(file[fields[i] + 0]) << 24) |
                            (static_cast<uint32_t>(file[fields[i] + 1]) << 16) |
                            (static_cast<uint32_t>(file[fields[i] + 2]) << 8) |
                            static_cast<uint32_t>(file[fields[i] + 3]);
    put32(image, bgd_at + fields[i], kFakeBgd + offset);
  }
  return image;
}

void attached_tests() {
  const std::vector<uint8_t> image = a_loaded_room();
  const dolphin::Read read = [&image](uint32_t at, void* dst, size_t n) {
    if (at < kFakeBgw) return false;
    const size_t from = at - kFakeBgw;
    if (from + n > image.size()) return false;
    std::memcpy(dst, image.data() + from, n);
    return true;
  };

  const dolphin::Dzb got = dolphin::dzb_at(read, kFakeBgw);
  ok(got.ok, "a room is followed out of a process through its six pointers");

  tww_engine::RoomDzb from_ram, from_disc;
  std::string why;
  ok(search::room_of(got.located(), &from_ram, &why), "and the six read as a room the engine wants");
  ok(search::room_of(a_dzb(), &from_disc, &why), "as the same room off the disc does");
  ok(from_ram.v_tbl.size() == from_disc.v_tbl.size() &&
         from_ram.t_tbl.size() == from_disc.t_tbl.size() &&
         from_ram.b_tbl.size() == from_disc.b_tbl.size() &&
         from_ram.tree_tbl.size() == from_disc.tree_tbl.size() &&
         from_ram.g_tbl.size() == from_disc.g_tbl.size() &&
         from_ram.ti_tbl.size() == from_disc.ti_tbl.size(),
     "and the two halves come back with all six tables the same length");
  ok(from_ram.v_tbl[1].x == from_disc.v_tbl[1].x && from_ram.v_tbl[0].y == from_disc.v_tbl[0].y,
     "a vertex is the same number whichever half read it");
  ok(from_ram.tree_tbl[0].mBlock == from_disc.tree_tbl[0].mBlock &&
         from_ram.g_tbl[0].m_info == from_disc.g_tbl[0].m_info &&
         from_ram.ti_tbl[0].mPolyInf3 == from_disc.ti_tbl[0].mPolyInf3,
     "and so are the four tables a disc read for the screen throws away");

  const size_t nine =
      got.mesh.ground.size() + got.mesh.wall.size() + got.mesh.roof.size();
  ok(got.mesh.ok && nine == 9,
     "the room the grid is built from comes off the same read as the room the engine steps on");

  std::vector<uint8_t> no_bgd = a_loaded_room();
  put32(no_bgd, 0x94, 0x00000000);
  const dolphin::Read reads_no_bgd = [&no_bgd](uint32_t at, void* dst, size_t n) {
    if (at < kFakeBgw || (at - kFakeBgw) + n > no_bgd.size()) return false;
    std::memcpy(dst, no_bgd.data() + (at - kFakeBgw), n);
    return true;
  };
  ok(!dolphin::dzb_at(reads_no_bgd, kFakeBgw).ok,
     "RED: a room whose collision is not loaded yet is refused rather than followed to zero");

  std::vector<uint8_t> off_end = a_loaded_room();
  put32(off_end, (kFakeBgd - kFakeBgw) + 0x1C, 0x81FFFFF0u);
  const dolphin::Read reads_off_end = [&off_end](uint32_t at, void* dst, size_t n) {
    if (at < kFakeBgw || (at - kFakeBgw) + n > off_end.size()) return false;
    std::memcpy(dst, off_end.data() + (at - kFakeBgw), n);
    return true;
  };
  ok(!dolphin::dzb_at(reads_off_end, kFakeBgw).ok,
     "RED: a table that would run off the end of MEM1 is refused");

  std::vector<uint8_t> too_many = a_loaded_room();
  put32(too_many, (kFakeBgd - kFakeBgw) + 0x18, 1000000);
  const dolphin::Read reads_too_many = [&too_many](uint32_t at, void* dst, size_t n) {
    if (at < kFakeBgw || (at - kFakeBgw) + n > too_many.size()) return false;
    std::memcpy(dst, too_many.data() + (at - kFakeBgw), n);
    return true;
  };
  ok(!dolphin::dzb_at(reads_too_many, kFakeBgw).ok,
     "RED: a count no room has is refused before it is multiplied by a row");
}

/** Reference for `search::float_steps`: one `nextafter` at a time. */
static long long float_steps_walked(double a, double b) {
  float from = static_cast<float>(a), to = static_cast<float>(b);
  if (from == to) return 0;
  if (from > to) std::swap(from, to);
  long long steps = 0;
  while (from < to && steps < 1000000) {
    from = std::nextafter(from, to);
    ++steps;
  }
  return steps;
}

void target_tests() {
  {
    const float anchors[] = {0.0f, -0.0f, 1e-45f, -1e-45f, 1e-38f, -1e-38f, 2.8709774f,
                             -2.8709774f, 666.5806f, -204015.22f, 0.25f, -0.25f};
    const float spans[] = {0.0f, 1e-45f, 3e-45f, 1e-40f, 1e-7f, 3e-7f, 0.01f, 0.2f, 1.0f, 5.0f};
    int pairs = 0, agree = 0;
    for (float a : anchors) {
      for (float d : spans) {
        for (int sign = -1; sign <= 1; sign += 2) {
          const double b = static_cast<double>(a) + sign * static_cast<double>(d);
          ++pairs;
          if (search::float_steps(a, b) == float_steps_walked(a, b)) ++agree;
        }
      }
    }
    ok(pairs == 240 && agree == pairs,
       "float steps are counted from the bits and agree with stepping one value at a time, "
       "across zero, the subnormals and the cap (" + std::to_string(agree) + " of " +
           std::to_string(pairs) + ")");
    ok(search::float_steps(-1e-45, 1e-45) == 2 && search::float_steps(-0.0, 0.0) == 0,
       "the smallest value either side of zero is two steps apart, and the two zeros none");
    ok(search::float_steps(-2.8709774, 5.0) == 1000000,
       "a miss a room wide is the cap, as it was");
  }

  search::Target point;
  point.x = 10.0;
  point.z = 20.0;
  ok(point.distance(13.0, 24.0) == 5.0, "a point is the straight line to it");
  ok(point.distance(10.0, 20.0) == 0.0, "and zero on it");
  double nx = 0, nz = 0;
  point.nearest(13.0, 24.0, &nx, &nz);
  ok(nx == 10.0 && nz == 20.0, "the nearest place a point allows is the point");

  search::Target free_z = point;
  free_z.has_z = false;
  ok(free_z.distance(13.0, 24.0) == 3.0,
     "a freed axis is not measured - a target free in z is off by its x and nothing else");
  ok(free_z.distance(10.0, -9999.0) == 0.0,
     "so every value of the freed axis arrives, which is what the checkbox says");
  free_z.nearest(13.0, 24.0, &nx, &nz);
  ok(nx == 10.0 && nz == 24.0,
     "and the nearest place on a freed axis is where he already is, not a coordinate he never "
     "asked about");

  search::Target anywhere = point;
  anywhere.has_x = false;
  anywhere.has_z = false;
  ok(anywhere.distance(-500.0, 900.0) == 0.0,
     "both axes freed is every place in the room - a question whose whole answer is the facing");

  search::Target box;
  box.ranged = true;
  box.x0 = -10.0; box.x1 = 10.0; box.z0 = -5.0; box.z1 = 5.0;
  ok(box.distance(0.0, 0.0) == 0.0 && box.distance(10.0, 5.0) == 0.0,
     "inside a range is not off by anything, edges included");
  ok(box.distance(13.0, 0.0) == 3.0, "outside it is the distance to the nearest edge");
  ok(box.distance(13.0, 9.0) == 5.0, "and to the nearest corner past a corner");
  box.nearest(13.0, 9.0, &nx, &nz);
  ok(nx == 10.0 && nz == 5.0, "whose nearest place is that corner");

  search::Target strip = box;
  strip.has_x = false;
  ok(strip.distance(99999.0, 0.0) == 0.0,
     "a range free in x is every x - a place far outside it is not off by anything");
  ok(strip.distance(99999.0, 9.0) == 4.0,
     "RED: and the freed axis is not a term, so the miss is the z alone");
  strip.nearest(99999.0, 9.0, &nx, &nz);
  ok(nx == 99999.0 && nz == 5.0,
     "whose nearest place keeps the freed coordinate he already has");

  /* As geom::Mesh::box: min x, y, z then max x, y, z. */
  const float room[6] = {-1000.0f, 0.0f, -2000.0f, 1000.0f, 0.0f, 2000.0f};
  double min_x = 0, min_z = 0, max_x = 0, max_z = 0;
  point.extent(room, &min_x, &min_z, &max_x, &max_z);
  ok(min_x == 10.0 && max_x == 10.0 && min_z == 20.0 && max_z == 20.0,
     "a point occupies a point");
  free_z.extent(room, &min_x, &min_z, &max_x, &max_z);
  ok(min_x == 10.0 && max_x == 10.0 && min_z == -2000.0 && max_z == 2000.0,
     "a freed axis takes the room's own extent - a goal with no ends is a grid with no ends");

  search::Target loose;
  loose.ranged = true;
  loose.z0 = -5.0; loose.z1 = 5.0;
  loose.has_x = false;
  loose.extent(room, &min_x, &min_z, &max_x, &max_z);
  ok(min_x == -1000.0 && max_x == 1000.0 && min_z == -5.0 && max_z == 5.0,
     "RED: a range's freed axis takes the room as well, so no corridor is ever laid over a float "
     "with no end to it");

  search::EndFacing any;
  ok(any.holds(0) && any.holds(40000), "no facing asked for holds every facing");
  search::EndFacing one;
  one.any = false;
  one.a = 16384;
  ok(one.holds(16384) && !one.holds(16385), "a single facing is that facing and no other");
  search::EndFacing arc;
  arc.any = false;
  arc.a = 16384;
  arc.span = 16384;
  ok(arc.holds(16384) && arc.holds(24000) && arc.holds(32768) && !arc.holds(32769),
     "an arc sweeps positive from a, which is the way the room draws the fan");
  search::EndFacing wrapped;
  wrapped.any = false;
  wrapped.a = 60000;
  wrapped.span = 10000;
  ok(wrapped.holds(60000) && wrapped.holds(4464) && !wrapped.holds(5000),
     "and an arc over the wrap is still one arc");

  const search::Corridor was = search::corridor(0.0, 0.0, 10.0, 20.0, 50.0);
  const search::Corridor now = search::corridor(0.0, 0.0, point, room, 50.0);
  ok(!now.boxed && now.ax == was.ax && now.bx == was.bx && now.bz == was.bz &&
         now.min_x == was.min_x && now.max_z == was.max_z,
     "a point's corridor IS the corridor it always was - same spine, same box, no second arm");

  search::Target wide_box;
  wide_box.ranged = true;
  wide_box.x0 = -900.0; wide_box.x1 = 900.0; wide_box.z0 = 1000.0; wide_box.z1 = 1000.0;
  const search::Corridor over = search::corridor(0.0, 0.0, wide_box, room, 50.0);
  ok(over.boxed && over.min_x <= -950.0 && over.max_x >= 950.0,
     "a range's corridor reaches the whole range and not just the middle of it");
  ok(over.from_spine(900.0, 1000.0) == 0.0,
     "RED: a place at the far end of the range is ON the corridor - off the spine, but the goal "
     "is the box and not the spine");
  ok(search::corridor(0.0, 0.0, 0.0, 1000.0, 50.0).from_spine(900.0, 1000.0) > 50.0,
     "which the spine alone says it is not, and that is the selection the range would have lost");

  const search::Corridor band = search::corridor(0.0, 0.0, free_z, room, 50.0);
  ok(band.boxed && band.min_z <= -2000.0 && band.max_z >= 2000.0,
     "a freed axis is a band across the room, bounded by the room and by nothing else");
}

/** A List as the seam builds one: the rows, and their box as the range. */
search::Target listed(const std::vector<double>& rows) {
  search::Target t;
  t.ranged = true;
  t.list = rows;
  t.x0 = t.x1 = rows[0];
  t.z0 = t.z1 = rows[1];
  for (size_t i = 0; i + 1 < rows.size(); i += 2) {
    t.x0 = std::min(t.x0, rows[i]); t.x1 = std::max(t.x1, rows[i]);
    t.z0 = std::min(t.z0, rows[i + 1]); t.z1 = std::max(t.z1, rows[i + 1]);
  }
  return t;
}

void list_tests() {
  geom::Mesh half;
  quad(half.ground, -200, -200, 0, 200, 0);
  const search::Corridor c = search::corridor(-300, 0, 300, 0, 60);
  search::Limits limits;
  limits.cell = 25.0;
  const search::Selection s = search::select(half, c);
  const search::Grid g = search::build(s, c, limits);

  /* Two rows on the floor and one past its edge. */
  search::Target three = listed({-100, 0, -20, 50, 150, 0});
  ok(three.distance(150.0, 10.0) == 10.0,
     "before the floor is read, a list is as far as its nearest row");
  ok(search::resolve(&three, g, s.ground, 5.0) && three.ground.kept > 0,
     "a list keeps the floor under its rows, and a row with none is dropped without refusing it");
  ok(three.distance(-100.0, 0.0) == 0.0 && three.distance(-100.0, 30.0) == 25.0,
     "each row is a point with the tolerance about it, as a point target is");
  ok(three.distance(-25.0, 40.0) == 5.0,
     "RED: the nearest row is the goal, not the first one");
  ok(three.distance(-60.0, 25.0) > 0.0,
     "RED: the box around the rows is not a place; between them is off the goal");
  double nx = 0, nz = 0;
  three.nearest(-22.0, 60.0, &nx, &nz);
  ok(std::fabs(nx + 22.0) < 1e-9 && std::fabs(nz - 55.0) < 1e-9,
     "the nearest place is on the nearest row's own floor");

  search::Target none = listed({150, 0, 180, 40});
  ok(!search::resolve(&none, g, s.ground, 5.0),
     "a list with no row on any floor is refused, the same as a point with none");
}

void ground_tests() {
  geom::Mesh two;
  quad(two.ground, -400, -400, 400, 400, 0);
  quad(two.ground, -400, -400, 0, 400, 300);
  const search::Corridor c = search::corridor(-300, 0, 300, 0, 60);
  search::Limits limits;
  limits.cell = 25.0;
  const search::Selection s = search::select(two, c);
  const search::Grid g = search::build(s, c, limits);

  search::Target flat;
  flat.ranged = true;
  flat.x0 = -120.0; flat.x1 = -80.0; flat.z0 = -20.0; flat.z1 = 20.0;
  search::Target plan = flat;
  ok(search::resolve(&plan, g, s.ground, 0.0) && plan.ground.kept > 0,
     "a target over a floor keeps the cells it stands on");
  ok(plan.distance(-100.0, 0.0) == 0.0, "and is not off by anything inside them");

  search::Target low = flat;
  low.has_y = true;
  low.y0 = -1.0; low.y1 = 1.0;
  ok(search::resolve(&low, g, s.ground, 0.0) && low.ground.kept > 0,
     "a height the lower floor reaches keeps the cells of it");

  search::Target high = flat;
  high.has_y = true;
  high.y0 = 299.0; high.y1 = 301.0;
  ok(search::resolve(&high, g, s.ground, 0.0) && high.ground.kept > 0,
     "and a height the balcony reaches keeps them for the balcony");

  search::Target between = flat;
  between.has_y = true;
  between.y0 = 149.0; between.y1 = 151.0;
  ok(!search::resolve(&between, g, s.ground, 0.0) && between.ground.kept == 0,
     "RED: a height between two floors is ground nowhere, whatever the cell's own range says");

  search::Target balcony;
  balcony.ranged = true;
  balcony.x0 = -100.0; balcony.x1 = 100.0; balcony.z0 = -20.0; balcony.z1 = 20.0;
  balcony.has_y = true;
  balcony.y0 = 299.0; balcony.y1 = 301.0;
  ok(search::resolve(&balcony, g, s.ground, 0.0), "a box straddling an edge keeps the side that has floor");
  ok(balcony.distance(-50.0, 0.0) == 0.0,
     "a place over the balcony is inside the goal the bytes allow");
  ok(balcony.distance(80.0, 0.0) > 50.0,
     "RED: and one at the same height on the other side of the edge is not - which is the plan at "
     "the right x and z on the wrong floor");

  search::Target hole;
  hole.ranged = true;
  hole.x0 = 260.0; hole.x1 = 280.0; hole.z0 = -10.0; hole.z1 = 10.0;
  geom::Mesh holed;
  quad(holed.ground, -400, -400, 200, 400, 0);
  const search::Selection hs = search::select(holed, c);
  const search::Grid hg = search::build(hs, c, limits);
  ok(!search::resolve(&hole, hg, hs.ground, 0.0) && hole.ground.kept == 0,
     "a target past where the floor ends has no ground in it, which is a run that is refused");

  search::Target forgiving = hole;
  ok(search::resolve(&forgiving, hg, hs.ground, 100.0) && forgiving.ground.kept > 0,
     "RED: and the same target with tolerance enough to reach the floor is not refused");

  search::Target elsewhere;
  elsewhere.ranged = true;
  elsewhere.x0 = 90000.0; elsewhere.x1 = 90100.0; elsewhere.z0 = 90000.0; elsewhere.z1 = 90100.0;
  ok(!search::resolve(&elsewhere, g, s.ground, 0.0),
     "and a target nowhere near the room is refused rather than searched for");

  search::Target box_only = balcony;
  box_only.has_y = false;
  box_only.ground = search::Region();
  ok(balcony.distance(80.0, 0.0) >= box_only.distance(80.0, 0.0),
     "the goal narrowed to its floor is never nearer than the box it was cut from");
}

void clipped_tests() {
  geom::Mesh wedge;
  tri(wedge.ground, -200, 0, -200, -200, 0, 200, 200, 0, -200);
  const search::Corridor c = search::corridor(-300, 0, 300, 0, 60);
  search::Limits limits;
  limits.cell = 50.0;
  const search::Selection s = search::select(wedge, c);
  const search::Grid g = search::build(s, c, limits);

  search::Target over;
  over.ranged = true;
  over.x0 = -200.0; over.x1 = 200.0; over.z0 = -200.0; over.z1 = 200.0;
  ok(search::resolve(&over, g, s.ground, 0.0) && over.ground.kept == 1,
     "one ground triangle inside the region is ONE polygon, not the cells it crosses");
  ok(over.distance(-150.0, -150.0) == 0.0, "a place on the floor is in the goal");

  /* To the diagonal x + z = 0: 240 / sqrt(2). */
  const double off = over.distance(120.0, 120.0);
  ok(off > 160.0 && off < 175.0,
     "RED: a place inside the BOX but past where the floor ends is a real distance from the goal, "
     "which a cell kept whole answered as zero");

  /* A zero-width slab is clipped twice at one x; the second clip must keep what the first rounded
     just outside. */
  const double grid_z0 = g.origin_z, grid_z1 = g.origin_z + g.cell * g.nz;
  int short_lines = 0, lines = 0;
  for (int k = 0; k < 4000; ++k) {
    search::Target line;
    line.has_x = true;
    line.has_z = false;
    line.x = static_cast<double>(static_cast<float>(-150.0 + k * 0.0499));
    const double want_lo = std::max(-200.0, grid_z0), want_hi = std::min(-line.x, grid_z1);
    ++lines;
    if (!search::resolve(&line, g, s.ground, 0.0)) {
      ++short_lines;
      continue;
    }
    double lo = 1e300, hi = -1e300;
    for (size_t i = 2; i < line.ground.vert.size(); i += 3) {
      lo = std::min(lo, line.ground.vert[i]);
      hi = std::max(hi, line.ground.vert[i]);
    }
    if (lo > want_lo + 1e-6 || hi < want_hi - 1e-6) ++short_lines;
  }
  ok(lines > 0 && short_lines == 0,
     "RED: a line with no thickness keeps the whole of where the floor crosses it, at every x");

  search::Target inside;
  inside.ranged = true;
  inside.x0 = -150.0; inside.x1 = -50.0; inside.z0 = -150.0; inside.z1 = -50.0;
  ok(search::resolve(&inside, g, s.ground, 0.0) && inside.ground.kept == 1,
     "a region strictly inside one triangle is that region");
  const double west = inside.distance(-160.0, -100.0), east = inside.distance(-40.0, -100.0);
  const double north = inside.distance(-100.0, -160.0), south = inside.distance(-100.0, -40.0);
  ok(std::fabs(west - 10.0) < 1e-9 && std::fabs(east - 10.0) < 1e-9 &&
         std::fabs(north - 10.0) < 1e-9 && std::fabs(south - 10.0) < 1e-9,
     "RED: MUTATION - ten units outside each of its four sides is ten units from the goal, so "
     "every wall of the region cut and none of them was a wall the triangle already had");

  geom::Mesh slope;
  ramp(slope.ground, -200, -200, 200, 200, 0, 400);
  const search::Selection ss = search::select(slope, c);
  const search::Grid sg = search::build(ss, c, limits);
  search::Target band;
  band.ranged = true;
  band.x0 = -200.0; band.x1 = 200.0; band.z0 = -200.0; band.z1 = 200.0;
  band.has_y = true;
  band.y0 = 100.0; band.y1 = 200.0;
  ok(search::resolve(&band, sg, ss.ground, 0.0) && band.ground.kept > 0,
     "a floor climbing through the allowed band is kept where it is in it");
  /* y = 200 + x, so heights 100 to 200 are x -100 to 0. */
  ok(band.distance(-50.0, 0.0) == 0.0, "the part of the ramp at the allowed height is the goal");
  ok(band.distance(180.0, 0.0) > 150.0,
     "RED: and the part of the same triangle above that height is not - which a test on the "
     "cell's own range answered as ground everywhere the cell reached");

  bool on_plane = true;
  for (size_t i = 0; i + 2 < band.ground.vert.size(); i += 3) {
    const double want = 200.0 + band.ground.vert[i];
    if (std::fabs(band.ground.vert[i + 1] - want) > 1e-6) on_plane = false;
  }
  ok(on_plane,
     "RED: every vertex of the region lies in the plane of the triangle it was cut from, so it "
     "cannot float above the floor or cut through it");

  search::Target point;
  point.x = -150.0;
  point.z = 0.0;
  ok(search::resolve(&point, g, s.ground, 30.0) && point.ground.kept == 1,
     "a point is a region too - its tolerance about a place");
  search::Target adrift;
  adrift.x = 150.0;
  adrift.z = 0.0;
  ok(!search::resolve(&adrift, g, s.ground, 30.0),
     "and a point whose tolerance reaches no floor is refused, the same as a box that reaches "
     "none");

  search::Target line;
  line.z = 0.0;
  line.has_x = false;
  ok(search::resolve(&line, g, s.ground, 20.0) && line.ground.kept == 1,
     "a freed axis is the floor along that line");
  ok(line.distance(-190.0, 0.0) == 0.0 && line.distance(190.0, 0.0) > 0.0,
     "RED: which stops at the floor's own edge rather than running the width of the room");

  /* `distance` walks the region's index and stops early; hold it to a walk of every polygon, over
     many triangles with a hole so the nearest piece is often not the one underfoot. */
  geom::Mesh many;
  for (int mx = 0; mx < 8; ++mx) {
    for (int mz = 0; mz < 8; ++mz) {
      const float qx = -200.0f + 50.0f * mx, qz = -200.0f + 50.0f * mz;
      if (mx >= 3 && mx <= 4 && mz >= 3 && mz <= 4) continue;  // the hole
      quad(many.ground, qx, qz, qx + 50.0f, qz + 50.0f, 0.0f);
    }
  }
  search::Target field;
  field.ranged = true;
  field.x0 = -200.0; field.x1 = 200.0; field.z0 = -200.0; field.z1 = 200.0;
  const search::Corridor mc = search::corridor(-300, 0, field, many.box, 60);
  const search::Selection ms = search::select(many, mc);
  const search::Grid mg = search::build(ms, mc, limits);
  ok(search::resolve(&field, mg, ms.ground, 0.0) && field.ground.kept > 100,
     "a floor of many triangles is many polygons, one for each");

  bool agrees = true;
  double worst = 0.0;
  for (int i = 0; i < 400; ++i) {
    const double px = -300.0 + 1.507 * i, pz = -300.0 + 0.931 * ((i * 7) % 400);
    double best = 1e300;
    for (int p = 0; p < field.ground.kept; ++p) {
      const int from = field.ground.at[static_cast<size_t>(p)];
      const int to = field.ground.at[static_cast<size_t>(p) + 1];
      double d = 1e300;
      bool pos = false, neg = false;
      for (int e = from; e < to; ++e) {
        const int f = (e + 1 < to) ? e + 1 : from;
        const double ax = field.ground.vert[static_cast<size_t>(e) * 3];
        const double az = field.ground.vert[static_cast<size_t>(e) * 3 + 2];
        const double ex = field.ground.vert[static_cast<size_t>(f) * 3] - ax;
        const double ez = field.ground.vert[static_cast<size_t>(f) * 3 + 2] - az;
        const double side = ex * (pz - az) - ez * (px - ax);
        if (side > 0) pos = true;
        if (side < 0) neg = true;
        const double len2 = ex * ex + ez * ez;
        double t = len2 > 0 ? ((px - ax) * ex + (pz - az) * ez) / len2 : 0.0;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        const double dx = px - (ax + ex * t), dz = pz - (az + ez * t);
        d = std::min(d, std::sqrt(dx * dx + dz * dz));
      }
      if (!(pos && neg)) d = 0.0;
      best = std::min(best, d);
    }
    const double got = field.distance(px, pz);
    worst = std::max(worst, std::fabs(got - best));
    if (std::fabs(got - best) > 1e-9) agrees = false;
  }
  ok(agrees,
     "RED: MUTATION - the index answers what a walk of every polygon answers, at four hundred "
     "places on the floor, in the hole and off every edge of it - a walk that stopped early "
     "would be a goal that is nearer than it is");
}

void engine_tests() {
  const tww_engine::RoomDzb floor = tww_engine::flat_floor_dzb(0.0f);

  tww_engine::Init init;
  init.pos.set(0.0f, 0.0f, 0.0f);
  init.proc = daPy_lk_c::daPyProc_WAIT_e;

  tww_engine::Session session(init, &floor);
  const tww_engine::StepResult r = session.step(tww_engine::Pad());

  ok(r == tww_engine::StepResult::Ok, "a frame runs through tww_engine from inside this binary");
  ok(session.state().current.pos.y == 0.0f,
     "and the floor holds a standstill where it was put, which is the descent's own answer");

  /* RED: Ground::None is the game's GRND_NONE. */
  tww_engine::RunOptions falling;
  falling.ground = tww_engine::RunOptions::Ground::None;
  tww_engine::Session no_floor(init, &floor, falling);
  no_floor.step(tww_engine::Pad());
  ok(no_floor.state().current.pos.y < 0.0f,
     "with the ground stage off he falls, so the check above is about the floor");
}

void catalogue_tests() {
  const std::vector<search::Move> all = search::roster(0);
  ok(all.size() == 974, "the roster is 974 moves - the twenty-four named plus 950 sword combos");

  std::set<std::string> ids;
  for (size_t i = 0; i < all.size(); ++i) ids.insert(all[i].id);
  ok(ids.size() == all.size(), "and every id in it is distinct");

  /* Must match the window's `moves.ts`, in order. */
  const char* named[24] = {"jumpslash",       "jumpslash_qs",    "backflip",
                           "backflip_qs",     "sidehop_l",       "sidehop_r",
                           "dry_roll",        "dry_roll_r",
                           "dry_roll_r_free", "target_slash_r",  "target_slash",
                           "target_slash_qs", "neutral_slash_r", "neutral_slash",
                           "crawl",           "crawl_r",
                           "fine_turn",       "cup_cdown_turnaround", "cup_b_settle_turnaround",
                           "cup_b_early_turnaround", "l_cdown_turnaround",
                           "ess_up_turn", "ess_left_turn", "ess_right_turn"};
  bool in_order = true;
  for (int i = 0; i < 24; ++i) {
    if (all[i].id != named[i]) in_order = false;
  }
  ok(in_order, "and the first twenty-four are moves.ts's own ids, in its own order");

  int combo = 0, combo_none = 0, combo_rest = 0;
  for (size_t i = 24; i < all.size(); ++i) {
    if (all[i].type == "combo") ++combo;
    if (all[i].type == "combo_none") ++combo_none;
    if (all[i].type == "combo_rest") ++combo_rest;
  }
  ok(combo == 127 && combo_none == 193 && combo_rest == 630,
     "the combos split 127 / 193 / 630 the way moves.ts's three literals do");

  /* The ender ignores the lock, so NtNtNtN is NtNtNtNt and NtN is NtLt. */
  ok(ids.count("NtNtNtNt") == 1 && ids.count("NtNtNtN") == 0 && ids.count("NtLt") == 1 &&
         ids.count("NtN") == 0,
     "and a move that can be spelled locked on every press is kept under that spelling");
  int targeted_four = 0;
  for (size_t i = 24; i < all.size(); ++i) {
    if (all[i].type == "combo" && all[i].presses == 4) ++targeted_four;
  }
  ok(targeted_four == 64, "and the targeted group holds 64 four-press combos");

  ok(ids.count("NL") == 1 && ids.count("NR") == 1 && ids.count("NU") == 1 && ids.count("ND") == 1,
     "and the four untargeted directions survive the dedup as four distinct moves");

  /* `all` has no camera, so a move pressed on a camera cardinal cannot be aimed. */
  bool shapes = true;
  int unaimed = 0;
  for (size_t i = 0; i < all.size(); ++i) {
    const bool stated = all[i].is_stated();
    if (all[i].unaimed) {
      ++unaimed;
      if (stated || !all[i].action.empty()) shapes = false;
      continue;
    }
    if (stated != all[i].action.empty()) shapes = false;
    if (stated != (all[i].rotates != 0)) shapes = false;
  }
  ok(shapes, "every move is pressed for, priced by hand, or could not be aimed, and never two of "
             "the three");
  ok(unaimed == 7,
     "and with no camera to read it against, exactly seven rows cannot be aimed - the three ways "
     "out of the C up view, the held-L chain and the three ESS turns");

  search::Move crawl_r;
  ok(search::move_of("crawl_r", 0, &crawl_r), "crawl_r is in the roster");
  bool re_press = false;
  for (size_t i = 1; i + 1 < crawl_r.action.size(); ++i) {
    if (crawl_r.action[i - 1].buttons != 0 && crawl_r.action[i].buttons == 0 &&
        crawl_r.action[i + 1].buttons != 0) {
      re_press = true;
    }
  }
  ok(re_press, "and its column releases a button and presses it again, which is a rising edge");

  search::Move crawl;
  search::move_of("crawl", 0, &crawl);
  bool plain_re_press = false;
  for (size_t i = 1; i + 1 < crawl.action.size(); ++i) {
    if (crawl.action[i - 1].buttons != 0 && crawl.action[i].buttons == 0 &&
        crawl.action[i + 1].buttons != 0) {
      plain_re_press = true;
    }
  }
  ok(!plain_re_press, "RED: the plain crawl has no such release, so the check above is asking");

  const tww_engine::RoomDzb floor = tww_engine::flat_floor_dzb(0.0f);
  tww_engine::Init init;
  init.pos.set(0.0f, 0.0f, 0.0f);
  init.proc = daPy_lk_c::daPyProc_WAIT_e;

  search::Move backflip;
  search::move_of("backflip", 0, &backflip);
  const search::Drive flip = search::drive(backflip, init, &floor);
  ok(flip.ok && flip.rested, "the backflip runs every frame and comes back to a standstill");
  ok(flip.frames == 22,
     "and costs 22 frames, which is the number moves.ts carries off the Python sim");
  ok(flip.frames == flip.engine_frames + search::kInputDelay,
     "which is the engine's count plus the two frames of controller latency it does not model");
  ok(flip.dz < -269.0 && flip.dz > -271.0 && flip.facing_out == flip.facing_in,
     "and it travels about 270 units opposite the facing, leaving the facing alone");

  /* procCrawlEnd's bail reads spActionButton, so the re-press rides item_button and the edge is
     not tested here. */
  const search::Drive slow = search::drive(crawl, init, &floor);
  const search::Drive quick = search::drive(crawl_r, init, &floor);
  ok(slow.frames == 39 && quick.frames == 40,
     "the two crawls cost 39 and 40 frames, which is what moves.ts carries for them");
  ok(quick.dz > slow.dz + 10.0,
     "and the re-pressed one lands about 11 units further, so its second R press reached the game");

  /* An unported dispatch arm leaves Link standing with ok and rested set; it must be named. */
  search::Move slash;
  search::move_of("target_slash", 0, &slash);
  const search::Drive cut = search::drive(slash, init, &floor);
  ok(cut.ok && cut.rested, "the target slash runs every frame and comes back to a standstill");
  ok(cut.dispatch_calls == 0 && cut.dz > 1.0,
     "and it moves");

  bool arms_honest = true;
  const std::vector<search::Move> roster = search::roster(0);
  std::map<std::string, search::Drive> driven;
  for (size_t i = 0; i < roster.size(); ++i) {
    if (!roster[i].blocked.empty() || roster[i].is_stated()) continue;
    const search::Drive d = search::drive(roster[i], init, &floor);
    driven[roster[i].id] = d;
    if (d.dispatch_calls > 0 && d.dx == 0.0 && d.dz == 0.0 && d.ok && d.rested) {
      if (d.dispatch_arm == 0) arms_honest = false;
    }
  }
  ok(arms_honest,
     "and any move that stands still while reporting ok names the arm that swallowed it");

  /* R held past a slash's exit is read by the standstill as a crouch. */
  int with_r = 0, early = 0;
  for (std::map<std::string, search::Drive>::const_iterator it = driven.begin();
       it != driven.end(); ++it) {
    const std::string& id = it->first;
    std::string plain;
    if (id.size() > 2 && id.compare(id.size() - 2, 2, "+R") == 0) {
      plain = id.substr(0, id.size() - 2);
    } else if (id == "target_slash_r" || id == "neutral_slash_r") {
      plain = id.substr(0, id.size() - 2);
    } else {
      continue;
    }
    const std::map<std::string, search::Drive>::const_iterator without = driven.find(plain);
    if (without == driven.end()) continue;
    ++with_r;
    bool crouched = false;
    for (size_t p = 0; p < it->second.procs.size(); ++p) {
      if (it->second.procs[p] == daPy_lk_c::daPyProc_CROUCH_e) crouched = true;
    }
    if (it->second.rested && !crouched && it->second.frames < without->second.frames) ++early;
  }
  ok(with_r == 148, "every move that ends a slash with R is driven beside the same move without it");
  ok(early == with_r,
     "and each one rests sooner than without R, and never crouches on an R held past the exit");

  /* No RED control: stepping a blocked move hits JUT_ASSERT and aborts the process. */
  search::Move qs;
  search::move_of("backflip_qs", 0, &qs);
  const search::Drive spun = search::drive(qs, init, &floor);
  ok(spun.blocked && spun.engine_frames == 0,
     "the landing quickspin is not stepped, because tww_engine aborts four frames into CUT_TURN");
  ok(!qs.blocked.empty(),
     "and it carries the reason, so the flag cannot outlive the thing that put it there");

  search::Move fine;
  search::move_of("fine_turn", 0, &fine);
  const search::Drive turn = search::drive(fine, init, &floor);
  ok(turn.frames == 46 && turn.engine_frames == 0 && !turn.ok,
     "the C up turn reports its stated 46 frames and no driven frame at all");
  ok(turn.facing_out == 655 && fine.stands == 200,
     "and it rotates 655, standing for the 200 turns moves.ts says it does - a hundred steps "
     "each way, and 655 times a hundred is 65,500 rather than the whole circle, so the two "
     "directions reach 200 facings between them and never the same one twice");
}

void approx_tests() {
  const search::BaseTable base = search::base_table(false);

  bool accounted = true;
  for (size_t i = 0; i < base.move.size(); ++i) {
    if (base.move[i].driven == base.move[i].why.empty()) continue;
    accounted = false;
  }
  ok(accounted, "every move in the base table is either driven or carries why it was not");
  ok(base.driven + base.not_driven == static_cast<int>(base.move.size()),
     "and the two counts add up to the roster, so nothing was skipped without being counted");

  bool sums = true;
  for (size_t i = 0; i < base.move.size(); ++i) {
    const search::BaseMove& m = base.move[i];
    if (!m.driven) continue;
    double ahead = 0, side = 0;
    for (size_t f = 0; f < m.step.size(); ++f) {
      ahead += m.step[f].ahead;
      side += m.step[f].side;
    }
    if (std::fabs(ahead - m.ahead) > 1e-9 || std::fabs(side - m.side) > 1e-9) sums = false;
  }
  ok(sums, "and every driven move's frames sum to the net that run reported");

  {
    /* Every cell marked and no ground: Solid hands every move off, None must not read it. */
    const search::Slab hostile = search::slab(0.0, 0.0, 0.0, 8192.0);
    const search::Corridor all = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
    search::Selection none_sel;
    const search::Grid hostile_grid = search::build(none_sel, all, search::Limits());
    (void)hostile;

    const search::Calibration cal = search::Calibration::measured();
    int driven = 0;
    bool arithmetic = true, clean = true, priced = true;
    for (size_t i = 0; i < base.move.size(); ++i) {
      const search::BaseMove& row = base.move[i];
      if (!row.driven) continue;
      ++driven;
      /* Off the quarter marks, where cM_ssin and std::sin differ. */
      const int facing = 0x1B6A;
      const search::Stepped got =
          search::step_move(row, hostile_grid, none_sel, cal, 100.0, 7.0, -250.0, facing,
                            search::Collision::None);
      /* Summed per frame in order and rotated by the game's cM_ssin/cM_scos, as the engine is. */
      const double sin_f = static_cast<double>(cM_ssin(static_cast<s16>(facing)));
      const double cos_f = static_cast<double>(cM_scos(static_cast<s16>(facing)));
      double want_x = 100.0, want_z = -250.0;
      for (size_t f = 0; f < row.step.size(); ++f) {
        want_x += row.step[f].ahead * sin_f + row.step[f].side * cos_f;
        want_z += row.step[f].ahead * cos_f - row.step[f].side * sin_f;
      }
      if (!got.ok || got.x != want_x || got.z != want_z || got.y != 7.0) arithmetic = false;
      if (got.handed_off || got.left || got.why != 0 || got.sloped_frames != 0 ||
          got.wall_near || got.air_landing) {
        clean = false;
      }
      if (got.frames != row.frames) priced = false;
    }
    ok(driven > 0, "there are driven rows to ask about with the room left out");
    ok(arithmetic,
       "with collision None a move is the base table applied at the facing, summed the same way");
    ok(clean, "and nothing is handed off, because the two things a room does to a move were never read");
    ok(priced, "and a move costs the frames the table measured, the launches included");

    {
      double worst = 0.0;
      int worst_at = 0;
      for (int f = 0; f < 65536; f += 7) {
        const double rad = static_cast<double>(f) * 2.0 * 3.14159265358979323846 / 65536.0;
        const double apart = std::fabs(std::sin(rad) -
                                       static_cast<double>(cM_ssin(static_cast<s16>(f))));
        if (apart > worst) {
          worst = apart;
          worst_at = f;
        }
      }
      (void)worst_at;
      ok(worst > 0.0,
         "RED: the game's own sine and the library's are different numbers, so which one the model "
         "rotates by decides where a move lands");
      ok(worst * base.widest > 1e-3,
         "and over the widest move's reach that difference is more than a thousandth of a unit - "
         "which at a tolerance of zero is hundreds of thousands of float steps");

      search::Question aimed;
      aimed.aim = search::Aim::Overhead;
      bool aims_by_the_game = true;
      for (int f = 0; f < 65536; f += 1021) {
        double ax = 0, az = 0;
        search::aim_point(aimed, 0.0, 0.0, f, &ax, &az);
        const double s = static_cast<double>(cM_ssin(static_cast<s16>(f)));
        const double c = static_cast<double>(cM_scos(static_cast<s16>(f)));
        if (ax != c * 30.0 + s * 20.0) aims_by_the_game = false;
        if (az != c * 20.0 - s * 30.0) aims_by_the_game = false;
      }
      ok(aims_by_the_game,
         "the item's offset is turned by the game's own sine and cosine, to the bit, the same way "
         "the base table is - a target aimed at what he carries cannot be exact otherwise");
    }

    bool solid_notices = false;
    for (size_t i = 0; i < base.move.size(); ++i) {
      const search::BaseMove& row = base.move[i];
      if (!row.driven) continue;
      const search::Stepped got =
          search::step_move(row, hostile_grid, none_sel, cal, 100.0, 7.0, -250.0, 0x2000,
                            search::Collision::Solid);
      if (got.handed_off || got.left) solid_notices = true;
    }
    ok(solid_notices,
       "RED: and the same room with collision on IS read, so the check above is about the switch");
  }

  /* cM_atan2s reads its table at (int)(ratio * 1024), so a rise under 1/1024 is angle 0 and is
     priced as level. */
  {
    const search::BaseTable& b = base;
    const search::Calibration cal = search::Calibration::measured();
    /* U_GetAtanTable's index width. */
    const double kStep = 1.0 / 1024.0;
    struct Rung {
      double grade;
      bool charged;
    };
    const Rung rung[2] = {{kStep * 0.9, false}, {kStep * 1.6, true}};

    int asked = 0;
    bool level_free = true, rise_charged = true;
    for (int r = 0; r < 2; ++r) {
      const search::Slab world = search::slab(0.0, 0.0, rung[r].grade, 8192.0);
      const search::Corridor around = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
      const search::Selection sel = search::select(world.mesh, around);
      search::Limits open_enough;
      open_enough.slope_normal_y = 0.0;
      open_enough.height_step = 1e9;
      open_enough.normal_apart = 1e9;
      const search::Grid over = search::build(sel, around, open_enough);

      for (size_t i = 0; i < b.move.size(); ++i) {
        const search::BaseMove& row = b.move[i];
        if (!row.driven || row.air_from >= 0) continue;
        const search::MoveCal* mc = cal.of(row.id);
        if (mc == nullptr || mc->up_level > 0.999) continue;
        const search::Stepped got =
            search::step_move(row, over, sel, cal, 0.0, world.at(0.0, 0.0), 0.0, 0,
                              search::Collision::Floors);
        if (!got.ok) continue;
        ++asked;
        if (r == 0 && got.uphill_frames != 0) level_free = false;
        if (r == 1 && got.uphill_frames == 0) rise_charged = false;
      }
    }
    ok(asked > 0, "there are moves the slope correction can be asked about");
    ok(level_free,
       "a rise gentler than the game's own table can see is priced as level ground, because that "
       "is what the game prices it as");
    ok(rise_charged,
       "RED: and a rise past that same boundary IS charged, so the check above is about where the "
       "step is and not about the correction being switched off");
  }

  const search::Slab flat = search::slab(0.0, 0.0, 0.0, 8192.0);
  const search::Corridor wide = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
  const search::Selection flat_sel = search::select(flat.mesh, wide);
  search::Limits open;
  open.slope_normal_y = 0.0;
  open.height_step = 1e9;
  open.normal_apart = 1e9;
  const search::Grid flat_grid = search::build(flat_sel, wide, open);

  double worst_flat = 0.0;
  int flat_rows = 0;
  for (size_t i = 0; i < base.move.size(); ++i) {
    const search::BaseMove& m = base.move[i];
    if (!m.driven) continue;
    const search::Stepped guess =
        search::step_move(m, flat_grid, flat_sel, search::Calibration(), 0.0, 0.0, 0.0, 0);
    const double miss = std::sqrt((guess.x - m.side) * (guess.x - m.side) +
                                  (guess.z - m.ahead) * (guess.z - m.ahead));
    if (miss > worst_flat) worst_flat = miss;
    if (guess.frames != m.frames) worst_flat = 1e9;
    ++flat_rows;
  }
  ok(flat_rows > 0, "the flat replay has moves to run at all, which zero rows would not say");
  ok(worst_flat < 1e-6,
     "and the model reproduces every driven move on level ground, position and frame count");

  /* Not an L-held move: with no target dAttention_c aims at the camera's forward, which this port
     leaves at 0. */
  const search::BaseMove* crawl_row = base.of("crawl");
  ok(crawl_row != nullptr && crawl_row->driven, "the crawl is in the base table and was driven");
  double worst_turn = 0.0;
  for (int f = 1; f < 8 && crawl_row != nullptr && crawl_row->driven; ++f) {
    const int facing = (f * 8192) & 0xFFFF;
    search::Move m;
    search::move_of("crawl", facing, &m);
    tww_engine::Init at;
    at.pos.set(0.0f, 0.0f, 0.0f);
    at.shape_angle_y = static_cast<s16>(facing);
    at.travel_angle_y = static_cast<s16>(facing);
    at.proc = daPy_lk_c::daPyProc_WAIT_e;
    const search::Drive truth = search::drive(m, at, &flat.room);
    const search::Stepped guess = search::step_move(*crawl_row, flat_grid, flat_sel,
                                                    search::Calibration(), 0.0, 0.0, 0.0, facing);
    const double miss = std::sqrt((guess.x - truth.dx) * (guess.x - truth.dx) +
                                  (guess.z - truth.dz) * (guess.z - truth.dz));
    if (miss > worst_turn) worst_turn = miss;
  }
  ok(worst_turn < 0.01,
     "and a move measured once at facing 0 lands where the engine puts it at seven others");

  search::Move crawl_half;
  search::move_of("crawl", 32768, &crawl_half);
  tww_engine::Init back;
  back.pos.set(0.0f, 0.0f, 0.0f);
  back.shape_angle_y = static_cast<s16>(-32768);
  back.travel_angle_y = static_cast<s16>(-32768);
  back.proc = daPy_lk_c::daPyProc_WAIT_e;
  const search::Drive turned = search::drive(crawl_half, back, &flat.room);
  ok(turned.dz < -1.0 && crawl_row != nullptr &&
         std::fabs(turned.dz + crawl_row->ahead) < 0.01,
     "RED: driven the other way round the engine goes the other way, by the same distance");

  const search::Floor under = search::floor_at(flat_grid, flat_sel, 100.0, 100.0, 0.0);
  ok(under.inside && under.found && std::fabs(under.y) < 1e-9,
     "the floor under a point on the slab is the slab, at its own height");
  ok(std::fabs(under.ny - 1.0) < 1e-9, "and its normal is the flat up the grid computed");

  const search::Slab tilt = search::slab(0.0, 0.0, 0.25, 8192.0);
  const search::Selection tilt_sel = search::select(tilt.mesh, wide);
  const search::Grid tilt_grid = search::build(tilt_sel, wide, open);
  const search::Floor up = search::floor_at(tilt_grid, tilt_sel, 0.0, 200.0, 0.0);
  ok(up.found && std::fabs(up.y - tilt.at(0.0, 200.0)) < 0.01,
     "and on a tilted slab it is the plane evaluated at the point, not the cell's own corner");

  const search::Floor away = search::floor_at(flat_grid, flat_sel, 1e7, 1e7, 0.0);
  ok(!away.inside && !away.found,
     "a point the grid does not cover answers outside, which is a counted event and not a floor");

  const search::Stepped clear =
      search::step_move(*crawl_row, flat_grid, flat_sel, search::Calibration(),
                        flat_grid.origin_x + 25.0, 0.0, flat_grid.origin_z + 25.0, 0);
  ok(!clear.handed_off && clear.why == 0,
     "a move over unmarked ground answers for itself, which is what makes the model worth having");

  search::Grid corner = flat_grid;
  const double cell = corner.cell;
  const int cx = corner.nx / 2, cz = corner.nz / 2;
  corner.mark[static_cast<size_t>(corner.at(cx + 1, cz))] = search::kHeight;

  search::BaseMove dash;
  dash.id = "one frame across a corner";
  dash.driven = true;
  dash.frames = 1;
  search::BaseFrame one;
  one.ahead = cell * 0.9;
  one.side = cell * 0.9;
  dash.step.push_back(one);
  dash.ahead = one.ahead;
  dash.side = one.side;

  const double sx = corner.origin_x + (cx + 0.95) * cell;
  const double sz = corner.origin_z + (cz + 0.05) * cell;
  const search::Stepped cut =
      search::step_move(dash, corner, flat_sel, search::Calibration(), sx, 0.0, sz, 0);
  ok((cut.why & search::kHeight) != 0,
     "a frame that only clips a marked cell's corner is handed off, which endpoints alone miss");

  const search::Stepped uncut =
      search::step_move(dash, flat_grid, flat_sel, search::Calibration(), sx, 0.0, sz, 0);
  ok(uncut.why == 0, "RED: with nothing marked the same frame is not handed off");

  geom::Mesh walled = flat.mesh;
  tri(walled.wall, -400.0f, 0.0f, 500.0f, 400.0f, 0.0f, 500.0f, -400.0f, 200.0f, 500.0f);
  const search::Selection wall_sel = search::select(walled, wide);
  ok(wall_sel.wall_held == 1, "the wall the reach test runs against is in the corridor's selection");
  const search::Grid wall_grid = search::build(wall_sel, wide, open);
  ok(!wall_grid.wall_tri.empty() && wall_grid.wall_cells > 0,
     "and the grid bucketed it, so the broad phase has something to hand back");

  /* Inside the body's 35-unit radius. */
  ok(search::wall_in_reach(wall_grid, wall_sel, 0.0, 400.0, 0.0, 490.0, 0.0),
     "a frame ending ten units short of a wall can reach it");
  ok(!search::wall_in_reach(wall_grid, wall_sel, 0.0, 300.0, 0.0, 400.0, 0.0),
     "RED: a frame stopping a hundred units short of it cannot, so the test is not always true");

  ok(search::wall_in_reach(wall_grid, wall_sel, 0.0, 400.0, 0.0, 600.0, 0.0),
     "a frame that passes clean through a wall is caught by the sweep, not by its ends");

  ok(!search::wall_in_reach(wall_grid, wall_sel, 0.0, 400.0, 0.0, 600.0, 1000.0),
     "and a body a thousand units above that wall cannot touch it however close it passes");
  ok(search::wall_in_reach(wall_grid, wall_sel, 0.0, 400.0, 0.0, 600.0, 150.0),
     "RED: at a height that still overlaps it, the same path reaches it again");

  const search::Stepped into_wall = search::step_move(*crawl_row, wall_grid, wall_sel,
                                                      search::Calibration(), 0.0, 0.0, 495.0, 0);
  const search::Stepped away_from = search::step_move(*crawl_row, wall_grid, wall_sel,
                                                      search::Calibration(), 0.0, 0.0, 0.0, 0);
  ok(into_wall.wall_near && into_wall.handed_off,
     "a move whose frames run up to a wall is handed to the engine...");
  ok(!away_from.wall_near && !away_from.handed_off,
     "...and the same move out in the open is not, which is what the fast path is for");

  const search::BaseMove* flip = base.of("backflip");
  ok(flip != nullptr && flip->driven && flip->air_from >= 0,
     "the backflip is driven and the table knows which of its frames are off the ground");
  if (flip != nullptr && flip->driven) {
    const search::Stepped level =
        search::step_move(*flip, flat_grid, flat_sel, search::Calibration(), 0.0, 0.0, 0.0, 0);
    const search::Stepped sloped = search::step_move(
        *flip, tilt_grid, tilt_sel, search::Calibration(), 0.0, tilt.at(0.0, 0.0), 0.0, 0);
    ok(!level.air_landing, "a launch that lands on level ground answers for itself...");
    ok(sloped.air_landing && sloped.handed_off,
       "...and one that lands on a slope goes to the engine, because the frame it ends on is an "
       "inequality");

    /* An arc with no floor under it would fly to the frame cap, and search_tree prunes on frames,
       so it is charged the table's own. The ground ends 100 out; a backflip goes 270. */
    const search::Corridor ends = search::corridor(0.0, 0.0, 0.0, 0.0, 100.0);
    const search::Selection ends_sel = search::select(flat.mesh, ends);
    const search::Grid ends_grid = search::build(ends_sel, ends, open);
    const search::Stepped over_nothing =
        search::step_move(*flip, ends_grid, ends_sel, search::Calibration(), 0.0, 0.0, 0.0, 0);
    ok(over_nothing.ok && over_nothing.handed_off,
       "a launch whose arc runs out of floor is handed to the engine...");
    ok(over_nothing.frames == flip->frames,
       "...and is charged the table's own frames rather than integrated to the cap, because the "
       "frame bound prunes on that number and would drop a real answer");
    /* `reach` is the furthest any frame took him. */
    const double went = std::sqrt(over_nothing.x * over_nothing.x +
                                  over_nothing.z * over_nothing.z);
    ok(went <= flip->reach + 1.0,
       "RED: and it ends no further out than the move's own reach, which is what says the arc "
       "stopped rather than kept flying");
  }

  /* calibration.inc is generated by setupcore_calibrate; these catch it going stale. */
  const search::Calibration stored = search::Calibration::measured();
  ok(!stored.move.empty(), "the stored calibration has rows in it at all");

  bool named = true;
  for (size_t i = 0; i < stored.move.size(); ++i) {
    search::Move m;
    if (!search::move_of(stored.move[i].id, 0, &m)) named = false;
  }
  ok(named, "and every move it names is still in the roster");

  /* threshold() answers 0 for a move with no row, so every travelling move needs one. */
  struct Covers {
    static int missing(const search::BaseTable& t, const search::Calibration& c, int* read) {
      int gap = 0;
      *read = 0;
      for (size_t i = 0; i < t.move.size(); ++i) {
        const search::BaseMove& row = t.move[i];
        if (!row.driven || row.reach <= 1.0) continue;
        ++*read;
        if (c.of(row.id) == nullptr) ++gap;
      }
      return gap;
    }
  };
  int travelling = 0;
  ok(Covers::missing(base, stored, &travelling) == 0 && travelling > 0,
     "and every driven move that travels has a stored calibration row");
  {
    search::Calibration short_one = stored;
    short_one.move.erase(short_one.move.begin());
    int again = 0;
    ok(Covers::missing(base, short_one, &again) > 0,
       "RED: with one row taken out, a travelling move is found without one");
  }

  const double ny_edge = search::Limits().slope_normal_y;
  const double handed_at = std::sqrt(1.0 - ny_edge * ny_edge) / ny_edge;
  ok(stored.ladder >= handed_at,
     "and the stored ladder reaches the grade the grid starts handing moves to the engine at");

  /* The miss is jagged in grade: hold the charge at a grade on neither ladder, off the ring. */
  ok(stored.charge_steps > 10,
     "the stored charge was measured on a denser ladder than the ten rungs that fitted it");
  {
    const search::BaseTable every = search::base_table(true);
    /* The scale model alone, so no plane table answers for it. */
    search::Calibration scale_only = stored;
    scale_only.plane.clear();
    const double between = 0.478478;
    /* On no power-of-two ring. */
    const int off_ring[2] = {1000, 33000};
    std::vector<double> worst(stored.move.size(), 0.0);
    int driven = 0, charged = 0;
    for (int sign = -1; sign <= 1; sign += 2) {
      const search::Slab tilt = search::slab(0.0, 0.0, sign * between, 8192.0);
      const search::Selection tilt_sel = search::select(tilt.mesh, wide);
      const search::Grid tilt_grid = search::build(tilt_sel, wide, open);
      for (int fi = 0; fi < 2; ++fi) {
        const int facing = off_ring[fi];
        const std::vector<search::Move> ring = search::roster(facing, true);
        for (size_t k = 0; k < stored.move.size(); ++k) {
          const search::MoveCal& mc = stored.move[k];
          const search::BaseMove* row = every.of(mc.id);
          if (row == nullptr || !row->driven || row->reach <= 1.0 || mc.rows == 0) continue;
          if (sign < 0 && fi == 0) ++charged;
          const search::Move* m = nullptr;
          for (size_t r = 0; r < ring.size(); ++r) {
            if (ring[r].id == mc.id) m = &ring[r];
          }
          if (m == nullptr) continue;
          tww_engine::Init at;
          at.pos.set(0.0f, static_cast<f32>(tilt.at(0.0, 0.0)), 0.0f);
          at.shape_angle_y = static_cast<s16>(facing);
          at.travel_angle_y = static_cast<s16>(facing);
          at.proc = daPy_lk_c::daPyProc_WAIT_e;
          const search::Drive truth = search::drive(*m, at, &tilt.room);
          if (!truth.ok || !truth.rested || truth.dispatch_calls > 0) continue;
          const search::Stepped guess = search::step_move(*row, tilt_grid, tilt_sel, scale_only,
                                                          at.pos.x, at.pos.y, at.pos.z, facing);
          const double ex = truth.dx - (guess.x - at.pos.x);
          const double ez = truth.dz - (guess.z - at.pos.z);
          worst[k] = std::max(worst[k], std::sqrt(ex * ex + ez * ez));
          ++driven;
        }
      }
    }
    int past = 0, past_short = 0;
    std::string first_past;
    for (size_t k = 0; k < stored.move.size(); ++k) {
      if (worst[k] > stored.move[k].worst) {
        if (first_past.empty()) first_past = stored.move[k].id;
        ++past;
      }
      if (worst[k] > 0.95 * stored.move[k].worst) ++past_short;
    }
    ok(charged > 20 && driven > 0,
       "the table charges more than the named moves, and the engine ran them off the ring");
    ok(past == 0, "and no miss off the ladder and off the ring is past its move's stored charge" +
                      (first_past.empty() ? std::string() : " (first: " + first_past + ")"));
    ok(past_short > 0,
       "RED: with every charge five per cent short, some move's miss is past it");
  }

  const search::MoveCal* roll_cal = stored.of("dry_roll");
  const search::BaseMove* roll_row = base.of("dry_roll");
  ok(roll_cal != nullptr && roll_row != nullptr && roll_row->driven,
     "the roll has a stored calibration and a driven base row");
  if (roll_cal != nullptr && roll_row != nullptr && roll_row->driven) {
    /* The roll travels +z, so this rise is uphill. */
    const search::Slab rise = search::slab(0.0, 0.0, 0.2, 8192.0);
    const search::Selection rise_sel = search::select(rise.mesh, wide);
    const search::Grid rise_grid = search::build(rise_sel, wide, open);
    search::Move m;
    search::move_of("dry_roll", 0, &m);
    tww_engine::Init at;
    at.pos.set(0.0f, static_cast<f32>(rise.at(0.0, 0.0)), 0.0f);
    at.proc = daPy_lk_c::daPyProc_WAIT_e;
    const search::Drive truth = search::drive(m, at, &rise.room);
    const search::Stepped guess = search::step_move(*roll_row, rise_grid, rise_sel, stored,
                                                    at.pos.x, at.pos.y, at.pos.z, 0);
    ok(truth.ok && truth.rested, "and it runs uphill in the engine at all");
    ok(std::fabs((guess.z - at.pos.z) - truth.dz) <= roll_cal->worst + 0.01,
       "and the stored coefficients put it where the engine does, inside their own recorded error");

    const search::Stepped raw = search::step_move(*roll_row, rise_grid, rise_sel,
                                                  search::Calibration(), at.pos.x, at.pos.y,
                                                  at.pos.z, 0);
    ok(std::fabs((raw.z - at.pos.z) - truth.dz) > roll_cal->worst + 1.0,
       "RED: with no calibration the same roll misses by more, so the fit is doing work");

    /* Stairs (ground code 8), which the game prices as level. */
    const search::Slab stairs = search::slab(0.0, 0.0, 0.2, 8192.0, geom::kGroundStairs);
    const search::Selection stairs_sel = search::select(stairs.mesh, wide);
    const search::Grid stairs_grid = search::build(stairs_sel, wide, open);
    const search::Drive climbed = search::drive(m, at, &stairs.room);
    const search::Stepped on_stairs = search::step_move(*roll_row, stairs_grid, stairs_sel, stored,
                                                        at.pos.x, at.pos.y, at.pos.z, 0);
    ok(climbed.ok && climbed.rested && std::fabs(climbed.dz - truth.dz) > 1.0,
       "a roll up stairs runs in the engine, and not as far as the same rise that is not stairs");
    ok(on_stairs.stairs_frames > 0 &&
           std::fabs((on_stairs.x - at.pos.x) - climbed.dx) <= 1e-3 &&
           std::fabs((on_stairs.z - at.pos.z) - climbed.dz) <= 1e-3,
       "and the model puts it where the engine does");

    search::Selection unmarked = stairs_sel;
    unmarked.ground_code.clear();
    const search::Stepped as_slope = search::step_move(*roll_row, stairs_grid, unmarked, stored,
                                                       at.pos.x, at.pos.y, at.pos.z, 0);
    ok(as_slope.stairs_frames == 0 && std::fabs((as_slope.z - at.pos.z) - climbed.dz) > 1.0,
       "RED: priced as a slope, the same roll falls short of the engine's");
  }

  ok(stored.threshold("dry_roll", 0) == 0.0,
     "a move with no frame over sloped ground is owed no allowance at all");
  if (roll_cal != nullptr && roll_cal->frames > 1) {
    const double half = stored.threshold("dry_roll", roll_cal->frames / 2);
    const double all = stored.threshold("dry_roll", roll_cal->frames);
    ok(all > half && half > 0.0,
       "and one that spent half its frames on a slope is owed less than one that spent all of "
       "them");
    ok(std::fabs(all - roll_cal->worst) < 1e-9,
       "with the whole allowance being the error the ladder actually measured");
  }
  ok(stored.threshold("a move nobody has measured", 20) == 0.0,
     "RED: a move with no stored row is owed nothing rather than some other move's number");

  /* `drive` lends each thread one room, keyed on the data it was built from. */
  {
    search::Move m;
    search::move_of("dry_roll", 0, &m);
    const search::Slab one = search::slab(0.0, 0.0, 0.2, 8192.0);
    const search::Slab two = search::slab(0.0, 0.0, -0.3, 8192.0);
    struct At {
      static tww_engine::Init on(const search::Slab& s) {
        tww_engine::Init at;
        at.pos.set(0.0f, static_cast<f32>(s.at(0.0, 0.0)), 0.0f);
        at.proc = daPy_lk_c::daPyProc_WAIT_e;
        return at;
      }
    };
    const search::Drive first = search::drive(m, At::on(one), &one.room);
    const search::Drive other = search::drive(m, At::on(two), &two.room);
    const search::Drive again = search::drive(m, At::on(one), &one.room);
    ok(first.ok && other.ok && again.ok, "the roll runs on both slopes");
    ok(again.dx == first.dx && again.dy == first.dy && again.dz == first.dz &&
           again.frames == first.frames,
       "and driven again in the room it was lent, it ends exactly where the first drive did");
    ok(other.dz != first.dz,
       "RED: and in another room it is that room's drive, not the lent one's");

    /* Overwritten in place: same storage and sizes, other data. */
    tww_engine::RoomDzb reused = one.room;
    const search::Drive before = search::drive(m, At::on(one), &reused);
    const void* storage = reused.v_tbl.data();
    bool same_sizes = reused.v_tbl.size() == two.room.v_tbl.size() &&
                      reused.t_tbl.size() == two.room.t_tbl.size() &&
                      reused.b_tbl.size() == two.room.b_tbl.size() &&
                      reused.tree_tbl.size() == two.room.tree_tbl.size() &&
                      reused.g_tbl.size() == two.room.g_tbl.size() &&
                      reused.ti_tbl.size() == two.room.ti_tbl.size();
    if (same_sizes) {
      std::copy(two.room.v_tbl.begin(), two.room.v_tbl.end(), reused.v_tbl.begin());
      std::copy(two.room.t_tbl.begin(), two.room.t_tbl.end(), reused.t_tbl.begin());
      std::copy(two.room.b_tbl.begin(), two.room.b_tbl.end(), reused.b_tbl.begin());
      std::copy(two.room.tree_tbl.begin(), two.room.tree_tbl.end(), reused.tree_tbl.begin());
      std::copy(two.room.g_tbl.begin(), two.room.g_tbl.end(), reused.g_tbl.begin());
      std::copy(two.room.ti_tbl.begin(), two.room.ti_tbl.end(), reused.ti_tbl.begin());
    }
    const search::Drive after = search::drive(m, At::on(two), &reused);
    /* On a thread with no lent room yet. */
    search::Drive fresh;
    std::thread alone([&]() { fresh = search::drive(m, At::on(two), &two.room); });
    alone.join();
    ok(same_sizes && storage == reused.v_tbl.data() && before.dz == first.dz,
       "a room overwritten in place keeps its storage and its sizes, which is the case this is "
       "about");
    ok(after.dx == fresh.dx && after.dz == fresh.dz && after.frames == fresh.frames,
       "RED: and a drive in it is the new room's, not the previous room's");
  }

  if (roll_row != nullptr && roll_row->driven && roll_row->reach > 1.0) {
    search::Calibration without = stored;
    for (size_t i = 0; i < without.move.size(); ++i) {
      if (without.move[i].id == "dry_roll") {
        without.move.erase(without.move.begin() + static_cast<long>(i));
        break;
      }
    }
    const search::Slab rise = search::slab(0.0, 0.0, 0.2, 8192.0);
    const search::Selection rise_sel = search::select(rise.mesh, wide);
    const search::Grid rise_grid = search::build(rise_sel, wide, open);
    const double y0 = rise.at(0.0, 0.0);
    const search::Stepped kept =
        search::step_move(*roll_row, rise_grid, rise_sel, stored, 0.0, y0, 0.0, 0);
    const search::Stepped bare =
        search::step_move(*roll_row, rise_grid, rise_sel, without, 0.0, y0, 0.0, 0);
    ok(kept.sloped_frames > 0 && !kept.handed_off,
       "a calibrated roll over a slope is the model's own");
    ok(bare.handed_off && bare.unmeasured,
       "RED: the same roll with its row taken out goes to the engine, for having none");

    const search::Slab flat = search::slab(0.0, 0.0, 0.0, 8192.0);
    const search::Selection flat_sel = search::select(flat.mesh, wide);
    const search::Grid flat_grid = search::build(flat_sel, wide, open);
    const search::Stepped level =
        search::step_move(*roll_row, flat_grid, flat_sel, without, 0.0, flat.at(0.0, 0.0), 0.0, 0);
    ok(level.sloped_frames == 0 && !level.handed_off,
       "and on level ground a move with no row is still the model's, owed nothing and exact");
  }
}

void search_tests() {
  const search::BaseTable base = search::base_table(false);

  const search::Slab flat = search::slab(0.0, 0.0, 0.0, 8192.0);
  const search::Corridor wide = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
  const search::Selection flat_sel = search::select(flat.mesh, wide);
  search::Limits open;
  open.slope_normal_y = 0.0;
  open.height_step = 1e9;
  open.normal_apart = 1e9;
  const search::Grid flat_grid = search::build(flat_sel, wide, open);

  int turn_rows = 0, driven_turn_rows = 0;
  for (size_t i = 0; i < base.move.size(); ++i) {
    if (search::turn_steps(base.move[i]) > 0) ++turn_rows;
    if (base.move[i].driven && search::turn_steps(base.move[i]) > 0) ++driven_turn_rows;
  }
  ok(turn_rows == 1, "exactly one row of the table turns without being pressed for");
  ok(driven_turn_rows == 0,
     "RED: a driven row is never taken for one, however many frames it spends turning");
  const search::BaseMove* fine = base.of("fine_turn");
  ok(fine != nullptr && search::turn_steps(*fine) == fine->stands,
     "and it stands for as many steps as the catalogue says it does");

  search::Question all_moves;
  const std::vector<int> every = search::rows_for(all_moves, base);
  ok(static_cast<int>(every.size()) == base.driven + turn_rows,
     "an empty move list is every row the table can step, and nothing it cannot");
  bool every_steppable = true;
  for (size_t i = 0; i < every.size(); ++i) {
    const search::BaseMove& row = base.move[static_cast<size_t>(every[i])];
    if (!row.driven && search::turn_steps(row) == 0) every_steppable = false;
  }
  ok(every_steppable, "and every row it offers is one with a measurement or a stated price");

  search::Question named;
  named.moves.push_back("crawl");
  named.moves.push_back("jumpslash_qs");
  ok(search::rows_for(named, base).size() == 1,
     "a question naming two moves is offered only the one the table can step");
  const std::vector<std::string> left_out = search::unstepped_for(named, base);
  ok(left_out.size() == 1 && left_out[0] == "jumpslash_qs",
     "RED: and the other is named rather than dropped, so a move the person chose and the search "
     "never took cannot go unsaid");
  ok(search::unstepped_for(all_moves, base).empty(),
     "RED: a question that names no move leaves none of them out");

  const double rate = search::fastest_rate(base, every, flat_grid);
  bool rate_bounds = true;
  for (size_t i = 0; i < every.size(); ++i) {
    const search::BaseMove& row = base.move[static_cast<size_t>(every[i])];
    if (!row.driven || row.frames <= 0) continue;
    if (row.reach / row.frames > rate + 1e-9) rate_bounds = false;
  }
  ok(rate > 0.0 && rate_bounds,
     "the bound's rate is at least as fast as every chosen move's own reach a frame");
  std::vector<int> crawl_only;
  for (size_t i = 0; i < base.move.size(); ++i) {
    if (base.move[i].id == "crawl") crawl_only.push_back(static_cast<int>(i));
  }
  ok(search::fastest_rate(base, crawl_only, flat_grid) < rate,
     "RED: and it is the fastest move's rather than any move's, so a slow move list answers less");

  search::Question near_q;
  near_q.tolerance = 10.0;
  search::Question far_q;
  far_q.tolerance = 200.0;
  const search::Quanta tight = search::quanta_for(near_q, base, every);
  const search::Quanta loose = search::quanta_for(far_q, base, every);
  ok(tight.cell < loose.cell && tight.facing <= loose.facing,
     "a tighter tolerance quantises the state more finely, because the quantum is derived from it");
  ok(tight.cell <= near_q.tolerance / std::sqrt(2.0) + 1e-12,
     "and a cell's diagonal never exceeds the tolerance, so two answers a person can tell apart "
     "are never merged");
  search::Question exact_q;
  exact_q.tolerance = 0.0;
  ok(search::quanta_for(exact_q, base, every).facing == 1,
     "RED: an exact-hit question buckets no two facings together at all");
  /* At tolerance 0 the cell is a float step, subnormal at the origin; it must still index a room. */
  ok(8192.0 / search::quanta_for(exact_q, base, every).cell < 4.0e18,
     "RED: an exact-hit question from the origin still gives every place in a room its own cell");

  /* Only the room's camera check reads the camera a move starts from. Without it, two B Settle
     turnarounds and one ESS Up Turn that end on one facing are one state, and the cheaper wins. */
  ok(!search::quanta_for(exact_q, base, every).camera,
     "RED: with no camera check the camera is not part of the signature");
  const search::CamField some_room;
  search::Question checked_q = exact_q;
  checked_q.camera_clear = &some_room;
  ok(search::quanta_for(checked_q, base, every).camera,
     "RED: with one it is, because the check reads the camera a move starts from");

  /* The table grows with the memory ceiling and never reads the thread count (D13). */
  search::Question roomy = exact_q;
  roomy.memory = 8LL << 30;
  search::Question roomy_wide = roomy;
  roomy_wide.cores = 8;
  const size_t few = search::quanta_for(exact_q, base, every).slots;
  const size_t many = search::quanta_for(roomy, base, every).slots;
  ok(few == (size_t(1) << 14) && many > few && (many & (many - 1)) == 0,
     "RED: a larger memory ceiling gives a larger dominance table, a power of two");
  ok(search::quanta_for(roomy_wide, base, every).slots == many,
     "RED: and the thread count does not change it");

  /* Sized off the roll's measurement so the question stays answerable as the table changes. */
  const search::BaseMove* roll = base.of("dry_roll");
  ok(roll != nullptr && roll->driven && roll->frames > 0,
     "the roll is driven, which is what the two questions below are sized off");
  if (roll == nullptr || !roll->driven) return;

  search::Question q;
  q.start_facing = 0;
  q.target.x = 2.0 * roll->side;
  q.target.z = 2.0 * roll->ahead;
  q.tolerance = 20.0;
  q.frames = 3 * roll->frames;
  q.moves.push_back("crawl");
  q.moves.push_back("crawl_r");
  q.moves.push_back("dry_roll");
  q.moves.push_back("dry_roll_r");
  const search::Found run = search::search_tree(q, base, flat_grid, flat_sel,
                                                search::Calibration::measured());
  ok(run.exhausted, "a search inside its node guard explores its whole cost bound");
  ok(!run.candidate.empty(), "and this question has candidates at all, which zero would not say");
  ok(run.count.expanded > 0 && run.count.generated >= run.count.expanded,
     "with more states generated than expanded, which is what a dominance test does");

  /* Session::built() counts every engine run in the process; the walk may add only the camera
     tapes of the rows it reaches (`cup_tape::runs`). base_table drives the engine, so it is built
     before the count. */
  {
    search::Question walk;
    walk.start_facing = 0;
    /* Without a camera the turnarounds are unaimed and never stepped. */
    walk.has_camera = true;
    walk.start_camera = 0;
    walk.target.x = 2.0 * roll->side;
    walk.target.z = 2.0 * roll->ahead;
    walk.tolerance = 20.0;
    walk.frames = 3 * roll->frames;
    walk.moves.push_back("dry_roll");
    walk.moves.push_back("cup_b_settle_turnaround");
    walk.moves.push_back("l_cdown_turnaround");
    walk.moves.push_back("fine_turn");

    const std::vector<int> offered = search::rows_for(walk, base);
    ok(offered.size() == 4,
       "the walk's own question is offered all four rows, the two turnarounds and the turn among "
       "them, so what it measures is a walk that had every chance to consult");

    const unsigned long long before = tww_engine::Session::built();
    const unsigned long long tapes_before = cup_tape::runs();
    const search::Found walked = search::search_tree(walk, base, flat_grid, flat_sel,
                                                     search::Calibration::measured());
    const unsigned long long after = tww_engine::Session::built();
    const unsigned long long tapes = cup_tape::runs() - tapes_before;

    ok(walked.count.expanded > 0 && walked.count.generated > walked.count.expanded,
       "the walk that is being counted really walked, so zero engine runs is a measurement and "
       "not an empty loop");
    ok(after - before == tapes,
       "RED: and it asked the engine for no move - only the camera tapes of the rows it reached, " +
           std::to_string(tapes) + " of them, not a single frame of a single move");
  }

  search::Question nowhere = q;
  nowhere.tolerance = 0.0;
  /* Near enough to walk towards, on nothing these moves combine to exactly. */
  nowhere.target.x = q.target.x + 0.37;
  const search::Found lost = search::search_tree(nowhere, base, flat_grid, flat_sel,
                                                 search::Calibration::measured());
  ok(lost.candidate.empty(),
     "a target nothing can reach answers no candidate at all, which is the case the closest set "
     "exists for");
  ok(!lost.closest.empty(),
     "RED: and it still answers the closest the walk reached, so a run that gets nowhere is not an "
     "empty table");
  ok(lost.count.kept_closest == static_cast<long long>(lost.closest.size()),
     "with the count saying how many of them there are rather than being written after the fact");
  bool walked_to = true, ordered = true;
  for (size_t i = 0; i < lost.closest.size(); ++i) {
    if (lost.closest[i].path.empty()) walked_to = false;
    if (i > 0 && lost.closest[i].distance < lost.closest[i - 1].distance) ordered = false;
  }
  ok(walked_to,
     "each one carries the moves that reached it, because a landing with no plan under it is a row "
     "nobody can drive");
  ok(ordered, "and they come back closest first, so the twenty kept are the twenty nearest");

  const search::Verified said_lost = search::verify(lost, nowhere, base, &flat.room);
  ok(said_lost.closest.size() == lost.closest.size(),
     "every one the walk kept is driven through the engine, because the landing a person reads is "
     "never the model's guess at one");
  long long ran = 0;
  for (size_t i = 0; i < said_lost.closest.size(); ++i) {
    if (said_lost.closest[i].outcome != search::Outcome::CouldNotRun) ++ran;
  }
  ok(ran > 0,
     "RED: and the engine ran at least one of them to the end - every one could-not-run is a plan "
     "list with nothing in it however full the walk's own set was");

  {
    search::Question open_box = q;
    open_box.tolerance = 0.0;
    const search::Found wide_run = search::search_tree(open_box, base, flat_grid, flat_sel,
                                                       search::Calibration::measured());

    /* A box no roll can stay inside. */
    const double reach = std::sqrt(roll->side * roll->side + roll->ahead * roll->ahead);
    search::Question boxed = open_box;
    boxed.bounds.has_xmin = boxed.bounds.has_xmax = true;
    boxed.bounds.has_zmin = boxed.bounds.has_zmax = true;
    boxed.bounds.xmin = q.start_x - reach / 4.0;
    boxed.bounds.xmax = q.start_x + reach / 4.0;
    boxed.bounds.zmin = q.start_z - reach / 4.0;
    boxed.bounds.zmax = q.start_z + reach / 4.0;
    const search::Found boxed_run = search::search_tree(boxed, base, flat_grid, flat_sel,
                                                        search::Calibration::measured());

    ok(wide_run.count.generated > 0,
       "there are states to prune before the box is asked to prune any");
    ok(boxed_run.count.outside_bounds > 0,
       "a state a move put outside the box is counted as refused for being outside it");
    ok(boxed_run.count.expanded < wide_run.count.expanded,
       "RED: and the walk with the box explores less than the walk without one, which is the "
       "whole of what a prune is");
    ok(boxed_run.candidate.empty(),
       "a box nothing can move inside answers nothing, rather than answering off ground that was "
       "ruled out");

    search::Question roomy = open_box;
    roomy.bounds.has_xmin = roomy.bounds.has_xmax = true;
    roomy.bounds.xmin = q.start_x - 1e9;
    roomy.bounds.xmax = q.start_x + 1e9;
    const search::Found roomy_run = search::search_tree(roomy, base, flat_grid, flat_sel,
                                                        search::Calibration::measured());
    ok(roomy_run.count.outside_bounds == 0 &&
       roomy_run.count.expanded == wide_run.count.expanded,
       "RED: and a box wide enough to hold the whole walk refuses nothing and explores the same "
       "states, so the prune answers the box and not its own presence");
  }

  {
    search::Question him = q;
    him.tolerance = 0.0;
    him.aim = search::Aim::Player;
    search::Question held = him;
    held.aim = search::Aim::Overhead;

    double hx = 0, hz = 0, ix = 0, iz = 0;
    search::aim_point(him, 100.0, -250.0, 0x1B6A, &hx, &hz);
    search::aim_point(held, 100.0, -250.0, 0x1B6A, &ix, &iz);
    ok(hx == 100.0 && hz == -250.0,
       "aimed at him, what the target is measured from is where he is standing");
    const double apart = std::sqrt((ix - hx) * (ix - hx) + (iz - hz) * (iz - hz));
    ok(apart > 35.0 && apart < 37.0,
       "RED: and aimed at the item it is the 36 units away the game hangs it at, so the two aims "
       "are different questions");

    const search::Found run_held = search::search_tree(held, base, flat_grid, flat_sel,
                                                       search::Calibration::measured());
    const search::Verified said_held = search::verify(run_held, held, base, &flat.room);
    bool moved = !said_held.closest.empty(), judged = true;
    for (size_t i = 0; i < said_held.closest.size(); ++i) {
      const search::Consult& one = said_held.closest[i];
      if (one.outcome == search::Outcome::CouldNotRun) continue;
      const double off = std::sqrt((one.aim_x - one.x) * (one.aim_x - one.x) +
                                   (one.aim_z - one.z) * (one.aim_z - one.z));
      if (off < 35.0) moved = false;
      if (one.distance != held.target.distance(one.aim_x, one.aim_z)) judged = false;
      if (!one.stop.empty()) {
        const search::Reached& last = one.stop[one.stop.size() - 1];
        if (last.x != one.x || last.z != one.z) moved = false;
      }
    }
    ok(moved,
       "a consult reports where the engine left him, and what the target was about beside it - so "
       "the last stop and the landing are the same place and the item is neither");
    ok(judged,
       "RED: and the distance that admits or refutes is measured on the item, because that is the "
       "thing the question asked to be put somewhere");
  }

  struct Catching : search::Watching {
    long long lines = 0;
    size_t most_candidates = 0;
    size_t most_closest = 0;
    bool same_shape = true;
    bool walked(const search::Counters& count, int through, size_t candidates) {
      (void)count;
      (void)through;
      (void)candidates;
      return true;
    }
    bool wants_so_far() { return true; }
    void so_far(const search::Found& run) {
      ++lines;
      if (run.candidate.size() > most_candidates) most_candidates = run.candidate.size();
      if (run.closest.size() > most_closest) most_closest = run.closest.size();
      for (size_t i = 1; i < run.candidate.size(); ++i) {
        if (run.candidate[i].frames < run.candidate[i - 1].frames) same_shape = false;
      }
      for (size_t i = 1; i < run.closest.size(); ++i) {
        if (run.closest[i].distance < run.closest[i - 1].distance) same_shape = false;
      }
      if (run.exhausted) same_shape = false;
    }
  };
  Catching catching;
  const search::Found handed = search::search_tree(q, base, flat_grid, flat_sel,
                                                   search::Calibration::measured(), nullptr,
                                                   &catching);
  ok(catching.lines > 0,
     "RED: a walk hands over what it has while it is still walking, rather than only when it stops");
  ok(catching.most_closest > 0,
     "RED: and what it hands over carries the closest it has reached, which is the half a run that "
     "reaches nothing has to show");
  ok(catching.same_shape,
     "each one is ranked the way the finished run is ranked and says it is not finished, so no row "
     "moves when the walk ends");
  ok(handed.candidate.size() == run.candidate.size(),
     "and being watched changes nothing about the answer");

  /* A tolerance nothing meets and a budget that prunes nothing, so the check range alone decides
     what is recorded. */
  {
    search::Question narrow = q;
    narrow.tolerance = 1e-9;
    narrow.frames = 1000000;
    narrow.steps = 9;
    narrow.check_range = 0.0;
    /* The steps bound counts the range, so with it on a narrower range walks less. */
    narrow.distance_bound = false;
    search::Question wide = narrow;
    wide.check_range = 1e6;
    const search::Found ran_narrow = search::search_tree(narrow, base, flat_grid, flat_sel,
                                                         search::Calibration::measured());
    const search::Found ran_wide = search::search_tree(wide, base, flat_grid, flat_sel,
                                                       search::Calibration::measured());
    ok(ran_narrow.count.generated > 0 && ran_wide.count.generated > 0,
       "two runs that walked something, which is what makes the two checks below about the range "
       "rather than about an empty question");
    ok(ran_wide.candidate.size() > ran_narrow.candidate.size(),
       "RED: a wider check range puts more of the tree in front of the engine, so the number the "
       "walk carried as a constant is a number somebody can turn");
    ok(ran_wide.count.generated == ran_narrow.count.generated &&
           ran_wide.count.expanded == ran_narrow.count.expanded,
       "RED: and it widens what is recorded rather than what is walked - the same tree either "
       "way, because it is the shortlist's width and not the search's");
  }

  /* The counters are compared too: a race can walk a different tree and still match the list. */
  {
    /* Target at the start, so nothing is pruned for distance and the tree is big enough for
       threads to overlap. */
    search::Question alone = q;
    alone.target.x = alone.start_x;
    alone.target.z = alone.start_z;
    alone.tolerance = 1.0;
    alone.frames = 1000000;
    alone.steps = 9;
    alone.cores = 1;
    search::Question crowd = alone;
    crowd.cores = 8;
    const search::Found ran_alone = search::search_tree(alone, base, flat_grid, flat_sel,
                                                        search::Calibration::measured());
    const search::Found ran_crowd = search::search_tree(crowd, base, flat_grid, flat_sel,
                                                        search::Calibration::measured());
    ok(!ran_alone.candidate.empty() && ran_alone.count.expanded > 0,
       "a question with a list to compare at all, which is what makes the two checks below mean "
       "something rather than matching two empty runs");

    bool same_list = ran_alone.candidate.size() == ran_crowd.candidate.size() &&
                     ran_alone.closest.size() == ran_crowd.closest.size();
    for (size_t i = 0; same_list && i < ran_alone.candidate.size(); ++i) {
      const search::Candidate& a = ran_alone.candidate[i];
      const search::Candidate& b = ran_crowd.candidate[i];
      if (a.path != b.path || a.frames != b.frames || a.distance != b.distance ||
          a.x != b.x || a.y != b.y || a.z != b.z || a.facing != b.facing) {
        same_list = false;
      }
    }
    for (size_t i = 0; same_list && i < ran_alone.closest.size(); ++i) {
      const search::Candidate& a = ran_alone.closest[i];
      const search::Candidate& b = ran_crowd.closest[i];
      if (a.path != b.path || a.frames != b.frames || a.distance != b.distance) same_list = false;
    }
    ok(same_list,
       "RED: one core and eight answer the same plans in the same order, and the same closest "
       "beside them - the thread count is the only thing on a question that cannot change what "
       "comes back");

    ok(ran_alone.count.generated == ran_crowd.count.generated &&
           ran_alone.count.expanded == ran_crowd.count.expanded &&
           ran_alone.count.dominance_kills == ran_crowd.count.dominance_kills &&
           ran_alone.count.bound_pruned == ran_crowd.count.bound_pruned &&
           ran_alone.count.key_collapses == ran_crowd.count.key_collapses &&
           ran_alone.exhausted == ran_crowd.exhausted,
       "RED: and they walked the same tree to get there, state for state and prune for prune, so "
       "the lists match by construction rather than by luck");

    /* Each report must aggregate every thread, so needs several threads to test. */
    struct Rising : search::Watching {
      long long lines = 0;
      long long most = 0;
      size_t most_plans = 0;
      bool fell = false;
      bool walked(const search::Counters& count, int through, size_t candidates) {
        (void)through;
        (void)candidates;
        if (count.generated < most) fell = true;
        if (count.generated > most) most = count.generated;
        return true;
      }
      bool wants_so_far() { return true; }
      void so_far(const search::Found& one) {
        ++lines;
        if (one.count.generated < most) fell = true;
        if (one.count.generated > most) most = one.count.generated;
        if (one.candidate.size() < most_plans) fell = true;
        if (one.candidate.size() > most_plans) most_plans = one.candidate.size();
      }
    };
    /* Deep enough to be reported more than once; the reporter polls on a clock. */
    search::Question long_enough = crowd;
    long_enough.steps = 16;
    /* The steps bound would cut this walk to under one poll. */
    long_enough.distance_bound = false;
    Rising rising;
    const search::Found watched = search::search_tree(long_enough, base, flat_grid, flat_sel,
                                                      search::Calibration::measured(), nullptr,
                                                      &rising);
    ok(rising.lines > 1 && rising.most > 0,
       "a run that handed over more than once while it walked, which is what makes the two checks "
       "below about the figures rather than about a run nobody saw");
    ok(!rising.fell,
       "RED: every figure a person reads is the whole run and never one thread's share of it, so "
       "no line is smaller than the one before it");
    ok(rising.most <= watched.count.generated,
       "RED: and no line ever claims more than the finished run generated, so the figure counts "
       "the run and never one thread twice");
  }

  bool inside = true;
  for (size_t i = 0; i < run.candidate.size(); ++i) {
    if (run.candidate[i].frames > q.frames) inside = false;
    if (run.candidate[i].distance > q.tolerance) inside = false;
  }
  ok(inside, "every candidate is within the frame bound and within the tolerance asked for");

  int cheapest = 1 << 30;
  for (size_t i = 0; i < run.candidate.size(); ++i) {
    if (run.candidate[i].frames < cheapest) cheapest = run.candidate[i].frames;
  }
  search::Question tighter = q;
  tighter.frames = cheapest - 1;
  const search::Found starved = search::search_tree(tighter, base, flat_grid, flat_sel,
                                                    search::Calibration::measured());
  ok(starved.candidate.empty(),
     "RED: a budget one frame under the cheapest candidate answers none of them");
  search::Question exactly = q;
  exactly.frames = cheapest;
  const search::Found just = search::search_tree(exactly, base, flat_grid, flat_sel,
                                                 search::Calibration::measured());
  ok(!just.candidate.empty(),
     "and a budget of exactly that cost answers it again, so the bound is not simply refusing");

  search::Question richer = q;
  richer.frames = q.frames + 30;
  const search::Found more = search::search_tree(richer, base, flat_grid, flat_sel,
                                                 search::Calibration::measured());

  struct Keys {
    static std::set<std::string> of(const std::vector<search::Candidate>& from, int at_most) {
      std::set<std::string> out;
      for (size_t i = 0; i < from.size(); ++i) {
        if (from[i].frames > at_most) continue;
        std::string k;
        const std::vector<search::Edge> c = search::canonical(from[i].path);
        for (size_t e = 0; e < c.size(); ++e) {
          char one[32];
          std::snprintf(one, sizeof one, "%d:%d,", c[e].row, c[e].steps);
          k += one;
        }
        out.insert(k);
      }
      return out;
    }
  };
  ok(!Keys::of(more.candidate, q.frames).empty() &&
         Keys::of(run.candidate, q.frames) == Keys::of(more.candidate, q.frames),
     "a budget of B answers exactly what a budget with room to spare answers at cost B or less, "
     "which is what an admissible bound means");

  /* Reachable only by turning first: one roll rotated by 20 C up turn steps. */
  const int steps_off = 20;
  const double aimed = static_cast<double>(fine == nullptr ? 0 : fine->turn) * steps_off;
  const double theta = aimed * (2.0 * 3.14159265358979323846 / 65536.0);
  search::Question turn_q;
  turn_q.target.x = roll->ahead * std::sin(theta) + roll->side * std::cos(theta);
  turn_q.target.z = roll->ahead * std::cos(theta) - roll->side * std::sin(theta);
  turn_q.tolerance = 20.0;
  turn_q.frames = (fine == nullptr ? 0 : fine->frames - 1 + steps_off) + roll->frames + 10;
  turn_q.moves.push_back("dry_roll");
  turn_q.moves.push_back("fine_turn");
  const search::Found turned = search::search_tree(turn_q, base, flat_grid, flat_sel,
                                                   search::Calibration::measured());
  int turn_edges = 0;
  bool never_twice = true;
  for (size_t i = 0; i < turned.candidate.size(); ++i) {
    const std::vector<search::Edge>& path = turned.candidate[i].path;
    for (size_t e = 0; e < path.size(); ++e) {
      const bool turns = search::turn_steps(base.move[static_cast<size_t>(path[e].row)]) > 0;
      if (!turns) continue;
      ++turn_edges;
      if (e + 1 < path.size() &&
          search::turn_steps(base.move[static_cast<size_t>(path[e + 1].row)]) > 0) {
        never_twice = false;
      }
    }
  }
  ok(!turned.candidate.empty() && turn_edges > 0,
     "a target only a turn can aim at is answered, and the answers use the C up turn");
  ok(never_twice,
     "and no candidate turns twice in a row: a run of turns is ONE edge with its steps summed");

  if (fine != nullptr) {
    const int one_turn = (fine->frames - 1) + 6;
    const int two_turns = 2 * (fine->frames - 1) + 6;
    ok(two_turns > one_turn,
       "RED: two turns of the same net cost strictly more than one, so collapsing is not a tidy-up");
  }

  std::vector<search::Edge> ab, ba;
  search::Edge a;
  a.row = 3;
  search::Edge b;
  b.row = 7;
  ab.push_back(a);
  ab.push_back(b);
  ba.push_back(b);
  ba.push_back(a);
  ok(search::canonical(ab) == search::canonical(ba),
     "a flipped twin has the same canonical key, which is the multiset and not the sequence");
  ok(!(ab == ba),
     "RED: while the paths themselves differ, so the key is doing the collapsing rather than "
     "the two being equal anyway");
  /* Two orders of one multiset are two plans: the engine can put them on different floats. */
  bool paths_distinct = true, orders_both_listed = false;
  for (size_t i = 0; i < run.candidate.size(); ++i) {
    for (size_t j = i + 1; j < run.candidate.size(); ++j) {
      if (run.candidate[i].path == run.candidate[j].path) paths_distinct = false;
      if (search::canonical(run.candidate[i].path) == search::canonical(run.candidate[j].path)) {
        orders_both_listed = true;
      }
    }
  }
  ok(paths_distinct, "no path is listed twice in one run");

  ok(orders_both_listed,
     "RED: two orders of the same moves the model cannot tell apart are both in its list");
  bool merged_in_budget = true;
  for (size_t i = 0; i < run.candidate.size(); ++i) {
    if (run.candidate[i].frames > q.frames) merged_in_budget = false;
  }
  ok(merged_in_budget, "and every plan it lists, spliced or walked, is inside the frames asked");

  /* Five moves is past the split, so some of the five orders are spliced rather than walked. */
  const search::BaseMove* crawl_row = base.of("crawl");
  if (crawl_row != nullptr && crawl_row->driven) {
    search::Question deep;
    deep.start_facing = 0;
    /* From z -100 the orders cross 256, where the float step doubles. */
    deep.start_z = -100.0;
    deep.target.x = 4.0 * roll->side + crawl_row->side;
    deep.target.z = deep.start_z + 4.0 * roll->ahead + crawl_row->ahead;
    deep.tolerance = 1.0;
    deep.frames = 4 * roll->frames + crawl_row->frames;
    deep.steps = 5;
    deep.moves.push_back("crawl");
    deep.moves.push_back("dry_roll");
    const search::Found deep_run = search::search_tree(deep, base, flat_grid, flat_sel,
                                                       search::Calibration::measured());
    std::set<std::vector<search::Edge> > orders;
    for (size_t i = 0; i < deep_run.candidate.size(); ++i) {
      if (deep_run.candidate[i].path.size() == 5) orders.insert(deep_run.candidate[i].path);
    }
    ok(orders.size() == 5,
       "RED: all five orders of four rolls and a crawl are listed, past the split as well");
    ok(deep_run.count.merged_orders > 0,
       "RED: and some of them were spliced onto a listed plan rather than walked");

    /* Each order's band rebuilt from its own moves, as the walk adds them. */
    const search::Calibration cal = search::Calibration::measured();
    int banded = 0, own_band = 0;
    for (size_t i = 0; i < deep_run.candidate.size(); ++i) {
      const search::Candidate& c = deep_run.candidate[i];
      if (c.path.size() != 5) continue;
      ++banded;
      double x = 0.0, y = 0.0, z = deep.start_z, band = 0.0;
      for (size_t e = 0; e < c.path.size(); ++e) {
        const search::BaseMove& row = base.move[static_cast<size_t>(c.path[e].row)];
        const search::Stepped st =
            search::step_move(row, flat_grid, flat_sel, cal, x, y, z, 0, deep.collision);
        const float m = static_cast<float>(std::max(std::fabs(st.x), std::fabs(st.z)));
        const double step = static_cast<double>(
            std::nextafter(m, std::numeric_limits<float>::infinity()) - m);
        band = band + cal.threshold(row.id, st.sloped_frames, st.tabled) +
               deep.check_range * step;
        x = st.x;
        y = st.y;
        z = st.z;
      }
      if (band == c.consult) ++own_band;
    }
    ok(banded == 5 && own_band == banded,
       "RED: and every one of the five is shortlisted on the band its own moves earn");
  }

  ok(run.count.held_bytes > 0, "a run that shortlisted plans says how much memory they hold");
  search::Question little = q;
  little.memory = run.count.held_bytes / 4;
  const search::Found tight_run = search::search_tree(little, base, flat_grid, flat_sel,
                                                      search::Calibration::measured());
  ok(!tight_run.exhausted,
     "RED: a run whose shortlist met the machine's memory says it is no longer exhaustive");
  ok(tight_run.count.held_bytes <= little.memory,
     "and it held no more than it was given");
  /* The plan that would pass the ceiling is refused, so held_bytes never reaches it. */
  ok(tight_run.count.memory_full,
     "RED: and it says its shortlist is full, though what it holds never reaches the ceiling");
  ok(!run.count.memory_full, "while a run with no ceiling never says so");

  const search::Slab small = search::slab(0.0, 0.0, 0.0, 300.0);
  const search::Corridor narrow = search::corridor(0.0, 0.0, 0.0, 0.0, 150.0);
  const search::Selection small_sel = search::select(small.mesh, narrow);
  const search::Grid small_grid = search::build(small_sel, narrow, open);
  search::Question off;
  off.target.z = 260.0;
  off.tolerance = 25.0;
  off.frames = 70;
  off.moves.push_back("dry_roll");
  off.moves.push_back("crawl");
  const search::Found walked = search::search_tree(off, base, small_grid, small_sel,
                                                   search::Calibration::measured());
  ok(walked.count.left_corridor > 0,
     "a run that walks off the grid counts every time it did, rather than rejecting it silently");
  const search::Found held = search::search_tree(off, base, flat_grid, flat_sel,
                                                 search::Calibration::measured());
  ok(held.count.left_corridor == 0,
     "RED: and the same question over a grid that holds the whole run counts none");

  ok(more.count.dominance_kills > 0,
     "states are killed by dominance at all, which is what keeps a frontier from blowing up");
  search::Question exacting = turn_q;
  exacting.tolerance = 1e-4;
  const search::Found picky = search::search_tree(exacting, base, flat_grid, flat_sel,
                                                  search::Calibration::measured());
  ok(picky.quanta.facing == 1 && turned.quanta.facing > fine->turn,
     "an exact-hit question buckets no two facings and a 20-unit one buckets a whole turn step");
  ok(picky.count.dominance_kills < turned.count.dominance_kills,
     "RED: and it kills fewer states for it, so the bucket is the tolerance's rather than a "
     "constant");

  struct Enough : search::Watching {
    long long seen = 0;
    int lowest = 1 << 30;
    int highest = -1;
    bool walked(const search::Counters& count, int through, size_t candidates) {
      (void)count;
      (void)candidates;
      if (through < lowest) lowest = through;
      if (through > highest) highest = through;
      ++seen;
      return false;
    }
  };
  Enough enough;
  const search::Found stopped = search::search_tree(richer, base, flat_grid, flat_sel,
                                                    search::Calibration::measured(), nullptr,
                                                    &enough);
  ok(!stopped.exhausted, "a run told to stop says its bound was not explored");
  ok(more.exhausted, "RED: while the same question left to run says it was");
  ok(enough.seen == 1 && enough.lowest >= 0 && enough.highest <= 1000,
     "and what it is told is a share of the whole tree, which cannot read past the whole of it");

  struct Taking : search::Watching {
    bool takes = false;
    std::mutex books;
    std::set<std::vector<search::Edge> > got;
    bool walked(const search::Counters& count, int through, size_t candidates) {
      (void)count;
      (void)through;
      (void)candidates;
      return true;
    }
    void found(const search::Candidate& one) {
      std::lock_guard<std::mutex> lock(books);
      got.insert(one.path);
    }
    bool takes_every_candidate() { return takes; }
  };
  Taking keeping;
  const search::Found kept_list = search::search_tree(q, base, flat_grid, flat_sel,
                                                      search::Calibration::measured(), nullptr,
                                                      &keeping);
  Taking taking;
  taking.takes = true;
  const search::Found took = search::search_tree(q, base, flat_grid, flat_sel,
                                                 search::Calibration::measured(), nullptr,
                                                 &taking);
  ok(!kept_list.candidate.empty(), "a watcher that reads the list afterwards still gets one");
  ok(took.candidate.empty(),
     "RED: a watcher that takes every candidate leaves the walk nothing to keep, so the "
     "candidate ceiling cannot end its run");
  ok(!taking.got.empty() && taking.got == keeping.got,
     "and it is handed every candidate the list would have held");
  ok(took.exhausted && took.count.generated == kept_list.count.generated,
     "and the run it watched is the same whole run");

  const search::Found again = search::search_tree(q, base, flat_grid, flat_sel,
                                                  search::Calibration::measured());
  bool identical = again.candidate.size() == run.candidate.size();
  for (size_t i = 0; identical && i < again.candidate.size(); ++i) {
    if (again.candidate[i].frames != run.candidate[i].frames) identical = false;
    if (again.candidate[i].path != run.candidate[i].path) identical = false;
  }
  ok(identical,
     "the same question answers the identical list twice, which is what `D13` needs before a "
     "second core or a pause can be believed");

  search::Question standing;
  standing.tolerance = 5.0;
  standing.frames = 60;
  standing.moves.push_back("crawl");
  const search::Found here = search::search_tree(standing, base, flat_grid, flat_sel,
                                                 search::Calibration::measured());
  bool empty_path = false;
  for (size_t i = 0; i < here.candidate.size(); ++i) {
    if (here.candidate[i].path.empty()) empty_path = true;
  }
  ok(empty_path, "a start already inside the tolerance is a candidate of no moves at all");

  const search::Found by_point = search::search_tree(q, base, flat_grid, flat_sel, search::Calibration::measured());

  /* Same tolerance: it is also the dominance quantum and the bound's slack. */
  search::Question ranged = q;
  ranged.target.ranged = true;
  ranged.target.x0 = q.target.x - q.tolerance;
  ranged.target.x1 = q.target.x + q.tolerance;
  ranged.target.z0 = q.target.z - q.tolerance;
  ranged.target.z1 = q.target.z + q.tolerance;
  const search::Found by_box = search::search_tree(ranged, base, flat_grid, flat_sel, search::Calibration::measured());
  ok(by_box.candidate.size() >= by_point.candidate.size() && !by_box.candidate.empty(),
     "a range around a point finds everything the point did - a box is a wider question and "
     "never a narrower one");

  search::Question freed = q;
  freed.target.has_z = false;
  const search::Found by_line = search::search_tree(freed, base, flat_grid, flat_sel, search::Calibration::measured());
  ok(by_line.candidate.size() >= by_point.candidate.size(),
     "and freeing an axis finds everything the point did, because it asks about less");
  bool off_the_point = false;
  for (size_t i = 0; i < by_line.candidate.size(); ++i) {
    if (std::fabs(by_line.candidate[i].z - q.target.z) > q.tolerance) off_the_point = true;
  }
  ok(off_the_point,
     "RED: and it finds landings the point refused - a z the question does not ask about, which "
     "is the whole of what the checkbox means");

  search::Question faced = freed;
  faced.end_facing.any = false;
  faced.end_facing.a = 1;
  faced.end_facing.span = 0;
  const search::Found by_facing = search::search_tree(faced, base, flat_grid, flat_sel, search::Calibration::measured());
  bool every_one = true;
  for (size_t i = 0; i < by_facing.candidate.size(); ++i) {
    if (by_facing.candidate[i].facing != 1) every_one = false;
  }
  ok(every_one, "an end facing keeps only the plans that finish on it");
  ok(by_facing.candidate.size() < by_line.candidate.size(),
     "RED: and the same question with no facing asked for keeps more, so the arc is doing the "
     "refusing rather than the walk running out");

  /* One step either way costs the row's stated frames; the long way round costs 94 more. */
  const search::BaseMove* fine_row = base.of("fine_turn");
  ok(fine_row != nullptr && search::turn_steps(*fine_row) == 200,
     "RED: the table's C up turn stands for 200 moves, which is what a hundred steps each way is");
  if (fine_row != nullptr) {
    search::Question turned;
    turned.start_facing = 0;
    turned.target.has_x = false;
    turned.target.has_z = false;
    turned.tolerance = 0.0;
    turned.frames = fine_row->frames + 4;
    turned.steps = 1;
    turned.moves.push_back("fine_turn");
    bool clockwise = false, anticlockwise = false;
    const int step = fine_row->turn;
    for (int way = 0; way < 2; ++way) {
      turned.end_facing.any = false;
      turned.end_facing.span = 0;
      turned.end_facing.a = ((way == 0 ? step : -step) % 65536 + 65536) % 65536;
      const search::Found one = search::search_tree(turned, base, flat_grid, flat_sel,
                                                    search::Calibration::measured());
      bool cheap = false;
      for (size_t i = 0; i < one.candidate.size(); ++i) {
        if (one.candidate[i].frames == fine_row->frames) cheap = true;
      }
      if (way == 0) clockwise = cheap; else anticlockwise = cheap;
    }
    ok(clockwise,
       "one step of the turn reaches the facing one step along, at the row's own stated price");
    ok(anticlockwise,
       "RED: and ONE step reaches the facing one step the other way too, at the same price - a "
       "walk that turns one way only reaches it in ninety-five and this is what says which");
  }
}


/* The verification: checks are keyed on the candidates handed in, never on how many answers a
   world holds. */

/* Every charged move and every slope-table node ends inside `travel_of`. */
void travel_tests() {
  const search::BaseTable base = search::base_table(true);
  const search::Calibration stored = search::Calibration::measured();
  bool inside = true, past_level = false, stepped_any = false;
  std::string worst;
  double worst_over = 0.0;
  int moves_read = 0;
  long long steps_read = 0;
  const double grades[3] = {0.3, 0.9, 1.7};
  const double ways[1][2] = {{1, 0}};
  for (double g : grades) {
    for (const auto& w : ways) {
      const search::Slab ground = search::slab(0.0, g * w[0], g * w[1], 8192.0);
      const search::Corridor wide = search::corridor(0.0, 0.0, 0.0, 0.0, 3000.0);
      const search::Selection sel = search::select(ground.mesh, wide);
      search::Limits open;
      open.slope_normal_y = 0.0;
      open.height_step = 1e9;
      open.normal_apart = 1e9;
      const search::Grid grid = search::build(sel, wide, open);
      const double drop = search::ground_drop(grid), steepest = search::ground_steepest(grid);
      for (const search::BaseMove& row : base.move) {
        if (!row.driven || row.step.empty()) continue;
        const search::MoveCal* mc = stored.of(row.id);
        if (mc == nullptr || mc->rows == 0) continue;
        const search::PlaneTable* pt = stored.plane_of(row.id);
        const double travel = search::travel_of(row, drop, mc, pt, steepest);
        ++moves_read;
        for (int k = 0; k < 16; ++k) {
          const int facing = k * 4096 + 1000;
          const search::Stepped st = search::step_move(row, grid, sel, mc, pt, 0.0,
                                                       ground.at(0.0, 0.0), 0.0, facing);
          if (!st.ok) continue;
          stepped_any = true;
          ++steps_read;
          const double went = std::hypot(st.x, st.z);
          if (went > travel && went - travel > worst_over) {
            inside = false;
            worst_over = went - travel;
            worst = row.id;
          }
          if (went > row.reach) past_level = true;
        }
        if (pt != nullptr) {
          for (size_t i = 0; i < pt->ahead.size(); ++i) {
            if (pt->key[i] == 255) continue;
            const double node = std::hypot(pt->ahead[i], pt->side[i]);
            if (node > travel) inside = false;
            if (node > row.reach) past_level = true;
          }
        }
      }
    }
  }
  std::printf("     travel checked on %d moves, %lld steps\n", moves_read, steps_read);
  ok(stepped_any && inside,
     "no charged move ends on steep ground further out than the distance bounds allow" +
         (worst.empty() ? std::string() : " - " + worst + " is past it"));
  ok(past_level,
     "RED: and somewhere one ends past its level reach");
}

/* Fewest: no plan shorter than it is recorded or kept as closest, and longer ones still are. */
void fewest_tests() {
  const search::BaseTable base = search::base_table(false);
  const search::Calibration stored = search::Calibration::measured();
  const double at = 1000.0;
  const search::Slab ground = search::slab(0.0, 0.0, 0.0, 8192.0);
  const search::Corridor wide = search::corridor(at, at, at, at, 3000.0);
  const search::Selection sel = search::select(ground.mesh, wide);
  search::Limits open;
  open.slope_normal_y = 0.0;
  open.height_step = 1e9;
  open.normal_apart = 1e9;
  const search::Grid grid = search::build(sel, wide, open);

  search::Question q;
  q.start_x = at;
  q.start_z = at;
  q.start_y = ground.at(at, at);
  q.target.x = at + 200.0;
  q.target.z = at + 180.0;
  q.tolerance = 400.0;
  q.frames = 100000;
  q.steps = 3;
  q.moves.push_back("dry_roll");
  q.moves.push_back("crawl");
  q.moves.push_back("fine_turn");

  auto shortest = [](const search::Found& f) {
    size_t n = 1000;
    for (const search::Candidate& c : f.candidate) n = std::min(n, c.path.size());
    for (const search::Candidate& c : f.closest) n = std::min(n, c.path.size());
    return n;
  };
  const search::Found all = search::search_tree(q, base, grid, sel, stored);
  q.fewest = 3;
  const search::Found three = search::search_tree(q, base, grid, sel, stored);
  std::printf("     fewest 3: %zu candidates against %zu, shortest %zu against %zu\n",
              three.candidate.size(), all.candidate.size(), shortest(three), shortest(all));
  ok(shortest(all) < 3, "without fewest, plans shorter than three are recorded");
  ok(!three.candidate.empty(), "with fewest 3, three-step plans are still recorded");
  ok(shortest(three) == 3, "RED: and none shorter than three is recorded or kept as closest");
  /* Shorter states are still walked, so every longer plan the unfiltered run finds is found. */
  std::set<std::vector<search::Edge> > kept;
  for (const search::Candidate& c : three.candidate) kept.insert(c.path);
  size_t lost = 0;
  for (const search::Candidate& c : all.candidate) {
    if (c.path.size() >= 3 && kept.count(c.path) == 0) ++lost;
  }
  ok(lost == 0, "RED: every plan of three or more the unfiltered run records is recorded");

  /* The frames floor, the same way: nothing quicker recorded, nothing slower lost. */
  q.fewest = 0;
  int quickest = 1 << 30, slowest = 0;
  for (const search::Candidate& c : all.candidate) {
    quickest = std::min(quickest, c.frames);
    slowest = std::max(slowest, c.frames);
  }
  q.least_frames = (quickest + slowest) / 2;
  const search::Found late = search::search_tree(q, base, grid, sel, stored);
  int least = 1 << 30;
  for (const search::Candidate& c : late.candidate) least = std::min(least, c.frames);
  for (const search::Candidate& c : late.closest) least = std::min(least, c.frames);
  std::printf("     least frames %d: %zu candidates against %zu, quickest %d against %d\n",
              q.least_frames, late.candidate.size(), all.candidate.size(), least, quickest);
  ok(quickest < q.least_frames, "without a floor, quicker plans are recorded");
  ok(!late.candidate.empty(), "with a floor, slower plans are still recorded");
  ok(least >= q.least_frames, "RED: and none quicker than the floor is recorded or kept as closest");
  std::set<std::vector<search::Edge> > slow;
  for (const search::Candidate& c : late.candidate) slow.insert(c.path);
  size_t gone = 0;
  for (const search::Candidate& c : all.candidate) {
    if (c.frames >= q.least_frames && slow.count(c.path) == 0) ++gone;
  }
  ok(gone == 0, "RED: every plan at or over the floor the unfiltered run records is recorded");
}

/* A run with the steps bound drives every order a run without it does, and no closest is farther. */
void steps_bound_tests() {
  /* 100 away, one roll of 90 left: no nearer than 10, inside the 5 + 10 its last step can earn. */
  {
    search::StepsLeft s;
    s.distance = 100.0;
    s.steps = 1;
    s.travel = 90.0;
    s.consult = 5.0;
    s.charge = 10.0;
    s.farthest_closest = 1.0;
    ok(!s.cannot_help(),
       "RED: a state whose last step could still earn its way into the near set is walked");
    s.distance = 106.0;
    ok(s.cannot_help(), "and one past what that step can earn, and past the closest, is not");
    s.distance = 100.0;
    s.charge = 0.0;
    s.farthest_closest = 20.0;
    ok(!s.cannot_help(),
       "RED: a state that could still come nearer than the farthest of the closest is walked");
  }

  const search::BaseTable base = search::base_table(false);
  const search::Calibration stored = search::Calibration::measured();
  /* The fourth world ties: every order of four rolls and a crawl lands on one state, so orders are
     spliced. */
  const double grades[4][2] = {{0.0, 0.0}, {0.12, -0.07}, {0.3, 0.1}, {0.0, 0.0}};
  const double tolerance[4] = {40.0, 40.0, 0.0, 1.0};
  const char* ground_is[4] = {"level ground", "a slope", "a steep slope at a tolerance of zero",
                              "level ground where orders tie"};
  /* Off the origin: the dominance cell is one float step there, a subnormal whose index wraps. */
  const double at = 1000.0;
  for (int g = 0; g < 4; ++g) {
    const search::Slab ground = search::slab(0.0, grades[g][0], grades[g][1], 8192.0);
    const search::Corridor wide = search::corridor(at, at, at, at, 3000.0);
    const search::Selection sel = search::select(ground.mesh, wide);
    search::Limits open;
    open.slope_normal_y = 0.0;
    open.height_step = 1e9;
    open.normal_apart = 1e9;
    const search::Grid grid = search::build(sel, wide, open);

    search::Question q;
    q.start_x = at;
    q.start_z = at;
    q.start_y = ground.at(at, at);
    q.target.x = at + 200.0;
    q.target.z = at + 180.0;
    q.tolerance = tolerance[g];
    q.frames = 100000;
    q.steps = 4;
    /* At zero, the roll with the widest charge, so the consult band holds something. */
    q.moves.push_back(tolerance[g] == 0.0 ? "dry_roll_r_free" : "dry_roll");
    q.moves.push_back("crawl");
    if (g == 3) {
      q.steps = 5;
      const search::BaseMove* roll = base.of("dry_roll");
      const search::BaseMove* crawl = base.of("crawl");
      if (roll != nullptr && crawl != nullptr) {
        search::Stepped st;
        st.x = at;
        st.y = q.start_y;
        st.z = at;
        for (int m = 0; m < 5; ++m) {
          st = search::step_move(m < 4 ? *roll : *crawl, grid, sel, stored, st.x, st.y, st.z,
                                 st.facing);
        }
        q.target.x = st.x;
        q.target.z = st.z;
      }
    } else {
      q.moves.push_back("fine_turn");
    }
    if (tolerance[g] == 0.0) {
      /* Half a unit off two rolls' landing: only the charges put anything in the near set. */
      const search::BaseMove* roll = base.of("dry_roll_r_free");
      if (roll != nullptr) {
        const search::Stepped one =
            search::step_move(*roll, grid, sel, stored, at, q.start_y, at, 0);
        const search::Stepped two =
            search::step_move(*roll, grid, sel, stored, one.x, one.y, one.z, one.facing);
        q.target.x = two.x + 0.5;
        q.target.z = two.z;
      }
    }
    search::Question uncut_q = q;
    uncut_q.distance_bound = false;
    const search::Found cut = search::search_tree(q, base, grid, sel, stored);
    const search::Found uncut = search::search_tree(uncut_q, base, grid, sel, stored);

    const char* where = ground_is[g];
    std::printf("     steps bound on %s: %lld states against %lld, %lld cut, %zu candidates "
                "against %zu, %zu closest\n",
                where, cut.count.generated, uncut.count.generated, cut.count.steps_pruned,
                cut.candidate.size(), uncut.candidate.size(), cut.closest.size());
    /* Not equality: a cut state never claims its cell, so a look-alike merged away there can be
       walked here and come nearer. */
    std::set<std::vector<search::Edge>> have;
    for (const search::Candidate& c : cut.candidate) have.insert(c.path);
    size_t missing = 0;
    for (const search::Candidate& c : uncut.candidate) {
      if (have.find(c.path) == have.end()) ++missing;
    }
    std::vector<double> near_cut, near_uncut;
    for (const search::Candidate& c : cut.closest) near_cut.push_back(c.distance);
    for (const search::Candidate& c : uncut.closest) near_uncut.push_back(c.distance);
    std::sort(near_cut.begin(), near_cut.end());
    std::sort(near_uncut.begin(), near_uncut.end());
    bool no_farther = near_cut.size() >= near_uncut.size();
    for (size_t i = 0; no_farther && i < near_uncut.size(); ++i) {
      if (near_cut[i] > near_uncut[i]) no_farther = false;
    }
    std::printf("       %lld orders spliced against %lld, %zu of the uncut run's missing\n",
                cut.count.merged_orders, uncut.count.merged_orders, missing);
    const std::string on = std::string(" - on ") + where;
    ok(cut.count.steps_pruned > 0 && cut.count.generated < uncut.count.generated &&
           uncut.count.steps_pruned == 0,
       ("the steps bound cuts states, and none with the bound off" + on).c_str());
    if (g == 3) {
      ok(uncut.count.merged_orders > 0,
         "the orders on the tying world are spliced at all, which is what makes the check below "
         "about splices");
    }
    ok(!uncut.closest.empty() && !uncut.candidate.empty() && missing == 0 && no_farther,
       ("RED: and drives every order the uncut run drives, with no closest farther than its" + on)
           .c_str());
  }
}

void verify_tests() {
  const search::BaseTable base = search::base_table(false);

  /* One slab is both the grid's mesh and the engine's room. */
  const search::Slab flat = search::slab(0.0, 0.0, 0.0, 8192.0);
  const search::Corridor wide = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
  const search::Selection flat_sel = search::select(flat.mesh, wide);
  search::Limits open;
  open.slope_normal_y = 0.0;
  open.height_step = 1e9;
  open.normal_apart = 1e9;
  const search::Grid flat_grid = search::build(flat_sel, wide, open);
  const search::Calibration stored = search::Calibration::measured();

  const search::BaseMove* roll = base.of("dry_roll");
  ok(roll != nullptr && roll->driven, "the roll is driven, which the questions below are sized off");
  if (roll == nullptr || !roll->driven) return;

  search::Question q;
  q.target.x = 2.0 * roll->side;
  q.target.z = 2.0 * roll->ahead;
  q.tolerance = 20.0;
  q.frames = 3 * roll->frames;
  q.moves.push_back("dry_roll");
  q.moves.push_back("dry_roll_r");
  const search::Found run = search::search_tree(q, base, flat_grid, flat_sel, search::Calibration::measured());
  const search::Verified said = search::verify(run, q, base, &flat.room);

  ok(said.count.consults > 0,
     "the verification consulted the engine at all - zero confirmed is a result, zero consulted "
     "is the bug");
  ok(static_cast<size_t>(said.count.consults) == run.candidate.size() &&
         said.consult.size() == run.candidate.size(),
     "with the switch off every candidate the search found is consulted, and none twice");
  ok(said.count.confirmed + said.count.refuted + said.count.could_not_run == said.count.consults,
     "and each consult is counted under exactly one of `D9`'s three outcomes");
  ok(said.count.discarded == 0, "with nothing discarded, which is what the switch being off means");

  bool admitted_by_engine = true;
  for (size_t i = 0; i < said.answer.size(); ++i) {
    const search::Consult& one = said.consult[said.answer[i]];
    if (one.outcome != search::Outcome::Confirmed) admitted_by_engine = false;
    if (one.distance > q.tolerance) admitted_by_engine = false;
    if (one.drives != static_cast<int>(one.path.size())) admitted_by_engine = false;
    if (one.engine_frames <= 0 && !one.path.empty()) admitted_by_engine = false;
  }
  ok(!said.answer.empty() && admitted_by_engine,
     "every answer was stepped by the engine the whole way and is inside the tolerance on the "
     "engine's own distance");

  search::Question moved = q;
  moved.target.x = q.target.x + 5000.0;
  const search::Verified elsewhere = search::verify(run, moved, base, &flat.room);
  ok(elsewhere.count.consults == said.count.consults && elsewhere.answer.empty() &&
         elsewhere.count.refuted == elsewhere.count.consults,
     "RED: the same candidates against a target they do not reach are all refuted and none is "
     "admitted, so the verdict is the engine's rather than the list's");

  /* Plans that share first moves and diverge, so a memo keyed on too little mixes landings. */
  const search::BaseMove* crawl = base.of("crawl");
  ok(crawl != nullptr && crawl->driven, "the crawl is driven, which the remembered plans need");
  if (crawl != nullptr && crawl->driven) {
    search::Edge r, c;
    r.row = static_cast<int>(roll - &base.move[0]);
    c.row = static_cast<int>(crawl - &base.move[0]);
    const search::Edge plans[][3] = {{r, r, r}, {c, r, r}, {r, c, r}, {c, c, r},
                                     {r, r, c}, {c, r, c}, {r, c, c}, {c, c, c}};
    search::Found mixed;
    for (size_t p = 0; p < sizeof plans / sizeof plans[0]; ++p) {
      for (size_t n = 1; n <= 3; ++n) {
        search::Candidate one;
        one.path.assign(plans[p], plans[p] + n);
        mixed.candidate.push_back(one);
      }
    }
    const bool was = search::remember_drives(false);
    const search::Verified cold = search::verify(mixed, q, base, &flat.room);
    search::remember_drives(true);
    const search::Verified warm = search::verify(mixed, q, base, &flat.room);
    const search::DriveMemo memo = search::drive_memo();
    const search::Verified again = search::verify(mixed, q, base, &flat.room);
    const search::DriveMemo memo2 = search::drive_memo();
    search::remember_drives(was);

    struct Same {
      static bool as(const search::Verified& a, const search::Verified& b) {
        if (a.consult.size() != b.consult.size() || a.answer != b.answer) return false;
        for (size_t i = 0; i < a.consult.size(); ++i) {
          const search::Consult& p = a.consult[i];
          const search::Consult& r = b.consult[i];
          if (std::memcmp(&p.x, &r.x, sizeof p.x) != 0 || std::memcmp(&p.y, &r.y, sizeof p.y) != 0 ||
              std::memcmp(&p.z, &r.z, sizeof p.z) != 0 ||
              std::memcmp(&p.aim_x, &r.aim_x, sizeof p.aim_x) != 0 ||
              std::memcmp(&p.aim_z, &r.aim_z, sizeof p.aim_z) != 0 || p.facing != r.facing ||
              p.frames != r.frames || p.engine_frames != r.engine_frames ||
              p.drives != r.drives || p.outcome != r.outcome || p.why != r.why ||
              p.stop.size() != r.stop.size()) {
            return false;
          }
          for (size_t s = 0; s < p.stop.size(); ++s) {
            if (std::memcmp(&p.stop[s].x, &r.stop[s].x, sizeof p.stop[s].x) != 0 ||
                std::memcmp(&p.stop[s].z, &r.stop[s].z, sizeof p.stop[s].z) != 0 ||
                p.stop[s].facing != r.stop[s].facing || p.stop[s].frames != r.stop[s].frames) {
              return false;
            }
          }
        }
        return true;
      }
    };
    ok(memo.hits > 0 && memo.misses > 0,
       "the plans share moves, so some drives were read back and some were driven - zero read back "
       "is a memo nothing tested");
    ok(Same::as(cold, warm),
       "every plan verified through the memo ends where a fresh verification does, bit for bit");
    ok(memo2.misses == memo.misses && memo2.hits > memo.hits && Same::as(cold, again),
       "and a second pass drives nothing new and still lands every plan in the same place");
  }

  bool rows_whole = !said.consult.empty();
  double worst = 0.0;
  for (size_t i = 0; i < said.consult.size(); ++i) {
    const search::Consult& one = said.consult[i];
    if (one.outcome == search::Outcome::CouldNotRun) continue;
    if (std::fabs((one.predicted - one.distance) - one.error) > 1e-12) rows_whole = false;
    if (one.predicted_frames <= 0 && !one.path.empty()) rows_whole = false;
    if (std::fabs(one.error) > worst) worst = std::fabs(one.error);
  }
  ok(rows_whole,
     "every consult carries its own row of the error catalogue: what was predicted, what the "
     "engine answered, and the signed difference");
  ok(worst < q.tolerance,
     "and over level ground the model is inside the tolerance it was searched at, which is what "
     "makes a refutation here mean something");

  /* A tolerance of zero still reaches the engine: the consult radius is not the tolerance
     (`Candidate::consult`). The target is an engine landing, so an exact answer exists. */
  {
    search::Question reached = q;
    reached.tolerance = 0.0;
    reached.steps = 1;
    reached.frames = roll->frames;
    /* No floor read, so the allowance is exactly zero; a flat slab still reads a grade near 1e-9. */
    reached.collision = search::Collision::None;
    /* Off facing 0, where the game's trig is exact and the model matches the engine to the bit. */
    reached.start_facing = 7018;

    search::Candidate just_one;
    search::Edge only;
    only.row = -1;
    for (size_t i = 0; i < base.move.size(); ++i) {
      if (base.move[i].id == "dry_roll") only.row = static_cast<int>(i);
    }
    just_one.path.push_back(only);
    search::Found staged;
    staged.candidate.push_back(just_one);
    const search::Verified drove = search::verify(staged, reached, base, &flat.room);
    ok(drove.consult.size() == 1 && drove.consult[0].why.empty(),
       "the engine ran a single roll, which is what the exact question below is aimed at");
    if (drove.consult.size() == 1 && drove.consult[0].why.empty()) {
      reached.target.x = drove.consult[0].aim_x;
      reached.target.z = drove.consult[0].aim_z;

      const search::Found run0 =
          search::search_tree(reached, base, flat_grid, flat_sel, stored);
      const search::Verified said0 = search::verify(run0, reached, base, &flat.room);

      bool nothing_earned = true;
      for (size_t i = 0; i < run0.candidate.size(); ++i) {
        if (run0.candidate[i].allowance != 0.0) nothing_earned = false;
      }
      ok(nothing_earned,
         "with the room unread not one candidate earns an allowance, so this asks about the case "
         "the slab fixture above cannot reach");
      ok(!run0.candidate.empty(),
         "at a tolerance of zero the walk still records a shortlist, because what it records is "
         "the consult radius and not the tolerance");
      ok(said0.count.consults > 0,
         "so the engine is consulted at all, which is the whole failure the split exists for - "
         "zero confirmed is a result, zero consulted is the bug");
      ok(!said0.answer.empty(),
         "and the exact answer is admitted, on a landing the engine itself produced");

      bool every_answer_is_exact = !said0.answer.empty();
      for (size_t i = 0; i < said0.answer.size(); ++i) {
        const search::Consult& one = said0.consult[said0.answer[i]];
        if (reached.target.off(one.aim_x, one.aim_z) != 0) every_answer_is_exact = false;
      }
      ok(every_answer_is_exact,
         "with every one of them the same four bytes as the target, which is what zero asks and "
         "what a distance in units cannot say about two f32s");

      search::Question nudged = reached;
      nudged.target.x = static_cast<double>(
          std::nextafter(static_cast<float>(reached.target.x),
                         std::numeric_limits<float>::infinity()));
      const search::Verified beside = search::verify(run0, nudged, base, &flat.room);
      ok(beside.count.consults == said0.count.consults && beside.answer.empty(),
         "RED: the same candidates against a target one float step over are all refused, so zero "
         "means the bytes rather than a small number of units");
    }
  }

  /* `resolve`'s slack is the tolerance alone, never the aim's reach: the goal region must not
     swallow a landing past the tolerance but inside the reach. */
  {
    search::Question aimed;
    aimed.start_x = 0;
    aimed.start_z = 0;
    aimed.tolerance = 5.0;
    /* Only the item aim has any reach. */
    aimed.aim = search::Aim::Overhead;
    aimed.target.has_x = true;
    aimed.target.has_z = true;
    aimed.target.x = 40.0;
    aimed.target.z = 40.0;

    ok(search::aim_reach(aimed) > 30.0,
       "the item's aim really does reach tens of units past the place being measured, which is "
       "what would be swallowed if it were slack");

    search::Target walked = aimed.target;
    const bool stands = search::resolve(&walked, flat_grid, flat_sel.ground, aimed.tolerance);
    ok(stands, "the target has floor under it on the flat fixture, so this asks about a resolved "
               "goal and not about the unresolved arm");
    if (stands) {
      const double miss_x = aimed.target.x + 20.0;
      ok(walked.off(miss_x, aimed.target.z) > 0,
         "a landing twenty units off the goal in x is not off by zero float steps - the aim's "
         "reach is not a width the goal has");
      ok(walked.distance(miss_x, aimed.target.z) > 1.0,
         "and it is not at zero distance either, which is the same region read by the other "
         "measure");

      ok(walked.off(aimed.target.x, aimed.target.z) == 0,
         "while the goal itself is still off by nothing at all, which is what a tolerance of zero "
         "admits on");
    }
  }

  /* The question's camera reaches the engine. No yaw value is asserted: that is the engine's. */
  {
    search::Question asked = q;
    asked.tolerance = 0.0;
    asked.steps = 1;
    asked.frames = roll->frames;
    asked.collision = search::Collision::None;
    asked.start_facing = 7018;

    search::Candidate one_roll;
    search::Edge only;
    only.row = -1;
    for (size_t i = 0; i < base.move.size(); ++i) {
      if (base.move[i].id == "dry_roll") only.row = static_cast<int>(i);
    }
    one_roll.path.push_back(only);
    search::Found staged;
    staged.candidate.push_back(one_roll);

    search::Question blind = asked;
    blind.has_camera = false;
    search::Question seen = asked;
    seen.has_camera = true;
    seen.start_camera = 21422;

    const search::Verified without = search::verify(staged, blind, base, &flat.room);
    const search::Verified watched = search::verify(staged, seen, base, &flat.room);

    ok(without.consult.size() == 1 && watched.consult.size() == 1,
       "one roll runs with a camera and without one, so the two can be put beside each other");
    if (without.consult.size() == 1 && watched.consult.size() == 1) {
      const search::Consult& a = without.consult[0];
      const search::Consult& b = watched.consult[0];

      ok(a.why.empty() && b.why.empty(),
         "and a camera does not stop a move the engine could otherwise run");
      ok(!a.had_camera,
         "a question that names no camera answers that it had none rather than answering zero - "
         "a yaw of zero is a camera looking north");
      ok(b.had_camera, "and a question that names one says it had one");
      ok(!b.stop.empty() && b.stop[0].camera == 21422,
         "the camera the question named is the camera the run came back with, so the field is "
         "wired through to the engine rather than declared and dropped");

      ok(a.x == b.x && a.z == b.z && a.facing == b.facing && a.frames == b.frames,
         "and it moves him not at all - the camera watches the run, it does not take part in it");
    }
  }

  /* The C up turn moves the camera it carries (`dCamera_c::subjectCamera`), while its facing and
     price stay the stipulated ones. */
  {
    const search::BaseMove* turn = base.of("fine_turn");
    ok(turn != nullptr, "the table has the C up turn this block is about");
    if (turn != nullptr) {
      search::Question asked = q;
      asked.tolerance = 0.0;
      asked.steps = 1;
      asked.collision = search::Collision::None;
      asked.start_facing = 7018;
      asked.has_camera = true;
      asked.start_camera = 7018;

      search::Candidate one_turn;
      search::Edge only;
      only.row = -1;
      for (size_t i = 0; i < base.move.size(); ++i) {
        if (base.move[i].id == "fine_turn") only.row = static_cast<int>(i);
      }
      only.steps = 22;
      one_turn.path.push_back(only);
      search::Found staged;
      staged.candidate.push_back(one_turn);

      const search::Verified out = search::verify(staged, asked, base, &flat.room);
      ok(out.consult.size() == 1, "one turn runs, so there is a stop to read the camera off");
      if (out.consult.size() == 1 && !out.consult[0].stop.empty()) {
        const search::Consult& t = out.consult[0];
        const int landed = t.stop[0].camera;
        const int facing = t.stop[0].facing;

        ok(t.had_camera, "a turn asked with a camera says it had one");
        ok(landed != asked.start_camera,
           "and the turn moves it - the first-person view is entered, which is what the turn is, "
           "so a camera that came out where it went in is one nothing ran");
        ok(landed == facing,
           "onto the facing the turn ended on, which is what the view's entry chases");
        ok(facing == ((asked.start_facing + turn->turn * only.steps) & 0xFFFF),
           "RED: while the facing itself is still the stipulated arithmetic and nothing else");
        ok(t.stated_frames == t.frames && t.frames == (turn->frames - 1) + only.steps,
           "and the price is still the one somebody set by hand, every frame of it");
      }
    }
  }

  /* A turnaround is pressed at `camera + k * 0x4000`, not at the reversed facing. The sweep keeps
     the camera within 0x800 of the facing: the reversal gate is a strict `> 0x7800`. */
  {
    int examined = 0, at_a_cardinal = 0, unaimed = 0, not_a_half_circle = 0;
    /* A stride coprime with the circle, so facings are not multiples of the cardinals. */
    for (int facing = 0; facing < 65536; facing += 1013) {
      for (int off = -0x7F0; off <= 0x7F0; off += 0xFD) {
        const int camera = (facing + off) & 0xFFFF;
        search::Move turn;
        if (!search::move_of("cup_b_settle_turnaround", facing, &turn, &camera)) continue;
        if (turn.action.empty()) continue;
        ++examined;
        if (turn.unaimed) {
          ++unaimed;
          continue;
        }
        const int world = turn.action[0].bearing & 0xFFFF;
        for (int k = 0; k < 4; ++k) {
          if (world == ((camera + search::kTurnCardinals[k]) & 0xFFFF)) ++at_a_cardinal;
        }
        if (world != ((facing + 0x8000) & 0xFFFF)) ++not_a_half_circle;
      }
    }

    ok(examined > 0, "there are states to aim a turnaround from at all, which is the count that "
                     "makes the rest of this block mean anything");
    ok(unaimed == 0,
       "and over every state the game produces, none of them is one no cardinal reverses - which "
       "is a result the run states rather than a number nobody counted");
    ok(at_a_cardinal == examined,
       "every turnaround is pressed at one of the four camera cardinals, which is what a thumb "
       "can reach and what `facing + 0x8000` is not");
    ok(not_a_half_circle > 0,
       "and most of them are not a half circle at all - the offset from a true 180 is exactly how "
       "far the camera sits off the facing");

    const int held = 7018;
    const int near_by = 21422, further = near_by + 100;
    search::Move a, b;
    const bool both = search::move_of("cup_b_settle_turnaround", held, &a, &near_by) &&
                      search::move_of("cup_b_settle_turnaround", held, &b, &further);
    ok(both && !a.action.empty() && !b.action.empty(), "the same facing looked up under two "
                                                       "cameras answers a press either way");
    if (both && !a.action.empty() && !b.action.empty()) {
      ok(((b.action[0].bearing - a.action[0].bearing) & 0xFFFF) == 100,
         "and moving the camera 100 moves the press 100, so the camera is what the turnaround is "
         "aimed off");
    }

    const int between = (held + 0x800) & 0xFFFF;
    search::Move none;
    ok(!search::turn_bearing(held, between, 0),
       "RED: a camera the game never produced has no cardinal that reverses him");
    ok(search::move_of("cup_b_settle_turnaround", held, &none, &between) && none.unaimed,
       "RED: and the row still exists and says the state is wrong, rather than disappearing out "
       "of the roster and taking the plan with it");
  }

  /* The near set is the tolerance plus the allowance, which is zero on level ground, hence a slope.
     The target is one allowance past the model's end, at half that tolerance. */
  const search::Slab tilted = search::slab(0.0, 0.0, 0.08, 8192.0);
  const search::Corridor along = search::corridor(0.0, 0.0, 0.0, 1200.0, 900.0);
  const search::Selection tilted_sel = search::select(tilted.mesh, along);
  const search::Grid tilted_grid = search::build(tilted_sel, along, open);
  const search::Stepped one =
      search::step_move(*roll, tilted_grid, tilted_sel, stored, 0.0, 0.0, 0.0, 0);
  const double owed = stored.threshold("dry_roll", one.sloped_frames, one.tabled);
  ok(one.ok && one.sloped_frames > 0 && owed > 0.0,
     "a roll over ground that is not level is owed an allowance at all, which is what the rest of "
     "this block is built on");

  /* The bound's slack on a level room is the check range's floor alone; a rise under 1/1024 is
     level to the game. */
  {
    search::Question flat_q;
    flat_q.target.x = 300.0;
    flat_q.target.z = 300.0;
    flat_q.tolerance = 1.0;
    flat_q.frames = roll->frames;
    flat_q.moves.push_back("dry_roll");
    const search::Slab level = search::slab(0.0, 0.0, 0.0, 8192.0);
    const search::Selection level_sel = search::select(level.mesh, along);
    const search::Grid level_grid = search::build(level_sel, along, open);
    const search::Slab gentle = search::slab(0.0, 0.0, 0.0005, 8192.0);
    const search::Selection gentle_sel = search::select(gentle.mesh, along);
    const search::Grid gentle_grid = search::build(gentle_sel, along, open);
    const search::Found on_level =
        search::search_tree(flat_q, base, level_grid, level_sel, stored);
    const search::Found on_gentle =
        search::search_tree(flat_q, base, gentle_grid, gentle_sel, stored);
    const search::Found on_slope =
        search::search_tree(flat_q, base, tilted_grid, tilted_sel, stored);
    search::Question none_q = flat_q;
    none_q.check_range = 0.0;
    search::Question twice_q = flat_q;
    twice_q.check_range = flat_q.check_range * 2.0;
    const search::Found level_none =
        search::search_tree(none_q, base, level_grid, level_sel, stored);
    const search::Found level_twice =
        search::search_tree(twice_q, base, level_grid, level_sel, stored);
    search::Question gentle_none = none_q;
    const search::Found on_gentle_none =
        search::search_tree(gentle_none, base, gentle_grid, gentle_sel, stored);
    ok(level_none.slack == 0.0 && on_gentle_none.slack == 0.0 && on_gentle.slack > 0.0,
       "RED: a level room's bound is widened by no slope, and so is one rising under the game's "
       "own step");
    ok(on_level.slack > 0.0 && level_twice.slack == 2.0 * on_level.slack,
       "RED: and by the check range's floor, in proportion to it");
    ok(on_slope.slack > on_level.slack && search::level_ground(level_grid) &&
           !search::level_ground(tilted_grid),
       "and a sloped room's by the slope as well");

    /* The bound cuts nothing the check range records: a target two rolls ahead plus most of their
       consult radius. */
    const search::Stepped first =
        search::step_move(*roll, level_grid, level_sel, stored, 0.0, 0.0, 0.0, 0);
    const search::Stepped second = search::step_move(*roll, level_grid, level_sel, stored,
                                                     first.x, first.y, first.z, first.facing);
    search::Question on_it;
    on_it.target.x = second.x;
    on_it.target.z = second.z;
    /* Not 0: at the origin the dominance cell is a subnormal whose index wraps. */
    on_it.tolerance = 1e-6;
    on_it.frames = first.frames + second.frames;
    on_it.moves.push_back("dry_roll");
    on_it.distance_bound = false;
    const search::Found probe = search::search_tree(on_it, base, level_grid, level_sel, stored);
    double radius = 0.0;
    for (size_t i = 0; i < probe.candidate.size(); ++i) {
      if (probe.candidate[i].path.size() == 2 && probe.candidate[i].distance == 0.0) {
        radius = probe.candidate[i].consult;
      }
    }
    const double run = std::hypot(second.x, second.z);
    search::Question past = on_it;
    if (run > 0.0) {
      past.target.x = second.x + second.x / run * radius * 0.9;
      past.target.z = second.z + second.z / run * radius * 0.9;
    }
    const search::Found uncut = search::search_tree(past, base, level_grid, level_sel, stored);
    search::Question pruned_q = past;
    pruned_q.distance_bound = true;
    const search::Found cut = search::search_tree(pruned_q, base, level_grid, level_sel, stored);
    auto landings = [](const search::Found& f) {
      std::vector<std::vector<double>> out;
      for (size_t i = 0; i < f.candidate.size(); ++i) {
        const search::Candidate& c = f.candidate[i];
        out.push_back({c.x, c.z, static_cast<double>(c.frames), static_cast<double>(c.facing)});
      }
      std::sort(out.begin(), out.end());
      return out;
    };
    ok(first.ok && second.ok && radius > 0.0 && !uncut.candidate.empty() &&
           uncut.count.bound_pruned == 0,
       "a plan only the check range records is in a run the distance bound cut nothing from");
    ok(landings(cut) == landings(uncut),
       "RED: and a run the bound did cut keeps every plan of it");

    /* A launch from under the floor lands early; never earlier than `fewest_frames`. */
    int launches = 0, early = 0;
    bool never_under = true;
    for (size_t i = 0; i < base.move.size(); ++i) {
      const search::BaseMove& row = base.move[i];
      if (!row.driven || row.air_from < 0 || row.air_to <= row.air_from) continue;
      ++launches;
      const int least = search::fewest_frames(row, stored, true);
      for (int depth = 5; depth <= 60; depth += 5) {
        const search::Stepped st = search::step_move(row, level_grid, level_sel, stored, 0.0,
                                                     -static_cast<double>(depth), 0.0, 0);
        if (!st.ok) continue;
        if (st.frames < row.frames) ++early;
        if (st.frames < least) never_under = false;
      }
    }
    ok(launches > 0 && early > 0,
       "a launch under a floor lands before its table's count, which is the case this is about");
    ok(never_under, "RED: and never before the fewest frames the bound divides by");
  }

  /* Root-motion frames count as sloped too; the quick spin is nearly all root. */
  const search::BaseMove* spin = base.of("target_slash_qs");
  if (spin != nullptr && spin->driven) {
    int moving = 0, carried_by_speed = 0;
    for (size_t f = 0; f < spin->step.size(); ++f) {
      const search::BaseFrame& bf = spin->step[f];
      if (bf.above > 0.0 || (bf.ahead == 0.0 && bf.side == 0.0)) continue;
      ++moving;
      if (bf.speed_ahead != 0.0 || bf.speed_side != 0.0) ++carried_by_speed;
    }
    const search::Stepped spun =
        search::step_move(*spin, tilted_grid, tilted_sel, stored, 0.0, 0.0, 0.0, 0);
    ok(spun.ok && carried_by_speed < moving && spun.sloped_frames == moving,
       "every moving frame of a move over a slope counts toward its allowance, the root's as well "
       "as the ones carried by speedF");
  } else {
    ok(false, "the quick spin is in the base table, so the slope count can be checked on it");
  }

  /* The plane table, off its nodes and off facing 0. */
  {
    const search::BaseMove* free_roll = base.of("dry_roll_r_free");
    const search::PlaneTable* pt = stored.plane_of("dry_roll_r_free");
    const search::MoveCal* mc = stored.of("dry_roll_r_free");
    if (free_roll != nullptr && free_roll->driven && pt != nullptr && mc != nullptr) {
      const int facing = 7777;
      const search::Slab plane = search::slab(0.0, 0.05, 0.12, 8192.0);
      const search::Corridor around = search::corridor(0.0, 0.0, 0.0, 0.0, 900.0);
      const search::Selection plane_sel = search::select(plane.mesh, around);
      const search::Grid plane_grid = search::build(plane_sel, around, open);
      search::Move m;
      search::move_of("dry_roll_r_free", facing, &m);
      tww_engine::Init init;
      init.pos.set(0.0f, static_cast<f32>(plane.at(0.0, 0.0)), 0.0f);
      init.shape_angle_y = static_cast<s16>(facing);
      init.travel_angle_y = static_cast<s16>(facing);
      init.proc = daPy_lk_c::daPyProc_WAIT_e;
      const search::Drive truth = search::drive(m, init, &plane.room);
      const search::Stepped st = search::step_move(*free_roll, plane_grid, plane_sel, stored,
                                                   init.pos.x, init.pos.y, init.pos.z, facing);
      const double miss = std::hypot(st.x - init.pos.x - truth.dx, st.z - init.pos.z - truth.dz);
      ok(truth.ok && st.ok && st.tabled && miss <= pt->worst,
         "a roll over one plane at a facing the table was not built at ends where the engine ends "
         "it, to within the table's measured worst");
      ok(st.tabled &&
             stored.threshold("dry_roll_r_free", st.sloped_frames, true) == pt->worst &&
             pt->worst < mc->worst,
         "and it is owed the table's measured worst, which is less than the scale model's");

      const search::Slab level = search::slab(0.0, 0.0, 0.0, 8192.0);
      const search::Selection level_sel = search::select(level.mesh, around);
      const search::Grid level_grid = search::build(level_sel, around, open);
      const search::Stepped flat = search::step_move(*free_roll, level_grid, level_sel, stored, 0.0,
                                                     0.0, 0.0, facing);
      ok(flat.ok && !flat.tabled && flat.sloped_frames == 0,
         "over level ground the table is never read");
    } else {
      ok(false, "the free roll has a plane table, so its answer over a slope can be checked");
    }
  }

  if (one.ok && owed > 0.0) {
    search::Question edge_q;
    edge_q.target.x = one.x;
    edge_q.target.z = one.z + owed;
    edge_q.tolerance = owed / 2.0;
    edge_q.frames = roll->frames;
    edge_q.moves.push_back("dry_roll");
    const search::Found narrow =
        search::search_tree(edge_q, base, tilted_grid, tilted_sel, stored);
    bool inside_the_allowance = !narrow.candidate.empty();
    bool past_the_tolerance = false;
    for (size_t i = 0; i < narrow.candidate.size(); ++i) {
      const search::Candidate& c = narrow.candidate[i];
      if (c.distance > edge_q.tolerance + c.allowance) inside_the_allowance = false;
      if (c.distance > edge_q.tolerance) past_the_tolerance = true;
    }
    ok(inside_the_allowance && past_the_tolerance && narrow.count.allowed > 0,
     "a path the model puts past the tolerance and inside its own measured error is in the near "
     "set, and is counted as being there on the allowance");
    ok(!narrow.candidate.empty() && narrow.slack > 0.0 &&
           narrow.candidate[0].allowance == owed,
       "with the allowance being the threshold the ladder measured for that move and nothing "
       "chosen");

    search::Question edge_greedy = edge_q;
    edge_greedy.greedy = true;
    if (narrow.candidate.empty()) return;
    const search::Verified kept = search::verify(narrow, edge_q, base, &tilted.room);
    const search::Verified dropped = search::verify(narrow, edge_greedy, base, &tilted.room);
    ok(kept.count.discarded == 0 &&
           kept.count.consults == static_cast<long long>(narrow.candidate.size()),
       "the switch off consults every one of them, so nothing the engine might confirm is lost");
    ok(dropped.count.discarded == narrow.count.allowed &&
           dropped.count.consults == kept.count.consults - narrow.count.allowed,
       "RED: and the switch on discards exactly the ones the allowance put there, which is the "
       "only site in the run that drops a candidate without consulting it");
    ok(dropped.answer.size() <= kept.answer.size() && dropped.greedy && !kept.greedy,
       "so the greedy list is never longer than the exact one, and each run says which it was");
  }

  ok(search::consults_per_confirmed(said.count) >= 1.0,
     "the wasted-verification figure is consults over confirmed answers, so it is never under one");
  search::VerifyCounters none;
  none.consults = 7;
  ok(search::consults_per_confirmed(none) < 0.0,
     "RED: and a run that confirmed nothing says so rather than dividing by it");

  /* A start typed with fewer digits than its float runs from the float, not float plus remainder. */
  {
    search::Candidate two_rolls;
    search::Edge rolled;
    rolled.row = -1;
    for (size_t i = 0; i < base.move.size(); ++i) {
      if (base.move[i].id == "dry_roll_r") rolled.row = static_cast<int>(i);
    }
    two_rolls.path.push_back(rolled);
    two_rolls.path.push_back(rolled);
    search::Found staged;
    staged.candidate.push_back(two_rolls);

    search::Question typed_short = q;
    typed_short.start_x = 209.62949;
    typed_short.start_z = 549.97955;
    typed_short.start_facing = 46991;
    search::Question exact = typed_short;
    exact.start_x = static_cast<double>(static_cast<float>(typed_short.start_x));
    exact.start_z = static_cast<double>(static_cast<float>(typed_short.start_z));

    const search::Verified from_typed = search::verify(staged, typed_short, base, &flat.room);
    const search::Verified from_exact = search::verify(staged, exact, base, &flat.room);
    bool ran = from_typed.consult.size() == 1 && from_exact.consult.size() == 1 &&
               from_typed.consult[0].stop.size() == 2 && from_exact.consult[0].stop.size() == 2;
    ok(ran, "two rolls run from both starts, which is what the two checks below compare");
    bool on_floats = ran, same = ran;
    for (size_t s = 0; ran && s < 2; ++s) {
      const search::Reached& a = from_typed.consult[0].stop[s];
      const search::Reached& b = from_exact.consult[0].stop[s];
      if (static_cast<double>(static_cast<float>(a.x)) != a.x ||
          static_cast<double>(static_cast<float>(a.z)) != a.z) {
        on_floats = false;
      }
      if (a.x != b.x || a.z != b.z || a.facing != b.facing) same = false;
    }
    ok(on_floats, "every stop of a plan whose start was typed short is a float the game can hold");
    ok(same, "and it is the same float, stop for stop, as the plan started from the start's float");
  }

  /* An edited price (`priced`) is paid by the walk and added to a checked plan's engine run. */
  {
    const search::BaseMove* turn = base.of("fine_turn");
    ok(turn != nullptr, "the C up turn is in the table, which the prices below are set on");
    std::map<std::string, int> costs;
    costs["dry_roll"] = roll->frames + 7;
    costs["fine_turn"] = 60;
    const search::BaseTable dear = search::priced(base, costs);
    const search::BaseMove* dear_roll = dear.of("dry_roll");
    const search::BaseMove* dear_turn = dear.of("fine_turn");
    ok(dear_roll != nullptr && dear_roll->frames == roll->frames + 7 &&
           dear_roll->surcharge == 7 && base.of("dry_roll")->frames == roll->frames,
       "a retimed move is priced on a copy, and the measured table is left as it was");
    ok(dear_turn != nullptr && dear_turn->frames == 60,
       "and the C up turn takes the price it was given rather than the catalogue's own");

    search::Candidate roll_then_turn;
    search::Edge r, c;
    r.row = -1;
    c.row = -1;
    for (size_t i = 0; i < base.move.size(); ++i) {
      if (base.move[i].id == "dry_roll") r.row = static_cast<int>(i);
      if (base.move[i].id == "fine_turn") c.row = static_cast<int>(i);
    }
    c.steps = 3;
    roll_then_turn.path.push_back(r);
    roll_then_turn.path.push_back(c);
    search::Found staged;
    staged.candidate.push_back(roll_then_turn);
    const search::Verified cheap = search::verify(staged, q, base, &flat.room);
    const search::Verified paid = search::verify(staged, q, dear, &flat.room);
    const bool both = cheap.consult.size() == 1 && paid.consult.size() == 1 &&
                      cheap.consult[0].stop.size() == 2 && paid.consult[0].stop.size() == 2;
    ok(both, "the plan runs at both prices, which is what the two checks below compare");
    if (both && turn != nullptr) {
      ok(paid.consult[0].stop[0].frames == cheap.consult[0].stop[0].frames + 7,
         "RED: the retimed roll costs its price in a checked plan - the engine's run plus the 7");
      ok(paid.consult[0].stop[1].frames == (60 - 1) + 3 &&
             cheap.consult[0].stop[1].frames == (turn->frames - 1) + 3,
         "RED: and the three-step C up turn costs (price - 1) + 3, at either price");
    }

    search::Question tight = q;
    tight.moves.clear();
    tight.moves.push_back("dry_roll");
    tight.frames = 2 * roll->frames;
    std::map<std::string, int> over;
    over["dry_roll"] = roll->frames + 1;
    const search::BaseTable too_dear = search::priced(base, over);
    const search::Found at_measured =
        search::search_tree(tight, base, flat_grid, flat_sel, stored);
    const search::Found at_price =
        search::search_tree(tight, too_dear, flat_grid, flat_sel, stored);
    ok(!at_measured.candidate.empty() && at_price.candidate.empty(),
       "RED: a frame budget two rolls wide finds the two rolls at their measured price and none at "
       "a price one frame over it");
  }

  /* The recall audit is exponential, so two moves and a shallow depth. */
  search::Question shallow = q;
  shallow.frames = 2 * roll->frames;
  const search::Found short_run = search::search_tree(shallow, base, flat_grid, flat_sel, search::Calibration::measured());
  const search::Verified short_said = search::verify(short_run, shallow, base, &flat.room);
  const search::Recall audit =
      search::recall_audit(shallow, base, flat_grid, flat_sel, stored, short_said, &flat.room);
  ok(audit.complete && audit.drives > 0 && !audit.truth.empty(),
     "the audit re-runs the same question with the engine in the model's place and finds the "
     "answers that are really there");
  ok(audit.lost.empty() && audit.gained.empty() && audit.skipped.empty(),
     "and over this question the approximation lost none of them, which is what recall means");

  search::Verified holed = short_said;
  if (!holed.answer.empty()) {
    holed.answer.pop_back();
    const search::Recall missed =
        search::recall_audit(shallow, base, flat_grid, flat_sel, stored, holed, &flat.room);
    ok(missed.lost.size() == 1,
       "RED: an answer list one short is reported as one answer lost, so the audit can see the "
       "failure it exists for");
  }
}

/** The float four bytes are, high byte first; not shared with the walk on purpose. */
float typed(unsigned b0, unsigned b1, unsigned b2, unsigned b3) {
  const unsigned bits = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
  float f = 0.0f;
  std::memcpy(&f, &bits, sizeof f);
  return f;
}

void y_range(const search::Region& kept, double* lo, double* hi) {
  *lo = 1e300;
  *hi = -1e300;
  for (size_t i = 1; i < kept.vert.size(); i += 3) {
    if (kept.vert[i] < *lo) *lo = kept.vert[i];
    if (kept.vert[i] > *hi) *hi = kept.vert[i];
  }
}

/* An address is a byte mask, not a box. */
void mask_tests() {
  /* Y of `__ 80 05 33`: a blank high byte allows heights an octave apart. */
  ok(typed(0x45, 0x80, 0x05, 0x33) == 4096.64990234375f &&
         typed(0x44, 0x80, 0x05, 0x33) == 1024.1624755859375f &&
         typed(0x43, 0x80, 0x05, 0x33) == 256.0406188964844f &&
         typed(0x42, 0x80, 0x05, 0x33) == 64.0101547241211f,
     "a blank high byte allows heights an octave apart, not an interval between them");
  ok(typed(0xC5, 0x80, 0x05, 0x33) == -4096.64990234375f &&
         typed(0x3E, 0x80, 0x05, 0x33) == 0.250039666891098f,
     "and the same going down, and on the other side of zero");
  ok(typed(0xC5, 0x9C, 0x40, 0x00) == -5000.0f && typed(0x00, 0x00, 0x00, 0x00) == 0.0f,
     "the seabed and the sea surface are bytes of their own, and the address matches neither");

  /* Three stacked floors; only the last is at a height the bytes allow. */
  geom::Mesh three;
  quad(three.ground, -400, -400, 400, 400, -5000.0f);
  quad(three.ground, -400, -400, 400, 400, 2000.0f);
  quad(three.ground, -400, -400, 400, 400, typed(0x45, 0x80, 0x05, 0x33));
  const search::Corridor c = search::corridor(-300, 0, 300, 0, 60);
  search::Limits limits;
  limits.cell = 25.0;
  const search::Selection s = search::select(three, c);
  const search::Grid g = search::build(s, c, limits);

  /* The hull the window sends (`spanOf` over those bytes). */
  search::Target box;
  box.ranged = true;
  box.x0 = -120.0; box.x1 = -80.0; box.z0 = -20.0; box.z1 = 20.0;
  box.has_y = true;
  box.y0 = -8.50841e37; box.y1 = 8.50841e37;

  search::Target hulled = box;
  ok(search::resolve(&hulled, g, s.ground, 0.0) && hulled.ground.kept > 0,
     "RED: the hull of those bytes keeps floor");
  double lo = 0.0, hi = 0.0;
  y_range(hulled.ground, &lo, &hi);
  ok(lo == -5000.0 && hi == static_cast<double>(typed(0x45, 0x80, 0x05, 0x33)),
     "RED: and it keeps all three floors, the seabed included - a box over a blank high byte "
     "excludes nothing at all");

  search::Target masked = box;
  masked.mask.known[1][1] = true; masked.mask.byte[1][1] = 0x80;
  masked.mask.known[1][2] = true; masked.mask.byte[1][2] = 0x05;
  masked.mask.known[1][3] = true; masked.mask.byte[1][3] = 0x33;
  ok(search::resolve(&masked, g, s.ground, 0.0) && masked.ground.kept > 0,
     "the bytes keep the floor at a height they allow");
  y_range(masked.ground, &lo, &hi);
  ok(std::fabs(lo - 4096.64990234375) < 1.001 && std::fabs(hi - 4096.64990234375) < 1.001,
     "and nothing else: the seabed and the floor at 2000 are not heights those bytes can hold");

  /* `__ 80 05 33` also allows 1.18e-38, and `kHeightSlack` is a whole unit, so 0 is kept. */
  geom::Mesh sea;
  quad(sea.ground, -400, -400, 400, 400, 0.0f);
  const search::Selection sea_sel = search::select(sea, c);
  const search::Grid sea_grid = search::build(sea_sel, c, limits);
  search::Target at_zero = masked;
  ok(search::resolve(&at_zero, sea_grid, sea_sel.ground, 0.0) && at_zero.ground.kept > 0,
     "a floor at zero is kept, because the values those bytes allow crowd in around it");

  /* The region is only a prune; at zero tolerance the landing's f32 must carry every typed byte. */
  search::Question exact;
  exact.target = at_zero;
  exact.tolerance = 0.0;
  const double on_bytes = static_cast<double>(typed(0x41, 0x80, 0x05, 0x33));
  ok(search::reaches(exact, -100.0, on_bytes, 0.0),
     "a landing on the typed Y bytes reaches");
  ok(!search::reaches(exact, -100.0, 0.0, 0.0),
     "a landing on the floor at zero does not, though the region kept that floor");
  ok(!search::reaches(exact, -100.0, 16.0, 0.0),
     "nor one at 16, a hair off 41 80 05 33 and inside the unit of slack");
  search::Question loose = exact;
  loose.tolerance = 0.5;
  ok(search::reaches(loose, -100.0, 16.0, 0.0) && !search::reaches(loose, -100.0, 17.0, 0.0),
     "and above zero the height is within the tolerance or it is not");
  /* The region cuts a blank byte as its hull; the landing's bytes hold the typed ones exactly. */
  search::Target comb_bytes;
  comb_bytes.mask.known[0][0] = true; comb_bytes.mask.byte[0][0] = 0xC2;
  comb_bytes.mask.known[0][1] = true; comb_bytes.mask.byte[0][1] = 0xC8;
  comb_bytes.mask.known[0][3] = true; comb_bytes.mask.byte[0][3] = 0x01;
  ok(comb_bytes.holds_bytes(static_cast<double>(typed(0xC2, 0xC8, 0x7A, 0x01)), 0.0, 0.0) &&
         !comb_bytes.holds_bytes(static_cast<double>(typed(0xC2, 0xC8, 0x7A, 0x02)), 0.0, 0.0),
     "a typed low byte is held exactly, whatever the blank one beside it is");

  /* A run straddling Y and Z is not a float: its miss is the gap between the four bytes as one
     number. */
  search::Target run4;
  run4.mask.known[1][1] = true; run4.mask.byte[1][1] = 0x80;
  run4.mask.known[1][2] = true; run4.mask.byte[1][2] = 0x05;
  run4.mask.known[1][3] = true; run4.mask.byte[1][3] = 0x33;
  run4.mask.known[2][0] = true; run4.mask.byte[2][0] = 0xC8;
  int at = 0, len = 0;
  ok(run4.mask.run(&at, &len) && at == 5 && len == 4, "a run is found where it starts");
  const double z_c8 = static_cast<double>(typed(0xC8, 0x40, 0x00, 0x00));
  ok(run4.miss(0.0, static_cast<double>(typed(0x41, 0x80, 0x05, 0x33)), z_c8) == 0,
     "a landing on the four bytes misses by nothing");
  ok(run4.miss(0.0, static_cast<double>(typed(0x42, 0x1A, 0x84, 0x00)), z_c8) ==
         static_cast<double>(0x800533C8LL - 0x1A8400C8LL),
     "and one on 1A 84 00 C8 misses by the gap between the two as numbers");
  search::Target run3 = run4;
  run3.mask.known[2][0] = false;
  ok(run3.mask.run(&at, &len) && at == 5 && len == 3 &&
         run3.miss(0.0, static_cast<double>(typed(0x41, 0x80, 0x05, 0x34)), 0.0) == 1.0,
     "a run shorter than four is an address, and its miss counts its own last byte as one");
  search::Target run5 = run4;
  run5.mask.known[2][1] = true; run5.mask.byte[2][1] = 0x40;
  ok(!run5.mask.run(&at, &len),
     "RED: a run of five bytes is not an address - four is the most one is");
  search::Target gap = run4;
  gap.mask.known[1][3] = false; gap.mask.known[2][1] = true;
  search::Target none;
  ok(!gap.mask.run(&at, &len) && !none.mask.run(&at, &len),
     "a hole in the run, or nothing typed, is not one");
  search::Target run4_gap = run4;
  ok(run4_gap.miss(0.0, static_cast<double>(typed(0x41, 0x80, 0x05, 0x33)),
                   static_cast<double>(typed(0xC7, 0x40, 0x00, 0x00))) == 1.0,
     "four bytes are a gap taken exactly, down to their last byte");

  search::Target elsewhere = box;
  elsewhere.mask.known[1][0] = true; elsewhere.mask.byte[1][0] = 0x44;
  elsewhere.mask.known[1][1] = true; elsewhere.mask.byte[1][1] = 0x80;
  elsewhere.mask.known[1][2] = true; elsewhere.mask.byte[1][2] = 0x05;
  elsewhere.mask.known[1][3] = true; elsewhere.mask.byte[1][3] = 0x33;
  ok(!search::resolve(&elsewhere, g, s.ground, 0.0) && elsewhere.ground.kept == 0,
     "a height with no floor at it keeps nothing, whatever the hull around it holds");

  /* Aimed at the item, carried 140 over his feet, the floor that fits 256.04 is at 116.04. */
  {
    const float allowed = typed(0x43, 0x80, 0x05, 0x33);
    geom::Mesh two;
    quad(two.ground, -400, -400, 400, 400, allowed);
    quad(two.ground, -400, -400, 400, 400, allowed - 140.0f);
    const search::Selection ts = search::select(two, c);
    const search::Grid tg = search::build(ts, c, limits);
    search::Target at_256 = box;
    at_256.mask.known[1][0] = true; at_256.mask.byte[1][0] = 0x43;
    at_256.mask.known[1][1] = true; at_256.mask.byte[1][1] = 0x80;
    at_256.mask.known[1][2] = true; at_256.mask.byte[1][2] = 0x05;
    at_256.mask.known[1][3] = true; at_256.mask.byte[1][3] = 0x33;

    search::Question player, item;
    item.aim = search::Aim::Overhead;
    search::Target feet = at_256;
    feet.aim_lift = search::aim_lift(player);
    feet.aim_reach = search::aim_reach(player);
    ok(search::resolve(&feet, tg, ts.ground, 0.0), "aimed at him, the bytes keep floor");
    y_range(feet.ground, &lo, &hi);
    ok(std::fabs(lo - allowed) < 1.001 && std::fabs(hi - allowed) < 1.001,
       "aimed at him, the floor kept is the one at the height the bytes allow");

    search::Target carried = at_256;
    carried.aim_lift = search::aim_lift(item);
    carried.aim_reach = search::aim_reach(item);
    ok(search::aim_lift(item) == 140.0 && search::aim_lift(player) == 0.0,
       "the item is carried 140 over his feet, and he is carried over nothing");
    ok(search::resolve(&carried, tg, ts.ground, 0.0), "aimed at the item, the bytes keep floor");
    y_range(carried.ground, &lo, &hi);
    ok(std::fabs(lo - (allowed - 140.0)) < 1.001 && std::fabs(hi - (allowed - 140.0)) < 1.001,
       "RED: aimed at the item, the floor kept is the one 140 under that height, and not the "
       "floor at it");

    /* `distance` cannot tell the two floors apart; `distance_here` must. */
    ok(carried.distance(-100.0, 0.0) == 0.0 &&
           carried.distance_here(-100.0, allowed - 140.0, 0.0) == 0.0,
       "on the floor that fits, the goal is here");
    ok(std::isinf(carried.distance_here(-100.0, allowed, 0.0)),
       "RED: and on the floor over it, the goal is not on his floor at all");

    /* A steep ramp elsewhere must not widen a flat floor 26.7 short into a match. */
    geom::Mesh flat_and_steep;
    quad(flat_and_steep.ground, -140, -40, -60, 40, (allowed - 140.0f) - 26.7f);
    flat_and_steep.ground.insert(flat_and_steep.ground.end(),
                                 {100.0f, (allowed - 140.0f) - 150.0f, -40.0f, 300.0f, (allowed - 140.0f) + 190.0f, 40.0f,
                                  300.0f, (allowed - 140.0f) + 190.0f, -40.0f});
    flat_and_steep.ground.insert(flat_and_steep.ground.end(),
                                 {100.0f, (allowed - 140.0f) - 150.0f, -40.0f, 100.0f, (allowed - 140.0f) - 150.0f, 40.0f,
                                  300.0f, (allowed - 140.0f) + 190.0f, 40.0f});
    const search::Selection fs = search::select(flat_and_steep, c);
    const search::Grid fg = search::build(fs, c, limits);
    search::Target across = carried;
    across.x0 = -300.0; across.x1 = 300.0;
    ok(search::resolve(&across, fg, fs.ground, 0.0), "the steep ramp crosses the height");
    bool flat_kept = false;
    for (size_t i = 0; i < across.ground.vert.size(); i += 3) {
      if (across.ground.vert[i] < 0.0) flat_kept = true;
    }
    ok(!flat_kept,
       "RED: and the flat floor 26.7 short of it is not kept, though the ramp is steep enough to "
       "have covered it from under his feet");

    /* Two floors that both fit `__ 80 05 33`, carried: 116.04 and -75.99. */
    geom::Mesh levels;
    const float high = typed(0x43, 0x80, 0x05, 0x33) - 140.0f;
    const float low = typed(0x42, 0x80, 0x05, 0x33) - 140.0f;
    quad(levels.ground, -140, -40, -60, 40, high);
    quad(levels.ground, 0, -40, 200, 40, low);
    const search::Selection ls = search::select(levels, c);
    const search::Grid lg = search::build(ls, c, limits);
    search::Target two_floors = at_256;
    two_floors.mask.known[1][0] = false;
    two_floors.x0 = -300.0; two_floors.x1 = 300.0;
    two_floors.aim_lift = search::aim_lift(item);
    two_floors.aim_reach = search::aim_reach(item);
    ok(search::resolve(&two_floors, lg, ls.ground, 0.0), "both floors fit those bytes");
    ok(two_floors.distance(-100.0, 0.0) == 0.0 &&
           two_floors.distance_here(-100.0, high, 0.0) == 0.0,
       "under the high floor, the goal is overhead, and on it the goal is here");
    ok(std::fabs(two_floors.distance_here(-100.0, low, 0.0) - 100.0) < 1e-9,
       "RED: on the low floor's level, the goal on his floor is the low stretch a hundred off, "
       "not the floor overhead");

    /* A ramp is cut only where his feet cross the height; the reach does not widen it in y. */
    geom::Mesh ramp;
    const float mid = allowed - 140.0f;
    ramp.ground.insert(ramp.ground.end(), {-400.0f, mid - 50.0f, -400.0f, 400.0f, mid + 50.0f, 400.0f,
                                           400.0f, mid + 50.0f, -400.0f});
    ramp.ground.insert(ramp.ground.end(), {-400.0f, mid - 50.0f, -400.0f, -400.0f, mid - 50.0f, 400.0f,
                                           400.0f, mid + 50.0f, 400.0f});
    const search::Selection rs = search::select(ramp, c);
    const search::Grid rg = search::build(rs, c, limits);
    search::Target sloped = carried;
    sloped.x0 = -300.0; sloped.x1 = 300.0;
    ok(search::resolve(&sloped, rg, rs.ground, 0.0), "aimed at the item, a slope through the "
                                                     "height keeps floor");
    y_range(sloped.ground, &lo, &hi);
    ok(lo >= mid - 1.001 && hi <= mid + 1.001,
       "and keeps only the strip where his feet are at that height, a unit either side");

    /* In x and z the region is widened by the item's reach. */
    ok(carried.from_feet && !feet.from_feet,
       "aimed at the item an address is answered where he stands, and aimed at him it is not");
    double wx0 = 1e300, wx1 = -1e300;
    for (size_t i = 0; i < carried.ground.vert.size(); i += 3) {
      wx0 = std::min(wx0, carried.ground.vert[i]);
      wx1 = std::max(wx1, carried.ground.vert[i]);
    }
    const double r = search::aim_reach(item);
    ok(std::fabs(wx0 - (-120.0 - r)) < 1e-6 && std::fabs(wx1 - (-80.0 + r)) < 1e-6,
       "RED: where he stands reaches the item's reach past the box on each side");

    search::Question carried_q;
    carried_q.aim = search::Aim::Overhead;
    carried_q.target = carried;
    carried_q.tolerance = 0.0;
    ok(search::reaches(carried_q, 300.0, static_cast<double>(allowed), 0.0),
       "RED: an item on the typed bytes reaches, though it is nowhere over the floor he stands on");
    ok(!search::reaches(carried_q, -100.0, static_cast<double>(allowed) + 1.0, 0.0),
       "and one over that floor off the bytes does not");
  }

  /* A tie on distance is broken on the bytes. */
  ok(search::closeness(0.0, 5.0, 0.0, 9.0) < 0 && search::closeness(0.0, 9.0, 0.0, 5.0) > 0,
     "RED: at the same distance, the smaller gap in the bytes is nearer");
  ok(search::closeness(1.0, 0.0, 2.0, 99.0) < 0 && search::closeness(0.0, 3.0, 0.0, 3.0) == 0,
     "and distance still comes first, and a tie on both is a tie");

  /* Below the model's measured error the bytes are rounding noise, so a tie. */
  ok(search::beyond_error(0.5, 1.0) == 0.0 && search::beyond_error(0.9, 1.0) == 0.0,
     "RED: two misses inside the model's error are the same miss");
  ok(search::beyond_error(3.0, 1.0) == 2.0, "and one outside it is what lies past it");
  {
    search::Target y_bytes;
    y_bytes.mask.known[1][0] = true; y_bytes.mask.byte[1][0] = 0x43;
    y_bytes.mask.known[1][1] = true; y_bytes.mask.byte[1][1] = 0x80;
    y_bytes.mask.known[1][2] = true; y_bytes.mask.byte[1][2] = 0x05;
    y_bytes.mask.known[1][3] = true; y_bytes.mask.byte[1][3] = 0x33;
    geom::Mesh one;
    quad(one.ground, -400, -400, 400, 400, 116.0f);
    const search::Corridor oc = search::corridor(-300, 0, 300, 0, 60);
    search::Limits ol;
    ol.cell = 25.0;
    const search::Selection os = search::select(one, oc);
    const search::Grid og = search::build(os, oc, ol);
    y_bytes.ranged = true;
    y_bytes.x0 = -100; y_bytes.x1 = 100; y_bytes.z0 = -20; y_bytes.z1 = 20;
    y_bytes.aim_lift = 140.0;
    search::resolve(&y_bytes, og, os.ground, 0.0);
    const double at = static_cast<double>(typed(0x43, 0x80, 0x05, 0x33));
    ok(y_bytes.bytes_off(0.0, at, 0.0) == 0.0 &&
           std::fabs(y_bytes.bytes_off(5.0, at + 1.5, -7.0) - 1.5) < 1e-9,
       "RED: the miss from the bytes is in units of height, and an untyped axis adds nothing");
  }

  /* A blank high byte on x leaves lines with no area (`Region::solid`): `__ 80 00 00` over 50 to
     300 is 64 and 256. */
  geom::Mesh strip;
  quad(strip.ground, 50, -50, 300, 50, 0.0f);
  search::Target comb;
  comb.ranged = true;
  comb.x0 = 50.0; comb.x1 = 300.0; comb.z0 = -50.0; comb.z1 = 50.0;
  const search::Corridor sc = search::corridor(0, 0, comb, strip.box, 60);
  const search::Selection ss = search::select(strip, sc);
  const search::Grid sg = search::build(ss, sc, limits);

  search::Target whole = comb;
  ok(search::resolve(&whole, sg, ss.ground, 0.0) && whole.distance(150.0, 0.0) == 0.0,
     "RED: with no bytes typed the whole strip is the goal, 150 included");

  comb.mask.known[0][1] = true; comb.mask.byte[0][1] = 0x80;
  comb.mask.known[0][2] = true; comb.mask.byte[0][2] = 0x00;
  comb.mask.known[0][3] = true; comb.mask.byte[0][3] = 0x00;
  ok(search::resolve(&comb, sg, ss.ground, 0.0) && comb.ground.kept > 0,
     "the bytes on x keep the floor where it crosses the values they allow");
  bool only_allowed = true;
  bool any_solid = false;
  for (size_t i = 0; i < comb.ground.vert.size(); i += 3) {
    const double vx = comb.ground.vert[i];
    if (std::fabs(vx - 64.0) > 1e-3 && std::fabs(vx - 256.0) > 1e-3) {
      only_allowed = false;
    }
  }
  for (size_t i = 0; i < comb.ground.solid.size(); ++i) {
    if (comb.ground.solid[i] != 0) any_solid = true;
  }
  ok(only_allowed && !any_solid,
     "and they are lines on the floor - every vertex at a value the bytes hold, and no polygon "
     "with an inside to be in");
  /* Not `== 0.0`: a clipped vertex is interpolated, a few parts in 1e15 off. */
  ok(comb.distance(64.0, 0.0) < 1e-9 && comb.distance(150.0, 0.0) > 1.0,
     "so a place on one of those lines has arrived and a place between two of them has not");

  search::Target blank = comb;
  blank.mask = search::Mask();
  ok(search::resolve(&blank, sg, ss.ground, 0.0) && blank.distance(150.0, 0.0) == 0.0,
     "no byte typed leaves the region the box");

  /* The bytes cut with no span beside them, inside the room's own floor range. */
  search::Target no_span;
  no_span.ranged = true;
  no_span.x0 = -120.0; no_span.x1 = -80.0; no_span.z0 = -20.0; no_span.z1 = 20.0;
  no_span.mask.known[1][1] = true; no_span.mask.byte[1][1] = 0x80;
  no_span.mask.known[1][2] = true; no_span.mask.byte[1][2] = 0x05;
  no_span.mask.known[1][3] = true; no_span.mask.byte[1][3] = 0x33;
  ok(!no_span.has_y, "the bytes arrive with no height span beside them");
  ok(search::resolve(&no_span, g, s.ground, 0.0) && no_span.ground.kept > 0,
     "and they still cut the floor");
  y_range(no_span.ground, &lo, &hi);
  ok(std::fabs(lo - 4096.64990234375) < 1.001 && std::fabs(hi - 4096.64990234375) < 1.001,
     "to the one height they allow, and not to every floor under the question");

  search::Target not_a_number;
  not_a_number.ranged = true;
  not_a_number.x0 = -120.0; not_a_number.x1 = -80.0; not_a_number.z0 = -20.0;
  not_a_number.z1 = 20.0;
  not_a_number.mask.known[1][0] = true; not_a_number.mask.byte[1][0] = 0x7F;
  not_a_number.mask.known[1][1] = true; not_a_number.mask.byte[1][1] = 0x80;
  ok(!search::resolve(&not_a_number, g, s.ground, 0.0) && not_a_number.ground.kept == 0,
     "bytes that are not a number at all keep no floor, rather than keeping all of it");

  /* A typed low byte under blank high bytes is a comb, kept as its places, never its hull. */
  geom::Mesh tile;
  quad(tile.ground, 1000, -100, 1400, 100, 0.0f);
  search::Target comb_x;
  comb_x.ranged = true;
  comb_x.x0 = 1000.0; comb_x.x1 = 1400.0;
  comb_x.z0 = -100.0; comb_x.z1 = 100.0;
  const search::Corridor tc = search::corridor(1000, 0, comb_x, tile.box, 60);
  const search::Selection ts = search::select(tile, tc);
  search::Limits wide;
  wide.cell = 50.0;
  const search::Grid tg = search::build(ts, tc, wide);
  search::Target coarse = comb_x;
  ok(search::resolve(&coarse, tg, ts.ground, 0.0) && coarse.ground.kept > 0,
     "RED: the same question with no byte typed keeps the tile");
  comb_x.mask.known[0][3] = true; comb_x.mask.byte[0][3] = 0x33;
  ok(search::resolve(&comb_x, tg, ts.ground, 0.0) && comb_x.ground.kept > 100,
     "at zero it keeps the places that hold the byte");
  bool every_one_holds = true;
  for (size_t i = 0; i < comb_x.ground.vert.size(); i += 3) {
    const float x = static_cast<float>(comb_x.ground.vert[i]);
    uint32_t bits = 0;
    std::memcpy(&bits, &x, sizeof bits);
    if ((bits & 0xFFu) != 0x33u) every_one_holds = false;
  }
  ok(every_one_holds,
     "RED: and every place it keeps holds the typed byte - never widened back to its hull");

  /* A hull 8.5e37 wide is clamped to the room, or the grid would be one cell. */
  const float room_box[6] = {-400.0f, -50.0f, -400.0f, 400.0f, 50.0f, 400.0f};
  search::Target huge;
  huge.ranged = true;
  huge.x0 = -8.50841e37; huge.x1 = 8.50841e37;
  huge.z0 = -8.50841e37; huge.z1 = 8.50841e37;
  const search::Corridor wide_c = search::corridor(0, 0, huge, room_box, 60);
  ok(wide_c.boxed && wide_c.gx0 == -400.0 && wide_c.gx1 == 400.0 && wide_c.gz0 == -400.0 &&
         wide_c.gz1 == 400.0,
     "a goal wider than the room is bounded by the room, so the corridor stays a corridor");
  /* The clamp is in `corridor`, not `extent`: `resolve` hands `extent` the grid laid over it. */
  search::Target small = huge;
  small.x0 = -100.0; small.x1 = 100.0; small.z0 = -30.0; small.z1 = 30.0;
  const search::Corridor small_c = search::corridor(0, 0, small, room_box, 60);
  ok(small_c.gx0 == -100.0 && small_c.gx1 == 100.0 && small_c.gz0 == -30.0 &&
         small_c.gz1 == 30.0,
     "and a goal inside the room is left exactly as it was asked");
}
}  // namespace

/** `cup_exit_table.inc` against console captures: B on 17 turns from facing 0, C-down on 7. The
 *  settle counts from the view's end; the stretch is 7 frames wherever a creep follows. */
void cup_exit_tests() {
  struct Console {
    int steps, cdown, settled, settle, first;
  };
  const Console turns[] = {
      {1, -1, 652, 0, 652},         {2, 1298, 1298, 7, 1307},     {3, -1, 1963, 0, 1963},
      {6, -1, 3921, 0, 3921},       {12, -1, 7856, 0, 7856},      {14, 9177, 9206, 22, 9170},
      {23, 15077, 15117, 25, 15067}, {25, -1, 16384, 7, 16374},   {31, -1, 20295, 0, 20295},
      {39, -1, 25584, 18, 25549},   {40, 26209, 26254, 26, 26201}, {48, 31451, 31501, 27, 31441},
      {50, -1, 32768, 7, 32758},    {-12, -1, 57680, 0, 57680},   {-14, 56359, 56330, 22, 56366},
      {-31, -1, 45241, 0, 45241},   {-40, 39327, 39282, 26, 39335},
  };
  int b = 0, stretch = 0, cdown = 0, cdowns = 0, naive = 0;
  for (const Console& c : turns) {
    const int dir = c.steps < 0 ? -1 : 1;
    const int end = (655 * c.steps) & 0xFFFF;
    cup_exit::Leave s, f, d;
    const bool read = cup_exit::leave(end, dir, cup_exit::Way::Settled, &s) &&
                      cup_exit::leave(end, dir, cup_exit::Way::Stretch, &f) &&
                      cup_exit::leave(end, dir, cup_exit::Way::CDown, &d);
    if (read && s.csangle == c.settled && s.frames - s.wait - 2 == c.settle) ++b;
    const int window = c.first == c.settled ? 0 : 7;
    if (read && f.csangle == c.first && f.window == window) ++stretch;
    if (c.cdown >= 0) {
      ++cdowns;
      if (read && d.csangle == c.cdown) ++cdown;
    }
    if (c.settled == end) ++naive;
  }
  ok(b == 17, "cup exit: B settles where the console did, as late as it did, on " +
                  std::to_string(b) + " of 17 turns");
  ok(stretch == 17, "cup exit: the first stretch after B is the console's value and length on " +
                        std::to_string(stretch) + " of 17");
  ok(cdown == cdowns && cdowns == 7,
     "cup exit: C-down holds the console's value on " + std::to_string(cdown) + " of " +
         std::to_string(cdowns));
  /* The naive rule, camera on the facing, must disagree with the console. */
  ok(naive < 17, "cup exit: the camera on the facing is not what the console did, on " +
                     std::to_string(17 - naive) + " of 17");

  /* A lowering turn at F reads the raising turn at -F, negated. */
  int mirrored = 0;
  for (int end = 0; end < 65536; end += 97) {
    cup_exit::Leave up, down;
    const int m = (0x10000 - end) & 0xFFFF;
    if (cup_exit::leave(end, 1, cup_exit::Way::Settled, &up) &&
        cup_exit::leave(m, -1, cup_exit::Way::Settled, &down) &&
        ((up.csangle + down.csangle) & 0xFFFF) == 0 && up.frames == down.frames) {
      ++mirrored;
    }
  }
  ok(mirrored == (65536 + 96) / 97, "cup exit: a lowering turn reads the raising turn's mirror");

  /* A 14-step C up turn from facing 0, then each way out: walk, model and engine agree. */
  const search::BaseTable base = search::base_table(false);
  const search::Slab flat = search::slab(0.0, 0.0, 0.0, 8192.0);
  const search::Corridor wide = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
  const search::Selection sel = search::select(flat.mesh, wide);
  search::Limits open;
  open.slope_normal_y = 0.0;
  open.height_step = 1e9;
  open.normal_apart = 1e9;
  const search::Grid grid = search::build(sel, wide, open);
  const search::BaseMove* fine = base.of("fine_turn");
  const char* ways[] = {"cup_cdown_turnaround", "cup_b_settle_turnaround",
                        "cup_b_early_turnaround"};
  const cup_exit::Way way_of[] = {cup_exit::Way::CDown, cup_exit::Way::Settled,
                                  cup_exit::Way::Stretch};
  int found_it = 0, confirmed = 0, modelled = 0;
  for (int w = 0; w < 3 && fine != nullptr; ++w) {
    cup_exit::Leave l;
    int reversed = 0;
    if (!cup_exit::leave(9170, 1, way_of[w], &l) || !search::turn_bearing(9170, l.csangle, &reversed)) {
      continue;
    }
    const search::BaseMove* block = base.of(ways[w]);
    if (block == nullptr) continue;
    search::Question q;
    q.start_facing = 0;
    q.has_camera = true;
    q.start_camera = 0;
    q.target.has_x = false;
    q.target.has_z = false;
    q.tolerance = 0.0;
    q.steps = 2;
    q.moves.push_back("fine_turn");
    q.moves.push_back(ways[w]);
    q.end_facing.any = false;
    q.end_facing.span = 0;
    q.end_facing.a = reversed;
    const int price = (fine->frames - 1) + 14 + block->frames + l.frames;
    q.frames = price;
    const search::Found run = search::search_tree(q, base, grid, sel,
                                                  search::Calibration::measured());
    /* At most the price: another order can reach the same facing for less. */
    bool here = false;
    for (const search::Candidate& c : run.candidate) {
      if (c.facing == reversed && c.frames <= price) here = true;
    }
    if (here) ++found_it;
    search::Pose was, turned, left;
    was.facing = 0;
    if (search::model_step("fine_turn", fine->turn, 14, was, &turned) &&
        search::model_step(ways[w], 0, 1, turned, &left) && left.facing == reversed &&
        left.camera == l.csangle && left.frames == l.frames) {
      ++modelled;
    }
    const search::Verified said = search::verify(run, q, base, &flat.room);
    for (const search::Consult& c : said.consult) {
      if (c.outcome == search::Outcome::Confirmed && c.facing == reversed && c.frames <= price) {
        ++confirmed;
        break;
      }
    }
  }
  ok(modelled == 3, "cup exit: a C up turn then each way out lands on the table's facing and "
                    "frames, on " + std::to_string(modelled) + " of 3");
  ok(found_it == 3, "cup exit: the walk reaches each way out's facing for no more than its price, on " +
                        std::to_string(found_it) + " of 3");
  ok(confirmed == 3, "cup exit: and the engine confirms it on the same facing and frames, on " +
                         std::to_string(confirmed) + " of 3");

  /* With no turn before it, a way out reads the no-turn row and needs no carried camera. */
  int alone_found = 0, alone_confirmed = 0;
  for (int w = 0; w < 3; ++w) {
    cup_exit::Leave l;
    int reversed = 0;
    if (!cup_exit::leave(9170, 0, way_of[w], &l) || !search::turn_bearing(9170, l.csangle, &reversed)) {
      continue;
    }
    const search::BaseMove* block = base.of(ways[w]);
    if (block == nullptr) continue;
    search::Question alone;
    alone.start_facing = 9170;
    alone.has_camera = false;
    alone.target.has_x = false;
    alone.target.has_z = false;
    alone.tolerance = 0.0;
    alone.steps = 1;
    alone.moves.push_back(ways[w]);
    alone.end_facing.any = false;
    alone.end_facing.span = 0;
    alone.end_facing.a = reversed;
    const int price = block->frames + l.frames;
    alone.frames = price;
    const search::Found run = search::search_tree(alone, base, grid, sel,
                                                  search::Calibration::measured());
    for (const search::Candidate& c : run.candidate) {
      if (c.facing == reversed && c.frames == price && c.path.size() == 1) {
        ++alone_found;
        break;
      }
    }
    const search::Verified said = search::verify(run, alone, base, &flat.room);
    for (const search::Consult& c : said.consult) {
      if (c.outcome == search::Outcome::Confirmed && c.facing == reversed && c.frames == price) {
        ++alone_confirmed;
        break;
      }
    }
  }
  ok(alone_found == 3 && alone_confirmed == 3,
     "cup exit: with no C up turn before it, each way out reaches and confirms the no-turn row, on " +
         std::to_string(alone_found) + " and " + std::to_string(alone_confirmed) + " of 3");

  search::Move plain;
  const int any_camera = 0;
  ok(!search::move_of("turnaround", 0, &plain, &any_camera) && base.of("turnaround") == nullptr,
     "cup exit: RED: the plain turnaround is in neither the roster nor the table");

  /* `moves.ts` shows each way out at the table's lower median price. */
  {
    std::string app = __FILE__;
    app = app.substr(0, app.find_last_of("/\\")) + "/../../app/src/core/moves.ts";
    std::string text;
    if (FILE* f = std::fopen(app.c_str(), "rb")) {
      char buf[4096];
      size_t n = 0;
      while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
      std::fclose(f);
    }
    int held = 0;
    std::string wanted;
    for (int w = 0; w < 3; ++w) {
      std::vector<int> prices;
      for (int dir = -1; dir <= 1; ++dir) {
        for (int end = 0; end < 65536; ++end) {
          cup_exit::Leave l;
          if (cup_exit::leave(end, dir, way_of[w], &l)) prices.push_back(l.frames);
        }
      }
      std::sort(prices.begin(), prices.end());
      const int median = prices.empty() ? 0 : prices[(prices.size() - 1) / 2];
      const search::BaseMove* block = base.of(ways[w]);
      const std::string row = std::string("{id:'") + ways[w] + "'";
      const size_t at = text.find(row);
      const size_t fr = at == std::string::npos ? at : text.find("frames:", at);
      const int want = block != nullptr ? block->frames + median : -1;
      wanted += std::string(w ? ", " : "") + ways[w] + " " + std::to_string(want);
      if (block != nullptr && fr != std::string::npos && std::atoi(text.c_str() + fr + 7) == want) {
        ++held;
      }
    }
    ok(!text.empty() && held == 3,
       "cup exit: moves.ts shows each way out at the median price the table holds, on " +
           std::to_string(held) + " of 3 (" + wanted + ")");
  }
}

void camera_type_tests() {
  /* Every rung of the lookup answers a different name, so a fall-through shows. */
  disc::CameraNames names;
  names.stage_id = 1;
  names.stage_list = {"Room", "OverLook"};
  names.room_list = {"DStairs"};
  std::string why;
  auto name_of = [](int type) {
    return type >= 0 ? std::string(dCamera_c::types[type].name) : std::string("refused");
  };
  const search::FloorCamera none;
  ok(name_of(search::camera_type(names, none, &why)) == "OverLook",
     "camera type: a floor naming no camera runs the stage's own type");
  search::FloorCamera room;
  room.id = 0;
  room.room = 0;
  ok(name_of(search::camera_type(names, room, &why)) == "DStairs",
     "camera type: an id from the room cam id reads the room's list");
  search::FloorCamera stage;
  stage.id = 0;
  ok(name_of(search::camera_type(names, stage, &why)) == "Room",
     "camera type: an id from the poly cam id reads the stage's list, not the room's or the default");
  search::FloorCamera past;
  past.id = 5;
  past.room = 0;
  ok(name_of(search::camera_type(names, past, &why)) == "OverLook",
     "camera type: an id past its list falls back to the stage's own type");
  search::FloorCamera moving;
  moving.cam_move_bg = 21;
  ok(name_of(search::camera_type(names, moving, &why)) == "MiniIsland",
     "camera type: cam-move BG 21 is mvBGTypes' MiniIsland");
  search::FloorCamera high;
  high.id = 0x1FF;
  ok(search::camera_type(names, high, &why) == -1,
     "camera type: a player too far above the floor keeps a type a posed room does not know - refused");
  disc::CameraNames bare;
  ok(search::camera_type(bare, none, &why) == 7,
     "camera type: a stage whose id reaches no entry is Field, type 7");
  disc::CameraNames keep = names;
  keep.room_list = {"Keep"};
  ok(search::camera_type(keep, room, &why) == -1, "camera type: Keep is refused, not guessed");
}

namespace {

/* SETUPCORE_ISO, else the app's settings; empty when neither names one. */
std::string the_disc() {
  const char* env = std::getenv("SETUPCORE_ISO");
  if (env != nullptr && *env) return env;
  std::string json, why;
  if (!settings::read(&json, &why) || json.empty()) return "";
  const seam::Json j = seam::parse(json);
  return j.ok() && j.has("iso") ? j.at("iso").as_str() : "";
}

/* Entries in a file's `RCAM` (and, in a stage file, `CAMR`) chunks, not via the reader under test. */
uint32_t camera_entries(const std::vector<uint8_t>& file, bool stage_file) {
  if (file.size() < 4) return 0;
  const uint32_t n = geom::be32(&file[0]);
  uint32_t most = 0;
  for (uint32_t c = 0; c < n && 4 + (c + 1) * 12 <= file.size(); ++c) {
    const uint8_t* head = &file[4 + c * 12];
    const bool is = std::memcmp(head, "RCAM", 4) == 0 || (stage_file && std::memcmp(head, "CAMR", 4) == 0);
    if (is) most = std::max(most, geom::be32(head + 4));
  }
  return most;
}

}  // namespace

void camera_list_tests() {
  const std::string path = the_disc();
  std::string why;
  const auto iso = path.empty() ? nullptr : disc::Iso::open(path, &why);
  ok(iso != nullptr, "camera lists: the disc opened (SETUPCORE_ISO, or the app's own setting) - " +
                         (path.empty() ? std::string("none is named") : path));
  if (!iso) return;
  const std::vector<disc::Where> rooms = disc::rooms_on(*iso);
  std::set<std::string> stages_seen;
  int stages = 0, rooms_read = 0, camr = 0, lost = 0;
  std::string first_lost;
  for (const disc::Where& w : rooms) {
    disc::CameraNames names;
    if (!disc::camera_names_of(*iso, w.stage, w.room, &names, &why)) continue;
    std::vector<uint8_t> archive, file;
    if (stages_seen.insert(w.stage).second &&
        iso->file("res/Stage/" + w.stage + "/Stage.arc", &archive) &&
        disc::from_rarc(archive, ".dzs", &file)) {
      ++stages;
      if (camera_entries(file, true) > 0 && camera_entries(file, false) == 0) ++camr;
      if (camera_entries(file, true) > 0 && names.stage_list.empty()) {
        ++lost;
        if (first_lost.empty()) first_lost = w.stage;
      }
    }
    if (iso->file("res/Stage/" + w.stage + "/Room" + std::to_string(w.room) + ".arc", &archive) &&
        disc::from_rarc(archive, ".dzr", &file)) {
      ++rooms_read;
      if (camera_entries(file, false) > 0 && names.room_list.empty()) {
        ++lost;
        if (first_lost.empty()) first_lost = w.stage + " room " + std::to_string(w.room);
      }
    }
  }
  ok(stages > 0 && rooms_read > 0, "camera lists: read " + std::to_string(stages) + " stage file(s) and " +
                                       std::to_string(rooms_read) + " room file(s)");
  ok(lost == 0, "camera lists: every file holding a camera list comes back with it (" +
                    std::to_string(camr) + " stage file(s) keep theirs only as CAMR); lost " +
                    std::to_string(lost) + (first_lost.empty() ? "" : ", first " + first_lost));
  /* On console, Kaisen's camera is type 57, `Room`. */
  disc::CameraNames kaisen;
  ok(disc::camera_names_of(*iso, "Kaisen", 0, &kaisen, &why) && kaisen.stage_id == 0 &&
         search::camera_type(kaisen, search::FloorCamera(), &why) == 57,
     "camera lists: a Kaisen floor naming no camera is the Room camera the console reads there");
}

void l_chain_tests() {
  /* Console values at facing 9170. */
  int v0 = 0, v1 = 0, vc = 0, f0 = 0, f1 = 0;
  const bool h0 = l_chain::held(9170, 0, &v0, &f0);
  const bool h1 = l_chain::held(9170, 1, &v1, &f1);
  const bool c1 = l_chain::cdown(9170, 0, 1, &vc, nullptr);
  ok(h0 && v0 == 9177, "l chain: held L at 9170 reads the console's 9177, read " +
                           std::to_string(v0));
  ok(h1 && v1 == 9188, "l chain: and one tap with C-down held reads the console's 9188, read " +
                           std::to_string(v1));
  ok(c1 && vc == 9194,
     "l chain: one tap after the C-down exit with no turn reads the console's 9194, read " +
         std::to_string(vc));
  ok(f1 == f0 + 3, "l chain: a tap costs its three frames");

  /* At 9170 the held chain tops out at 9206 after three taps. */
  int top = 0;
  ok(l_chain::held(9170, 3, &top, nullptr) && top == 9206 &&
         !l_chain::held(9170, 4, nullptr, nullptr),
     "l chain: RED: the tap after the chain's top is refused");

  search::Pose was, now;
  was.facing = 9170;
  was.taps = 2;
  int reversed = 0, v2 = 0, f2 = 0;
  l_chain::held(9170, 2, &v2, &f2);
  search::turn_bearing(9170, v2, &reversed);
  ok(search::model_step("l_cdown_turnaround", 0, 1, was, &now) && now.facing == reversed &&
         now.camera == v2 && now.frames == f2 && now.taps == 0,
     "l chain: model_step lands two taps on the table's facing, camera and frames");

  /* Held L needs no camera in the question. */
  const search::BaseTable base = search::base_table(false);
  const search::BaseMove* block = base.of("l_cdown_turnaround");
  ok(block != nullptr, "l chain: the row is in the base table");
  const search::Slab flat = search::slab(0.0, 0.0, 0.0, 8192.0);
  const search::Corridor wide = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
  const search::Selection sel = search::select(flat.mesh, wide);
  search::Limits open;
  open.slope_normal_y = 0.0;
  open.height_step = 1e9;
  open.normal_apart = 1e9;
  const search::Grid grid = search::build(sel, wide, open);
  int found = 0, confirmed = 0;
  for (int taps = 0; taps <= 2 && block != nullptr; ++taps) {
    int v = 0, frames = 0, facing = 0;
    if (!l_chain::held(9170, taps, &v, &frames) || !search::turn_bearing(9170, v, &facing)) continue;
    search::Question q;
    q.start_facing = 9170;
    q.has_camera = false;
    q.target.has_x = false;
    q.target.has_z = false;
    q.tolerance = 0.0;
    q.steps = 1;
    q.moves.push_back("l_cdown_turnaround");
    q.end_facing.any = false;
    q.end_facing.span = 0;
    q.end_facing.a = facing;
    const int price = block->frames + frames;
    q.frames = price;
    const search::Found run = search::search_tree(q, base, grid, sel,
                                                  search::Calibration::measured());
    for (const search::Candidate& c : run.candidate) {
      if (c.facing == facing && c.frames == price && c.path.size() == 1 &&
          c.path[0].taps == taps) {
        ++found;
        break;
      }
    }
    const search::Verified said = search::verify(run, q, base, &flat.room);
    for (const search::Consult& c : said.consult) {
      if (c.outcome == search::Outcome::Confirmed && c.facing == facing && c.frames == price) {
        ++confirmed;
        break;
      }
    }
  }
  ok(found == 3 && confirmed == 3,
     "l chain: the walk reaches 0, 1 and 2 taps at their price and the engine confirms them, on " +
         std::to_string(found) + " and " + std::to_string(confirmed) + " of 3");

  /* A turn's child copies the state before it, so it must not inherit the chain's taps. */
  if (block != nullptr) {
    search::Question q;
    q.start_facing = 9170;
    q.has_camera = false;
    q.target.has_x = false;
    q.target.has_z = false;
    q.tolerance = 0.0;
    q.steps = 2;
    q.moves.push_back("l_cdown_turnaround");
    q.moves.push_back("fine_turn");
    q.end_facing.any = true;
    q.frames = 400;
    const search::Found run = search::search_tree(q, base, grid, sel,
                                                  search::Calibration::measured());
    int turns_after = 0, tapped_turns = 0;
    const int fine_row = static_cast<int>(base.of("fine_turn") - &base.move[0]);
    for (const search::Candidate& c : run.candidate) {
      for (size_t i = 1; i < c.path.size(); ++i) {
        if (c.path[i].row == fine_row && c.path[i - 1].taps > 0) {
          ++turns_after;
          if (c.path[i].taps != 0) ++tapped_turns;
        }
      }
    }
    ok(turns_after > 0 && tapped_turns == 0,
       "l chain: RED: a C up turn after a tapped chain carries no taps, over " +
           std::to_string(turns_after) + " such turns");
  }

  /* After a C up turn onto 9170: the C-down exit's wait plus the chain's frames. */
  {
    cup_exit::Leave l;
    int v = 0, frames = 0, facing = 0;
    search::Pose after, left;
    after.facing = 9170;
    after.cup_dir = 1;
    after.taps = 1;
    const bool have = cup_exit::leave(9170, 1, cup_exit::Way::CDown, &l) &&
                      l_chain::cdown(9170, 1, 1, &v, &frames) &&
                      search::turn_bearing(9170, v, &facing);
    ok(have && search::model_step("cup_cdown_turnaround", 0, 1, after, &left) &&
           left.facing == facing && left.frames == l.wait + frames,
       "l chain: the C up C-down row with one tap reverses off the chain after the wait");
  }

  /* `moves.ts` shows the row at the lower median no-tap price. */
  {
    std::string app = __FILE__;
    app = app.substr(0, app.find_last_of("/\\")) + "/../../app/src/core/moves.ts";
    std::string text;
    if (FILE* f = std::fopen(app.c_str(), "rb")) {
      char buf[4096];
      size_t n = 0;
      while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
      std::fclose(f);
    }
    std::vector<int> prices;
    for (int facing = 0; facing < 65536; ++facing) {
      int frames = 0;
      if (l_chain::held(facing, 0, nullptr, &frames)) prices.push_back(frames);
    }
    std::sort(prices.begin(), prices.end());
    const int median = prices.empty() ? 0 : prices[(prices.size() - 1) / 2];
    const size_t at = text.find("{id:'l_cdown_turnaround'");
    const size_t fr = at == std::string::npos ? at : text.find("frames:", at);
    const int want = block != nullptr ? block->frames + median : -1;
    ok(block != nullptr && fr != std::string::npos && std::atoi(text.c_str() + fr + 7) == want,
       "l chain: moves.ts shows the held-L row at the median price the table holds (" +
           std::to_string(want) + ")");
  }
}

/** A floor at y 0 and, unless `wall_z` is 0, a wall across x at that z. No polygon names a camera. */
tww_engine::RoomDzb cam_room_of(float wall_z) {
  tww_engine::RoomDzb r;
  auto vtx = [&](float x, float y, float z) {
    cBgD_Vtx_t v;
    v.x = x;
    v.y = y;
    v.z = z;
    r.v_tbl.push_back(v);
    return static_cast<u16>(r.v_tbl.size() - 1);
  };
  auto face = [&](u16 a, u16 b, u16 c) {
    cBgD_Tri_t t;
    t.vtx0 = a;
    t.vtx1 = b;
    t.vtx2 = c;
    t.id = 0;
    t.grp = 0;
    r.t_tbl.push_back(t);
  };
  cBgD_Ti_t ti;
  ti.mPolyInf0 = 0xFF;
  ti.mPolyInf1 = 0;
  ti.mPolyInf2 = 0xFFFF;
  ti.mPolyInf3 = 0;
  r.ti_tbl.push_back(ti);
  const u16 a = vtx(-4000, 0, -4000), b = vtx(-4000, 0, 4000), c = vtx(4000, 0, -4000),
            d = vtx(4000, 0, 4000);
  face(a, b, c);
  face(d, c, b);
  if (wall_z != 0.0f) {
    const u16 e = vtx(-1000, -50, wall_z), f = vtx(1000, -50, wall_z),
              g = vtx(-1000, 1000, wall_z), h = vtx(1000, 1000, wall_z);
    face(e, f, g);
    face(h, g, f);
  }
  return r;
}

/** Where the engine's last frame left him, read off the player rather than off `Drive`. */
struct LastFrame : search::Watcher {
  bool seen = false;
  f32 x = 0, y = 0, z = 0;
  void frame(int, const daPy_lk_c& lk, bool) override {
    seen = true;
    x = lk.current.pos.x;
    y = lk.current.pos.y;
    z = lk.current.pos.z;
  }
};

void exits_at_tests() {
  /* At the origin the engine's exits are the tables', so the tape and the pose change nothing. */
  const cup_tape::Spot origin = {0.0f, 0.0f, 0.0f};
  const cup_exit::Way ways[] = {cup_exit::Way::CDown, cup_exit::Way::Settled,
                                cup_exit::Way::Stretch};
  int tried = 0, same = 0, taps_tried = 0, taps_same = 0;
  for (int i = 0; i < 12; ++i) {
    const int f = static_cast<int>((static_cast<uint32_t>(i) * 2654435761u) & 0xFFFF);
    for (int dir = -1; dir <= 1; ++dir) {
      for (cup_exit::Way w : ways) {
        cup_exit::Leave table, engine;
        if (!cup_exit::leave(f, dir, w, &table)) continue;
        ++tried;
        if (cup_exit::leave_at(origin, f, dir, w, &engine) && engine.csangle == table.csangle &&
            engine.frames == table.frames) {
          ++same;
        }
      }
      for (int taps = 1; taps <= 2; ++taps) {
        int table = 0, engine = 0, tf = 0, ef = 0;
        if (!l_chain::cdown(f, dir, taps, &table, &tf)) continue;
        ++taps_tried;
        if (l_chain::cdown_at(origin, f, dir, taps, &engine, &ef) && engine == table && ef == tf) {
          ++taps_same;
        }
      }
    }
  }
  ok(tried > 50 && same == tried, "exits at: the engine's exit at the origin is the table's on " +
                                      std::to_string(same) + " of " + std::to_string(tried));
  ok(taps_tried > 20 && taps_same == taps_tried,
     "exits at: the engine's taps at the origin are the table's on " + std::to_string(taps_same) +
         " of " + std::to_string(taps_tried));

  /* His console at z -204009: facing 15672, the view left by C-down with no turn, csangle 15682. */
  const cup_tape::Spot his = {-144.753647f, 668.66925f, -204009.375f};
  cup_exit::Leave table, there;
  ok(cup_exit::leave(15672, 0, cup_exit::Way::CDown, &table) && table.csangle == 15672,
     "exits at: the table, built at the origin, reads 15672 there");
  const bool read = cup_exit::leave_at(his, 15672, 0, cup_exit::Way::CDown, &there);
  ok(read && there.csangle == 15682,
     "exits at: the engine where he stood reads the console's 15682, got " +
         std::to_string(there.csangle));
  int seated = 0, frames = 0;
  ok(search::ess_reseat(15672, 0, &seated, &frames, &his) && seated == 15682,
     "exits at: the ESS reseat where he stood is the console's 15682");

  /* The verification turns ESS Right onto the reseat the engine reads there, not the table's. */
  const search::BaseTable base = search::base_table(false);
  int row = -1;
  for (size_t i = 0; i < base.move.size(); ++i) {
    if (base.move[i].id == "ess_right_turn") row = static_cast<int>(i);
  }
  const search::Slab ground = search::slab(static_cast<double>(his.y), 0.0, 0.0, 262144.0);
  search::Question q;
  q.start_x = his.x;
  q.start_y = his.y;
  q.start_z = his.z;
  q.start_facing = 15672;
  q.has_camera = false;
  q.target.has_x = false;
  q.target.has_z = false;
  q.steps = 1;
  q.moves.push_back("ess_right_turn");
  q.end_facing.any = true;
  search::Found one;
  search::Candidate c;
  search::Edge e;
  e.row = row;
  c.path.push_back(e);
  one.candidate.push_back(c);
  const search::Verified said = search::verify(one, q, base, &ground.room);
  const int turned = said.consult.empty() ? -1 : said.consult[0].facing;
  ok(row >= 0 && turned == ((15682 + 0xC000) & 0xFFFF),
     "exits at: verified, ESS Right where he stood ends on the console's reseat less a quarter, "
     "got " + std::to_string(turned));
}

void ess_tests() {
  const char* ids[3] = {"ess_up_turn", "ess_left_turn", "ess_right_turn"};
  const int offsets[3] = {0x0000, 0x4000, 0xC000};

  ok(search::kEssDistance == 15.0f / 54.0f,
     "ess: the stick is byte 158, 15 past the dead zone on the 54 range");

  /* The engine turns each one in place onto the camera plus its offset, at the hold's price. */
  int drove = 0, landed = 0;
  const int facings[] = {0, 9170, 0x8000, 50001};
  const int gaps[] = {1, -7, 300, -2048, 2048, 0x3800, -0x4800, 0x7800};
  for (int s = 0; s < 3; ++s) {
    for (int f : facings) {
      for (int g : gaps) {
        const int target = (f + g) & 0xFFFF;
        const int camera = (target - offsets[s]) & 0xFFFF;
        search::Move m;
        if (!search::move_of(ids[s], f, &m, &camera) || m.unaimed) continue;
        tww_engine::Init init;
        init.pos.set(100.0f, 0.0f, -200.0f);
        init.shape_angle_y = static_cast<s16>(f);
        init.travel_angle_y = static_cast<s16>(f);
        init.proc = daPy_lk_c::daPyProc_WAIT_e;
        const tww_engine::RoomDzb flat = tww_engine::flat_floor_dzb(0.0f);
        const search::Drive d = search::drive(m, init, &flat);
        ++drove;
        if (d.ok && d.rested && d.facing_out == target && d.dx == 0.0 && d.dz == 0.0 &&
            d.reach == 0.0 && d.frames == search::ess_hold(g) + search::kInputDelay) {
          ++landed;
        }
      }
    }
  }
  ok(drove == 96 && landed == drove,
     "ess: the engine lands every turn on its target in place at its price, " +
         std::to_string(landed) + " of " + std::to_string(drove));

  /* The reseat at 9170 with no turn: the C-down exit's table reads 9182. */
  int seated = 0, reseat = 0;
  ok(search::ess_reseat(9170, 0, &seated, &reseat) && seated == 9182,
     "ess: the reseat at 9170 reads the exit table's 9182, read " + std::to_string(seated));
  search::Pose was, now;
  was.facing = 9170;
  was.camera = 30000;
  was.has_camera = false;
  ok(search::model_step("ess_up_turn", 0, 1, was, &now) && now.facing == 9182 &&
         now.camera == 9182 && now.has_camera && now.frames == reseat + search::ess_hold(12) - 1,
     "ess: model_step reseats, turns up onto the exit's camera and keeps it, with no camera before");
  ok(search::model_step("ess_left_turn", 0, 1, was, &now) &&
         now.facing == ((9182 + 0x4000) & 0xFFFF) && now.camera == 9182,
     "ess: and left onto it plus a quarter");
  was.facing = 0;
  ok(search::ess_reseat(0, 0, &seated, nullptr) && seated == 0 &&
         !search::model_step("ess_up_turn", 0, 1, was, &now),
     "ess: RED: where the exit lands on the facing, up does not turn");
  ok(!search::ess_target(search::Seat::EssRight, 9170, true, 9170 + 0x4000, nullptr),
     "ess: RED: a half turn is a turnaround, not this");
  ok(!search::runs_camera(search::Seat::EssUp) && !search::runs_camera(search::Seat::EssLeft) &&
         !search::runs_camera(search::Seat::EssRight),
     "ess: the room's camera check does not apply to the reseat");

  const search::BaseTable base = search::base_table(false);
  bool priced = true;
  for (int s = 0; s < 3; ++s) {
    const search::BaseMove* row = base.of(ids[s]);
    if (row == nullptr || !row->driven || row->frames != 1 + search::kInputDelay) priced = false;
  }
  ok(priced, "ess: the base table drives all three at a one-frame hold");

  /* The walk reaches it at its price and the engine confirms it. */
  {
    int reseat_9170 = 0;
    search::ess_reseat(9170, 0, nullptr, &reseat_9170);
    const search::Slab flat = search::slab(0.0, 0.0, 0.0, 8192.0);
    const search::Corridor wide = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
    const search::Selection sel = search::select(flat.mesh, wide);
    search::Limits open;
    open.slope_normal_y = 0.0;
    open.height_step = 1e9;
    open.normal_apart = 1e9;
    const search::Grid grid = search::build(sel, wide, open);
    search::Question q;
    q.start_facing = 9170;
    /* The reseat needs no camera: at 9170 it reads 9182. */
    q.has_camera = false;
    q.target.has_x = false;
    q.target.has_z = false;
    q.tolerance = 0.0;
    q.steps = 1;
    q.moves.push_back("ess_right_turn");
    q.end_facing.any = false;
    q.end_facing.span = 0;
    q.end_facing.a = (9182 - 0x4000) & 0xFFFF;
    const int turn = search::ess_hold(12 - 0x4000);
    const int price = reseat_9170 + turn + search::kInputDelay;
    q.frames = price;
    const search::Found run =
        search::search_tree(q, base, grid, sel, search::Calibration::measured());
    bool found = false, confirmed = false;
    for (const search::Candidate& c : run.candidate) {
      if (c.facing == q.end_facing.a && c.frames == price && c.path.size() == 1) found = true;
    }
    const search::Verified said = search::verify(run, q, base, &flat.room);
    for (const search::Consult& c : said.consult) {
      if (c.outcome == search::Outcome::Confirmed && c.facing == q.end_facing.a &&
          c.frames == price) {
        confirmed = true;
      }
    }
    ok(found && confirmed, "ess: the walk reaches a right turn at " + std::to_string(price) +
                               " frames and the engine confirms it");

    /* A person's price is the shown median moved; the same turn moves with it. */
    std::map<std::string, int> retimed;
    retimed["ess_right_turn"] = search::ess_median(search::Seat::EssRight) + 10;
    const search::BaseTable shifted = search::priced(base, retimed);
    q.frames = price + 10;
    const search::Found later =
        search::search_tree(q, shifted, grid, sel, search::Calibration::measured());
    const search::Verified again = search::verify(later, q, shifted, &flat.room);
    bool moved = false;
    for (const search::Consult& c : again.consult) {
      if (c.outcome == search::Outcome::Confirmed && c.frames == price + 10) moved = true;
    }
    ok(moved, "ess: ten frames on the typed price are ten on every turn, walk and engine alike");
    /* Far under the median, a turn still costs its own hold: the floor is the row's one frame. */
    retimed["ess_right_turn"] = 1;
    const search::BaseTable floored = search::priced(base, retimed);
    const int least = reseat_9170 + turn;
    q.frames = least;
    const search::Found cheap =
        search::search_tree(q, floored, grid, sel, search::Calibration::measured());
    const search::Verified held = search::verify(cheap, q, floored, &flat.room);
    bool at_hold = false;
    for (const search::Consult& c : held.consult) {
      if (c.outcome == search::Outcome::Confirmed && c.frames == least) at_hold = true;
    }
    ok(at_hold, "ess: RED: a price of 1 still costs the turn its reseat and hold, " +
                    std::to_string(least) + " frames, walk and engine alike");
  }

  /* `moves.ts` shows each at `ess_median`. */
  {
    std::string app = __FILE__;
    app = app.substr(0, app.find_last_of("/\\")) + "/../../app/src/core/moves.ts";
    std::string text;
    if (FILE* f = std::fopen(app.c_str(), "rb")) {
      char buf[4096];
      size_t n = 0;
      while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
      std::fclose(f);
    }
    bool shown = true;
    const search::Seat seats[3] = {search::Seat::EssUp, search::Seat::EssLeft,
                                   search::Seat::EssRight};
    for (int s = 0; s < 3; ++s) {
      const int median = search::ess_median(seats[s]);
      const size_t at = text.find(std::string("{id:'") + ids[s] + "'");
      const size_t fr = at == std::string::npos ? at : text.find("frames:", at);
      if (fr == std::string::npos || std::atoi(text.c_str() + fr + 7) != median) shown = false;
    }
    ok(shown, "ess: moves.ts shows each turn at its median price");
  }
}

void seed_push_tests() {
  /* A wall at z 20, inside Link's wall circle from the origin: the collision pass pushes the seed
     before frame 1. */
  tww_engine::RoomDzb room = cam_room_of(0.0f);
  auto vtx = [&](float x, float y, float z) {
    cBgD_Vtx_t v;
    v.x = x;
    v.y = y;
    v.z = z;
    room.v_tbl.push_back(v);
    return static_cast<u16>(room.v_tbl.size() - 1);
  };
  auto face = [&](u16 a, u16 b, u16 c) {
    cBgD_Tri_t t;
    t.vtx0 = a;
    t.vtx1 = b;
    t.vtx2 = c;
    t.id = 0;
    t.grp = 0;
    room.t_tbl.push_back(t);
  };
  {
    const u16 e = vtx(-1000, -50, 20), f = vtx(1000, -50, 20), g = vtx(-1000, 1000, 20),
              h = vtx(1000, 1000, 20);
    face(e, g, f);
    face(h, f, g);
  }
  room.b_tbl.resize(1);
  room.b_tbl[0].startTri = 0;
  room.tree_tbl.resize(1);
  room.tree_tbl[0].mFlag = 1;
  room.tree_tbl[0].mParent = 0xFFFF;
  room.tree_tbl[0].mBlock = 0;
  room.g_tbl.resize(1);
  cBgD_Grp_t& g = room.g_tbl[0];
  std::memset(&g, 0, sizeof(g));
  g.m_name = nullptr;
  g.m_scale.x = g.m_scale.y = g.m_scale.z = 1.0f;
  g.m_parent = 0xFFFF;
  g.m_next_sibling = 0xFFFF;
  g.m_first_child = 0xFFFF;

  const int facing = 0x8000;
  search::Move crawl;
  ok(search::move_of("crawl", facing, &crawl), "seed push: the crawl is in the roster");
  tww_engine::Init at;
  at.pos.set(0.0f, 0.0f, 0.0f);
  at.shape_angle_y = static_cast<s16>(facing);
  at.travel_angle_y = static_cast<s16>(facing);
  at.proc = daPy_lk_c::daPyProc_WAIT_e;

  {
    tww_engine::Session seeded(at, &room, tww_engine::RunOptions());
    ok(seeded.state().current.pos.z < -1.0f,
       "seed push: a standstill seeded inside the wall's reach is pushed off it before frame 1");
  }

  LastFrame last;
  const search::Drive d =
      search::drive(crawl, at, &room, tww_engine::RunOptions(), 96, &last);
  ok(d.ok && d.rested && last.seen, "seed push: the crawl runs and rests");
  ok(static_cast<f32>(at.pos.x + d.dx) == last.x && static_cast<f32>(at.pos.y + d.dy) == last.y &&
         static_cast<f32>(at.pos.z + d.dz) == last.z,
     "RED: the seed plus the drive's net is the engine's own landing, the push included");

  const search::BaseTable base = search::base_table(false);
  int row = -1;
  for (size_t i = 0; i < base.move.size(); ++i) {
    if (base.move[i].id == "crawl") row = static_cast<int>(i);
  }
  ok(row >= 0, "seed push: the crawl is a row of the base table");
  if (row < 0) return;
  search::Question q;
  q.start_x = 0.0;
  q.start_y = 0.0;
  q.start_z = 0.0;
  q.start_facing = facing;
  q.target.x = 0.0;
  q.target.z = -100.0;
  q.tolerance = 0.0;
  q.steps = 1;
  q.frames = 100000;
  search::Candidate one;
  search::Edge edge;
  edge.row = row;
  one.path.push_back(edge);
  search::Found found;
  found.candidate.push_back(one);
  const search::Verified said = search::verify(found, q, base, &room, nullptr);
  ok(!said.consult.empty() && said.consult[0].stop.size() == 1,
     "seed push: the verification drove the crawl");
  if (said.consult.empty() || said.consult[0].stop.size() != 1) return;
  const search::Reached& stop = said.consult[0].stop[0];
  ok(static_cast<f32>(stop.x) == last.x && static_cast<f32>(stop.y) == last.y &&
         static_cast<f32>(stop.z) == last.z,
     "RED: and the crawl's stop is where the engine left him, not where he was seeded plus the net");
}

void landing_tests() {
  search::Consult a, b, c, d;
  a.x = 10.0;
  a.z = 20.0;
  a.facing = 9170;
  a.frames = 60;
  b = a;
  b.frames = 45;
  c = a;
  c.x = static_cast<double>(std::nextafter(10.0f, 11.0f));
  c.frames = 90;
  d = a;
  d.facing = 9171;
  const std::vector<search::Consult> kept = search::one_per_landing({a, b, c, d});
  ok(kept.size() == 3 && kept[0].frames == 45 && kept[1].frames == 90 && kept[2].facing == 9171,
     "landing: two plans on one landing keep the faster, and a float step or a facing apart stay");
  const std::vector<search::Consult> swapped = search::one_per_landing({b, a});
  ok(swapped.size() == 1 && swapped[0].frames == 45,
     "landing: and the faster plan wins whichever was driven first");
}

void cam_clear_tests() {
  /* The eye sits half a circle from the csangle: 0 looks from 0x8000, direction 33 of 64. */
  ok(search::CamField::bins(0, 0, 0) == (1ULL << 32),
     "cam clear: a csangle of 0 needs the 33rd direction");
  ok(search::CamField::bins(0, 0, 1024) == (7ULL << 31),
     "cam clear: and a margin of one direction takes one either side");
  ok(search::CamField::bins(0x7E00, 0x400, 0) == ((1ULL << 63) | 1ULL),
     "cam clear: a span across the eye's zero takes the last direction and the first");
  ok(search::CamField::bins(0, 65000, 0) == ~0ULL,
     "cam clear: a span of nearly the circle takes all");

  disc::CameraNames names;
  const tww_engine::RoomDzb open_room = cam_room_of(0.0f);
  const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  const search::CamRoom cams = search::cam_room(open_room, &names, "");
  const double measured =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  ok(cams.ok && cams.types.size() == 1 && cams.types[0] == 7 && !cams.every_type,
     "cam clear: a floor naming no camera on a stage with none runs the Field type");
  /* B is FN01's 480. C-down pulls MM01 back 20 a frame for 15 frames a step: from the view's 280
     to 580, then 700 by the first tap; from the follow camera's 480, 700 at once. */
  const auto reach_at = [&](int level) {
    return level >= 0 && level < static_cast<int>(cams.level.size()) ? cams.level[level].reach
                                                                      : -1.0;
  };
  const double view0 = reach_at(cams.from_view[0]), view1 = reach_at(cams.from_view[1]);
  const double view8 = reach_at(cams.from_view[8]), held0 = reach_at(cams.from_follow[0]);
  ok(reach_at(0) == 480.0 && view0 >= 540.0 && view0 < 700.0 && view1 == 700.0 &&
         view8 == 700.0 && held0 == 700.0,
     "cam clear: B reaches 480, C Up C Down 580 with no taps and 700 from one, held L 700");
  for (size_t l = 0; l < cams.level.size(); ++l) {
    const search::CamBounds& b = cams.level[l];
    std::printf("     level %zu reach %.1f rise %.2f..%.2f centre %.1f..%.1f off %.1f\n", l,
                b.reach, b.rise_lo, b.rise_hi, b.centre_lo, b.centre_hi, b.centre_off);
  }
  std::printf("     margin %d, measured in %.2f s\n", cams.margin, measured);

  std::vector<uint64_t> here(1, search::CamField::key_at(25.0, 0.0, 25.0));
  search::CamField::Built built;
  const search::CamField open = search::cam_field(open_room, cams, here, "", 1, &built);
  bool all_open = open.levels() == static_cast<int>(cams.level.size());
  for (int l = 0; l < open.levels(); ++l) all_open = all_open && open.clear(l, 25.0, 0.0, 25.0, ~0ULL);
  ok(all_open, "cam clear: on open ground every direction is clear at every level");
  ok(!open.clear(open.levels(), 25.0, 0.0, 25.0, 1ULL) && !open.clear(-1, 25.0, 0.0, 25.0, 1ULL) &&
         !search::CamField().clear(0, 25.0, 0.0, 25.0, 1ULL),
     "cam clear: RED: a level the field does not hold, or a field never built, is not clear");
  ok(open.clear(0, 25.0, 100.0, 25.0, ~0ULL) && open.clear(0, 5000.0, 0.0, 25.0, ~0ULL),
     "cam clear: and a place it was not built for is worked out, not refused");

  const uint64_t behind = search::CamField::bins(0, 0, cams.margin);
  const uint64_t ahead = search::CamField::bins(0x8000, 0, cams.margin);
  const tww_engine::RoomDzb near_room = cam_room_of(-300.0f);
  const search::CamField near_field = search::cam_field(near_room, cams, here, "", 1, nullptr);
  bool near_met = true, near_ahead = true;
  for (int l = 0; l < near_field.levels(); ++l) {
    near_met = near_met && !near_field.clear(l, 25.0, 0.0, 25.0, behind);
    near_ahead = near_ahead && near_field.clear(l, 25.0, 0.0, 25.0, ahead);
  }
  ok(near_met, "cam clear: RED: a wall 300 behind him meets every camera");
  ok(near_ahead, "cam clear: and none looking from ahead of him");
  const tww_engine::RoomDzb far_room = cam_room_of(-650.0f);
  const search::CamField far_field = search::cam_field(far_room, cams, here, "", 1, nullptr);
  ok(far_field.clear(search::CamField::follow_level(), 25.0, 0.0, 25.0, behind) &&
         far_field.clear(far_field.view_level(0), 25.0, 0.0, 25.0, behind),
     "cam clear: a wall 650 behind meets neither B nor C Up C Down with no taps");
  ok(!far_field.clear(far_field.view_level(1), 25.0, 0.0, 25.0, behind) &&
         !far_field.clear(far_field.follow_start_level(0), 25.0, 0.0, 25.0, behind),
     "cam clear: RED: and meets C Up C Down after a tap, and held L");

  /* Over the roster itself, so a new camera row with no level goes red. */
  {
    int camera_rows = 0, unlevelled = 0;
    const search::BaseTable roster = search::base_table(false);
    for (const search::BaseMove& row : roster.move) {
      const search::Seat seat = search::seat_of(row.id);
      if (!search::runs_camera(seat)) continue;
      ++camera_rows;
      const int most = search::takes_taps(seat) ? l_chain::kTaps : 0;
      for (int taps = 0; taps <= most; ++taps) {
        if (search::camera_level(open, seat, taps) < 0) ++unlevelled;
      }
    }
    ok(camera_rows >= 4 && unlevelled == 0,
       "cam clear: every row that runs a camera reads a level at every tap count, over " +
           std::to_string(camera_rows) + " rows");
    ok(search::camera_level(open, search::Seat::Kept, 0) < 0 &&
           search::camera_level(open, search::Seat::OnTheFacing, 0) < 0,
       "cam clear: RED: and a row that runs none reads none");
  }

  {
    const search::CamField one = search::cam_field(near_room, cams, here, "", 1, nullptr);
    const std::vector<uint64_t> both = {search::CamField::key_at(25.0, 0.0, 25.0),
                                        search::CamField::key_at(2025.0, 0.0, 25.0)};
    const search::CamField built_both = search::cam_field(near_room, cams, both, "", 1, nullptr);
    const bool far_built = built_both.clear(1, 2025.0, 0.0, 25.0, behind);
    const bool far_later = one.clear(1, 2025.0, 0.0, 25.0, behind);
    const bool near_later = one.clear(1, 25.0, 0.0, 275.0, behind);
    ok(one.later_places() > 0 && far_later == far_built && far_later && !near_later,
       "cam clear: a place outside the field is worked out when the walk asks, as a build would");
  }

  {
    std::vector<uint64_t> wide_places;
    for (int ix = -40; ix < 40; ix += 7) {
      for (int iz = -40; iz < 40; iz += 7) wide_places.push_back(search::CamField::key_of(ix, iz, 0));
    }
    const search::CamField all = search::cam_field(near_room, cams, wide_places, "", 4, nullptr);
    int same = 0;
    for (uint64_t k : wide_places) {
      const search::CamField alone =
          search::cam_field(near_room, cams, std::vector<uint64_t>(1, k), "", 1, nullptr);
      const double x = (static_cast<double>(static_cast<long long>((k - 1) & 0x1FFFFF) - (1LL << 20)) + 0.5) * 50.0;
      const double z = (static_cast<double>(static_cast<long long>(((k - 1) >> 21) & 0x1FFFFF) - (1LL << 20)) + 0.5) * 50.0;
      bool agree = true;
      for (int l = 0; l < all.levels(); ++l) {
        for (int b = 0; b < 64; ++b) {
          agree = agree && all.clear(l, x, 0.0, z, 1ULL << b) == alone.clear(l, x, 0.0, z, 1ULL << b);
        }
      }
      same += agree ? 1 : 0;
    }
    ok(same == static_cast<int>(wide_places.size()),
       "cam clear: a field built over many tiles answers every place as one built for it alone, " +
           std::to_string(same) + " of " + std::to_string(wide_places.size()));
  }

  {
    const char* t = std::getenv("TEMP");
    const std::string cut = std::string(t ? t : ".") + "/setupcore_cam_clear_cut.bin";
    std::remove(cut.c_str());
    search::cam_field(near_room, cams, here, cut, 1, nullptr);
    std::string bytes;
    if (FILE* f = std::fopen(cut.c_str(), "rb")) {
      char buf[4096];
      size_t n = 0;
      while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) bytes.append(buf, n);
      std::fclose(f);
    }
    if (FILE* f = std::fopen(cut.c_str(), "wb")) {
      std::fwrite(bytes.data(), 1, bytes.size() - 10, f);
      std::fclose(f);
    }
    search::CamField::Built again_cut;
    search::cam_field(near_room, cams, here, cut, 1, &again_cut);
    ok(!bytes.empty() && again_cut.from_disc == 0 && again_cut.computed == 1,
       "cam clear: RED: a file cut short is worked out again rather than half believed");
    std::remove(cut.c_str());
  }

  {
    tww_engine::RoomDzb evented = cam_room_of(0.0f);
    evented.ti_tbl[0].mPolyInf2 = 0x00FF;  // room cam id 0, no cam-move BG
    disc::CameraNames only_event;
    only_event.room_list.push_back("Event");
    const search::CamRoom fell = search::cam_room(evented, &only_event, "");
    ok(fell.ok && !fell.every_type &&
           std::find(fell.types.begin(), fell.types.end(), 7) != fell.types.end(),
       "cam clear: floors naming only Event fall back to Field rather than refusing");
  }

  /* Camera names read from game RAM through the loaded stage's and room's pointers. */
  {
    std::map<uint32_t, uint8_t> ram;
    auto put8 = [&](uint32_t at, uint8_t v) { ram[at] = v; };
    auto put32 = [&](uint32_t at, uint32_t v) {
      for (int i = 0; i < 4; ++i) put8(at + static_cast<uint32_t>(i), static_cast<uint8_t>(v >> (24 - 8 * i)));
    };
    auto name = [&](uint32_t at, const char* text) {
      for (int i = 0; i < 0x14; ++i) put8(at + static_cast<uint32_t>(i), 0);
      for (int i = 0; text[i] != 0; ++i) put8(at + static_cast<uint32_t>(i), static_cast<uint8_t>(text[i]));
    };
    const uint32_t stage_list = 0x80500000, stage_rows = 0x80500100, stag = 0x80500200;
    const uint32_t room_list = 0x80500300, room_rows = 0x80500400;
    const int room_no = 3;
    put32(dolphin::addr::kStageData + 0x04, stage_list);
    put32(dolphin::addr::kStageData + 0x48, stag);
    put32(stage_list, 2);
    put32(stage_list + 4, stage_rows);
    name(stage_rows, "Room");
    name(stage_rows + 0x14, "Field");
    put8(stag + 0x08, 1);
    put32(dolphin::addr::kRoomStatus + room_no * 0x114 + 0x2C, room_list);
    put32(room_list, 1);
    put32(room_list + 4, room_rows);
    name(room_rows, "DStairs");
    const dolphin::Read read = [&ram](uint32_t at, void* dst, size_t n) {
      uint8_t* out = static_cast<uint8_t*>(dst);
      for (size_t i = 0; i < n; ++i) {
        std::map<uint32_t, uint8_t>::const_iterator it = ram.find(at + static_cast<uint32_t>(i));
        out[i] = it == ram.end() ? 0 : it->second;
      }
      return true;
    };
    disc::CameraNames got;
    ok(dolphin::camera_names_at(read, room_no, &got) && got.stage_id == 1 &&
           got.stage_list.size() == 2 && got.stage_list[0] == "Room" &&
           got.stage_list[1] == "Field" && got.room_list.size() == 1 &&
           got.room_list[0] == "DStairs",
       "cam clear: the camera names come out of an attached game's stage and room");
    put32(room_list + 4, 0x12345678);
    ok(!dolphin::camera_names_at(read, room_no, &got),
       "cam clear: RED: and a list whose rows lead out of the game's memory is not a list");
  }

  const char* temp = std::getenv("TEMP");
  const std::string file = std::string(temp ? temp : ".") + "/setupcore_cam_clear_test.bin";
  std::remove(file.c_str());
  search::CamField::Built first, second;
  search::cam_field(near_room, cams, here, file, 1, &first);
  const search::CamField again = search::cam_field(near_room, cams, here, file, 1, &second);
  const search::CamRoom back = search::cam_room(near_room, &names, file);
  ok(first.computed == 1 && first.saved && second.from_disc == 1 && second.computed == 0 &&
         again.clear(1, 25.0, 0.0, 25.0, ahead) && !again.clear(1, 25.0, 0.0, 25.0, behind),
     "cam clear: a room's field is kept on the disc and read back rather than computed");
  bool same_levels = back.level.size() == cams.level.size();
  for (size_t l = 0; same_levels && l < cams.level.size(); ++l) {
    same_levels = back.level[l].reach == cams.level[l].reach &&
                  back.level[l].rise_hi == cams.level[l].rise_hi;
  }
  ok(back.ok && back.from_disc && back.margin == cams.margin && same_levels &&
         back.from_view[0] == cams.from_view[0] && back.from_follow[8] == cams.from_follow[8],
     "cam clear: and so are its cameras' bounds, measured once");
  std::remove(file.c_str());

  /* Held L from no camera needs every direction clear. */
  const search::BaseTable base = search::base_table(false);
  const search::Slab flat = search::slab(0.0, 0.0, 0.0, 8192.0);
  const search::Corridor wide = search::corridor(0.0, 0.0, 0.0, 0.0, 400.0);
  const search::Selection sel = search::select(flat.mesh, wide);
  search::Limits lim;
  lim.slope_normal_y = 0.0;
  lim.height_step = 1e9;
  lim.normal_apart = 1e9;
  const search::Grid grid = search::build(sel, wide, lim);
  const std::vector<uint64_t> places = search::cam_places(sel, wide, true, 0.0);
  search::CamField::Built over;
  const search::CamField walled = search::cam_field(near_room, cams, places, "", 4, &over);
  const search::CamField unwalled = search::cam_field(open_room, cams, places, "", 4, nullptr);
  std::printf("     the field over %zu places in %.2f s\n", over.places, over.seconds);
  search::Question q;
  q.start_facing = 9170;
  q.has_camera = false;
  q.target.has_x = false;
  q.target.has_z = false;
  q.tolerance = 0.0;
  q.steps = 1;
  q.moves.push_back("l_cdown_turnaround");
  q.end_facing.any = true;
  q.frames = 400;
  q.camera_clear = &walled;
  const search::Found met =
      search::search_tree(q, base, grid, sel, search::Calibration::measured());
  q.camera_clear = &unwalled;
  const search::Found clear =
      search::search_tree(q, base, grid, sel, search::Calibration::measured());
  /* The start is always a candidate; count only ones that move. */
  size_t met_moved = 0, clear_moved = 0;
  for (const search::Candidate& c : met.candidate) met_moved += c.path.empty() ? 0 : 1;
  for (const search::Candidate& c : clear.candidate) clear_moved += c.path.empty() ? 0 : 1;
  ok(met_moved == 0 && met.count.camera_met > 0 && clear_moved > 0 &&
         clear.count.camera_met == 0,
     "cam clear: the walk refuses held L beside the wall and expands it in the open, " +
         std::to_string(met.count.camera_met) + " refused");
}

void x_range(const search::Region& kept, double* lo, double* hi) {
  *lo = 1e300;
  *hi = -1e300;
  for (size_t i = 0; i < kept.vert.size(); i += 3) {
    if (kept.vert[i] < *lo) *lo = kept.vert[i];
    if (kept.vert[i] > *hi) *hi = kept.vert[i];
  }
}

void within_tests() {
  /* `resolve` must match the window's `withinRange` (`app/src/core/address.ts`). */
  geom::Mesh flat;
  quad(flat.ground, -400, -400, 400, 400, 0.0f);
  const search::Corridor c = search::corridor(-300, 0, 300, 0, 400);
  search::Limits limits;
  limits.cell = 25.0;
  const search::Selection s = search::select(flat, c);
  const search::Grid g = search::build(s, c, limits);

  struct Axis { bool typed; double a, b; };
  const Axis axes[] = {{false, 0, 0}, {true, -50, 50}, {true, 10, 30}, {true, -300, -200},
                       {true, 95, 105}, {true, 100, 100}};
  const double ranges[][2] = {{-20, 20}, {0, 100}, {-250, -150}, {40, 40}};
  const double tols[] = {0.0, 12.5};
  int run = 0, wrong = 0;
  for (const Axis& ax : axes) {
    for (const auto& r : ranges) {
      for (double tol : tols) {
        const double lo = ax.typed ? std::max(ax.a, r[0]) : r[0];
        const double hi = ax.typed ? std::min(ax.b, r[1]) : r[1];
        if (ax.typed && lo > hi) continue;  // disjoint: below, where bytes decide it
        search::Target t;
        t.ranged = true;
        t.has_x = ax.typed;
        t.x0 = ax.a; t.x1 = ax.b;
        t.z0 = -30; t.z1 = 30;
        t.has_within = true;
        t.wx0 = r[0]; t.wx1 = r[1]; t.wz0 = -200; t.wz1 = 200;
        ++run;
        if (!search::resolve(&t, g, s.ground, tol)) { ++wrong; continue; }
        double klo = 0, khi = 0;
        x_range(t.ground, &klo, &khi);
        if (std::fabs(klo - (lo - tol)) > 1e-6 || std::fabs(khi - (hi + tol)) > 1e-6) ++wrong;
      }
    }
  }
  ok(run > 0 && wrong == 0,
     "the region keeps the overlap of the address and the range, widened by the tolerance, and "
     "a freed axis keeps the range");

  search::Target out;
  out.ranged = true;
  out.x0 = 256.0; out.x1 = 256.0; out.z0 = -30; out.z1 = 30;
  out.mask.known[0][0] = true; out.mask.byte[0][0] = 0x43;
  out.mask.known[0][1] = true; out.mask.byte[0][1] = 0x80;
  out.mask.known[0][2] = true; out.mask.byte[0][2] = 0x00;
  out.mask.known[0][3] = true; out.mask.byte[0][3] = 0x00;
  out.has_within = true;
  out.wx0 = -100; out.wx1 = 100; out.wz0 = -200; out.wz1 = 200;
  ok(!search::resolve(&out, g, s.ground, 0.0) && out.ground.kept == 0,
     "an address wholly outside the range keeps no floor");
  search::Target free_range = out;
  free_range.has_within = false;
  ok(search::resolve(&free_range, g, s.ground, 0.0) && free_range.ground.kept > 0,
     "RED: and without the range the same address keeps its floor");

  /* The range narrows arriving, never where the walk may go; that is the Bounds' job. */
  search::Target wide;
  wide.ranged = true;
  wide.x0 = -350; wide.x1 = 350; wide.z0 = -350; wide.z1 = 350;
  search::Target cut = wide;
  cut.has_within = true;
  cut.wx0 = -10; cut.wx1 = 10; cut.wz0 = -10; cut.wz1 = 10;
  const search::Corridor cw = search::corridor(0, 0, wide, flat.box, 60);
  const search::Corridor cc = search::corridor(0, 0, cut, flat.box, 60);
  ok(cw.min_x == cc.min_x && cw.max_x == cc.max_x && cw.min_z == cc.min_z && cw.max_z == cc.max_z,
     "the range leaves the corridor as the address's own box lays it");

  ok(cut.in_within(10, -10, 0.0) && !cut.in_within(10.5, 0, 0.0) && cut.in_within(10.5, 0, 1.0) &&
         wide.in_within(1e9, 1e9, 0.0),
     "a place is inside the range widened by the tolerance, and anywhere is when there is none");
}

float of_bits(uint32_t b) {
  float f;
  std::memcpy(&f, &b, sizeof f);
  return f;
}

void type_axis(search::Target* t, int axis, uint32_t bits) {
  for (int i = 0; i < 4; ++i) {
    t->mask.known[axis][i] = true;
    t->mask.byte[axis][i] = static_cast<uint8_t>(bits >> (24 - 8 * i));
  }
}

void crease_tests() {
  /* Three triangles of sea room 11, as the game's floats. His place is inside `lower`, 0.022
     outside `upper`, and the game puts his feet at BA8020C8. */
  const uint32_t upper[9] = {0xC5711CE7, 0x00000000, 0xC844604F, 0xC549EDB3, 0x41200000,
                             0xC8449143, 0xC5525C80, 0x00000000, 0xC844BDCF};
  const uint32_t beside[9] = {0xC5711CE7, 0x00000000, 0xC844604F, 0xC565719C, 0x41200000,
                              0xC8443562, 0xC549EDB3, 0x41200000, 0xC8449143};
  const uint32_t lower[9] = {0xC5525C80, 0x00000000, 0xC844BDCF, 0xC58A98BC, 0xC3C80000,
                             0xC844E50B, 0xC5711CE7, 0x00000000, 0xC844604F};
  const uint32_t his_x = 0xC56B79B3, his_y = 0xBA8020C8, his_z = 0xC8447175;
  /* The lower plane alone at his place, in the game's arithmetic. */
  const uint32_t lower_y = 0xBC102906;
  const double x = of_bits(his_x), z = of_bits(his_z);
  const search::Corridor c = search::corridor(x - 100, z, x + 100, z, 200);
  search::Limits limits;

  const auto address = [&](uint32_t y_bits) {
    search::Target t;
    t.ranged = true;
    t.x0 = t.x1 = x;
    t.z0 = t.z1 = z;
    t.has_y = true;
    t.y0 = t.y1 = of_bits(y_bits);
    type_axis(&t, 0, his_x);
    type_axis(&t, 1, y_bits);
    type_axis(&t, 2, his_z);
    return t;
  };
  /* `*exact`: every place kept is his x and z with those Y bytes. */
  const auto keeps = [&](std::initializer_list<const uint32_t*> tris, uint32_t y_bits,
                         bool* exact) {
    geom::Mesh m;
    for (const uint32_t* t : tris) {
      for (int i = 0; i < 9; ++i) m.ground.push_back(of_bits(t[i]));
    }
    const search::Selection s = search::select(m, c);
    const search::Grid g = search::build(s, c, limits);
    search::Target t = address(y_bits);
    const bool kept = search::resolve(&t, g, s.ground, 0.0) && t.ground.kept > 0;
    *exact = kept;
    for (size_t i = 0; i < t.ground.vert.size(); i += 3) {
      const float px = static_cast<float>(t.ground.vert[i]);
      const float py = static_cast<float>(t.ground.vert[i + 1]);
      const float pz = static_cast<float>(t.ground.vert[i + 2]);
      uint32_t bx, by, bz;
      std::memcpy(&bx, &px, 4);
      std::memcpy(&by, &py, 4);
      std::memcpy(&bz, &pz, 4);
      if (bx != his_x || by != y_bits || bz != his_z) *exact = false;
    }
    return kept;
  };

  /* Negative coordinates: a run's open test cannot be `open >= 0` on a float's rank. */
  bool exact = false;
  ok(keeps({lower}, lower_y, &exact) && exact,
     "RED: on the lower triangle alone, the height it gives at his place is kept, there and "
     "nowhere else");

  /* `cM3d_CrossY_Tri_Front` admits edge cross products down to -20, so a place can stand on a
     neighbour's plane past its edge; the highest such plane is the floor. */
  ok(keeps({upper, beside, lower}, his_y, &exact) && exact,
     "RED: his twelve bytes keep his place, on the upper plane past its edge");
  ok(!keeps({upper, beside, lower}, lower_y, &exact),
     "RED: and the lower plane's height there is not kept, because the upper plane is higher");

  geom::Mesh m;
  for (const uint32_t* t : {upper, beside, lower}) {
    for (int i = 0; i < 9; ++i) m.ground.push_back(of_bits(t[i]));
  }
  const search::Selection s = search::select(m, c);
  const search::Grid g = search::build(s, c, limits);
  search::Question his;
  his.target = address(his_y);
  his.tolerance = 0.0;
  search::resolve(&his.target, g, s.ground, 0.0);
  ok(his.target.ground.kept > 0 && search::reaches(his, x, of_bits(his_y), z),
     "RED: and a landing on his twelve bytes reaches the target");

  /* A piece `split_seabed` cut is read on its source triangle's plane: at the shore that gives 0,
     the piece's own plane -0.0088. */
  const uint32_t shore[9] = {0xC53776F3, 0x00000000, 0xC844EB90, 0xC56FF3D5, 0xC3C80000,
                             0xC8455A22, 0xC5525C80, 0x00000000, 0xC844BDCF};
  const uint32_t at_x = 0xC5525CFB, at_z = 0xC844BDCF;
  geom::Mesh sea;
  for (const uint32_t* t : {upper, shore}) {
    for (int i = 0; i < 9; ++i) sea.ground.push_back(of_bits(t[i]));
  }
  geom::split_seabed(sea);
  const double sx = of_bits(at_x), sz = of_bits(at_z);
  const search::Corridor sc = search::corridor(sx - 100, sz, sx + 100, sz, 200);
  const search::Selection ss = search::select(sea, sc);
  const search::Grid sg = search::build(ss, sc, limits);
  const auto on_shore = [&](uint32_t y_bits, bool sources) {
    search::Target t;
    t.ranged = true;
    t.x0 = t.x1 = sx;
    t.z0 = t.z1 = sz;
    t.has_y = true;
    t.y0 = t.y1 = of_bits(y_bits);
    type_axis(&t, 0, at_x);
    type_axis(&t, 1, y_bits);
    type_axis(&t, 2, at_z);
    return search::resolve(&t, sg, ss.ground, 0.0, nullptr,
                           sources ? &ss.ground_source : nullptr);
  };
  ok(ss.ground_source.size() == ss.ground.size() && ss.ground.size() > 18,
     "the sea cut the beach, and each piece carries the triangle it was cut from");
  ok(on_shore(0x00000000, true), "RED: at the shore the cut triangle's own height, 0, is kept");
  ok(!on_shore(0x00000000, false),
     "and it is the source that does it: the piece's own plane never gives 0 there");
  ok(!on_shore(his_y, true),
     "RED: and the upper plane's height there is not kept, because the cut triangle is higher");

  /* The same along a row: x free, z and the upper plane's height typed. */
  search::Target row;
  row.ranged = true;
  row.x0 = sx - 50; row.x1 = sx + 50;
  row.z0 = row.z1 = sz;
  row.has_y = true;
  row.y0 = row.y1 = of_bits(his_y);
  type_axis(&row, 1, his_y);
  type_axis(&row, 2, at_z);
  ok(search::resolve(&row, sg, ss.ground, 0.0, nullptr, &ss.ground_source) &&
         row.ground.kept > 0 && row.off(sx, sz) != 0,
     "RED: along the row the upper plane keeps that height, but not beside the shore vertex");

  /* Y `B8 __ __ __` with no height hull, as the window sends a Y typed alone: all of it is under
     this room's lowest vertex, 0, and the game gives B8001494 here. */
  geom::Mesh gentle;
  tri(gentle.ground, -2483.1167f, 10.0f, -201673.797f, -2771.5603f, 0.0f, -202142.547f,
      -2767.34058f, 0.0f, -201828.953f);
  const uint32_t gx = 0xC52D385C, gz = 0xC84566EB;
  const double ex = of_bits(gx), ez = of_bits(gz);
  const search::Corridor gc = search::corridor(ex - 100, ez, ex + 100, ez, 400);
  const search::Selection gs = search::select(gentle, gc);
  const search::Grid gg = search::build(gs, gc, limits);
  search::Target b8;
  b8.ranged = true;
  b8.x0 = ex - 300; b8.x1 = ex + 300; b8.z0 = ez - 300; b8.z1 = ez + 300;
  b8.mask.known[1][0] = true;
  b8.mask.byte[1][0] = 0xB8;
  search::Target whole = b8;
  whole.mask.known[1][1] = whole.mask.known[1][2] = whole.mask.known[1][3] = true;
  whole.mask.byte[1][1] = 0x00;
  whole.mask.byte[1][2] = 0x14;
  whole.mask.byte[1][3] = 0x94;
  ok(search::resolve(&whole, gg, gs.ground, 0.0) && whole.off(ex, ez) == 0,
     "RED: a height a hair under the room's lowest floor is kept where the game gives it");
  ok(search::resolve(&b8, gg, gs.ground, 0.0) && b8.off(ex, ez) == 0,
     "RED: Y B8 alone keeps the place, a hair under the lowest floor");
}

/* Which facings can land the item on a target's own f32. */
void item_facing_tests() {
  search::Question q;
  q.aim = search::Aim::Overhead;
  q.tolerance = 0.0;
  q.target.has_x = true;
  q.target.x = static_cast<double>(-2.9293828f);
  q.target.has_z = false;
  const std::vector<uint8_t> lands = search::item_facings(q);
  long long count = 0;
  for (uint8_t one : lands) count += one;
  ok(lands.size() == 65536, "an X held to its f32 by the item has a table over every facing");
  ok(count == 9072,
     "C03B7B02 (low bits 10) is in reach on 9072 facings: the solid bands and their patchy edges");
  ok(lands[23000] == 1 && lands[54000] == 1,
     "the offset lying along the line makes Link's own X small, and his fine steps reach it");
  ok(lands[0] == 0 && lands[16384] == 0 && lands[50000] == 0,
     "an offset 8 or more across leaves both sides on a 2^-20 grid, which ends in 0, 4, 8 or C");

  /* Every facing, against every Link coordinate 64 float steps either side of the one the table
     tries: the sum is monotonic in his coordinate, so a landing the table missed would be in that
     window. Each target is one axis, at a spread of magnitudes. */
  const float lines[] = {-2.9293828f, 0.3710937f, 3.0000002f, -57.15f, 1234.5671f, -200623.31f};
  long long both_ways = 0, wrong = 0;
  for (float line : lines) {
    for (int axis = 0; axis < 2; ++axis) {
      search::Question at = q;
      at.target.has_x = axis == 0;
      at.target.has_z = axis == 1;
      at.target.x = at.target.z = static_cast<double>(line);
      const std::vector<uint8_t> table = search::item_facings(at);
      for (int f = 0; f < 65536; ++f) {
        double ox = 0, oy = 0, oz = 0;
        search::item_point(0.0, 0.0, 0.0, 0, f, 0, &ox, &oy, &oz);
        const float o = static_cast<float>(axis == 0 ? ox : oz);
        float l = static_cast<float>(static_cast<double>(line) - static_cast<double>(o));
        for (int k = 0; k < 64; ++k) l = std::nextafter(l, -1e30f);
        bool found = false;
        for (int k = 0; k <= 128 && !found; ++k, l = std::nextafter(l, 1e30f)) {
          double ax = 0, ay = 0, az = 0;
          search::item_point(axis == 0 ? l : 0.0, 0.0, axis == 1 ? l : 0.0, 0, f, 0, &ax, &ay, &az);
          if (static_cast<float>(axis == 0 ? ax : az) == line) found = true;
        }
        if (found != (table[static_cast<size_t>(f)] == 1)) ++wrong;
        ++both_ways;
      }
    }
  }
  ok(both_ways == 12LL * 65536 && wrong == 0,
     "RED: on six lines each way, a facing is in the table exactly when the game's own sum can put "
     "the item on the line from some coordinate of his");

  search::Question him = q;
  him.aim = search::Aim::Player;
  search::Question loose = q;
  loose.tolerance = 1.0;
  search::Question free_both = q;
  free_both.target.has_x = false;
  ok(search::item_facings(him).empty() && search::item_facings(loose).empty() &&
         search::item_facings(free_both).empty(),
     "Link himself, a tolerance past zero or no axis asked leaves every facing open");

  /* Dropping childless states the item cannot land from loses no plan. The target is the item's
     X over a landing the model itself reaches, so the question has plans at all. */
  const search::BaseTable base = search::base_table(false);
  const search::Slab flat = search::slab(0.0, 0.0, 0.0, 8192.0);
  const search::Corridor wide = search::corridor(0.0, 0.0, 0.0, 0.0, 2000.0);
  const search::Selection sel = search::select(flat.mesh, wide);
  search::Limits open;
  open.slope_normal_y = 0.0;
  open.height_step = 1e9;
  open.normal_apart = 1e9;
  const search::Grid grid = search::build(sel, wide, open);
  const search::Calibration cal = search::Calibration::measured();
  search::Question near;
  near.aim = search::Aim::Overhead;
  near.start_facing = 0x1234;
  near.frames = 120;
  near.tolerance = 40.0;
  near.target.x = 0.5;
  near.target.z = 60.0;
  for (const char* id : {"crawl", "crawl_r", "dry_roll", "dry_roll_r", "fine_turn"}) {
    near.moves.push_back(id);
  }
  const search::Found seed = search::search_tree(near, base, grid, sel, cal);
  ok(!seed.candidate.empty(), "a loose question over flat ground has a landing to aim at");
  if (seed.candidate.empty()) return;
  /* Near X = 0 the float steps are fine, so the item's sum rules most facings out. */
  search::Question exact = near;
  exact.tolerance = 0.0;
  exact.target.has_z = false;
  long long open_facings = 65536;
  for (const search::Candidate& at : seed.candidate) {
    double ix = 0, iy = 0, iz = 0;
    search::item_point(at.x, at.y, at.z, 0, at.facing, 0, &ix, &iy, &iz);
    exact.target.x = static_cast<double>(static_cast<float>(ix));
    open_facings = 0;
    for (uint8_t one : search::item_facings(exact)) open_facings += one;
    if (open_facings < 65536 / 2) break;
  }
  ok(open_facings < 65536 / 2, "one landing's item X rules most facings out");
  /* A wide check range gives the near set a population; the facing filter still applies to it. */
  exact.check_range = 1e5;
  search::Question kept_all = exact;
  kept_all.drop_unlandable = false;
  const search::Found dropped = search::search_tree(exact, base, grid, sel, cal);
  const search::Found every = search::search_tree(kept_all, base, grid, sel, cal);
  std::set<std::vector<search::Edge>> a, b;
  for (const search::Candidate& c : dropped.candidate) a.insert(c.path);
  for (const search::Candidate& c : every.candidate) b.insert(c.path);
  ok(!b.empty() && a == b && dropped.exhausted && every.exhausted,
     "RED: the same exact question finds the same plans with childless unlandable states dropped "
     "as without");
  ok(dropped.count.unlandable > 0 && every.count.unlandable == 0,
     "and it does drop some, so the check above compares two different walks");
}

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
  /* `SETUPCORE_ONLY=ess` runs the roster, ESS and exit checks alone, `fewest` the fewest checks
     and `search` the walk's own; the whole suite takes minutes. */
  if (const char* only = std::getenv("SETUPCORE_ONLY")) {
    if (std::string(only) == "list") {
      list_tests();
      std::printf("%s\n", failed == 0 ? "all checks passed" : "CHECKS FAILED");
      return failed == 0 ? 0 : 1;
    }
    if (std::string(only) == "item") {
      item_facing_tests();
      std::printf("%s\n", failed == 0 ? "all checks passed" : "CHECKS FAILED");
      return failed == 0 ? 0 : 1;
    }
    if (std::string(only) == "search") {
      search_tests();
      std::printf("%s\n", failed == 0 ? "all checks passed" : "CHECKS FAILED");
      return failed == 0 ? 0 : 1;
    }
    if (std::string(only) == "fewest") {
      fewest_tests();
      std::printf("%s\n", failed == 0 ? "all checks passed" : "CHECKS FAILED");
      return failed == 0 ? 0 : 1;
    }
    if (std::string(only) == "ess") {
      catalogue_tests();
      ess_tests();
      exits_at_tests();
      std::printf("%s\n", failed == 0 ? "all checks passed" : "CHECKS FAILED");
      return failed == 0 ? 0 : 1;
    }
  }
  corridor_tests();
  selection_tests();
  grid_tests();
  mark_tests();
  cover_tests();
  ceiling_tests();
  adapter_tests();
  attached_tests();
  target_tests();
  ground_tests();
  list_tests();
  item_facing_tests();
  clipped_tests();
  mask_tests();
  within_tests();
  crease_tests();
  engine_tests();
  catalogue_tests();
  approx_tests();
  search_tests();
  travel_tests();
  steps_bound_tests();
  fewest_tests();
  verify_tests();
  cup_exit_tests();
  exits_at_tests();
  l_chain_tests();
  ess_tests();
  camera_type_tests();
  camera_list_tests();
  cam_clear_tests();
  seed_push_tests();
  landing_tests();
  std::printf("%s\n", failed == 0 ? "all checks passed" : "CHECKS FAILED");
  return failed == 0 ? 0 : 1;
}
