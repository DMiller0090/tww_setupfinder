#include "cam_clear.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "boundary/game_boundary.h"
#include "camera_type.h"
#include "l_chain.h"
#include "d/actor/d_a_player_main.h"
#include "d/d_cam_param.h"
#include "d/d_camera.h"
#include "engine/session.h"
#include "m_Do/m_Do_controller_pad.h"

namespace search {

static_assert(kCamSteps == l_chain::kTaps + 1, "one C-down step a tap, and one for the press");

namespace {

const double kPi = 3.14159265358979323846;
const double kInf = std::numeric_limits<double>::infinity();

int s16_of(int v) { return static_cast<int16_t>(static_cast<uint16_t>(v)); }

/* `dBgS::GetPolyCamId` is attribute word 0's low byte; `GetCamMoveBG` and `GetRoomCamId` word 2's
   low two bytes (`d_bg_s.cpp`). Only ground polygons choose a type. */
std::vector<int> room_types(const tww_engine::RoomDzb& dzb, const disc::CameraNames& names) {
  std::vector<char> seen(static_cast<size_t>(dCamera_c::type_num), 0);
  for (const cBgD_Tri_t& t : dzb.t_tbl) {
    if (t.vtx0 >= dzb.v_tbl.size() || t.vtx1 >= dzb.v_tbl.size() ||
        t.vtx2 >= dzb.v_tbl.size() || t.id >= dzb.ti_tbl.size()) {
      continue;
    }
    const cBgD_Vtx_t& a = dzb.v_tbl[t.vtx0];
    const cBgD_Vtx_t& b = dzb.v_tbl[t.vtx1];
    const cBgD_Vtx_t& c = dzb.v_tbl[t.vtx2];
    const double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    const double vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len <= 0.0 || ny / len < 0.5) continue;
    const cBgD_Ti_t& ti = dzb.ti_tbl[t.id];
    /* `checkGroundInfo`'s order: room cam id into the room's list, else poly cam id the stage's. */
    FloorCamera floor;
    const int room_cam = static_cast<int>((ti.mPolyInf2 >> 8) & 0xFF);
    floor.id = room_cam != 0xFF ? room_cam : static_cast<int>(ti.mPolyInf0 & 0xFF);
    floor.room = room_cam != 0xFF ? 0 : -1;
    floor.cam_move_bg = static_cast<int>(ti.mPolyInf2 & 0xFF);
    std::string why;
    const int type = camera_type(names, floor, &why);
    if (type >= 0 && type < dCamera_c::type_num) seen[static_cast<size_t>(type)] = 1;
  }
  std::vector<int> out;
  for (int i = 0; i < dCamera_c::type_num; ++i) {
    if (seen[static_cast<size_t>(i)]) out.push_back(i);
  }
  return out;
}

/* -1 unless the type's style in `mode` runs `algorithm`. */
int style_in(int type, int mode, int algorithm) {
  const int s = dCamera_c::types[type].mStyles[mode];
  if (s < 0 || s >= dCamParam_c::style_num) return -1;
  return dCamParam_c::styles[s].engineIdx == algorithm ? s : -1;
}

float param(int style, int i) { return dCamParam_c::styles[style].styleParam[i]; }

/* Tapes of each block's inputs, run on flat ground at the origin, reading where the camera's
   centre and eye go relative to Link. */
struct TapeFrame {
  float l = 0.0f;
  bool cdown = false;
  bool view = false;
  float stick = 0.0f;
  int facing = 0;
};

struct Tape {
  std::vector<TapeFrame> frame;
  size_t from = 0;
  /** -1 when C-down is never pressed. */
  long press = -1;
};

void push(Tape* t, int n, int facing, float l, bool cdown, bool view, float stick = 0.0f) {
  for (int i = 0; i < n; ++i) {
    TapeFrame f;
    f.l = l;
    f.cdown = cdown;
    f.view = view;
    f.stick = stick;
    f.facing = facing & 0xFFFF;
    t->frame.push_back(f);
  }
}

/* A C up turn of `steps` frames toward `dir`; the block follows it. */
int lead_turn(Tape* t, int start, int dir, int steps) {
  push(t, 11, start, 0, false, false);
  push(t, 3, start, 0, false, false);
  push(t, 20, start, 0, false, true);
  int f = start;
  for (int i = 0; i < steps; ++i) {
    f = (f + dir * 655) & 0xFFFF;
    push(t, 1, f, 0, false, true, dir > 0 ? -1.0f : 1.0f);
  }
  t->from = t->frame.size();
  push(t, 40, f, 0, false, true);
  return f;
}

void exit_b(Tape* t, int f) {
  push(t, 2, f, 0, false, true);
  push(t, 150, f, 0, false, false);
}

/* The C-down press and every L tap, 15 frames a step; one tape answers each step count by prefix. */
void cdown_steps(Tape* t, int f, bool view_first) {
  t->press = static_cast<long>(t->frame.size());
  if (view_first) {
    push(t, 4, f, 0, true, true);
    push(t, kCamStepFrames - 4, f, 0, true, false);
  } else {
    push(t, kCamStepFrames, f, 0, true, false);
  }
  for (int i = 1; i < kCamSteps; ++i) {
    push(t, 2, f, 1.0f, true, false);
    push(t, kCamStepFrames - 2, f, 0, true, false);
  }
  push(t, 20, f, 0, false, false);
}

struct Read {
  CamBounds env;
  bool any = false;
  int over = 0;
  double attention = 0;
  double step_reach[kCamSteps] = {0};
};

void widen(Read* r, double centre_y, double off, double reach, double rise) {
  if (!r->any) {
    r->env.centre_lo = r->env.centre_hi = centre_y;
    r->env.rise_lo = kInf;
    r->env.rise_hi = -kInf;
    r->any = true;
  }
  r->env.centre_lo = std::min(r->env.centre_lo, centre_y);
  r->env.centre_hi = std::max(r->env.centre_hi, centre_y);
  r->env.centre_off = std::max(r->env.centre_off, off);
  r->env.reach = std::max(r->env.reach, reach);
  if (reach > 1.0) {
    r->env.rise_lo = std::min(r->env.rise_lo, rise);
    r->env.rise_hi = std::max(r->env.rise_hi, rise);
  }
}

/* `over` is how far the eye's direction strays outside the start and tappable csangles (a frame
   whose csangle equals the one before). */
void run(int type, int seat, float radius, const Tape& tape, Read* r) {
  tww_engine::Init init;
  init.pos.set(0.0f, 0.0f, 0.0f);
  init.shape_angle_y = static_cast<s16>(tape.frame[0].facing);
  init.travel_angle_y = static_cast<s16>(tape.frame[0].facing);
  init.normal_speed = 0.0f;
  init.speed_f = 0.0f;
  init.proc = daPy_lk_c::daPyProc_WAIT_e;
  tww_engine::RunOptions opts;
  opts.ground = tww_engine::RunOptions::Ground::Supplied;
  opts.floor_y = 0.0f;
  opts.camera = true;
  opts.camera_yaw = static_cast<s16>(seat);
  opts.camera_type = type;
  opts.camera_radius = radius;
  tww_engine::Session session(init, NULL, opts);
  session.bind();
  r->attention = std::max(r->attention, static_cast<double>(session.lk.attention_info.position.y));
  bool lock = false;
  std::vector<int> eye_cs, yaw;
  int start_cam = 0;
  for (size_t i = 0; i < tape.frame.size(); ++i) {
    const TapeFrame& f = tape.frame[i];
    session.lk.shape_angle.y = static_cast<s16>(f.facing);
    session.lk.current.angle.y = static_cast<s16>(f.facing);
    g_mDoCPd_cpadInfo[PAD_1].mMainStickPosX = f.stick;
    g_mDoCPd_cpadInfo[PAD_1].mTriggerLeft = f.l;
    g_mDoCPd_cpadInfo[PAD_1].mCStickPosX = 0.0f;
    g_mDoCPd_cpadInfo[PAD_1].mCStickPosY = f.cdown ? -1.0f : 0.0f;
    g_mDoCPd_cpadInfo[PAD_1].mCStickValue = f.cdown ? 1.0f : 0.0f;
    if (f.l > 0.9f) lock = true;
    if (f.l < 0.6f) lock = false;
    cam_stub_lockon = lock;
    if (f.view) {
      session.lk.stub_player_status0 |= daPyStts0_SUBJECT_e;
    } else {
      session.lk.stub_player_status0 &= ~daPyStts0_SUBJECT_e;
    }
    session.runCamera(1);
    const tww_engine::Session::CameraFacts facts = session.cameraFacts();
    if (i + 1 == tape.from) start_cam = facts.yaw & 0xFFFF;
    if (i < tape.from) continue;
    const double cx = facts.center[0], cy = facts.center[1], cz = facts.center[2];
    const double dx = facts.eye[0] - cx, dy = facts.eye[1] - cy, dz = facts.eye[2] - cz;
    const double reach = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double rise = reach > 1.0 ? std::asin(std::max(-1.0, std::min(1.0, dy / reach))) *
                                          180.0 / kPi
                                    : 0.0;
    widen(r, cy, std::sqrt(cx * cx + cz * cz), reach, rise);
    for (int s = 1; s <= kCamSteps; ++s) {
      if (tape.press >= 0 &&
          static_cast<long>(i) >= tape.press + static_cast<long>(s) * kCamStepFrames) {
        continue;
      }
      r->step_reach[s - 1] = std::max(r->step_reach[s - 1], reach);
    }
    const int az = static_cast<int>(std::lround(std::atan2(dx, dz) * 32768.0 / kPi));
    eye_cs.push_back((az + 0x8000) & 0xFFFF);
    yaw.push_back(facts.yaw & 0xFFFF);
  }
  cam_stub_lockon = false;
  g_mDoCPd_cpadInfo[PAD_1].mMainStickPosX = 0.0f;
  g_mDoCPd_cpadInfo[PAD_1].mTriggerLeft = 0.0f;
  g_mDoCPd_cpadInfo[PAD_1].mCStickPosY = 0.0f;
  g_mDoCPd_cpadInfo[PAD_1].mCStickValue = 0.0f;

  const int face = tape.frame[tape.from].facing;
  for (size_t f = 1; f < yaw.size(); ++f) {
    if (yaw[f] != yaw[f - 1]) continue;
    int lo = 0, hi = 0;
    for (int v : {start_cam, yaw[f]}) {
      const int d = s16_of(v - face);
      lo = std::min(lo, d);
      hi = std::max(hi, d);
    }
    for (size_t g = 0; g <= f; ++g) {
      const int d = s16_of(eye_cs[g] - face);
      r->over = std::max(r->over, std::max(d - hi, lo - d));
    }
  }
}

/* `follow`: B out of the C up view. `view`: C-down out of it. `held`: C-down out of the follow
   camera (held L, the plain C down turnaround). */
void measure(int type, Read* view, Read* held, Read* follow) {
  std::vector<float> radii(1, 0.0f);
  const int f0 = style_in(type, 0, dCamAlg_FOLLOW_CAMERA_e);
  if (f0 >= 0) radii.push_back(param(f0, 0xb));
  const int kSeats[] = {0, 0x3000, -0x3000, 0x6000, -0x6000, 0x7F00, -0x7F00};
  for (float radius : radii) {
    for (int start : {0, 9170}) {
      for (int dir : {1, -1}) {
        for (int steps : {1, 17, 60}) {
          Tape b;
          exit_b(&b, lead_turn(&b, start, dir, steps));
          run(type, start, radius, b, follow);
          Tape c;
          cdown_steps(&c, lead_turn(&c, start, dir, steps), true);
          run(type, start, radius, c, view);
        }
      }
    }
    const int facing = 9170;
    for (int off : kSeats) {
      const int seat = (facing + off) & 0xFFFF;
      /* The view with no turn. */
      Tape b;
      push(&b, 11, facing, 0, false, false);
      b.from = b.frame.size();
      push(&b, 3, facing, 0, false, false);
      push(&b, 20, facing, 0, false, true);
      exit_b(&b, facing);
      run(type, seat, radius, b, follow);
      Tape c;
      push(&c, 11, facing, 0, false, false);
      c.from = c.frame.size();
      push(&c, 3, facing, 0, false, false);
      push(&c, 20, facing, 0, false, true);
      cdown_steps(&c, facing, true);
      run(type, seat, radius, c, view);
      /* Held L, C-down on its last 2 frames and kept held, then taps. */
      Tape l;
      push(&l, 11, facing, 0, false, false);
      l.from = l.frame.size();
      push(&l, 18, facing, 1.0f, false, false);
      l.press = static_cast<long>(l.frame.size());
      push(&l, 2, facing, 1.0f, true, false);
      push(&l, kCamStepFrames - 2, facing, 0, true, false);
      for (int i = 1; i < kCamSteps; ++i) {
        push(&l, 2, facing, 1.0f, true, false);
        push(&l, kCamStepFrames - 2, facing, 0, true, false);
      }
      push(&l, 20, facing, 0, false, false);
      run(type, seat, radius, l, held);
      /* The plain C down turnaround. */
      Tape d;
      push(&d, 11, facing, 0, false, false);
      d.from = d.frame.size();
      d.press = static_cast<long>(d.frame.size());
      push(&d, kCamStepFrames * kCamSteps, facing, 0, true, false);
      push(&d, 20, facing, 0, false, false);
      run(type, seat, radius, d, held);
    }
  }
}

void grow(CamBounds* b, const CamBounds& with) {
  b->reach = std::max(b->reach, with.reach);
  b->rise_lo = std::min(b->rise_lo, with.rise_lo);
  b->rise_hi = std::max(b->rise_hi, with.rise_hi);
  b->centre_lo = std::min(b->centre_lo, with.centre_lo);
  b->centre_hi = std::max(b->centre_hi, with.centre_hi);
  b->centre_off = std::max(b->centre_off, with.centre_off);
}

/* A voxel is set when a camera polygon's plane crosses it within the polygon's box. Each direction
   is a template of voxel columns covering every line the camera can draw from a cell's layer, each
   column tagged with its nearest ring; reach R is clear where the first set ring is at or past
   `ceil(R / kRing)`. */
const double kVoxel = 25.0;
const int kPer = 2;  // voxels a cell, and a layer

struct P2 {
  double x, z;
};

double cross(const P2& o, const P2& a, const P2& b) {
  return (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
}

std::vector<P2> hull(std::vector<P2> p) {
  std::sort(p.begin(), p.end(), [](const P2& a, const P2& b) {
    return a.x < b.x || (a.x == b.x && a.z < b.z);
  });
  if (p.size() < 3) return p;
  std::vector<P2> h(2 * p.size());
  size_t k = 0;
  for (size_t i = 0; i < p.size(); ++i) {
    while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0) --k;
    h[k++] = p[i];
  }
  for (size_t i = p.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross(h[k - 2], h[k - 1], p[i - 1]) <= 0) --k;
    h[k++] = p[i - 1];
  }
  h.resize(k - 1);
  return h;
}

/* `poly` convex, counter-clockwise. */
bool meets(const std::vector<P2>& poly, double x0, double z0, double x1, double z1) {
  double px0 = kInf, pz0 = kInf, px1 = -kInf, pz1 = -kInf;
  for (const P2& p : poly) {
    px0 = std::min(px0, p.x);
    pz0 = std::min(pz0, p.z);
    px1 = std::max(px1, p.x);
    pz1 = std::max(pz1, p.z);
  }
  if (px1 < x0 || px0 > x1 || pz1 < z0 || pz0 > z1) return false;
  const P2 corner[4] = {{x0, z0}, {x1, z0}, {x1, z1}, {x0, z1}};
  for (size_t i = 0; i < poly.size(); ++i) {
    const P2& a = poly[i];
    const P2& b = poly[(i + 1) % poly.size()];
    bool all_out = true;
    for (const P2& c : corner) {
      if (cross(a, b, c) >= 0) {
        all_out = false;
        break;
      }
    }
    if (all_out) return false;
  }
  return true;
}

struct Column {
  int16_t dx, dz, lo, hi;
  int16_t ring;
};

/* Relative to the voxel at a cell's and layer's low corner, nearest ring first. */
std::vector<std::vector<Column>> templates(const CamBounds& b) {
  std::vector<std::vector<Column>> out(static_cast<size_t>(CamField::kBins));
  const double side = CamField::kCell;
  const double off = b.centre_off;
  const P2 square[4] = {{-off, -off}, {side + off, -off}, {side + off, side + off},
                        {-off, side + off}};
  const double half = kPi / CamField::kBins;
  const double tan_lo = std::tan(std::max(-80.0, std::min(80.0, b.rise_lo)) * kPi / 180.0);
  const double tan_hi = std::tan(std::max(-80.0, std::min(80.0, b.rise_hi)) * kPi / 180.0);
  for (int bin = 0; bin < CamField::kBins; ++bin) {
    const double a0 = 2.0 * kPi * bin / CamField::kBins;
    const double a1 = a0 + 2.0 * half;
    const double am = a0 + half;
    std::unordered_map<int32_t, Column> cols;
    const int rings = static_cast<int>(std::ceil(b.reach / CamField::kRing));
    for (int k = 0; k < rings; ++k) {
      const double h0 = CamField::kRing * k;
      const double h1 = CamField::kRing * (k + 1);
      /* The cell swept through the annulus h0..h1; the outer arc is bounded by its tangents. */
      const P2 arc[5] = {{h0 * std::sin(a0), h0 * std::cos(a0)},
                         {h0 * std::sin(a1), h0 * std::cos(a1)},
                         {h1 * std::sin(a0), h1 * std::cos(a0)},
                         {h1 * std::sin(a1), h1 * std::cos(a1)},
                         {h1 / std::cos(half) * std::sin(am), h1 / std::cos(half) * std::cos(am)}};
      std::vector<P2> pts;
      for (const P2& s : square) {
        for (const P2& a : arc) pts.push_back({s.x + a.x, s.z + a.z});
      }
      const std::vector<P2> poly = hull(pts);
      const double y_lo = b.centre_lo + std::min(h0 * tan_lo, h1 * tan_lo);
      const double y_hi = CamField::kLayer + b.centre_hi + std::max(h0 * tan_hi, h1 * tan_hi);
      const int v_lo = static_cast<int>(std::floor(y_lo / kVoxel));
      const int v_hi = static_cast<int>(std::floor(y_hi / kVoxel));
      double px0 = kInf, pz0 = kInf, px1 = -kInf, pz1 = -kInf;
      for (const P2& p : poly) {
        px0 = std::min(px0, p.x);
        pz0 = std::min(pz0, p.z);
        px1 = std::max(px1, p.x);
        pz1 = std::max(pz1, p.z);
      }
      for (int vx = static_cast<int>(std::floor(px0 / kVoxel));
           vx <= static_cast<int>(std::floor(px1 / kVoxel)); ++vx) {
        for (int vz = static_cast<int>(std::floor(pz0 / kVoxel));
             vz <= static_cast<int>(std::floor(pz1 / kVoxel)); ++vz) {
          if (!meets(poly, vx * kVoxel, vz * kVoxel, (vx + 1) * kVoxel, (vz + 1) * kVoxel)) {
            continue;
          }
          const int32_t id = (vx + 32768) * 65536 + (vz + 32768);
          std::unordered_map<int32_t, Column>::iterator it = cols.find(id);
          if (it == cols.end()) {
            Column c;
            c.dx = static_cast<int16_t>(vx);
            c.dz = static_cast<int16_t>(vz);
            c.lo = static_cast<int16_t>(v_lo);
            c.hi = static_cast<int16_t>(v_hi);
            c.ring = static_cast<int16_t>(k);
            cols[id] = c;
          } else {
            it->second.lo = std::min<int16_t>(it->second.lo, static_cast<int16_t>(v_lo));
            it->second.hi = std::max<int16_t>(it->second.hi, static_cast<int16_t>(v_hi));
          }
        }
      }
    }
    std::vector<Column>& list = out[static_cast<size_t>(bin)];
    for (std::unordered_map<int32_t, Column>::const_iterator it = cols.begin(); it != cols.end();
         ++it) {
      list.push_back(it->second);
    }
    std::sort(list.begin(), list.end(), [](const Column& a, const Column& c) {
      return a.ring < c.ring ||
             (a.ring == c.ring && (a.dx < c.dx || (a.dx == c.dx && a.dz < c.dz)));
    });
  }
  return out;
}

struct Occupancy {
  long long x0 = 0, z0 = 0, y0 = 0;
  long long nx = 0, nz = 0, ny = 0, words = 0;
  std::vector<uint64_t> bit;

  bool any(long long vx, long long vz, long long lo, long long hi) const {
    if (vx < x0 || vz < z0 || vx >= x0 + nx || vz >= z0 + nz) return false;
    lo -= y0;
    hi -= y0;
    if (lo < 0) lo = 0;
    if (hi >= ny) hi = ny - 1;
    if (hi < lo) return false;
    const uint64_t* col = &bit[static_cast<size_t>(((vz - z0) * nx + (vx - x0)) * words)];
    const long long w0 = lo >> 6, w1 = hi >> 6;
    for (long long w = w0; w <= w1; ++w) {
      uint64_t m = ~0ULL;
      if (w == w0) m &= ~0ULL << (lo & 63);
      if (w == w1 && (hi & 63) != 63) m &= (1ULL << ((hi & 63) + 1)) - 1;
      if (col[w] & m) return true;
    }
    return false;
  }

  void set(long long vx, long long vy, long long vz) {
    if (vx < x0 || vz < z0 || vy < y0 || vx >= x0 + nx || vz >= z0 + nz || vy >= y0 + ny) return;
    const long long y = vy - y0;
    bit[static_cast<size_t>(((vz - z0) * nx + (vx - x0)) * words + (y >> 6))] |= 1ULL << (y & 63);
  }
};

/* `ChkPolyThrough` passes a camera through attribute word 3 bit 0; every other polygon is kept. */
void voxelise(const tww_engine::RoomDzb& dzb, Occupancy* o) {
  const double h = kVoxel * 0.5;
  for (const cBgD_Tri_t& t : dzb.t_tbl) {
    if (t.vtx0 >= dzb.v_tbl.size() || t.vtx1 >= dzb.v_tbl.size() ||
        t.vtx2 >= dzb.v_tbl.size()) {
      continue;
    }
    if (t.id < dzb.ti_tbl.size() && (dzb.ti_tbl[t.id].mPolyInf3 & 0x01)) continue;
    const cBgD_Vtx_t& a = dzb.v_tbl[t.vtx0];
    const cBgD_Vtx_t& b = dzb.v_tbl[t.vtx1];
    const cBgD_Vtx_t& c = dzb.v_tbl[t.vtx2];
    const double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    const double vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len <= 0.0) continue;
    nx /= len;
    ny /= len;
    nz /= len;
    const double reach = h * (std::fabs(nx) + std::fabs(ny) + std::fabs(nz));
    const long long bx0 = static_cast<long long>(std::floor(std::min({a.x, b.x, c.x}) / kVoxel));
    const long long bx1 = static_cast<long long>(std::floor(std::max({a.x, b.x, c.x}) / kVoxel));
    const long long by0 = static_cast<long long>(std::floor(std::min({a.y, b.y, c.y}) / kVoxel));
    const long long by1 = static_cast<long long>(std::floor(std::max({a.y, b.y, c.y}) / kVoxel));
    const long long bz0 = static_cast<long long>(std::floor(std::min({a.z, b.z, c.z}) / kVoxel));
    const long long bz1 = static_cast<long long>(std::floor(std::max({a.z, b.z, c.z}) / kVoxel));
    const long long ex0 = std::max(bx0, o->x0), ex1 = std::min(bx1, o->x0 + o->nx - 1);
    const long long ez0 = std::max(bz0, o->z0), ez1 = std::min(bz1, o->z0 + o->nz - 1);
    const long long ey0 = std::max(by0, o->y0), ey1 = std::min(by1, o->y0 + o->ny - 1);
    for (long long x = ex0; x <= ex1; ++x) {
      for (long long z = ez0; z <= ez1; ++z) {
        for (long long y = ey0; y <= ey1; ++y) {
          const double cx = (x + 0.5) * kVoxel - a.x;
          const double cy = (y + 0.5) * kVoxel - a.y;
          const double cz = (z + 0.5) * kVoxel - a.z;
          if (std::fabs(nx * cx + ny * cy + nz * cz) <= reach) o->set(x, y, z);
        }
      }
    }
  }
}

void unpack(uint64_t k, long long* ix, long long* iz, long long* iy) {
  const uint64_t v = k - 1;
  *ix = static_cast<long long>(v & 0x1FFFFF) - (1LL << 20);
  *iz = static_cast<long long>((v >> 21) & 0x1FFFFF) - (1LL << 20);
  *iy = static_cast<long long>((v >> 42) & 0x1FFFFF) - (1LL << 20);
}

void first_rings(const std::vector<std::vector<Column>>& tmpl, const Occupancy& o, long long vx,
                 long long vz, long long vy, uint8_t* out) {
  for (int bin = 0; bin < CamField::kBins; ++bin) {
    uint8_t first = CamField::kOpen;
    for (const Column& c : tmpl[static_cast<size_t>(bin)]) {
      if (o.any(vx + c.dx, vz + c.dz, vy + c.lo, vy + c.hi)) {
        first = static_cast<uint8_t>(std::min<int>(c.ring, CamField::kOpen - 1));
        break;
      }
    }
    out[bin] = first;
  }
}

/* Cache: one file a room, named by a collision hash; a head of bounds (a mismatch is ignored), then
   one key and row a place. */
const uint32_t kMagic = 0x464D4143;  // "CAMF"
const uint32_t kVersion = 7;
const size_t kRow = 2 * CamField::kBins;

uint64_t fnv(uint64_t h, const void* p, size_t n) {
  const unsigned char* b = static_cast<const unsigned char*>(p);
  for (size_t i = 0; i < n; ++i) {
    h ^= b[i];
    h *= 0x100000001b3ULL;
  }
  return h;
}

uint64_t room_hash(const tww_engine::RoomDzb& dzb) {
  uint64_t h = 0xcbf29ce484222325ULL;
  for (const cBgD_Vtx_t& v : dzb.v_tbl) {
    const float xyz[3] = {v.x, v.y, v.z};
    h = fnv(h, xyz, sizeof(xyz));
  }
  for (const cBgD_Tri_t& t : dzb.t_tbl) {
    const uint16_t w[5] = {t.vtx0, t.vtx1, t.vtx2, t.id, t.grp};
    h = fnv(h, w, sizeof(w));
  }
  for (const cBgD_Ti_t& t : dzb.ti_tbl) {
    const uint32_t w[4] = {t.mPolyInf0, t.mPolyInf1, t.mPolyInf2, t.mPolyInf3};
    h = fnv(h, w, sizeof(w));
  }
  return h;
}

/* Covers the types and their style parameters; engine camera changes need a `kVersion` bump. */
uint64_t types_hash(const std::vector<int>& types) {
  uint64_t h = 0xcbf29ce484222325ULL;
  for (int t : types) {
    h = fnv(h, &t, sizeof(t));
    for (int mode : {0, 1, 12}) {
      const int st = dCamera_c::types[t].mStyles[mode];
      h = fnv(h, &st, sizeof(st));
      if (st >= 0 && st < dCamParam_c::style_num) {
        h = fnv(h, dCamParam_c::styles[st].styleParam, sizeof(dCamParam_c::styles[st].styleParam));
      }
    }
  }
  return h;
}

struct Head {
  uint64_t types = 0;
  int64_t margin = 0;
  std::vector<double> bounds;
  std::vector<int32_t> steps;
  uint64_t count = 0;

  static size_t row_bytes() { return sizeof(uint64_t) + kRow; }

  static Head of(const CamRoom& room) {
    Head h;
    h.types = types_hash(room.types);
    h.margin = room.margin;
    for (const CamBounds& b : room.level) {
      for (double v : {b.reach, b.rise_lo, b.rise_hi, b.centre_lo, b.centre_hi, b.centre_off}) {
        h.bounds.push_back(v);
      }
    }
    for (int s = 0; s < kCamSteps; ++s) h.steps.push_back(room.from_view[s]);
    for (int s = 0; s < kCamSteps; ++s) h.steps.push_back(room.from_follow[s]);
    return h;
  }

  bool same(const Head& o) const {
    return types == o.types && margin == o.margin && bounds == o.bounds && steps == o.steps;
  }

  void write(std::ostream& out) const {
    const uint32_t fixed[2] = {kMagic, kVersion};
    out.write(reinterpret_cast<const char*>(fixed), sizeof(fixed));
    out.write(reinterpret_cast<const char*>(&types), sizeof(types));
    out.write(reinterpret_cast<const char*>(&margin), sizeof(margin));
    const uint64_t n = bounds.size() / 6;
    out.write(reinterpret_cast<const char*>(&n), sizeof(n));
    out.write(reinterpret_cast<const char*>(bounds.data()),
              static_cast<std::streamsize>(bounds.size() * sizeof(double)));
    out.write(reinterpret_cast<const char*>(steps.data()),
              static_cast<std::streamsize>(steps.size() * sizeof(int32_t)));
    out.write(reinterpret_cast<const char*>(&count), sizeof(count));
  }

  bool read(std::istream& in) {
    uint32_t fixed[2] = {0, 0};
    uint64_t n = 0;
    if (!in.read(reinterpret_cast<char*>(fixed), sizeof(fixed)) || fixed[0] != kMagic ||
        fixed[1] != kVersion || !in.read(reinterpret_cast<char*>(&types), sizeof(types)) ||
        !in.read(reinterpret_cast<char*>(&margin), sizeof(margin)) ||
        !in.read(reinterpret_cast<char*>(&n), sizeof(n)) || n == 0 || n > 64) {
      return false;
    }
    bounds.assign(static_cast<size_t>(n) * 6, 0.0);
    steps.assign(2 * kCamSteps, 0);
    return in.read(reinterpret_cast<char*>(bounds.data()),
                   static_cast<std::streamsize>(bounds.size() * sizeof(double))) &&
           in.read(reinterpret_cast<char*>(steps.data()),
                   static_cast<std::streamsize>(steps.size() * sizeof(int32_t))) &&
           in.read(reinterpret_cast<char*>(&count), sizeof(count)) && count < (1ULL << 32);
  }
};

/* The farthest C-down pulls the manual camera in `frames`: `manualCamera` adds parameter 14 a
   frame, clamped to parameters 11 and 12, and the eye chases the radius from below. */
double pulled(const std::vector<int>& types, bool out_of_view, int frames) {
  double most = 0;
  for (int type : types) {
    const int m = style_in(type, 12, dCamAlg_MANUAL_CAMERA_e);
    if (m < 0) continue;
    double start = param(m, 0xb);
    for (int mode = 0; mode < 2; ++mode) {
      const int f = style_in(type, mode, dCamAlg_FOLLOW_CAMERA_e);
      if (f < 0) continue;
      start = std::max(start, static_cast<double>(param(f, out_of_view ? 0xb : 10)));
    }
    const double upper = param(m, 0xc);
    most = std::max(most, std::min(upper, start + param(m, 0xe) * frames));
  }
  return most;
}

}  // namespace

CamRoom cam_room(const tww_engine::RoomDzb& dzb, const disc::CameraNames* names,
                 const std::string& cache) {
  CamRoom room;
  if (names != nullptr) {
    room.types = room_types(dzb, *names);
  }
  /* No follow-camera type on the floors: the game's fallback, the stage's type, else Field (7). */
  if (names != nullptr) {
    bool follows = false;
    for (int t : room.types) follows = follows || style_in(t, 0, dCamAlg_FOLLOW_CAMERA_e) >= 0;
    if (!follows) {
      std::string why;
      const int stage = camera_type(*names, FloorCamera(), &why);
      const int fall = stage >= 0 && style_in(stage, 0, dCamAlg_FOLLOW_CAMERA_e) >= 0 ? stage : 7;
      room.types.push_back(fall);
      std::sort(room.types.begin(), room.types.end());
      room.types.erase(std::unique(room.types.begin(), room.types.end()), room.types.end());
    }
  }
  if (room.types.empty()) {
    room.every_type = true;
    for (int t = 0; t < dCamera_c::type_num; ++t) {
      if (style_in(t, 0, dCamAlg_FOLLOW_CAMERA_e) >= 0) room.types.push_back(t);
    }
  }

  if (!cache.empty()) {
    std::ifstream in(cache, std::ios::binary);
    Head got;
    if (in && got.read(in) && got.types == types_hash(room.types)) {
      for (size_t i = 0; i + 6 <= got.bounds.size(); i += 6) {
        CamBounds b;
        b.reach = got.bounds[i];
        b.rise_lo = got.bounds[i + 1];
        b.rise_hi = got.bounds[i + 2];
        b.centre_lo = got.bounds[i + 3];
        b.centre_hi = got.bounds[i + 4];
        b.centre_off = got.bounds[i + 5];
        room.level.push_back(b);
      }
      for (int s = 0; s < kCamSteps; ++s) {
        room.from_view[s] = got.steps[static_cast<size_t>(s)];
        room.from_follow[s] = got.steps[static_cast<size_t>(kCamSteps + s)];
      }
      room.margin = static_cast<int>(got.margin);
      room.stray = std::max(0, (room.margin - 64) / 2);
      room.from_disc = true;
      room.ok = true;
      return room;
    }
  }

  Read view, held, follow;
  for (int type : room.types) {
    if (style_in(type, 0, dCamAlg_FOLLOW_CAMERA_e) < 0) continue;
    measure(type, &view, &held, &follow);
  }
  if (!view.any || !held.any || !follow.any) {
    room.why = "no camera type on the room's floors runs a follow camera";
    return room;
  }
  /* Widen what the tapes measured; reach is not widened, being clamped by the style. */
  for (Read* r : {&view, &held, &follow}) {
    r->env.rise_lo -= 1.0;
    r->env.rise_hi += 1.0;
    r->env.centre_lo -= 5.0;
    r->env.centre_hi += 5.0;
    r->env.centre_off += 5.0;
  }

  /* Follow reach: parameter 10. Manual: latitude 16 and 17, centre height 6 and 7. */
  CamBounds follow_b = follow.env;
  CamBounds manual_b = view.env;
  grow(&manual_b, held.env);
  for (int type : room.types) {
    for (int mode = 0; mode < 2; ++mode) {
      const int f = style_in(type, mode, dCamAlg_FOLLOW_CAMERA_e);
      if (f < 0) continue;
      follow_b.reach = std::max(follow_b.reach, static_cast<double>(param(f, 10)));
    }
    const int m = style_in(type, 12, dCamAlg_MANUAL_CAMERA_e);
    if (m < 0) continue;
    CamBounds s;
    s.reach = 0;
    s.rise_lo = std::min(param(m, 0x10), param(m, 0x11));
    s.rise_hi = std::max(param(m, 0x10), param(m, 0x11));
    s.centre_lo = view.attention + std::min(param(m, 6), param(m, 7));
    s.centre_hi = view.attention + std::max(param(m, 6), param(m, 7));
    s.centre_off = std::sqrt(static_cast<double>(param(m, 0)) * param(m, 0) +
                             static_cast<double>(param(m, 1)) * param(m, 1));
    grow(&manual_b, s);
  }

  /* One reach per step count and start, never below what the tapes reached; equal ones share. */
  room.level.push_back(follow_b);
  std::vector<double> reaches;
  double at_view[kCamSteps], at_follow[kCamSteps];
  for (int s = 1; s <= kCamSteps; ++s) {
    at_view[s - 1] = std::max(pulled(room.types, true, s * kCamStepFrames), view.step_reach[s - 1]);
    at_follow[s - 1] =
        std::max(pulled(room.types, false, s * kCamStepFrames), held.step_reach[s - 1]);
    at_view[s - 1] = std::ceil(at_view[s - 1] / CamField::kRing - 1e-9) * CamField::kRing;
    at_follow[s - 1] = std::ceil(at_follow[s - 1] / CamField::kRing - 1e-9) * CamField::kRing;
    reaches.push_back(at_view[s - 1]);
    reaches.push_back(at_follow[s - 1]);
  }
  std::sort(reaches.begin(), reaches.end());
  reaches.erase(std::unique(reaches.begin(), reaches.end()), reaches.end());
  for (double r : reaches) {
    CamBounds b = manual_b;
    b.reach = r;
    room.level.push_back(b);
  }
  for (int s = 0; s < kCamSteps; ++s) {
    room.from_view[s] = 1 + static_cast<int>(
        std::lower_bound(reaches.begin(), reaches.end(), at_view[s]) - reaches.begin());
    room.from_follow[s] = 1 + static_cast<int>(
        std::lower_bound(reaches.begin(), reaches.end(), at_follow[s]) - reaches.begin());
  }

  /* Doubled, and 64 more, for tapes not run. */
  room.stray = std::max(std::max(view.over, held.over), follow.over);
  room.margin = 2 * room.stray + 64;
  room.ok = true;
  return room;
}

uint64_t CamField::key_of(long long ix, long long iz, long long iy) {
  const uint64_t x = static_cast<uint64_t>(ix + (1LL << 20)) & 0x1FFFFF;
  const uint64_t z = static_cast<uint64_t>(iz + (1LL << 20)) & 0x1FFFFF;
  const uint64_t y = static_cast<uint64_t>(iy + (1LL << 20)) & 0x1FFFFF;
  return (x | (z << 21) | (y << 42)) + 1;
}

uint64_t CamField::key_at(double x, double y, double z) {
  return key_of(static_cast<long long>(std::floor(x / kCell)),
                static_cast<long long>(std::floor(z / kCell)),
                static_cast<long long>(std::floor(y / kLayer)));
}

uint64_t CamField::bins(int lo, int span, int margin) {
  /* Kept positive so the shifts floor. */
  const long long a = static_cast<long long>(lo & 0xFFFF) + 0x8000 - margin + 65536LL * 4;
  const long long b = a + span + 2LL * margin;
  if (span < 0 || b - a >= 65536 - 1024) return ~0ULL;
  const long long first = a >> 10, last = b >> 10;
  const long long n = last - first + 1;
  if (n >= kBins) return ~0ULL;
  const uint64_t run = (1ULL << n) - 1;
  const int at = static_cast<int>(first & 63);
  return at == 0 ? run : ((run << at) | (run >> (64 - at)));
}

void CamField::fill(const std::vector<uint64_t>& keys, const std::vector<uint8_t>& rings,
                    const CamRoom& room) {
  size_t cap = 16;
  while (cap < keys.size() * 2) cap <<= 1;
  levels_ = static_cast<int>(room.level.size());
  key_.assign(cap, 0);
  ring_.assign(cap * kRowBytes, 0);
  mask_ = cap - 1;
  size_ = 0;
  margin_ = room.margin;
  from_view_.assign(room.from_view, room.from_view + kCamSteps);
  from_follow_.assign(room.from_follow, room.from_follow + kCamSteps);
  need_.clear();
  for (const CamBounds& l : room.level) {
    need_.push_back(static_cast<int>(std::ceil(l.reach / kRing - 1e-9)));
  }
  for (size_t n = 0; n < keys.size(); ++n) {
    size_t i = static_cast<size_t>(mix(keys[n])) & mask_;
    while (key_[i] != 0 && key_[i] != keys[n]) i = (i + 1) & mask_;
    if (key_[i] == 0) ++size_;
    key_[i] = keys[n];
    std::copy(rings.begin() + static_cast<std::ptrdiff_t>(n * kRowBytes),
              rings.begin() + static_cast<std::ptrdiff_t>((n + 1) * kRowBytes),
              ring_.begin() + static_cast<std::ptrdiff_t>(i * kRowBytes));
  }
}

std::vector<uint64_t> cam_places(const Selection& selection, const Corridor& corridor,
                                 bool reads_floors, double start_y) {
  std::vector<uint64_t> out;
  const double c = CamField::kCell, l = CamField::kLayer;
  const long long cx0 = static_cast<long long>(std::floor(corridor.min_x / c));
  const long long cx1 = static_cast<long long>(std::floor(corridor.max_x / c));
  const long long cz0 = static_cast<long long>(std::floor(corridor.min_z / c));
  const long long cz1 = static_cast<long long>(std::floor(corridor.max_z / c));
  if (reads_floors) {
    const std::vector<float>& g = selection.ground;
    for (size_t t = 0; t + 9 <= g.size(); t += 9) {
      const double x0 = std::min({g[t], g[t + 3], g[t + 6]});
      const double x1 = std::max({g[t], g[t + 3], g[t + 6]});
      const double y0 = std::min({g[t + 1], g[t + 4], g[t + 7]});
      const double y1 = std::max({g[t + 1], g[t + 4], g[t + 7]});
      const double z0 = std::min({g[t + 2], g[t + 5], g[t + 8]});
      const double z1 = std::max({g[t + 2], g[t + 5], g[t + 8]});
      const long long ix0 = std::max(cx0, static_cast<long long>(std::floor(x0 / c)));
      const long long ix1 = std::min(cx1, static_cast<long long>(std::floor(x1 / c)));
      const long long iz0 = std::max(cz0, static_cast<long long>(std::floor(z0 / c)));
      const long long iz1 = std::min(cz1, static_cast<long long>(std::floor(z1 / c)));
      const long long iy0 = static_cast<long long>(std::floor(y0 / l));
      const long long iy1 = static_cast<long long>(std::floor(y1 / l));
      for (long long ix = ix0; ix <= ix1; ++ix) {
        for (long long iz = iz0; iz <= iz1; ++iz) {
          for (long long iy = iy0; iy <= iy1; ++iy) out.push_back(CamField::key_of(ix, iz, iy));
        }
      }
    }
  }
  const long long sy = static_cast<long long>(std::floor(start_y / l));
  if (!reads_floors) {
    for (long long ix = cx0; ix <= cx1; ++ix) {
      for (long long iz = cz0; iz <= cz1; ++iz) out.push_back(CamField::key_of(ix, iz, sy));
    }
  } else {
    out.push_back(CamField::key_at(corridor.ax, start_y, corridor.az));
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

std::string cam_cache_path(const std::string& dir, const tww_engine::RoomDzb& dzb) {
  if (dir.empty()) return std::string();
  char name[64];
  std::snprintf(name, sizeof(name), "camera-%016llx.bin",
                static_cast<unsigned long long>(room_hash(dzb)));
  return (std::filesystem::path(dir) / name).string();
}

namespace {

long long floor_div(long long v, long long by) { return v >= 0 ? v / by : -((-v + by - 1) / by); }

/* All or nothing: a file cut short or with trailing bytes is ignored. */
std::unordered_map<uint64_t, std::vector<uint8_t>> read_cache(const std::string& cache,
                                                              const Head& want) {
  std::unordered_map<uint64_t, std::vector<uint8_t>> held;
  if (cache.empty()) return held;
  std::ifstream in(cache, std::ios::binary);
  Head got;
  if (!in || !got.read(in) || !got.same(want)) return held;
  std::vector<uint8_t> row(kRow);
  for (uint64_t n = 0; n < got.count; ++n) {
    uint64_t k = 0;
    if (!in.read(reinterpret_cast<char*>(&k), sizeof(k)) ||
        !in.read(reinterpret_cast<char*>(row.data()), static_cast<std::streamsize>(kRow))) {
      held.clear();
      return held;
    }
    held[k] = row;
  }
  if (in.peek() != std::char_traits<char>::eof()) held.clear();
  return held;
}

/* Through a uniquely named part file renamed over `cache`, so concurrent writers never interleave. */
bool write_cache(const std::string& cache, const Head& want,
                 const std::unordered_map<uint64_t, std::vector<uint8_t>>& rows) {
  if (cache.empty()) return false;
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(cache).parent_path(), ec);
  static std::atomic<unsigned> written(0);
  char tag[64];
  std::snprintf(tag, sizeof(tag), ".%llx.%u.part",
                static_cast<unsigned long long>(
                    std::chrono::steady_clock::now().time_since_epoch().count()),
                written.fetch_add(1));
  const std::string part = cache + tag;
  bool ok = false;
  {
    std::ofstream out(part, std::ios::binary | std::ios::trunc);
    Head h = want;
    h.count = rows.size();
    h.write(out);
    std::vector<uint64_t> order;
    for (const auto& r : rows) order.push_back(r.first);
    std::sort(order.begin(), order.end());
    for (uint64_t k : order) {
      const std::vector<uint8_t>& row = rows.at(k);
      out.write(reinterpret_cast<const char*>(&k), sizeof(k));
      out.write(reinterpret_cast<const char*>(row.data()), static_cast<std::streamsize>(row.size()));
    }
    ok = static_cast<bool>(out);
  }
  if (!ok) {
    std::filesystem::remove(part, ec);
    return false;
  }
  std::filesystem::rename(part, cache, ec);
  if (ec) {
    std::filesystem::remove(cache, ec);
    std::filesystem::rename(part, cache, ec);
  }
  if (ec) std::filesystem::remove(part, ec);
  return !ec;
}

}  // namespace

/* A tile is `kTile` cells a side; it voxelises only what its own places reach, bounding memory. */
struct CamField::Builder {
  static constexpr long long kTile = 16;
  const tww_engine::RoomDzb* dzb = nullptr;
  std::vector<std::vector<Column>> tf, tm;
  int ext = 0, ylo = 0, yhi = 0;

  Builder(const tww_engine::RoomDzb& room_dzb, const CamRoom& room) : dzb(&room_dzb) {
    /* One template a camera, to its farthest level's reach. */
    CamBounds manual_b = room.level[1];
    for (size_t l = 1; l < room.level.size(); ++l) {
      manual_b.reach = std::max(manual_b.reach, room.level[l].reach);
    }
    tf = templates(room.level[0]);
    tm = templates(manual_b);
    for (const std::vector<std::vector<Column>>* t : {&tf, &tm}) {
      for (const std::vector<Column>& list : *t) {
        for (const Column& c : list) {
          ext = std::max(ext, std::max(std::abs(static_cast<int>(c.dx)),
                                       std::abs(static_cast<int>(c.dz))));
          ylo = std::min(ylo, static_cast<int>(c.lo));
          yhi = std::max(yhi, static_cast<int>(c.hi));
        }
      }
    }
  }

  static std::pair<long long, long long> tile_of(uint64_t key) {
    long long ix, iz, iy;
    unpack(key, &ix, &iz, &iy);
    return {floor_div(ix, kTile), floor_div(iz, kTile)};
  }

  /** `keys` all in one tile; `kRowBytes` a key into `rows`. */
  void tile(const std::vector<uint64_t>& keys, uint8_t* rows) const {
    long long x0 = std::numeric_limits<long long>::max(), x1 = std::numeric_limits<long long>::min();
    long long z0 = x0, z1 = x1, y0 = x0, y1 = x1;
    for (uint64_t k : keys) {
      long long ix, iz, iy;
      unpack(k, &ix, &iz, &iy);
      x0 = std::min(x0, ix * kPer);
      x1 = std::max(x1, ix * kPer + kPer - 1);
      z0 = std::min(z0, iz * kPer);
      z1 = std::max(z1, iz * kPer + kPer - 1);
      y0 = std::min(y0, iy * kPer);
      y1 = std::max(y1, iy * kPer);
    }
    Occupancy o;
    o.x0 = x0 - ext - 1;
    o.z0 = z0 - ext - 1;
    o.y0 = y0 + ylo - 1;
    o.nx = (x1 + ext + 2) - o.x0;
    o.nz = (z1 + ext + 2) - o.z0;
    o.ny = (y1 + yhi + 2) - o.y0;
    o.words = (o.ny + 63) / 64;
    o.bit.assign(static_cast<size_t>(o.nx * o.nz * o.words), 0);
    voxelise(*dzb, &o);
    for (size_t n = 0; n < keys.size(); ++n) {
      long long ix, iz, iy;
      unpack(keys[n], &ix, &iz, &iy);
      uint8_t* row = rows + n * kRow;
      first_rings(tf, o, ix * kPer, iz * kPer, iy * kPer, row);
      first_rings(tm, o, ix * kPer, iz * kPer, iy * kPer, row + CamField::kBins);
    }
  }
};

struct CamField::Later {
  std::mutex lock;
  std::unordered_map<uint64_t, std::vector<uint8_t>> rows;
};

bool CamField::later(uint64_t k, int level, uint64_t need) const {
  if (!builder_ || !later_) return false;
  std::lock_guard<std::mutex> hold(later_->lock);
  std::unordered_map<uint64_t, std::vector<uint8_t>>::const_iterator it = later_->rows.find(k);
  if (it == later_->rows.end()) {
    /* The whole tile on this layer: the walk is about to reach the neighbours. */
    long long ix, iz, iy;
    unpack(k, &ix, &iz, &iy);
    const long long tx = floor_div(ix, Builder::kTile), tz = floor_div(iz, Builder::kTile);
    std::vector<uint64_t> keys;
    for (long long x = tx * Builder::kTile; x < (tx + 1) * Builder::kTile; ++x) {
      for (long long z = tz * Builder::kTile; z < (tz + 1) * Builder::kTile; ++z) {
        const uint64_t key = key_of(x, z, iy);
        if (later_->rows.find(key) == later_->rows.end()) keys.push_back(key);
      }
    }
    std::vector<uint8_t> rows(keys.size() * kRowBytes);
    builder_->tile(keys, rows.data());
    for (size_t n = 0; n < keys.size(); ++n) {
      later_->rows[keys[n]] = std::vector<uint8_t>(
          rows.begin() + static_cast<std::ptrdiff_t>(n * kRowBytes),
          rows.begin() + static_cast<std::ptrdiff_t>((n + 1) * kRowBytes));
    }
    it = later_->rows.find(k);
  }
  return open(it->second.data(), level, need);
}

size_t CamField::later_places() const {
  if (!later_) return 0;
  std::lock_guard<std::mutex> hold(later_->lock);
  return later_->rows.size();
}

CamField cam_field(const tww_engine::RoomDzb& dzb, const CamRoom& room,
                   const std::vector<uint64_t>& places, const std::string& cache, int threads,
                   CamField::Built* built, const std::function<bool()>* stop) {
  const std::chrono::steady_clock::time_point began = std::chrono::steady_clock::now();
  CamField::Built local;
  CamField::Built& b = built != nullptr ? *built : local;
  b = CamField::Built();
  b.places = places.size();
  CamField field;
  if (room.level.size() < 2) {
    b.why = "the room's cameras were not read";
    return field;
  }
  const Head want = Head::of(room);
  std::unordered_map<uint64_t, std::vector<uint8_t>> held = read_cache(cache, want);

  std::vector<uint64_t> keys;
  std::vector<uint8_t> rings;
  std::map<std::pair<long long, long long>, std::vector<uint64_t>> tiles;
  for (uint64_t k : places) {
    std::unordered_map<uint64_t, std::vector<uint8_t>>::const_iterator it = held.find(k);
    if (it != held.end()) {
      keys.push_back(k);
      rings.insert(rings.end(), it->second.begin(), it->second.end());
      ++b.from_disc;
    } else {
      tiles[CamField::Builder::tile_of(k)].push_back(k);
    }
  }

  std::shared_ptr<CamField::Builder> builder = std::make_shared<CamField::Builder>(dzb, room);
  if (!tiles.empty()) {
    std::vector<const std::vector<uint64_t>*> work;
    for (const auto& t : tiles) work.push_back(&t.second);
    std::vector<std::vector<uint8_t>> done(work.size());
    std::atomic<size_t> next(0);
    std::atomic<bool> stopped(false);
    std::mutex asking;
    auto run = [&]() {
      for (;;) {
        if (stop != nullptr) {
          std::lock_guard<std::mutex> hold(asking);
          if (stopped || (*stop)()) {
            stopped = true;
            return;
          }
        }
        const size_t n = next.fetch_add(1);
        if (n >= work.size()) return;
        done[n].resize(work[n]->size() * CamField::kRowBytes);
        builder->tile(*work[n], done[n].data());
      }
    };
    const int n = std::max(1, threads);
    std::vector<std::thread> pool;
    for (int t = 1; t < n; ++t) pool.push_back(std::thread(run));
    run();
    for (std::thread& t : pool) t.join();
    /* Tiles a stop cut short are left to `later`. */
    for (size_t w = 0; w < work.size(); ++w) {
      if (done[w].empty()) continue;
      for (size_t i = 0; i < work[w]->size(); ++i) {
        const uint64_t k = (*work[w])[i];
        keys.push_back(k);
        const std::vector<uint8_t> row(
            done[w].begin() + static_cast<std::ptrdiff_t>(i * CamField::kRowBytes),
            done[w].begin() + static_cast<std::ptrdiff_t>((i + 1) * CamField::kRowBytes));
        rings.insert(rings.end(), row.begin(), row.end());
        held[k] = row;
        ++b.computed;
      }
    }
    if (b.computed > 0) b.saved = write_cache(cache, want, held);
  }

  field.fill(keys, rings, room);
  field.room_ = room;
  field.builder_ = builder;
  field.later_ = std::make_shared<CamField::Later>();
  b.seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
  return field;
}

bool cam_keep(const CamField& field, const std::string& cache) {
  if (field.levels_ == 0 || !field.later_ || field.later_places() == 0) return false;
  std::unordered_map<uint64_t, std::vector<uint8_t>> rows;
  for (size_t i = 0; i < field.key_.size(); ++i) {
    if (field.key_[i] == 0) continue;
    rows[field.key_[i]] = std::vector<uint8_t>(
        field.ring_.begin() + static_cast<std::ptrdiff_t>(i * CamField::kRowBytes),
        field.ring_.begin() + static_cast<std::ptrdiff_t>((i + 1) * CamField::kRowBytes));
  }
  {
    std::lock_guard<std::mutex> hold(field.later_->lock);
    for (const auto& r : field.later_->rows) rows.insert(r);
  }
  return write_cache(cache, Head::of(field.room_), rows);
}

}  // namespace search
