/* Collision read out of an attached, paused game. Offsets: `cBgS_ChkElm` (`c_bg_s.h`), `cBgW`
 * (`c_bg_w.h`).
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "../disc/disc.h"
#include "../geom/dzb.h"
#include "../geom/mesh.h"
#include "dolphin.h"

namespace dolphin {

namespace addr {
/** `dBgS`: `cBgS_ChkElm m_chk_element[256]` at its start. */
constexpr uint32_t kBgS = 0x803B93A8;
/** `dStage_roomControl_c::mStatus[64]`. */
constexpr uint32_t kRoomStatus = 0x803B1188;
/** `dComIfG_play_c::mStageData` (play + 0x3EB0, `d/d_com_inf_game.h`). */
constexpr uint32_t kStageData = 0x803BD258;
}  // namespace addr

/** `meshes` is how many of `dBgS`'s 256 slots were taken. */
using Room = geom::Mesh;

/** The room's own mesh (`mStatus[room].mpBgW`: 0x114 apart, pointer at +0x110, `d/d_stage.h`) plus
 *  everything that moves. Other stage meshes are left out: on `sea` the ocean floor dwarfs the room. */
Room room_of(const Mem& mem, int room);

/** The room's own `cBgD_t`, all six tables plus the mesh decoded from the same read. Moving
 *  collision is not included, matching a disc read. */
struct Dzb {
  bool ok = false;
  std::string why;
  /** Big-endian, as the process holds them. */
  std::vector<uint8_t> v, t, b, tree, g, ti;
  int32_t v_num = 0, t_num = 0, b_num = 0, tree_num = 0, g_num = 0, ti_num = 0;
  geom::Mesh mesh;
  /** Spans into this object, so built on demand rather than stored. */
  geom::bgd::Tables located() const;
};

Dzb dzb_of(const Mem& mem, int room);

/** `dzb_of` with the reads injected, so a test can supply its own MEM1. */
using Read = std::function<bool(uint32_t gc_addr, void* dst, size_t n)>;
Dzb dzb_at(const Read& read, uint32_t bgw);

/** `disc::camera_names_of` from RAM: stage `mpCamera` (+0x04), `mpStagInfo` (+0x48,
 *  `mCameraMapToolID` at +0x08), room `mStatus[room].mRoomDt.mpCamera` (+0x2C). A null list is
 *  empty; false only for a pointer outside MEM1 or an implausible count. */
bool camera_names_at(const Read& read, int room, disc::CameraNames* out);
bool camera_names_of(const Mem& mem, int room, disc::CameraNames* out);

}  // namespace dolphin
