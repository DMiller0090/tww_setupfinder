/* `cBgD_t`, the DZB header (`c_bg_w.h`): six counts and six tables. On disc the tables are file
 * offsets; in RAM `cBgS::ConvDzb` has made them pointers, and nothing else differs.
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "mesh.h"

namespace geom {

/** Byte offsets into `cBgD_t`; every field is four bytes, big-endian. */
namespace bgd {

constexpr uint32_t kVertCount = 0x00;  // s32 m_v_num
constexpr uint32_t kVertTable = 0x04;  // cBgD_Vtx_t* m_v_tbl
constexpr uint32_t kTriCount = 0x08;   // s32 m_t_num
constexpr uint32_t kTriTable = 0x0C;   // cBgD_Tri_t* m_t_tbl
constexpr uint32_t kBlkCount = 0x10;   // s32 m_b_num
constexpr uint32_t kBlkTable = 0x14;   // cBgD_Blk_t* m_b_tbl
constexpr uint32_t kTreeCount = 0x18;  // s32 m_tree_num
constexpr uint32_t kTreeTable = 0x1C;  // cBgD_Tree_t* m_tree_tbl
constexpr uint32_t kGrpCount = 0x20;   // s32 m_g_num
constexpr uint32_t kGrpTable = 0x24;   // cBgD_Grp_t* m_g_tbl
constexpr uint32_t kTiCount = 0x28;    // s32 m_ti_num
constexpr uint32_t kTiTable = 0x2C;    // cBgD_Ti_t* m_ti_tbl

/** Through `flag` at 0x30. */
constexpr uint32_t kHeaderBytes = 0x34;

/** The six tables located, not owned: raw big-endian rows. Whoever fills this guarantees each span
 *  is as long as its count; readers bound by the counts alone. */
struct Tables {
  const uint8_t* v = nullptr;
  const uint8_t* t = nullptr;
  const uint8_t* b = nullptr;
  const uint8_t* tree = nullptr;
  const uint8_t* g = nullptr;
  const uint8_t* ti = nullptr;
  int32_t v_num = 0;
  int32_t t_num = 0;
  int32_t b_num = 0;
  int32_t tree_num = 0;
  int32_t g_num = 0;
  int32_t ti_num = 0;
};

}  // namespace bgd
}  // namespace geom
