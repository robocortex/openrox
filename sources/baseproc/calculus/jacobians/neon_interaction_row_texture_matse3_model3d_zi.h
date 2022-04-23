//==============================================================================
//
//    OPENROX   : File neon_interaction_row_texture_matse3_model3d_zi.h
//
//    Contents  : API of neon_interaction_row_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_NEON_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__
#define __OPENROX_NEON_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__

#include <system/vectorisation/neon.h>

int rox_neon_interaction_row_texture_matse3_model3d_zi (
   float32x4_t * neon_Lk,
   const float32x4_t neon_ur,
   const float32x4_t neon_vr,
   const float32x4_t neon_Iu,
   const float32x4_t neon_Iv,
   const float32x4_t neon_zi,
   const float32x4_t neon_ziu,
   const float32x4_t neon_ziv,
   const float32x4_t neon_fu,
   const float32x4_t neon_fv,
   const float32x4_t neon_cu,
   const float32x4_t neon_cv,
   const float32x4_t neon_tau1,
   const float32x4_t neon_tau2,
   const float32x4_t neon_tau3
);

#endif // __OPENROX_NEON_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__
