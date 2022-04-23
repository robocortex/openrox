//==============================================================================
//
//    OPENROX   : File avx_interaction_row_texture_matse3_model3d_zi.h
//
//    Contents  : API of avx_interaction_row_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_AVX_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__
#define __OPENROX_AVX_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__

#include <system/vectorisation/avx.h>
#include <system/memory/datatypes.h>

ROX_API Rox_ErrorCode rox_avx_interaction_row_texture_matse3_model3d_zi (
   __m256 * avx_Lk,
   const __m256 avx_ur,
   const __m256 avx_vr,
   const __m256 avx_Iu,
   const __m256 avx_Iv,
   const __m256 avx_Zi,
   const __m256 avx_Ziu,
   const __m256 avx_Ziv,
   const __m256 avx_fu,
   const __m256 avx_fv,
   const __m256 avx_cu,
   const __m256 avx_cv,
   const __m256 avx_tau1,
   const __m256 avx_tau2,
   const __m256 avx_tau3
);

#endif // __OPENROX_AVX_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__
