//==============================================================================
//
//    OPENROX   : File sse_interaction_row_texture_matse3_model3d_zi.h
//
//    Contents  : API of sse_interaction_row_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SSE_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__
#define __OPENROX_SSE_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__

#include <system/vectorisation/sse.h>
#include <system/memory/datatypes.h>

ROX_API Rox_ErrorCode rox_sse_interaction_row_texture_matse3_model3d_zi (
   __m128 * sse_Lk,
   const __m128 sse_ur,
   const __m128 sse_vr,
   const __m128 sse_Iu,
   const __m128 sse_Iv,
   const __m128 sse_zi,
   const __m128 sse_ziu,
   const __m128 sse_ziv,
   const __m128 sse_fu,
   const __m128 sse_fv,
   const __m128 sse_cu,
   const __m128 sse_cv,
   const __m128 sse_tau1,
   const __m128 sse_tau2,
   const __m128 sse_tau3
);

#endif // __OPENROX_SSE_INTERACTION_ROW_TEXTURE_MATSE3_MODEL3D_ZI__
