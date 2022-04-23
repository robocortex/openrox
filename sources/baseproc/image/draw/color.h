//============================================================================
//
//    OPENROX   : File color.h
//
//    Contents  : API of color module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_COLOR__
#define __OPENROX_COLOR__

#include <system/errors/errors.h>
#include <system/memory/datatypes.h>

//! \ingroup Color
//! \addtogroup Color
//! @{

//! R,G,B,A belongs to the range [0,255] 
#define ROX_MAKERGBA(R,G,B,A) (R + (G << 8) + (B << 16) + (A << 24))

ROX_API Rox_ErrorCode rox_color_get_uchar_r_g_b_a_from_uint_rgba (
   Rox_Uchar * r, 
   Rox_Uchar * g, 
   Rox_Uchar * b, 
   Rox_Uchar * a, 
   Rox_Uint rgba
);
//! @} 

#endif // __OPENROX_COLOR__
