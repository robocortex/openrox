//==============================================================================
//
//    OPENROX   : File ansi_scale.h
//
//    Contents  : API of ansi scale module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ansi_scale__
#define __OPENROX_ansi_scale__

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>

//! \ingroup Matrix
//! \addtogroup Scale
//! @{

//! Scale all elements of an array float : inpuout = scale*inpuout
ROX_API int rox_ansi_array_float_scale_inplace ( float * inpout, const int size, const float scale );

//! @} 

#endif // __OPENROX_ansi_scale__
