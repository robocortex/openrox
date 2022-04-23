//==============================================================================
//
//    OPENROX   : File ansi_array2d_print.h
//
//    Contents  : API of ansi array2d print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_ARRAY2D_PRINT__
#define __OPENROX_ANSI_ARRAY2D_PRINT__

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>

//! \addtogroup Array2D
//! @{

ROX_API int rox_ansi_array2d_float_print ( float ** input_data, int rows, int cols );

//! @} 

#endif
