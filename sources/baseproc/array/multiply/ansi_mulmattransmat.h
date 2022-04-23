//==============================================================================
//
//    OPENROX   : File ansi_mulmattransmat.h
//
//    Contents  : API of ansi mulmattransmat module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ansi_mulmattransmat__
#define __OPENROX_ansi_mulmattransmat__

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>

//! \ingroup Matrix
//! \addtogroup Transpose
//! @{

ROX_API int rox_ansi_array_double_mulmattransmat ( double * res_data, int res_rows, int res_cols, double * one_data, double * two_data, int two_rows );
ROX_API int rox_ansi_array_float_mulmattransmat ( float * res_data, int res_rows, int res_cols, float * one_data, float * two_data, int two_rows );

ROX_API int rox_ansi_array2d_double_mulmattransmat ( double ** res_data, int res_rows, int res_cols, double ** one_data, double ** two_data, int two_rows );

//! @} 

#endif
