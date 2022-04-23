//==============================================================================
//
//    OPENROX   : File ansi_array2d_save.h
//
//    Contents  : API of ansi array2d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_ARRAY2D_SAVE__
#define __OPENROX_ANSI_ARRAY2D_SAVE__

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>
#include <stdio.h>

//! \addtogroup Array2D
//! @{

ROX_API int rox_ansi_array2d_float_fscan ( float ** data, int rows, int cols, FILE * file );
ROX_API int rox_ansi_array2d_float_padded_fscan ( float ** data, int rows, int cols, int used, FILE * file );
ROX_API int rox_ansi_array2d_float_fscan_transpose ( float ** data, int rows, int cols, FILE * file );
ROX_API int rox_ansi_array2d_uint_fscan ( unsigned int  ** data, int rows, int cols, FILE * file );
ROX_API int rox_ansi_array2d_float_save ( const char * filename, float ** input_data, int rows, int cols );
ROX_API int rox_ansi_array2d_uint_save ( const char *filename, unsigned int ** input_data, int rows, int cols );
ROX_API int rox_ansi_array2d_float_fprint ( FILE *file, float ** input_data, int rows, int cols );


//! @} 

#endif
