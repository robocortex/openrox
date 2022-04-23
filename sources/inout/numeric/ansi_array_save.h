//==============================================================================
//
//    OPENROX   : File ansi_array_save.h
//
//    Contents  : API of ansi array save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_ARRAY_SAVE__
#define __OPENROX_ANSI_ARRAY_SAVE__

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>
#include <stdio.h>

//! \addtogroup Array2D
//! @{

ROX_API int rox_ansi_array_float_fscan ( float * data, int size, FILE * file );
ROX_API int rox_ansi_array_uint_fscan ( unsigned int  * data, int size, FILE * file );
ROX_API int rox_ansi_array_float_save ( const char * filename, float * input_data, int size );
ROX_API int rox_ansi_array_uint_save ( const char *filename, unsigned int * input_data, int size );
ROX_API int rox_ansi_array_float_fprint ( FILE *file, float * input_data, int size );


//! @} 

#endif
