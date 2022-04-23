//==============================================================================
//
//    OPENROX   : File array_save.h
//
//    Contents  : API of array print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ARRAY_SAVE__
#define __OPENROX_ARRAY_SAVE__

#include <system/memory/datatypes.h>

//! \addtogroup Array
//! @{

// These are all ansi functions, should be prefixed by ansi

ROX_API Rox_ErrorCode rox_array_uint_save ( const Rox_Char * filename, Rox_Uint * input_data, Rox_Sint input_size );

ROX_API Rox_ErrorCode rox_array_float_save ( const Rox_Char * filename, Rox_Float * input_data, Rox_Sint input_size );

ROX_API Rox_ErrorCode rox_array_double_save ( const Rox_Char * filename, Rox_Double * input_data, Rox_Sint input_size );

ROX_API Rox_ErrorCode rox_array_double_save_append ( const Rox_Char * filename, Rox_Double * input_data, Rox_Sint input_size );

// ---------------------------------------------------------------------------------------------------

ROX_API Rox_ErrorCode rox_array_uchar_save_as_array2d ( const Rox_Char * filename, Rox_Uchar * input_data, Rox_Size input_rows, Rox_Size input_cols );

ROX_API Rox_ErrorCode rox_array_uint_save_as_array2d ( const Rox_Char * filename, Rox_Uint * input_data, Rox_Size input_rows, Rox_Size input_cols );

ROX_API Rox_ErrorCode rox_array_float_save_as_array2d ( const Rox_Char * filename, Rox_Float * input_data, Rox_Size input_rows, Rox_Size input_cols );

ROX_API Rox_ErrorCode rox_array_double_save_as_array2d ( const Rox_Char * filename, Rox_Double * input_data, Rox_Size input_rows, Rox_Size input_cols );

// ---------------------------------------------------------------------------------------------------

ROX_API Rox_ErrorCode rox_array_float_padded_save_as_array2d ( const Rox_Char * filename, Rox_Float * input_data, Rox_Size input_rows, Rox_Size input_cols, Rox_Size input_used );

//! @} 

#endif
