//==============================================================================
//
//    OPENROX   : File array2d_save.h
//
//    Contents  : API of array2d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ARRAY2D_SAVE__
#define __OPENROX_ARRAY2D_SAVE__

#include <stdio.h>

#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_float.h>
#include <generated/array2d_double.h>

//! \addtogroup Array2D
//! @{

//! Save a 2D array on ascii file
//! \param  [in ]  filename        the file name
//! \param  [in ]  input           the array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_save ( const Rox_Char * filename, const Rox_Array2D_Float input );

//! Save a 2D array on ascii file
//! \param  [in] filename        the file name
//! \param  [in] input           the array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_save (const Rox_Char * filename, const Rox_Array2D_Double input);

//! Append a 2D array on ascii file
//! \param  [in] filename        the file name
//! \param  [in] input           the array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_save_append (const Rox_Char * filename, const Rox_Array2D_Double input);

//! Save a 2D array on ascii file
//! \param  [in ]  filename       The output file
//! \param  [in ]  input          The array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_save ( const Rox_Char * filename, const Rox_Array2D_Uint input);

//! Save a 2D array on ascii file
//! \param  [in] filename        the output file
//! \param  [in] input           the array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_save ( const Rox_Char *filename, const Rox_Array2D_Uchar input);

//! Save a 2D array on file
//! \param [in] file             the output file
//! \param [in] input            the array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_fprint(FILE* file, const Rox_Array2D_Double input);

//! Save a 2D array on file
//! \param [in] file             the output file
//! \param [in] input            the array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_fprint(FILE* file, const Rox_Array2D_Float input);

//! Save a 2D array on file
//! \param  [in] file            the output file
//! \param  [in] input           the array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_fprint(FILE *file, const Rox_Array2D_Uint input);

//! Save a 2D array on file
//! \param  [in] file            the output file
//! \param  [in] input           the array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_fprint(FILE *file, const Rox_Array2D_Uchar input);

//! Display a 2D array on file
//! \param [in] input            the array to save
//! \param [in] file             the output file
//! \param [in] precision        number of digits after the dot
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_fprint_precision(Rox_Array2D_Double input, FILE * file, Rox_Uint precision);

//! Display a pretty 2D array on file
//! \param [in] input            the array to save
//! \param [in] file             the output file
//! \param [in] precision        number of digits after the dot
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_save_pretty ( const Rox_Char * filename, Rox_Array2D_Double input);

//! Read a 2D array from ascii file
//! \param  [out]  input           the array to read
//! \param  [in ]  filename        the input file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_read ( const Rox_Array2D_Uint input, const Rox_Char * filename );

//! Read a 2D array from ascii file
//! \param  [out]  input           the array to read
//! \param  [in ]  filename        the input file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_read ( const Rox_Array2D_Float input, const Rox_Char * filename );

//! Read a 2D array from ascii file
//! \param  [out]  input           the array to read
//! \param  [in ]  filename        the input file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_read ( const Rox_Array2D_Double input, const Rox_Char * filename );

//! @} 

#endif
