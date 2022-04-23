//============================================================================
//
//    OPENROX   : File pgmfile.h
//
//    Contents  : API of pgmfile module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_PGMFILE__
#define __OPENROX_PGMFILE__

#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_float.h>
#include <stdio.h>


//! \ingroup File
//! \addtogroup PGM
//! \brief Tools to read / save pgm files
//! @{

//! Create a new image and fills values using a binary PGM image file
//! \param  [out]  out            The created image
//! \param  [in ]  path           Path to image file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_new_pgm(Rox_Array2D_Uchar * out, const char * path);

//! Create a new image and fills values using a binary PGM image file
//! \param  [out]  out           The created image
//! \param  [in ]  path          Path to image file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_rgba_new_pgm(Rox_Array2D_Uint * out, const char * path);

//! Create a new mask and fills values using a binary PGM image file
//! \param  [out]  out            The created mask
//! \param  [in ]  path           Path to mask file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_mask_new_pgm(Rox_Array2D_Uint * out, const char * path);

//! Load image from pgm file
//! \param  [out]  in             Image to load
//! \param  [in ]  path           Path to image file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_read_pgm(Rox_Array2D_Uchar in, const char * path);

//! Load color image from pgm file
//! \param  [out]  in             Image to load
//! \param  [in ]  path           Path to image file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_rgba_read_pgm(Rox_Array2D_Uint in, const char * path);

//! Load mask from pgm file
//! \param  [out]  in             Mask to load
//! \param  [in ]  path           Path to mask file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_mask_read_pgm(Rox_Array2D_Uint in, const char * path);

//! Save image to pgm file
//! \param  [out]  path           Path to image file
//! \param  [in ]  in             Image to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_save_pgm(const char * path, Rox_Array2D_Uchar in);

//! Save color image to pgm file
//! \param  [out]  path           Path to image file
//! \param  [in ]  in             Image to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_rgba_save_pgm(const char * path, Rox_Array2D_Uint in);

//! Save mask to pgm file
//! \param  [out]  path           Path to mask file
//! \param  [in ]  in             Mask to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_mask_save_pgm(const char * path, Rox_Array2D_Uint in);

//! Read the comments of the opened file
//! \param  [in]  input           The opened file
//! \return An error code
ROX_API Rox_ErrorCode rox_pgm_strip_comments(FILE * input);

//! Read the header of a pgm file
//! \param  [in ]  input          The opened file
//! \param  [out]  rowsp          A pointer to return the image height
//! \param  [out]  colsp          A pointer ro return the image width
//! \param  [out]  maxp           A pointer of return the maximum grayscale
//! \return An error code
ROX_API Rox_ErrorCode rox_pgm_read_header(FILE* input, Rox_Sint* rowsp, Rox_Sint * colsp, Rox_Sint * maxp);

//! Save the header of a pgm file
//! \param  [in ]  input          The opened file
//! \param  [out]  rows           The image height
//! \param  [out]  cols           The image width
//! \return An error code
ROX_API Rox_ErrorCode rox_pgm_save_header(FILE* input, Rox_Sint rows, Rox_Sint cols);

//! Read a pgm file
//! \param  [out]  out            The buffer to store the read values
//! \param  [in ]  in             The file to be read
//! \param  [in ]  cols           The image width
//! \param  [in ]  rows           The image height
//! \return An error code
ROX_API Rox_ErrorCode rox_pgm_read_content(Rox_Uchar ** out, FILE * in, const Rox_Sint cols, const Rox_Sint rows);

//! Write a pgm file from a 2d array data
//! \param  [out]  out            The opened file
//! \param  [in ]  in             The buffer to write
//! \param  [in ]  cols           The image width
//! \param  [in ]  rows           The image height
//! \return An error code
ROX_API Rox_ErrorCode rox_pgm_write_content(FILE * out, Rox_Uchar **in, const Rox_Sint cols, const Rox_Sint rows);

//! Write a pgm file from a contiguous data buffer
//! \param  [out]  out            The opened file
//! \param  [in ]  in             The buffer to write
//! \param  [in ]  cols           The image width
//! \param  [in ]  rows           The image height
//! \return An error code
ROX_API Rox_ErrorCode rox_pgm_write_content_contiguous(FILE * out, Rox_Uchar *in, const Rox_Sint cols, const Rox_Sint rows);

//! Read a pgm file into a normalized float image
//! \param [out]  out      The opened file
//! \param [in]   path     The path to image file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_normalize_read_pgm(Rox_Array2D_Float out, const char * path);

//! @} 

#endif
