//==============================================================================
//
//    OPENROX   : File ppmfile.h
//
//    Contents  : API of ppmfile module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PPMFILE__
#define __OPENROX_PPMFILE__

#include <baseproc/image/image_rgba.h>
#include <stdio.h>

//! \ingroup InOut
//! \defgroup File File

//! \ingroup File
//! \addtogroup PPM
//! \brief Tools to read/ save ppm files
//! @{

//! Create a new color image and fills values using a binary PPM image file
//! \param  [out]  out         The created image
//! \param  [in ]  path        The path to image file
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_new_read_ppm(Rox_Image_RGBA * out, const Rox_Char * path);

//! Load color image from ppm file
//! \param  [out]  in             The image to load
//! \param  [in ]  path           The path to image file
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_read_ppm(Rox_Image_RGBA in, const Rox_Char * path);

//! Save color image to ppm file
//! \param  [out]  path           path to image file
//! \param  [in ]  in             image to save
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_save_ppm ( const Rox_Char * path, Rox_Image_RGBA in );

//! Save color image to ppm file without any additional allocation (slower)
//! \param  [out] path            path to image file
//! \param  [in ]  in             image to save
//! \return An error code
ROX_API Rox_ErrorCode rox_image_rgba_save_noalloc_ppm(const Rox_Char * path, Rox_Image_RGBA in);

//! Read the comments of the opened file
//! \param  [in ]  input          The opened file
//! \return An error code
ROX_API Rox_ErrorCode rox_ppm_strip_comments(FILE * input);

//! Read the header of a ppm file
//! \param  [in ]  input          The opened file
//! \param  [out]  rowsp          A pointer to return the image height
//! \param  [out]  colsp          A pointer ro return the image width
//! \param  [out]  maxp           A pointer of return the maximum grayscale
//! \return An error code
ROX_API Rox_ErrorCode rox_ppm_read_header(FILE * input, Rox_Sint * rowsp, Rox_Sint * colsp, Rox_Sint * maxp);

//! Save the header of a ppm file
//! \param  [out]  input       The opened file
//! \param  [out]  rows        The image height
//! \param  [out]  cols        The image width
//! \return An error code
ROX_API Rox_ErrorCode rox_ppm_save_header(FILE * input, Rox_Sint rows, Rox_Sint cols);

//! Read a ppm file
//! \param  [out]  out         The buffer to store the read values
//! \param  [in ]  in          The file to be read
//! \param  [in ]  cols        The image width
//! \param  [in ]  rows        The image height
//! \return An error code
ROX_API Rox_ErrorCode rox_ppm_read_content(Rox_Uchar * out, FILE * in, Rox_Sint cols, Rox_Sint rows);

//! Write a ppm file from a single contigous memory block describing RGB channels.
//! \param  [out]  out         The opened file
//! \param  [in ]  in          The buffer to write
//! \param  [in ]  cols        The image width
//! \param  [in ]  rows        The image height
//! \return An error code
ROX_API Rox_ErrorCode rox_ppm_write_content(FILE * out, Rox_Uchar *in, Rox_Sint cols, Rox_Sint rows);

//! Write a ppm file extracting RGB values from a single contigous memory block describing RGBA channels.
//! \param  [out]  out         The opened file
//! \param  [in ]  in          The buffer to write
//! \param  [in ]  cols        The image width
//! \param  [in ]  rows        The image height
//! \return An error code
ROX_API Rox_ErrorCode rox_ppm_write_content_rgba(FILE * out, Rox_Uint *in, Rox_Sint cols, Rox_Sint rows);

//! @} 

#endif
