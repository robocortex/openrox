//==============================================================================
//
//    OPENROX   : File image.h
//
//    Contents  : API of image module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE__
#define __OPENROX_IMAGE__

#include <generated/array2d_uchar.h>

//!  \ingroup Vision
//!  \addtogroup Image
//!  @{

//!  Define the different image format available 
//! @warning Please update plugins documentation/comments if this enum changes (ATM image_plugin_interface.hpp & rox_plugin_prediction_so3.h)
enum Rox_Image_Format
{
     Rox_Image_Format_Grays = 0,
     Rox_Image_Format_YUV422,
     Rox_Image_Format_RGBA,
     Rox_Image_Format_BGRA,
     Rox_Image_Format_ARGB,
     Rox_Image_Format_BGR,
     Rox_Image_Format_RGB,
     Rox_Image_Format_RGBA_FlippedUpsideDown,
     Rox_Image_Format_Alpha8_32bits,                     // Alpha8 is 1 byte per pixel but stored in the alpha value of a rgba buffer (32bits)
     Rox_Image_Format_Alpha8_32bits_FlippedUpsideDown    // Alpha8 is 1 byte per pixel but stored in the alpha value of a rgba buffer (32bits)
};

//! Define the Rox_Image object : Image grays in [0, 255]
typedef struct _Rox_Array2D_Uchar * Rox_Image;

//! Define the Rox_Image_Float object : Image grays in [0, 1]
typedef struct _Rox_Array2D_Float * Rox_Image_Float;

//! Create a grayscale image
//! \param  [in ]  image          The object to create
//! \param  [in ]  cols           The image width in pixels
//! \param  [in ]  rows           The image height in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_new ( Rox_Image * image, const Rox_Sint cols, const Rox_Sint rows );

//! Delete a grayscale image
//! \param  [in ]  image          The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_del ( Rox_Image *image );

//! Read a PGM file and fill the Rox_Image object.
//! \param  [in ]  image          The object to create
//! \param  [in ]  filename       The PGM filename to read
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_new_read_pgm ( Rox_Image * image, const Rox_Char * filename );

//! Read a PGM file
//! \param  [in ]  image          The allocated image to fill
//! \param  [in ]  filename       The PGM filename to read
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_read_pgm ( Rox_Image image, const Rox_Char * filename );

//! Read a PPM file and fill the Rox_Image object.
//! \param  [in ]  image          The object to create
//! \param  [in ]  filename       The PPM filename to read
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_new_read_ppm ( Rox_Image * image, const Rox_Char *filename );

//! Read a PPM file
//! \param  [in ]  image          The allocated image to fill
//! \param  [in ]  filename       The PPM filename to read
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_read_ppm ( Rox_Image image, const Rox_Char *filename );

//! Get the image width in pixels
//! \param  [out]  cols           The image width in pixels
//! \param  [in ]  image          The image object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_get_cols ( Rox_Sint * cols, const Rox_Image image );

//! Get the image height in pixels
//! \param  [out]  rows           The image height in pixels
//! \param  [in ]  image          The image object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_get_rows ( Rox_Sint * rows, const Rox_Image image );

//! Get the image height in pixels
//! \param  [out]  rows           The image height in pixels
//! \param  [out]  cols           The image height in pixels
//! \param  [in ]  image          The image object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_get_size ( 
  Rox_Sint * rows, 
  Rox_Sint * cols, 
  const Rox_Image image 
);

ROX_API Rox_ErrorCode rox_image_match_size ( 
     const Rox_Image image_1, 
     const Rox_Image image_2 
);

//! Check image size in pixels
//! \param  [in ]  image          The image object
//! \param  [out]  rows           The image height in pixels
//! \param  [out]  cols           The image height in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_check_size ( 
   const Rox_Image image, 
   const Rox_Sint rows, 
   const Rox_Sint cols 
);


//! Get the image bytes per row
//! \param  [out]  bytesPerRow     The image bytesperrow (also called stride)
//! \param  [in ]  image           The image object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_get_bytesperrow ( 
   Rox_Sint * bytesPerRow, 
   const Rox_Image image
);

//! Get the image row pointers
//! \param  [out]  rowsptr        An array of pointers to rows
//! \param  [in ]  image          The image object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_get_data_pointer_to_pointer (
     Rox_Uchar *** rowsptr, 
     const Rox_Image image
);

//! Set the image from an external pixel buffer
//! \param  [out]  image          The image to set
//! \param  [in ]  data           The pixel buffer
//! \param  [in ]  bytesPerRow    The octet size of one image row (exemple for RGBA image bytesPerRow = 4*cols, for a GRAYS image bytesPerRow = cols)
//! \param  [in ]  format         The pixel format of the input buffer
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_set_data ( 
     Rox_Image image, 
     const Rox_Uchar * data, 
     const Rox_Sint bytesPerRow, 
     const enum Rox_Image_Format format
    );

//! Get the image data
//! \param  [out]  data           The pixel buffer
//! \param  [in ]  image          The image to copy
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_get_data ( Rox_Uchar * data, const Rox_Image image );

//! Copy an image in another (Both must be valid images with same size)
//! \param  [out]  dest           The destination image
//! \param  [in ]  source         The source image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_copy ( Rox_Image dest, const Rox_Image source );

//! Save image to a PGM file
//! \param  [in ]  filename       The PGM filename
//! \param  [in ]  image          The image to save
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_save_pgm ( const Rox_Char * filename, const Rox_Image image );

//! Flip the image vertically (inverse the order of lines)
//! \param  [out]  output         The flipped image
//! \param  [in ]  input          The matrix to flip
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_copy_flip ( Rox_Image output, Rox_Image input);

//! Flip the image vertically (inverse the order of lines)
//! \param  [in, out] inout       The flipped image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_flip ( Rox_Image inout );

//! Create a new image with the input image centered in it
//! \param  [out]  image_out      The output centered square image
//! \param  [in ]  image_inp      The input image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_new_square_centered  ( Rox_Image * image_out, const Rox_Image image_inp );

ROX_API Rox_ErrorCode rox_image_convert_uchar_to_uchar_nostride ( unsigned char * data_uchar, Rox_Image image );

ROX_API Rox_ErrorCode rox_image_convert_uchar_to_float_nostride ( float * data_float, Rox_Image uchar );

//! @}

#endif // __OPENROX_IMAGE__
