//============================================================================
//
//    OPENROX   : File image_rgba.h
//
//    Contents  : API of image_rgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_IMAGE_RGBA__
#define __OPENROX_IMAGE_RGBA__

#include <baseproc/image/image.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>

#include <generated/dynvec_rect_sint.h>

//! \ingroup Vision
//! \addtogroup Image_Display
//! @{

//! Define the Rox_Image_RGBA object
typedef struct _Rox_Array2D_Uint * Rox_Image_RGBA;

//! Create a color image
//! \param  [in]  image           The object to create
//! \param  [in]  cols            The image width in pixels
//! \param  [in]  rows            The iamge height in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_new ( Rox_Image_RGBA * image, Rox_Sint cols, Rox_Sint rows );

//! Delete a color image
//! \param [in]   image          The object to delete
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_image_rgba_del ( Rox_Image_RGBA *image );

//! \brief Read a PGM file and fill the Rox_Image_RGBA object.
//! \param [in]   image          The object to create
//! \param [in]   filename       The PGM filename to read
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_image_rgba_new_read_pgm(Rox_Image_RGBA * image, const char * filename);

//! Read a PGM file
//! \param [in]   image          The allocated image to fill
//! \param [in]   filename       The PGM filename to read
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_image_rgba_read_pgm(Rox_Image_RGBA image, const char * filename);

//! Save color image to a PGM file
//! \param [out]  filename       The PGM filename
//! \param [in]   image          The color image to save
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_image_rgba_save_pgm(const char * filename, Rox_Image_RGBA image);

//! Copy
//! \param [out]  image_rgba_out       The output image
//! \param [in ]  image_rgba_inp       The input image
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_image_rgba_copy(Rox_Image_RGBA image_rgba_out, Rox_Image_RGBA image_rgba_inp);

//! Get the image width in pixels
//! \param [out]  cols           The image width in pixels
//! \param [in]   image          The image object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_get_cols(Rox_Sint * cols, Rox_Image_RGBA image);

//! Get the image height in pixels
//! \param [out]  rows           The image height in pixels
//! \param [in]   image          The image object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_get_rows(Rox_Sint * rows, Rox_Image_RGBA image);

//! Get the image height in pixels
//! \param  [out]  rows           The image height in pixels
//! \param  [out]  cols           The image width in pixels
//! \param  [in ]  image          The image object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_get_size(Rox_Sint * rows, Rox_Sint * cols, Rox_Image_RGBA image);

//! Get the image bytes per row
//! \param  [out]  bytesPerRow    The image bytesperrow
//! \param  [in ]  image          The image object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_get_bytesperrow ( Rox_Sint * bytesPerRow, Rox_Image_RGBA image );

//! Get the image row pointers
//! \param  [out]  rowsptr        An array of pointers to rows
//! \param  [in ]  image          The image object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_get_data_pointer_to_pointer (
   Rox_Uint *** rowsptr, 
   const Rox_Image_RGBA image
);

//! Get the pointer to the image data buffer
//! \param  [out]  rowsptr        The pointer to the pointer to the data
//! \param  [in ]  image          The color image to save
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_get_data_pointer ( 
   Rox_Uint ** rowsptr, 
   const Rox_Image_RGBA image
);

//! Set the image from an external pixel buffer
//! \param [out]  image          The image to set
//! \param [in]   data           The pixel buffer
//! \param [in]   bytesPerRow    The octet size of one image row
//! \param [in]   format         The pixel format of the input buffer
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_set_data(Rox_Image_RGBA image, Rox_Uchar *data, Rox_Sint bytesPerRow, enum Rox_Image_Format format);

//! Flip the image vertically (inverse the order of lines)
//! \param [out]  output         The flipped image
//! \param [in]   input          The matrix to flip
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_flip(Rox_Image_RGBA image_inout);

ROX_API Rox_ErrorCode rox_image_rgba_inlay(Rox_Image_RGBA image_out, Rox_Image_RGBA image_inp, Rox_Sint u, Rox_Sint v);

ROX_API Rox_ErrorCode rox_image_rgba_convert_image(Rox_Image_RGBA image_rgba_out, Rox_Image image_gray );

ROX_API Rox_ErrorCode rox_image_rgba_new_convert_image(Rox_Image_RGBA * image_rgba_out, Rox_Image image_gray );

ROX_API Rox_ErrorCode rox_image_convert_uint_to_uint_nostride ( unsigned int * data_int, Rox_Image_RGBA image_rgba );

//! @}

#endif // __OPENROX_IMAGE_DISPLAY__
