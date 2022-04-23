//==============================================================================
//
//    OPENROX   : File imask.h
//
//    Contents  : API of imask module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMASK__
#define __OPENROX_IMASK__

#include <generated/array2d_uint.h>
#include <baseproc/image/image.h>
#include <baseproc/geometry/rectangle/rectangle.h>
#include <baseproc/geometry/ellipse/ellipse2d.h>

//! \ingroup  Vision
//! \defgroup Mask Image Mask
//! \brief Image mask structures and methods.

//! \addtogroup Mask
//!   @{

//! Define the Rox_Imask object
typedef struct _Rox_Array2D_Uint * Rox_Imask;

//! Create a mask
//! \param  [out]  mask           The object to create
//! \param  [in ]  cols           The mask width in pixels
//! \param  [in ]  rows           The mask height in pixels
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imask_new ( Rox_Imask * mask, const Rox_Sint cols, const Rox_Sint rows );

//! Delete a mask
//! \param  [out]  mask           The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imask_del ( Rox_Imask * mask );

//! Read a PGM file and fill the Rox_Imask object.
//! \param  [out]  mask           The object to create
//! \param  [in ]  filename       The PGM filename to read
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imask_new_read_pgm ( Rox_Imask *mask, const char *filename );

//! Read a PGM file
//! \param  [out]  mask           The allocated mask to fill
//! \param  [in ]  filename       The PGM filename to read
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imask_read_pgm ( Rox_Imask mask, const char *filename );

//! Save mask to a PGM file
//! \param  [out]  filename       The PGM filename
//! \param  [in ]  mask           The mask to save
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imask_save_pgm (const char *filename, Rox_Imask mask );

//! Get the mask width in pixels
//! \param  [out]  cols           The mask width in pixels
//! \param  [in ]  mask           The mask object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imask_get_cols ( Rox_Sint * cols, const Rox_Imask mask );

//! Get the mask height in pixels
//! \param  [out]  rows           The mask height in pixels
//! \param  [in ]  mask           The mask object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imask_get_rows ( Rox_Sint * rows, const Rox_Imask mask );

//! Get the mask size in pixels
//! \param  [out]  rows           The mask height in pixels
//! \param  [out]  cols           The mask width in pixels
//! \param  [in ]  mask           The mask object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imask_get_size ( Rox_Sint * rows, Rox_Sint * cols, const Rox_Imask mask );

//! Set the mask from an external buffer
//! \param  [out]  mask           The mask to set
//! \param  [in ]  data           The mask buffer
//! \param  [in ]  bytesPerRow The octet size of one mask row
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imask_set_data ( Rox_Imask mask, Rox_Uint * data, const Rox_Sint bytesPerRow );

//! Set each element of the mask to zero
//! \param  [in ]  mask The mask to set
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imask_set_zero ( Rox_Imask mask );

//! Set each element of the mask to one
//! \param  [in ]  mask           The mask to set
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imask_set_ones ( Rox_Imask mask );

//! Set each element of the mask to the opposite value
//! \param  [in ]  mask           The mask to set
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imask_set_not ( Rox_Imask mask );

//! Set a mask as the result of a binary AND between two masks
//! \param  [out]  result         The result mask
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imask_set_and ( Rox_Imask result, Rox_Imask input_1, Rox_Imask input_2 );

//! Set the border of the mask
//! \param  [out] mask The mask to set
//! \param  [in] size The border size in pixels
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imask_set_border ( Rox_Imask mask, Rox_Sint size );


//! Set the mask by thresholding an image : set pixels below threshold to 1, above threshold to 0 
//! \param  [out] imask       The mask to set
//! \param  [in ] image       The image
//! \param  [in ] threshold   The graylevel threshold [0 ... 255]
//! \return An error code
//! \todo   to be tested

ROX_API Rox_ErrorCode rox_imask_set_from_image ( Rox_Imask imask, const Rox_Image image, const Rox_Sint grays_threshold );

//! Create a new mask with elliptic shape
//! \param  [out]  mask           The mask to be created
//! \param  [in ]  ellipse2d      The 2D ellipse
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imask_new_ellipse ( Rox_Imask * mask, const Rox_Ellipse2D ellipse2d );

ROX_API Rox_ErrorCode rox_imask_new_polygon ( Rox_Imask * mask, const Rox_Point2D_Double points_list, Rox_Sint const nb_points );

//! Set the mask with elliptic shape
//! \param  [in] mask The mask to set
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imask_set_centered_ellipse ( Rox_Imask mask );

//! Set a circle centered mask
//! \param [in] mask:       pointer to the mask object to modify
//! \param [in] radius:     radius of the circle in meters
//! \param [in] mask_sizex: size of X dimension of the mask(ed object) in meters
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_imask_set_centered_circle ( Rox_Imask mask, Rox_Double radius, Rox_Double mask_sizex );

//! Get the pointer to the pointer to the data
//! \param  [out]  data
//! \param  [in ]  mask
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imask_get_data_pointer_to_pointer ( Rox_Uint *** data, Rox_Imask mask );

//! Check if a mask is empty : all elemts are equal to 0
//! \param  [out] data
//! \param  [in ] mask
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imask_count_valid ( Rox_Sint * is_empty, Rox_Imask mask );

//! Copy a region of interest (roi) of an image in another 
//! \param  [out]  dest           The destination imask
//! \param  [in ]  roi            The region of interest (roi) 
//! \param  [in ]  source         The source imask
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imask_new_copy_roi ( Rox_Imask * dest, const Rox_Rect_Sint roi, const Rox_Imask source );

ROX_API Rox_ErrorCode rox_imask_get_roi ( Rox_Rect_Sint_Struct * roi, const Rox_Imask imask );

ROX_API Rox_ErrorCode rox_imask_check_size ( const Rox_Imask imask, const Rox_Sint rows, const Rox_Sint cols );


//! @}

#endif // __OPENROX_IMASK__
