//==============================================================================
//
//    OPENROX   : File array2d_point2d_float_txtfile.h
//
//    Contents  : API of array2d_point2d_float_txtfile module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ARRAY2D_POINT2D_FLOAT_TXTFILE__
#define __OPENROX_ARRAY2D_POINT2D_FLOAT_TXTFILE__

#include <generated/array2d_point2d_float.h>

//! Save a Rox_Array2D_Point2D_Float object to a text file
//! \param [in] 	filename the text filename
//! \param [in]	lut the object to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_point2d_float_save_txt(const char *filename, Rox_Array2D_Point2D_Float lut);

//! Load a Rox_Array2D_Point2D_Float object from a text file
//! \param [out] 	lut the object to load
//! \param [in]		filename the text filename
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_point2d_float_read_txt(Rox_Array2D_Point2D_Float lut, const char *filename);

//! Create and fill a Rox_Array2D_Point2D_Float object from a text file
//! \param [out]	lut the created object
//! \param [in]		filename the text filename
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_point2d_float_new_txt(Rox_Array2D_Point2D_Float *lut, char *filename);

#endif // __OPENROX_ARRAY2D_POINT2D_FLOAT_TXTFILE__
