//==============================================================================
//
//    OPENROX   : File detection_sl3_struct.h
//
//    Contents  : API of detection checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DETECTION_SL3_STRUCT__
#define __OPENROX_DETECTION_SL3_STRUCT__

#include <baseproc/geometry/point/points_struct.h>
#include <generated/array2d_double.h>
#include <baseproc/image/image.h>

//! \ingroup Vision
//! \addtogroup Bundle
//! @{

//! Detection Checkerboard structure
struct Rox_Detection_Sl3_Struct
{

   //! Pointer to the detected grid
   Rox_Array2D_Double _estimated_homography;

   //! Pointer to the detected grid
   Rox_Bool            _detected;

   //! Pointer to the detected grid
   int                 _image_width;

   //! Pointer to the detected grid
   int                 _image_height;

   //! Pointer to the detected grid
   Rox_Image   _image;
   
   //! Id of image
   Rox_Uint _id;
};

//! @}

#endif
