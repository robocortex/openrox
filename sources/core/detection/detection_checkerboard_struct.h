//==============================================================================
//
//    OPENROX   : File detection_checkerboard_struct.h
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

#ifndef __OPENROX_DETECTION_CHECKERBOARD_STRUCT__
#define __OPENROX_DETECTION_CHECKERBOARD_STRUCT__

#include <baseproc/geometry/point/point2d.h>
#include <generated/array2d_double.h>
#include <baseproc/image/image.h>
//#include <system/memory/datatypes.h>

//!   \ingroup Vision
//!   \addtogroup Bundle
//!   @{

//! Detection Checkerboard structure
struct Rox_Detection_Checkerboard_Struct
{
   //! Pointer to the detected grid
   Rox_Point2D_Double _detected_grid;

   //! Number of detected points
   int                 _n_points;

   //! Pointer to the detected grid
   Rox_Bool            _detected;

   //! Pointer to the detected grid
   int                 _image_width;

   //! Pointer to the detected grid
   int                 _image_height;

   //! Pointer to the detected grid
   Rox_Image   _image;
    
   //! id of image in order to get back the detection results (detected grid)
   Rox_Uint _id;
};

//! @}

#endif
