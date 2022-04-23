//==============================================================================
//
//    OPENROX   : File photoframe_struct.h
//
//    Contents  : API of photoframe module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PHOTOFRAME_STRUCT__
#define __OPENROX_PHOTOFRAME_STRUCT__

#include <generated/array2d_double.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>


//! \ingroup  Identification_Photoframe
//! \defgroup Photoframe Photoframe
//! \brief Photoframe structures and methods.

//! \addtogroup Photoframe
//! @{

//! The Rox_PhotoFrame_Struct object
struct Rox_PhotoFrame_Struct
{
   //! Am I detected ?
   Rox_Uint is_detected;

   //! Normalization homography
   Rox_Array2D_Double Hinsideframe;

   //! Valid homography to get the photoframe
   Rox_Array2D_Double Hfinal;

   //! Result homography to get the photoframe, only updated when a good match is found
   Rox_Array2D_Double resH;

   //! Result pose to get the photoframe, only updated when a good match is found
   Rox_MatSE3 pose;

   //! Result score in {0,1} : 0 not detected, 1 detected
   Rox_Double score;

   //! The size in meters of the observed photoframe
   Rox_Double width_meters;

   //! The size in meters of the observed photoframe
   Rox_Double height_meters;

   //! Buffer homography for intermediate computation
   Rox_Array2D_Double inplaneH[4];

   //! Reference template rotated
   Rox_Image templates[4];

   //! Reference theta
   Rox_Double theta[4];

   //! Warping grid
   // Rox_Array2D_Point2D_Sshort grid;
   Rox_MeshGrid2D_Float grid;

   //! Warping destination
   Rox_Image dest;

   //! The template calibration
   Rox_MatUT3 calib_template;

   //! The 2D-3D homography
   Rox_MatSL3 G;

   //! mask flag
   Rox_Uint has_mask;

   //! The image mask
   Rox_Imask masks[4];

   //! The score threshold
   Rox_Double score_threshold;
};

//! @}

#endif
