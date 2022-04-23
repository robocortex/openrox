//============================================================================
//
//    OPENROX   : File draw_cylinder.h
//
//    Contents  : API of draw_cylinder module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_DRAW_CYLINDER__
#define __OPENROX_DRAW_CYLINDER__

#include <baseproc/geometry/cylinder/cylinder2d.h>
#include <baseproc/geometry/cylinder/cylinder3d.h>
#include <baseproc/image/image_rgba.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>

//! \ingroup Image_Display
//! \addtogroup Draw
//! @{

//! Draw the 2D cylinder on the rgba image
//! \param  [out]  image_rgba     The rgba image to draw the cylinder
//! \param  [in ]  cylinder2d     The 2d cylinder
//! \param  [in ]  color          The color of the drawn cylinder
//! \return An error code
//! \ŧodo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_cylinder2d (
   Rox_Image_RGBA image_rgba, 
   Rox_Cylinder2D cylinder2d, 
   Rox_Uint color
);

//! Compute and draw the projection of the 3D cylinder on the rgba image
//! \param  [out]  image_rgba     The rgba image to draw the cylinder
//! \param  [in ]  cylinder3d     The 2d cylinder
//! \param  [in ]  calib          The intrinsics parameters
//! \param  [in ]  pose           The current point of view
//! \param  [in ]  color          The color of the drawn cylinder
//! \return An error code
//! \ŧodo   To be tested
ROX_API Rox_ErrorCode rox_image_rgba_draw_cylinder3d (
   Rox_Image_RGBA image_rgba, 
   Rox_MatUT3 calib, 
   Rox_MatSE3 pose, 
   Rox_Cylinder3D cylinder3d, 
   Rox_Uint color
);

//! @} 

#endif
