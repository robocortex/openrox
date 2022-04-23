//==============================================================================
//
//    OPENROX   : File unwrap_model.h
//
//    Contents  : API of unwrap_model module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_UNWRAP_MODEL__
#define __OPENROX_UNWRAP_MODEL__

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/plane/plane_struct.h>
#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>

//! \ingroup Vision
//! \addtogroup Unwrap
//! @{

//! Given an image of a quadrilateral and its associated 3D vertices, 
//! create a template which is parrallel to the image plane
//! Get the 3D transformation and calibration related.
//! \param  [out]  unwrapped_image     The newly created image (allocated in the function, delete it after use)
//! \param  [out]  unwrapped_imask     The newly created image mask (allocated in the function, delete it after use)
//! \param  [out]  plane               The plane parameters for the patch in the camera frame
//! \param  [out]  newpose             The new pose for the patch
//! \param  [out]  newcalib            The new calibration for the patch
//! \param  [in ]  source              The original template
//! \param  [in ]  vertices            The 4 3D vertices of the patch in the object frame
//! \param  [in ]  basesize            The minimal dimension of the result patch in pixels
//! \return An error code
ROX_API Rox_ErrorCode rox_unwrap_model ( 
   Rox_Image * unwrapped_image,
   Rox_Imask * unwrapped_imask, 
   Rox_Plane3D_Double plane, 
   Rox_MatSE3 newpose, 
   Rox_MatUT3 newcalib, 
   const Rox_Image image_model, 
   const Rox_Point3D_Double vertices, 
   const Rox_Uint basesize
);

//! @}

#endif
