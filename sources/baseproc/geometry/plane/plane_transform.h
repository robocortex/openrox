//==============================================================================
//
//    OPENROX   : File plane_transform.h
//
//    Contents  : API of plane_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PLANE_TRANSFORM__
#define __OPENROX_PLANE_TRANSFORM__

#include "plane_struct.h"

#include <baseproc/maths/linalg/matse3.h>

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>

//! \addtogroup Plane
//! @{

//! Transform a plane using a se(3) matrix
//! \param  [out]  transformed_a 		The transformed plane a parameter
//! \param  [out]  transformed_b 		The transformed plane b parameter
//! \param  [out]  transformed_c 		The transformed plane c parameter
//! \param  [out]  transformed_d 		The transformed plane d parameter
//! \param  [in ]  pose 					The transformation from the original to the new frame
//! \param  [in ]  a the plane a 		Component in the original frame
//! \param  [in ]  b the plane b 		Component in the original frame
//! \param  [in ]  c the plane c 		Component in the original frame
//! \param  [in ]  d the plane d 		Component in the original frame
//! \return An error code
ROX_API Rox_ErrorCode rox_plane_transform ( 
   Rox_Double * transformed_a, 
   Rox_Double * transformed_b, 
   Rox_Double * transformed_c, 
   Rox_Double * transformed_d, 
   const Rox_MatSE3 pose, 
   const Rox_Double a,
   const Rox_Double b, 
   const Rox_Double c, 
   const Rox_Double d
);

//! Transform a plane using a se(3) matrix
//! \param  [out]  plane3d_out			The transformed plane a parameter
//! \param  [in ]  pose 					The transformation from the original to the new frame
//! \param  [in ]  plane3d_inp			The plane in the original frame
//! \return An error code
ROX_API Rox_ErrorCode rox_plane3d_transform (
   Rox_Plane3D_Double plane3d_out, const Rox_MatSE3 pose, const Rox_Plane3D_Double plane3d_inp);

//! @}

#endif
