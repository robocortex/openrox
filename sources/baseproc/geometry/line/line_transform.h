//==============================================================================
//
//    OPENROX   : File line_transform.h
//
//    Contents  : API of line_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE_TRANSFORM__
#define __OPENROX_LINE_TRANSFORM__

#include <baseproc/geometry/line/line3d.h>
#include <baseproc/geometry/line/line3d_struct.h>
#include <baseproc/maths/linalg/matse3.h>

//! \addtogroup Line
//! @{

//! Transform a 3d line with planes coordinates given a pose
//! \param  [out]  line3d_out     The input 3d line
//! \param  [in ]  line3d_inp     The input 3d line
//! \param  [in ]  pose           The tranformation pose
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_planes_transform (
   Rox_Line3D_Planes line3d_out, 
   const Rox_Line3D_Planes line3d_inp, 
   const Rox_MatSE3 pose
);

//! Transform a 3d line with plucksr coordinates given a pose
//! \param  [out]  line3d_out     The input 3d line
//! \param  [in ]  line3d_inp     The input 3d line
//! \param  [in ]  pose           The tranformation pose
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_plucker_transform (
   Rox_Line3D_Plucker line3d_out, 
   const Rox_Line3D_Plucker line3d_inp, 
   const Rox_MatSE3 pose
);

//! @} 

#endif
