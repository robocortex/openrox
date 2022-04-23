//==============================================================================
//
//    OPENROX   : File line_project.h
//
//    Contents  : API of line_project module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE_PROJECT__
#define __OPENROX_LINE_PROJECT__

#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/geometry/line/line2d.h>
#include <baseproc/geometry/line/line3d.h>
#include <baseproc/geometry/line/line3d_struct.h>
#include <system/errors/errors.h>

//! \addtogroup Line
//! @{

//! Project a 3D line on the image plane
//! \param  [out]  res            The computed 2D line
//! \param  [in ]  line           The input 3d line
//! \param  [in ]  fu             The calibration fu
//! \param  [in ]  fv             The calibration fv
//! \param  [in ]  cu             The calibration cu
//! \param  [in ]  cv             The calibration cv
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_planes_project_ (
   Rox_Line2D_Normal res, 
   Rox_Line3D_Planes line, 
   Rox_Double fu, 
   Rox_Double fv, 
   Rox_Double cu, 
   Rox_Double cv
);


//! Project a 3D line on the image plane
//! \param  [out]  res            The computed 2D line
//! \param  [in ]  line           The input 3d line
//! \param  [in ]  K              The camera calibration matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_planes_project (
   Rox_Line2D_Normal res, 
   const Rox_Line3D_Planes line, 
   const Rox_MatUT3 K
);


//! Project a 3D line on the image plane
//! \param  [out]  res            The computed 2D line
//! \param  [in ]  line           The input 3d line
//! \param  [in ]  K              The camera calibration matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_planes_project_meters (
   Rox_Line2D_Normal res, 
   const Rox_Line3D_Planes line
);

//! @} 

#endif
