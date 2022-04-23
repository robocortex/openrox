//==============================================================================
//
//    OPENROX   : File ellipse_project.h
//
//    Contents  : API of ellipse_project module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ELLIPSE_PROJECT__
#define __OPENROX_ELLIPSE_PROJECT__

#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/ellipse/ellipse3d.h>

//! \addtogroup ellipse
//! @{

//! Project a 3D ellipse into a 2D ellipse on the image plane
//! \param  [out]  ellipse2d      The output 2d ellipse
//! \param  [in ]  ellipse3d      The input 3d ellipse
//! \param  [in ]  matct2         The intrinsic parameters
//! \return An error code
ROX_API Rox_ErrorCode rox_ellipse2d_project_ellipse3d(Rox_Ellipse2D ellipse2d, Rox_Array2D_Double matct2, Rox_Ellipse3D ellipse3d);

//! Project a 3D ellipse into a 2D ellipse on the image plane
//! \param [out]  ellipse2d       The output 2d ellipse
//! \param [in ]  ellipse3d       The input 3d ellipse
//! \param [in ]  matct2
//! \param [in ]  matse3
//! \return An error code
ROX_API Rox_ErrorCode rox_ellipse2d_transform_project_ellipse3d(Rox_Ellipse2D ellipse2d, Rox_Array2D_Double matct2, Rox_Array2D_Double matse3, Rox_Ellipse3D ellipse3d);

//! @} 

#endif
