//==============================================================================
//
//    OPENROX   : File cylinder_project.h
//
//    Contents  : API of cylinder_project module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CYLINDER_PROJECT__
#define __OPENROX_CYLINDER_PROJECT__

#include <baseproc/geometry/cylinder/cylinder2d.h>
#include <baseproc/geometry/cylinder/cylinder3d.h>

//! \addtogroup cylinder
//! @{

//! Project a 3D cylinder into a 2D cylinder on the image plane
//! \param [out]  cylinder2d the output 2d cylinder
//! \param [in]   cylinder3d the input 3d cylinder
//! \param [in]   matct2
//! \return An error code
ROX_API Rox_ErrorCode rox_cylinder2d_project_cylinder3d(Rox_Cylinder2D cylinder2d, Rox_Array2D_Double matct2, Rox_Cylinder3D cylinder3d);

//! Project a 3D cylinder into a 2D cylinder on the image plane
//! \param [out]  cylinder2d the output 2d cylinder
//! \param [in]   cylinder3d the input 3d cylinder
//! \param [in]   matct2
//! \param [in]   matse3
//! \return An error code
ROX_API Rox_ErrorCode rox_cylinder2d_transform_project_cylinder3d(Rox_Cylinder2D cylinder2d, Rox_Array2D_Double matct2, Rox_Array2D_Double matse3, Rox_Cylinder3D cylinder3d);

//! @} 

#endif
