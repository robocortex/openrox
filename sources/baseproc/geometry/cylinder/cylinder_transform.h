//==============================================================================
//
//    OPENROX   : File cylinder_transform.h
//
//    Contents  : API of cylinder_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CYLINDER_TRANSFORM__
#define __OPENROX_CYLINDER_TRANSFORM__

#include <baseproc/geometry/cylinder/cylinder2d.h>
#include <baseproc/geometry/cylinder/cylinder3d.h>
#include <generated/array2d_double.h>

//! \addtogroup cylinder
//! @{

//! Transform a 2d cylinder a calibration matrix
//! \param [out]  cylinder2d_out     The input 2d cylinder
//! \param [in ]  cylinder2d_inp     The input 2d cylinder
//! \param [in ]  matct2             The calibration matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_cylinder2d_transform ( Rox_Cylinder2D cylinder2d_out, const Rox_Array2D_Double matct2, const Rox_Cylinder2D cylinder3d_inp);

//! Transform a 3d cylinder with planes coordinates given a pose
//! \param [out]  cylinder3d_out     The input 3d cylinder
//! \param [in ]  cylinder3d_inp     The input 3d cylinder
//! \param [in ]  matse3             The tranformation pose
//! \return An error code
ROX_API Rox_ErrorCode rox_cylinder3d_transform ( Rox_Cylinder3D cylinder3d_out, const Rox_Array2D_Double matse3, const Rox_Cylinder3D cylinder3d_inp);

//! @}

#endif
