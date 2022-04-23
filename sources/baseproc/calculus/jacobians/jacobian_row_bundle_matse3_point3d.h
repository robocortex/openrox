//==============================================================================
//
//    OPENROX   : File jacobian_row_bundle_matse3_point3d.h
//
//    Contents  : API of jacobian_row_bundle_matse3_point3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_JACOBIAN_ROW_BUNDLE_MATSE3_POINT3D__
#define __OPENROX_JACOBIAN_ROW_BUNDLE_MATSE3_POINT3D__

#include <baseproc/geometry/point/point3d.h>

//! \ingroup Optimization
//! \defgroup Jacobians Jacobians

//! \addtogroup Jacobians
//! @{

//! Compute one row of the bundle adjustment jacobian
//! \param  [out]  rowuJpose      a pointer to a 6 cells array (Jac u wrt pose)
//! \param  [out]  rowvJpose      a pointer to a 6 cells array (Jac v wrt pose)
//! \param  [out]  rowuJpoint     a pointer to a 3 cells array (Jac u wrt point)
//! \param  [out]  rowvJpoint     a pointer to a 3 cells array (Jac v wrt point)
//! \param  [in ]  R              a pointer to the rotation matrix
//! \param  [in ]  transformed_point3d the 3d point to use (transformed)
//! \param  [in ]  point3d        the 3d point to use (original)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_se3bundle_row_from_points_float (
   Rox_Double * rowuJpose, 
   Rox_Double * rowvJpose, 
   Rox_Double * rowuJpoint, 
   Rox_Double * rowvJpoint, 
   Rox_Double ** R, 
   Rox_Point3D_Double transformed_point3d, 
   Rox_Point3D_Double point3d
);

//! @} 

#endif // __OPENROX_JACOBIAN_ROW_BUNDLE_MATSE3_POINT3D__
