//==============================================================================
//
//    OPENROX   : File point3d_print.h
//
//    Contents  : API of point3d display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINT3D_PRINT__
#define __OPENROX_POINT3D_PRINT__

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/point/point3d_struct.h>

//! \addtogroup Point3D
//! @{

//! Display a 3D point float on stdout
//! \param [in] 	point3D 		The point to print
//! \return An error code
ROX_API Rox_ErrorCode rox_point3d_float_print ( const Rox_Point3D_Float point3D );

//! Display a 3D point double on stdout
//! \param [in] 	point3D 		The point to print
//! \return An error code
ROX_API Rox_ErrorCode rox_point3d_double_print ( const Rox_Point3D_Double point3D );

ROX_API Rox_ErrorCode rox_point3d_float_vector_print ( Rox_Point3D_Float  points3D, const Rox_Sint nbpoints);

ROX_API Rox_ErrorCode rox_vector_point3d_double_print ( Rox_Point3D_Double  points3D, const Rox_Sint nbpoints);

//! @} 

#endif
