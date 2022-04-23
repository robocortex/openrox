//==============================================================================
//
//    OPENROX   : File point2d_save.h
//
//    Contents  : API of point2d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINT2D_SAVE__
#define __OPENROX_POINT2D_SAVE__

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point2d_struct.h>

//! \addtogroup point2d
//! @{

//! Save 3D points on file
//! \param [in] 	points3D 		The points to save
//! \return An error code
ROX_API Rox_ErrorCode rox_vector_point2d_double_save (
   const Rox_Char * filename, 
   const Rox_Point2D_Double point2d, 
   const Rox_Sint nb_points
);

//! Save 3D points appending on file
//! \param [in] 	points3D 		The points to save
//! \return An error code
ROX_API Rox_ErrorCode rox_vector_point2d_double_save_append ( 
   const Rox_Char * filename, 
   const Rox_Point2D_Double point2d, 
   const Rox_Sint nb_points
);

ROX_API Rox_ErrorCode rox_vector_point2d_double_load ( 
   Rox_Point2D_Double points2D, 
   const Rox_Sint nb_points,
   const Rox_Char * filename
);

//! @} 

#endif
