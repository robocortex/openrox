//==============================================================================
//
//    OPENROX   : File line3d_save.h
//
//    Contents  : API of line3d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE3D_SAVE__
#define __OPENROX_LINE3D_SAVE__

#include <baseproc/geometry/line/line3d.h>
#include <baseproc/geometry/line/line3d_struct.h>

//! \addtogroup Point3D
//! @{

//! Save 3D points on file
//! \param [in] 	points3D 		The points to save
//! \return An error code
ROX_API Rox_ErrorCode rox_vector_line3d_planes_save (
   const Rox_Char * filename, 
   const Rox_Line3D_Planes lines3d, 
   const Rox_Sint nb_lines
);

//! Save 3D points appending on file
//! \param [in] 	points3D 		The points to save
//! \return An error code
ROX_API Rox_ErrorCode rox_vector_line3d_planes_save_append ( 
   const Rox_Char * filename, 
   const Rox_Line3D_Planes lines3d, 
   const Rox_Sint nb_lines
);


ROX_API Rox_ErrorCode rox_vector_line3d_planes_load ( 
   Rox_Line3D_Planes lines3d, 
   const Rox_Sint nb_lines,
   const Rox_Char * filename 
);

//! @} 

#endif
