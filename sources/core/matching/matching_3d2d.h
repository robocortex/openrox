//==============================================================================
//
//    OPENROX   : File matching_3d2d.h
//
//    Contents  : API of matching 3d2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MATCHING_3D2D__
#define __OPENROX_MATCHING_3D2D__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus 

//=== INCLUDED HEADERS   =======================================================

#include <generated/dynvec_rect_sint_struct.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>

#include <system/memory/datatypes.h>

#include <baseproc/image/imask/imask.h>

#include <user/detection/motion/cluster.h>

//! \ingroup Vision
//! \defgroup Matching_3D2D Matching

//=== EXPORTED TYPESDEFS =======================================================

//! \defgroup Matching_3D2D Matching
//! \brief Matching 3D2D structure and methods.

//! \ingroup Matching_3D2D
//! \brief Matching 3D2D opaque object (pointer to a hidden structure)
typedef struct Rox_Matching_3Dto2D_Struct * Rox_Matching_3Dto2D;

//=== EXPORTED MACROS    =======================================================

//=== INTERNAL MACROS    =======================================================

//=== EXPORTED DATATYPES =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== EXPORTED FUNCTIONS =======================================================

//! \addtogroup Matching_3D2D
//! @{

//! Create and allocate the matching object.
//! \param  [out]  matching       The matching object to be created
//! \param  [in ]  points3D       The 3D points of the model to be matched
//! \param  [in ]  calib          The 3D camera calibration
//! \return An error code
ROX_API Rox_ErrorCode rox_matching_3d_to_2d_points_new(Rox_Matching_3Dto2D * matching, Rox_DynVec_Point3D_Double points3D, Rox_Array2D_Double calib);

//! Set the image mask from an user defined mask
//! \param  [in ]  matching 	    Object
//! \param  [in ]  points2D 	    The points to match
//! \return An error code
ROX_API Rox_ErrorCode rox_matching_3d_to_2d_points_make(Rox_Matching_3Dto2D matching, Rox_DynVec_Point2D_Double points2D);

//! Set the image mask from an user defined mask
//! \param  [out]  is_identified  The matching object to be created
//! \param  [in ]  pose 			 The pose
//! \param  [in ]  matching   	 The matching 
//! \return An error code
ROX_API Rox_ErrorCode rox_matching_3d_to_2d_points_get_pose(Rox_Uint * is_identified, Rox_Array2D_Double pose, Rox_Matching_3Dto2D matching);

//! Delete the blob detection structure.
//! \param  [in ]	 matching   	 The matching 
//! \return An error code
ROX_API Rox_ErrorCode rox_matching_3d_to_2d_points_del(Rox_Matching_3Dto2D * matching);

//====== INTERNAL FUNCTIONS =================================================

//! @} 

#ifdef __cplusplus
}
#endif // __cplusplus 

#endif // __OPENROX_MATCHING_3D2D__ 
