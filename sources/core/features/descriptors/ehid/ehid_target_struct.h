//==============================================================================
//
//    OPENROX   : File ehid_target_struct.h
//
//    Contents  : Structure of ehid_target module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_TARGET_STRUCT__
#define __OPENROX_EHID_TARGET_STRUCT__

#include <generated/array2d_double.h>
#include <system/memory/datatypes.h>
#include <generated/dynvec_ehid_match.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_uint.h>

//! \addtogroup EHID
//! @{

//! Target structure for identification 
struct Rox_Ehid_Target_Struct
{
   //! Target image width in pixels 
   Rox_Double width_pixels;

   //! Target image height in pixels 
   Rox_Double height_pixels;

   //! Target image width in meters 
   Rox_Double width_meters;

   //! Target image height in pixels 
   Rox_Double height_meters;

   //! Best detected viewpoint
   Rox_Uint bestvp;

   //! Number of primary match in the best viewpoint
   Rox_Uint bestvpcard;

   //! Is the pose for this target found ?
   Rox_Uint posefound;

   //!Best last score after p3p
   Rox_Double best_score_p3p;

   //! Best last score after minimization
   Rox_Double best_score_minimization;

   //! List of primary matches per viewpoints
   Rox_DynVec_Ehid_Match primarymatches[18];

   //! List of secondary matches per viewpoints
   Rox_DynVec_Ehid_Match secondarymatches[18];

   //! Container for minimization 
   Rox_Array2D_Double vpdifferences;

   //! Container for minimization 
   Rox_DynVec_Point3D_Float vprefs;

   //! Container for minimization 
   Rox_DynVec_Point2D_Float vprefs2d;

   //! Container for minimization 
   Rox_DynVec_Point2D_Float vpcurs;

   //! Container for minimization 
   Rox_DynVec_Point3D_Float allrefs;

   //! Container for minimization 
   Rox_DynVec_Point2D_Float allrefs2d;

   //! Container for minimization 
   Rox_DynVec_Point2D_Float allcurs;

   //! Container for minimization 
   Rox_DynVec_Uint used_cur;

   //! Calibration of the target
   Rox_Array2D_Double calib_input;

   //! Estimated poses for ransac
   Rox_Array2D_Double_Collection coarse_poses;

   //! Estimated pose
   Rox_Array2D_Double pose;

   //! Or estimated homography
   Rox_Array2D_Double best_homography;

   //! Container for optimisation 
   Rox_Array2D_Double homography;
};

//! define 
#define ROX_TYPE_EHID_TARGET (sizeof(struct Rox_Ehid_Target_Struct) << 2)

//! @} 

#endif
