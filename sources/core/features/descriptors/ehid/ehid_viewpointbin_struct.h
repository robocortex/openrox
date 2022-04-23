//==============================================================================
//
//    OPENROX   : File ehid_viewpointbin_struct.h
//
//    Contents  : API of ehid_viewpointbin module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_VIEWPOINTBIN_STRUCT__
#define __OPENROX_EHID_VIEWPOINTBIN_STRUCT__

#include <core/features/descriptors/ehid/ehid.h>

#include <generated/dynvec_point3d_double_struct.h>
#include <generated/dynvec_segment_point_struct.h>
#include <generated/objset_ehid_window_struct.h>
#include <generated/dynvec_ehid_dbindex_struct.h>
#include <generated/dynvec_ehid_point_struct.h>

//! \addtogroup EHID
//! @{

//! Viewpoint for ehid clustering structure 
struct Rox_Ehid_ViewpointBin_Struct
{
   //! Image size 
   Rox_Dim_Int_Struct image_size;

   //! minimum scale of this bin 
   Rox_Double min_scale;

   //! mean scale of this bin 
   Rox_Double mean_scale;

   //! max scale of this bin 
   Rox_Double max_scale;

   //! Maximum number of points  
   Rox_Uint max_points;

   //! Viewpoints vectors 
   Rox_DynVec_Point3D_Double vpvec;

   //! internal Transformations 
   Rox_Array2D_Double homography;

   //! internal Transformations 
   Rox_Array2D_Double invhomography;

   //! internal Transformations 
   Rox_Array2D_Double refhomography;

   //! internal Transformations 
   Rox_Array2D_Double warphomography;

   //! internal Transformations 
   Rox_Array2D_Double pose;

   //! internal Transformations 
   Rox_Array2D_Double calib_input;

   //! internal Transformations 
   Rox_Array2D_Double calib_output;

   //! Keypoints used to create points 
   Rox_DynVec_Segment_Point fast_points;

   //! Keypoints used to create points with non maximum suppression 
   Rox_DynVec_Segment_Point fast_points_nonmax;

   //! List of Ehid Windows 
   Rox_ObjSet_Ehid_Window windows;

   //! Global list of clustered features 
   Rox_DynVec_Ehid_Point clustered;

   //! Global list of indices for features 
   Rox_DynVec_Ehid_DbIndex clustered_index;

   //! Standard deviation of gaussian noise
   Rox_Float sigma;
};

//! @} 

#endif
