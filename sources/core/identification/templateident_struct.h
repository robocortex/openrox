//==============================================================================
//
//    OPENROX   : File templateident_struct.h
//
//    Contents  : API of templateident module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TEMPLATE_IDENT_STRUCT__
#define __OPENROX_TEMPLATE_IDENT_STRUCT__

#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <generated/dynvec_sint.h>
#include <generated/dynvec_sraiddesc.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/objset_dynvec_sraid_feature.h>
#include <generated/objset_dynvec_sraid_feature_struct.h>
#include <generated/dynvec_sraiddesc_struct.h>

//! \addtogroup Identification
//! @{

//! The Rox_Template_Ident_Struct object
struct Rox_Template_Ident_Struct
{
   //! Detected features
   Rox_ObjSet_DynVec_SRAID_Feature reference_features_subsets;

   //! Count matched features
   Rox_Sint count_matched;

   //! Point which have beed matched
   Rox_DynVec_Point2D_Float current_points_matched;

   //! Point which have beed matched
   Rox_DynVec_Point2D_Float reference_points_matched;

   //! Calibration for reference image
   Rox_Array2D_Double calib_input;

   //! Information about template dimensions
   Rox_Sint model_width_pixels;

   //! Information about template dimensions
   Rox_Sint model_height_pixels;
};

//! @}

#endif // __OPENROX_TEMPLATE_IDENT_STRUCT__
