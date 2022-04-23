//==============================================================================
//
//    OPENROX   : File essentialminimize.h
//
//    Contents  : API of essentialminimize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ESSENTIAL_MINIMIZE__
#define __OPENROX_ESSENTIAL_MINIMIZE__

#include <generated/array2d_double.h>
#include <generated/dynvec_uint.h>
#include <generated/dynvec_point2d_float.h>

//! \ingroup Geometry
//! \addtogroup Essential
//! @{

//! Given an initial pose and pairs of points, finds the pose which respects the most an epipolar constraints
//! cur' * E * ref ; E = [R]_x t
//! \param  [out]  pose          initial and result pose
//! \param  [in ]  mask          inlier flag per point
//! \param  [in ]  ref           reference points
//! \param  [in ]  cur           current points
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_essential_minimize ( 
   Rox_Array2D_Double pose, 
   Rox_DynVec_Uint mask, 
   Rox_DynVec_Point2D_Float ref, 
   Rox_DynVec_Point2D_Float cur
);

//! @} 

#endif
