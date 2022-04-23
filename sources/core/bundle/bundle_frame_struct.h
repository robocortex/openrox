//==============================================================================
//
//    OPENROX   : File bundle_frame_struct.h
//
//    Contents  : Structure of bundle_frame module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BUNDLE_FRAME_STRUCT__
#define __OPENROX_BUNDLE_FRAME_STRUCT__

#include <generated/dynvec_bundle_measure.h>
#include <generated/dynvec_bundle_measure_struct.h>

#include <baseproc/geometry/point/points_struct.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matrix.h>

//! \ingroup Vision
//! \addtogroup Bundle
//! @{

//! Bundle structure
struct Rox_Bundle_Frame_Struct
{
   //! index of the camera in global hessian
   Rox_Uint pos;

   //! Column associated
   Rox_Uint pos_jacobian;

   //! Is this frame estimated ?
   Rox_Uint is_fixed;

   //! Is this frame invalid ?
   Rox_Uint is_invalid;

   //! Measured in this frame
   Rox_DynVec_Bundle_Measure measures;

   //! Backup pose of this frame relative to common reference frame
   Rox_MatSE3 pose_previous;

   //! Pose of this frame relative to common reference frame
   Rox_MatSE3 pose;

   //! Hessian
   Rox_Matrix hessian;

   //! Hessian * error
   Rox_Matrix projected_error;

   //! Update information
   Rox_Double update[6];
};

//! @}

#endif // __OPENROX_BUNDLE_FRAME_STRUCT__
