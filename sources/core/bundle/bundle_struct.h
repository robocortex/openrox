//==============================================================================
//
//    OPENROX   : File bundle_struct.h
//
//    Contents  : Struct of bundle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BUNDLE_STRUCT__
#define __OPENROX_BUNDLE_STRUCT__

#include <generated/objset_bundle_measure_struct.h>
#include <generated/objset_bundle_camera_struct.h>
#include <generated/objset_bundle_frame_struct.h>
#include <generated/objset_bundle_point_struct.h>

//! \ingroup Vision
//! \addtogroup Bundle
//! @{

//! Checkerboard object 
struct Rox_Bundle_Struct
{
   //! Containers for cameras 
   Rox_ObjSet_Bundle_Camera cameras;

   //! Containers for frames 
   Rox_ObjSet_Bundle_Frame frames;

   //! Containers for cameras 
   Rox_ObjSet_Bundle_Point points;

   //! Containers for cameras 
   Rox_ObjSet_Bundle_Measure measures;

   //! Last iteration number of valid measures 
   Rox_Uint count_valid_measures;

   //! Last iteration number of valid points 
   Rox_Uint count_valid_points;

   //! Last iteration number of valid frames 
   Rox_Uint count_valid_frames;

   //! rho convergence factor 
   Rox_Double rho;

   //! Initial lambda factor 
   Rox_Double tau;

   //! LM damping factor 
   Rox_Double lambda;

   //! LM damping factor updater
   Rox_Double lambda_multiplier;

   //! Number of iterations allowed 
   Rox_Uint max_iterations;

   //! Numeric threshold 
   Rox_Double eps1;

   //! Numeric threshold 
   Rox_Double eps2;

   //! L2 Norm of error 
   Rox_Double norm_error;

   //! L2 Norm of error at previous iteration 
   Rox_Double norm_error_previous;
};

//! @} 

#endif
