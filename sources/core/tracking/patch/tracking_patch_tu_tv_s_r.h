//==============================================================================
//
//    OPENROX   : File tracking_patch_tu_tv_s_r.h
//
//    Contents  : API of tracking_patch_tu_tv_s_r module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRACKING_TU_TV_S_R_OBJECT__
#define __OPENROX_TRACKING_TU_TV_S_R_OBJECT__

#include <generated/array2d_float.h>

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matrix.h>

#include <core/patch/patchplane.h>

//! \ingroup Tracking
//! \addtogroup Patch
//! @{

//! The Rox_Tracking_Patch_tu_tv_s_r_Struct object 
struct Rox_Tracking_Patch_tu_tv_s_r_Struct
{
   //! The result Hessian Matrix  (J^t*J)
   Rox_Matrix JtJ;
   //! The result projected vector (J^t*diff) 
   Rox_Matrix Jtf;
   //! The inverse of (J^t*J) 
   Rox_Matrix iJtJ;
   //! The solution vector (homography + illumation changes) 
   Rox_Matrix solution;
   //! The solution vector (only homography) 
   Rox_Matrix solution_pose;
   //! The estimated homography 
   Rox_MatSL3 homography;

   //! Estimated TU 
   Rox_Double tu;
   //! Estimated TV 
   Rox_Double tv;
   //! Estimated S 
   Rox_Double s;
   //! Estimated R 
   Rox_Double r;
};

//! Define the pointer of the Rox_Tracking_Patch_tu_tv_s_r_Struct 
typedef struct Rox_Tracking_Patch_tu_tv_s_r_Struct * Rox_Tracking_Patch_tu_tv_s_r;

//! Create tracking plane object
//! \param [in] tracking the pointer to the tracking object
//! \return An error code
//! \todo to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_new(
   Rox_Tracking_Patch_tu_tv_s_r * tracking);

//! Delete tracking plane object
//! \param [in] tracking the pointer to the tracking object
//! \return An error code
//! \todo to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_del(
   Rox_Tracking_Patch_tu_tv_s_r * tracking);

//! Perform tracking on the given patch
//! \param [in] tracking the pointer to the tracking object
//! \param [in] patch the patch to track
//! \param [in] source the captured image to track into
//! \param [in] max_iters the maximum iteration count (if no convergence)
//! \return An error code
//! \todo to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_make(
   Rox_Tracking_Patch_tu_tv_s_r tracking, 
   Rox_PatchPlane patch, 
   Rox_Array2D_Float source, 
   Rox_Uint max_iters);

//! Set state parameters
//! \param [in] tracking the tracking object
//! \param [in] tu set tu value
//! \param [in] tv set tv value
//! \param [in] s set s value
//! \param [in] r set r value
//! \return An error code
//! \todo to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_set_state(
   Rox_Tracking_Patch_tu_tv_s_r tracking, 
   Rox_Double tu, Rox_Double tv, 
   Rox_Double s, 
   Rox_Double r
);

//! Get current parameters
//! \param [out] tu pointer to the tu result
//! \param [out] tv pointer to the tv value
//! \param [out] s pointer to the s value
//! \param [out] r pointer to the r value
//! \param [in] tracking the tracking object
//! \return An error code
//! \todo to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_get_state(
   Rox_Double * tu, 
   Rox_Double * tv, 
   Rox_Double * s, 
   Rox_Double * r, 
   Rox_Tracking_Patch_tu_tv_s_r tracking
);

//! Set H from tu tv s r
//! \param  [out]  H              the 3*3 matrix
//! \param  [in ]  tu             set tu value
//! \param  [in ]  tv             set tv value
//! \param  [in ]  s              set s value
//! \param  [in ]  r              set r value
//! \return An error code
//! \todo   to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_to_SL3(
   Rox_MatSL3 H, 
   Rox_Double tu, 
   Rox_Double tv, 
   Rox_Double s, 
   Rox_Double r
);

//! Get tu, tv, s, r from 3x3 matrix
//! \param  [out]  tu             pointer to the tu result
//! \param  [out]  tv             pointer to the tv value
//! \param  [out]  s              pointer to the s value
//! \param  [out]  r              pointer to the r value
//! \param  [in ]                 H the tracking object
//! \return An error code
//! \todo   to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_from_SL3(
   Rox_Double *tu, 
   Rox_Double *tv, 
   Rox_Double *s, 
   Rox_Double *r, 
   Rox_MatSL3 H
);

//! @} 

#endif // __OPENROX_TRACKING_TU_TV_S_R___
