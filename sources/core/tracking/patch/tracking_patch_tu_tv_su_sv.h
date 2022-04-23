//==============================================================================
//
//    OPENROX   : File tracking_patch_tu_tv_su_sv.h
//
//    Contents  : API of tracking_patch_tu_tv_su_sv module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __TRACKING_TU_TV_SU_SV__
#define __TRACKING_TU_TV_SU_SV__

#include <generated/array2d_float.h>

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matrix.h>

#include <core/patch/patchplane.h>

//! \ingroup Tracking
//! \addtogroup Patch
//! @{

//! The Rox_Tracking_Patch_tu_tv_su_sv_Struct object 
struct Rox_Tracking_Patch_tu_tv_su_sv_Struct
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
   Rox_Double su;

   //! Estimated R 
   Rox_Double sv;
};

//! Define the pointer of the Rox_Tracking_Patch_tu_tv_su_sv_Struct 
typedef struct Rox_Tracking_Patch_tu_tv_su_sv_Struct * Rox_Tracking_Patch_tu_tv_su_sv;

//! Create tracking plane object
//! \param  [in]  tracking        the pointer to the tracking object
//! \return An error code
//! \todo to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_new(
   Rox_Tracking_Patch_tu_tv_su_sv * tracking
   );

//! Delete tracking plane object
//! \param  [in ]  tracking       the pointer to the tracking object
//! \return An error code
//! \todo to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_del(
   Rox_Tracking_Patch_tu_tv_su_sv * tracking
   );

//! Perform tracking on the given patch
//! \param  [in ]  tracking       the pointer to the tracking object
//! \param  [in ]  patch          the patch to track
//! \param  [in ]  source         the captured image to track into
//! \param  [in ]  max_iters      the maximum iteration count (if no convergence)
//! \return An error code
//! \todo to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_make(
   Rox_Tracking_Patch_tu_tv_su_sv tracking, 
   Rox_PatchPlane patch, 
   Rox_Array2D_Float source, 
   Rox_Uint max_iters
   );

//! Set state parameters
//! \param  [in ]  tracking       the tracking object
//! \param  [in ]  tu             set tu value
//! \param  [in ]  tv             set tv value
//! \param  [in ]  su             set su value
//! \param  [in ]  sv             set sv value
//! \return An error code
//! \todo   to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_set_state(
   Rox_Tracking_Patch_tu_tv_su_sv tracking, 
   Rox_Double tu, 
   Rox_Double tv, 
   Rox_Double su, 
   Rox_Double sv
   );

//! Get current parameters
//! \param  [out]  tu             pointer to the tu result
//! \param  [out]  tv             pointer to the tv value
//! \param  [out]  su             pointer to the su value
//! \param  [out]  sv             pointer to the sv value
//! \param  [in ]  tracking       the tracking object
//! \return An error code
//! \todo   to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_get_state(
   Rox_Double * tu, 
   Rox_Double * tv, 
   Rox_Double * su, 
   Rox_Double * sv, 
   Rox_Tracking_Patch_tu_tv_su_sv tracking
   );

//! Set H from tu tv s r
//! \param  [in ]  H              The 3x3 homography matrix in SL3
//! \param  [in ]  tu             Set tu value
//! \param  [in ]  tv             Set tv value
//! \param  [in ]  su             Set su value
//! \param  [in ]  sv             Set sv value
//! \return An error code
//! \todo   To de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_to_SL3(
   Rox_MatSL3 H, 
   Rox_Double tu, 
   Rox_Double tv, 
   Rox_Double su, 
   Rox_Double sv
   );

//! Get tu, tv, s, r from 3x3 matrix
//! \param  [out]  tu 	          Pointer to the tu result
//! \param  [out]  tv 	          Pointer to the tv value
//! \param  [out]  su 	          Pointer to the su value
//! \param  [out]  sv 	          Pointer to the sv value
//! \param  [in ]  H              The 3x3 homography matrix in SL3
//! \return An error code
//! \todo   To de tested
ROX_API Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_from_SL3(
   Rox_Double * tu, 
   Rox_Double * tv, 
   Rox_Double * su, 
   Rox_Double * sv, 
   Rox_MatSL3 H
   );

//! @} 

#endif
