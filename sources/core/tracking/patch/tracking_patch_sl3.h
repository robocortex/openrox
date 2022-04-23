//==============================================================================
//
//    OPENROX   : File tracking_patch_sl3.h
//
//    Contents  : API of tracking_patch_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRACKING_PATCH_SL3__
#define __OPENROX_TRACKING_PATCH_SL3__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>

#include <baseproc/maths/linalg/matsl3.h>

#include <core/patch/patchplane.h>

//! \ingroup Tracking
//! \addtogroup Patch
//!	@{

//! The Rox_Tracking_Patch_SL3_Struct object 
struct Rox_Tracking_Patch_SL3_Struct
{
   //! The result Hessian Matrix  (J^t*J)
   Rox_Array2D_Double JtJ;
   //! The result projected vector (J^t*diff) 
   Rox_Array2D_Double Jtf;
   //! The inverse of (J^t*J) 
   Rox_Array2D_Double iJtJ;
   //! The solution vector (homography + illumation changes) 
   Rox_Array2D_Double solution;
   //! The solution vector (only homography) 
   Rox_Array2D_Double solution_pose;
   //! The estimated homography 
   Rox_Array2D_Double homography;
};

//! Define the pointer of the Rox_Tracking_Patch_SL3_Struct 
typedef struct Rox_Tracking_Patch_SL3_Struct * Rox_Tracking_Patch_SL3;

//! Create tracking plane object
//! \param  [in] obj the pointer to the tracking object
//! \return An error code
//! \todo   to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_sl3_new (
   Rox_Tracking_Patch_SL3 * tracking
   );

//! Delete tracking plane object
//! \param  [in] tracking the pointer to the tracking object
//! \return An error code
//! \todo   to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_sl3_del (
   Rox_Tracking_Patch_SL3 * tracking
   );

//! Perform tracking on the given patch
//! \param  [in] tracking the pointer to the tracking object
//! \param  [in] patch the patch to track
//! \param  [in] source the captured image to track into
//! \param  [in] max_iters the maximum iteration count (if no convergence)
//! \return An error code
//! \todo   to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_sl3_make (
   Rox_Tracking_Patch_SL3 tracking, 
   Rox_PatchPlane patch, 
   Rox_Array2D_Float source, 
   Rox_Uint max_iters
   );

//! Set plane homography
//! \param  [in] tracking the tracking object
//! \param  [in] homography the homography matrix
//! \return An error code
//! \todo   to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_sl3_set_homography (
   Rox_Tracking_Patch_SL3 tracking, 
   Rox_MatSL3 homography
   );

//! Get plane homography
//! \param  [out] homography the homography matrix
//! \param  [in] tracking the tracking object
//! \return An error code
//! \todo   to de tested
ROX_API Rox_ErrorCode rox_tracking_patch_sl3_get_homography (
   Rox_MatSL3 homography, 
   Rox_Tracking_Patch_SL3 tracking
);

//! @} 

#endif // __OPENROX_TRACKING_PATCH_SL3__
