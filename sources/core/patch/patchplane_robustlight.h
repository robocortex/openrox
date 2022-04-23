//==============================================================================
//
//    OPENROX   : File patchplane_robustlight.h
//
//    Contents  : API of patchplane_robustlight module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PATCHPLANE_ROBUSTLIGHT__
#define __OPENROX_PATCHPLANE_ROBUSTLIGHT__

#include <system/memory/datatypes.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_double.h>
#include <generated/objset_array2d_float.h>
#include <generated/objset_array2d_float_struct.h>
#include <generated/objset_array2d_uint.h>
#include <generated/objset_array2d_uint_struct.h>
#include <generated/dynvec_double.h>
#include <generated/dynvec_double_struct.h>
#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <baseproc/image/imask/imask.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

//! \ingroup Patch
//! \addtogroup PatchPlane_Robustlight
//! @{

//! The Rox_PatchPlane_RobustLight_Struct object 
struct Rox_PatchPlane_RobustLight_Struct
{
   //! The patch width 
   Rox_Sint width;

   //! The patch height 
   Rox_Sint height;

   // Global additive term
   //! The illumination beta parameter 
   Rox_Float beta;

   //! Error 
   Rox_Double error;

   // Reference info
   //! The template image 
   Rox_Array2D_Float reference;

   //! The template mask 
   Rox_Imask reference_mask;

   // Remap info
   //! The remap grid 
   Rox_MeshGrid2D_Float grid;

   // Warp
   //! The current template 
   Rox_Array2D_Float current;

   //! The current mask 
   Rox_Imask current_mask;

   // Gradient
   //! The gradient along x-axis 
   Rox_Array2D_Float gx;
   
   //! The gradient along y-axis 
   Rox_Array2D_Float gy;

   //! The gradient mask 
   Rox_Imask gradient_mask;

   // Misc
   //! The current template with illumination changes 
   Rox_Array2D_Float warped_lum;

   //! The difference between current and reference template 
   Rox_Array2D_Float difference;

   //! The mean of the reference template and the current template 
   Rox_Array2D_Float mean;

   //! The mean of the reference template and the current template 
   Rox_Array2D_Float mean_lum;

   // Subblocks
   //! The illumination alpha parameter of each subblock
   Rox_DynVec_Double alphas;
   //! The mean of each subblock
   Rox_DynVec_Double pixelmeans;
   //! The top left coordinate of each subblock 
   Rox_DynVec_Point2D_Double toplefts;
   //! The current template of each subblock 
   Rox_ObjSet_Array2D_Float subs_current;
   //! The current template with illumination changes of each subblock 
   Rox_ObjSet_Array2D_Float subs_warped_lum;
   //! The mean of the reference template and the current template for each subblock 
   Rox_ObjSet_Array2D_Float subs_mean;
   //! The difference between current and reference template for each subblock 
   Rox_ObjSet_Array2D_Float subs_difference;
   //! The gradient along x-axis for each subblock 
   Rox_ObjSet_Array2D_Float subs_gx;
   //! The gradient along y-axis for each subblock 
   Rox_ObjSet_Array2D_Float subs_gy;
   //! The gradient mask for each subblock
   Rox_ObjSet_Array2D_Uint subs_gradient_mask;
   //! The current mask for each subblock
   Rox_ObjSet_Array2D_Uint subs_current_mask;
};

//! Define the pointer of the Rox_PatchPlane_RobustLight_Struct 
typedef struct Rox_PatchPlane_RobustLight_Struct * Rox_PatchPlane_RobustLight;

//! Create patch plane object
//! \param [out] obj the pointer to the patch object
//! \param [in] height the patch height
//! \param [in] width the patch width
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_new(Rox_PatchPlane_RobustLight * obj, Rox_Sint height, Rox_Sint width);

//! \brief Delete patch plane object
//! \param [in] obj the pointer to the patch object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_del(Rox_PatchPlane_RobustLight * obj);

//! Set reference image plus mask
//! \param [out] obj the patch
//! \param [in] source the reference image (size defined in constructor)
//! \param [in] sourcemask the reference image mask (size defined in constructor)
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_set_reference(Rox_PatchPlane_RobustLight obj, Rox_Array2D_Float source, Rox_Imask sourcemask);

//! Reset affine luminance information
//! \param [out] obj the patch
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_reset_luminance(Rox_PatchPlane_RobustLight obj);

//! Prepare patch warp using an homography
//! \param [out] obj the patch
//! \param [in] homography the transformation matrix
//! \param [in] source the current image
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_prepare_sl3(Rox_PatchPlane_RobustLight obj, Rox_Array2D_Double homography, Rox_Array2D_Float source);

//! Finalize patch preparation
//! \param [out] obj the patch
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_prepare_finish(Rox_PatchPlane_RobustLight obj);

//! Compute zncc between reference and current buffers
//! \param [out] score Pointer to the result
//! \param [in] obj the patch
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_patchplane_robustlight_compute_score(Rox_Double * score, Rox_PatchPlane_RobustLight obj);

//! @} 

#endif
