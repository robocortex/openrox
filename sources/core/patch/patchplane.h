//==============================================================================
//
//    OPENROX   : File patchplane.h
//
//    Contents  : API of patchplane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PATCHPLANE__
#define __OPENROX_PATCHPLANE__

#include <system/memory/datatypes.h>
#include <generated/array2d_float.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Patch
//! \addtogroup PatchPlane
//! @{

//! The Rox_PatchPlane_Struct object 
struct Rox_PatchPlane_Struct
{
   //! The patch width 
   Rox_Sint width;

   //! The patch height 
   Rox_Sint height;

   //! The illumination alpha parameter 
   Rox_Float alpha;

   //! The illumination beta parameter 
   Rox_Float beta;

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
};

//! Define the pointer of the Rox_PatchPlane_Struct 
typedef struct Rox_PatchPlane_Struct * Rox_PatchPlane;

//! Create patch plane object
//! \param  [out]  obj            The pointer to the patch object
//! \param  [in ]  height         The patch height
//! \param  [in ]  width          The patch width
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_patchplane_new(Rox_PatchPlane * obj, Rox_Sint height, Rox_Sint width);

//! Delete patch plane object
//! \param  [in]   obj            The pointer to the patch object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_patchplane_del(Rox_PatchPlane * obj);

//! Set reference image plus mask
//! \param  [out] obj the patch
//! \param  [in]  image the reference image (size defined in constructor)
//! \param  [in]  imagemask the reference image mask (size defined in constructor)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_patchplane_set_reference (
   Rox_PatchPlane obj, 
   const Rox_Array2D_Float image, 
   const Rox_Imask imagemask
);

//! Reset affine luminance information
//! \param  [out]  obj            The patch
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_patchplane_reset_luminance(Rox_PatchPlane obj);

//! Prepare patch warp using an homography
//! \param  [out]  obj            The patch
//! \param  [in ]  homography     The transformation matrix
//! \param  [in ]  image          The current image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_patchplane_prepare_sl3(Rox_PatchPlane obj, Rox_MatSL3 homography, Rox_Array2D_Float image);

//! Finalize patch preparation
//! \param  [out]  obj            The patch
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_patchplane_prepare_finish(Rox_PatchPlane obj);

//! Compute zncc between reference and current buffers
//! \param  [out]  score          Pointer to the result
//! \param  [in ]  obj            The patch
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_patchplane_compute_score(Rox_Double * score, Rox_PatchPlane obj);

//! @} 

#endif
