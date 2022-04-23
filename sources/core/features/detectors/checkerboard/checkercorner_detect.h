//==============================================================================
//
//    OPENROX   : File checkercorner_detect.h
//
//    Contents  : API of checkercorner_detect module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CHECKERCORNER_DETECTOR__
#define __OPENROX_CHECKERCORNER_DETECTOR__

#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <generated/objset_array2d_float.h>
#include <generated/dynvec_checkercorner_struct.h>
#include <generated/dynvec_checkercorner.h>
#include <generated/objset_dynvec_sparse_value.h>
#include <baseproc/image/image.h>

//! \ingroup Detectors
//! \addtogroup CheckerCorner
//! @{

//! Checker corner detector structure
struct Rox_CheckerCorner_Detector_Struct
{
   //! Original image in float
   Rox_Array2D_Float source;

   //! Image gradient in float
   Rox_Array2D_Float gx;

   //! Image gradient in float
   Rox_Array2D_Float gy;

   //! Image gradient angle (should be in float ???)
   Rox_Array2D_Double angles;

   //! Image gradient magnitude (should be in float ???)
   Rox_Array2D_Double magnitudes;

   //! Image cornerness
   Rox_Array2D_Float cornerness;

   //! Image intermediate response
   Rox_Array2D_Float rep1;

   //! Image intermediate response
   Rox_Array2D_Float rep2;

   //! Image intermediate response
   Rox_Array2D_Float rep3;

   //! Image intermediate response
   Rox_Array2D_Float rep4;

   //! Image kernels
   Rox_ObjSet_DynVec_Sparse_Value kernels;

   //! List of raw kernels
   Rox_DynVec_CheckerCorner local_corners;

   //! List of kernels
   Rox_DynVec_CheckerCorner corners;

   //! Flag to set the maximum level of blurring for corners
   Rox_Uint kernel_blur_levels;

   //! Score blur
   Rox_Uint score_blur_levels;
};

//! Pointer to the structure
typedef struct Rox_CheckerCorner_Detector_Struct * Rox_CheckerCorner_Detector;

//! checkerboard object
typedef struct Rox_CheckerCorner_Struct * Rox_CheckerCorner;


//! Create a container object for checkerboard corner detection
//! \param  [out]  checkercorner_detector       The created container pointer
//! \param  [in ]  width                        The width  of the image needed to allocate internal buffers memory
//! \param  [in ]  height                       The height of the image needed to allocate internal buffers memory
//! \param  [in ]  kernel_blur_levels           The higher is the max level [1,2,...,8] the slower is the algorithm but more robust to corners which are not sharp
//! \param  [in ]  score_blur_levels            The higher is the max level [1,2,...,8] the slower is the algorithm but more robust to corners which are not sharp
//! \return An error code
ROX_API Rox_ErrorCode rox_checkercorner_detector_new (
   Rox_CheckerCorner_Detector * checkercorner_detector, 
   const Rox_Sint cols, 
   const Rox_Sint rows, 
   const Rox_Uint kernel_blur_levels, 
   const Rox_Uint score_blur_levels
);

//! Delete a container object for checkerboard corner detection
//! \param  [out]  checkercorner_detector     The container pointer to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_checkercorner_detector_del ( Rox_CheckerCorner_Detector * checkercorner_detector );

//! Extract checkerboard corner from an image
//! \param  [out]  checkercorner_detector     The container pointer
//! \param  [in ]  image                      The input image
//! \return An error code
ROX_API Rox_ErrorCode rox_checkercorner_detector_process (
   Rox_CheckerCorner_Detector checkercorner_detector, 
   const Rox_Image image
);

//! Build kernels for further corner detection
//! \param  [out]  checkercorner_detector     The container pointer to use
//! \return An error code
ROX_API Rox_ErrorCode rox_checkercorner_build_kernels ( Rox_CheckerCorner_Detector checkercorner_detector );

//! @}

#endif // __OPENROX_CHECKERCORNER_DETECTOR__
