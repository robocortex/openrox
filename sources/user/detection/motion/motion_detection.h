//==============================================================================
//
//    OPENROX   : File motion_detection.h
//
//    Contents  : Motion detection module definition
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MOTION_DETECTION__
#define __OPENROX_MOTION_DETECTION__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus 

// ====== INCLUDED HEADERS   =================================================
#include "cluster.h"
#include "motion_detection_params.h"

#include <generated/dynvec_rect_sint.h>
#include <generated/dynvec_point2d_float.h>

#include <system/memory/datatypes.h>

#include <baseproc/image/imask/imask.h>

//! \ingroup Vision
//! \defgroup Detection_Motion Motion Detection

// ====== EXPORTED TYPESDEFS =================================================

//! \defgroup Detection_Motion Motion Detection
//! \brief Motion Detection structure and methods.

//! \ingroup Detection_Motion
//! \brief Motion detection object

typedef struct Rox_Detection_Motion_Struct * Rox_Detection_Motion;

// ====== EXPORTED MACROS    =================================================

// ====== INTERNAL MACROS    =================================================

// ====== EXPORTED DATATYPES =================================================

// ====== INTERNAL DATATYPES =================================================

// ====== EXPORTED FUNCTIONS =================================================

//! \addtogroup Detection_Motion
//! @{

//! Create and allocate the motion detection structure.
//! \param  [out]  motion Motion Detection Parameters
//! \param  [in ]  model  Image model
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_new(Rox_Detection_Motion * motion, Rox_Image model);

//! Delete the motion detection structure.
//! \param  [in ] motion Motion Detection object
//! \return Void
ROX_API Rox_Void rox_detection_motion_del(Rox_Detection_Motion * motion);

//! Detect motion between the model and the given image
//! \param  [in ]  list   Motion Detection object
//! \param  [in ]  motion Motion Detection object
//! \param  [in ]  image  Image
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_set_window_list(Rox_DynVec_Rect_Sint list, Rox_Detection_Motion motion, Rox_Image image);

//! Set the image model from an user defined image
//! \param  [in ]  motion Motion Detection object
//! \param  [in ]  model  Image model
//! \return An error code
//! \warning The image size must be the same of the previously defined model
ROX_API Rox_ErrorCode rox_detection_motion_set_model(Rox_Detection_Motion motion, Rox_Image model);

//! Set the image mask from an user defined mask
//! \param  [in ]  motion Motion Detection object
//! \param  [in ]  mask   Image mask
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_set_imask(Rox_Detection_Motion motion, Rox_Imask mask);

//! Set the image mask from a window
//! \param  [in ]  motion Motion Detection object
//! \param  [in ]  window Window
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_set_imask_window(Rox_Detection_Motion motion, Rox_Rect_Sint window);

//! Set the image mask from a list of windows
//! \param  [in ]  motion Motion Detection object
//! \param  [in ]  list    Window list
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_set_imask_window_list(Rox_Detection_Motion motion, Rox_DynVec_Rect_Sint list);

//! Set the babdwidth for target segmentation
//! \param  [out]  motion Motion Detection object
//! \param  [in ]  sizu   Bandidth along the u axis
//! \param  [in ]  sizv   Bandidth along the v axis
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_set_bandwidth (
   Rox_Detection_Motion motion, 
   const Rox_Sint sizu, 
   const Rox_Sint sizv
);

//! Set the sensitivity for motion detection
//! \param  [out]  motion      Motion Detection object
//! \param  [in ]  sensitivity Sensitivity of motion detection in [0,1]
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_set_sensitivity (
   Rox_Detection_Motion motion, 
   const Rox_Float sensitivity
);

// ====== INTERNAL FUNCTIONS =================================================

//! Create and allocate the motion detection structure.
//! \param  [out]  motion         Motion Detection Parameters
//! \param  [in ]  model          Image model
//! \param  [in ]  params         Motion Detection Parameters
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_new_init_params (
   Rox_Detection_Motion * motion, 
   Rox_Image model, 
   Rox_Detection_Motion_Params params
);

//! @} 

#ifdef __cplusplus
}
#endif // __cplusplus 

#endif // __OPENROX_DETECTION_MOTION__ 
