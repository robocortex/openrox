//==============================================================================
//
//    OPENROX   : File motion_detection_params.h
//
//    Contents  : Motion detection parameters module definition
//
//    Author(s) : R&D department directed by Ezio MALIS
// 
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MOTION_DETECTION_PARAMS__
#define __OPENROX_MOTION_DETECTION_PARAMS__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus 

// ====== INCLUDED HEADERS   =================================================

// ====== EXPORTED TYPESDEFS =================================================

//! \ingroup  Detection_Motion
//! \defgroup Detection_Motion_Params Parameters for Motion Detection
//! \brief Parameters for motion detection.

//! \ingroup Detection_Motion_Params
//! \brief 

//! Object 
typedef struct Rox_Detection_Motion_Params_Struct* Rox_Detection_Motion_Params;

// ====== EXPORTED MACROS    =================================================

// ====== INTERNAL MACROS    =================================================

// ====== EXPORTED DATATYPES =================================================

// ====== INTERNAL DATATYPES =================================================

// ====== EXPORTED FUNCTIONS =================================================

//! \ingroup Detection_Motion_Params
//! \brief Create and allocate the motion detection parameters structure.
//! \param  [out]  params        Motion Detection Params object 
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_params_new (
   Rox_Detection_Motion_Params * params
);

//! \ingroup Detection_Motion_Params
//! \brief Delete the motion detection parameters structure.
//! \param  [out]  params        Motion Detection Params object 
//! \return Void
ROX_API Rox_Void rox_detection_motion_params_del (
   Rox_Detection_Motion_Params * params
);

//! \ingroup Detection_Motion_Params
//! \brief Set the bandwidth for target segmentation 
//! \param  [out]  params        Motion Detection Parameters object
//! \param  [in ]  sizu          Bandidth along the u axis
//! \param  [in ]  sizv          Bandidth along the v axis
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_params_set_bandwidth (
   Rox_Detection_Motion_Params params, 
   const Rox_Sint sizu, 
   const Rox_Sint sizv
);

//! \ingroup Detection_Motion_Params
//! \brief Set the sensitivity for motion detection 
//! \param  [out]  params        Motion Detection Parameters object
//! \param  [in ]  sensitivity   Sensitivity of motion detection in [0,1]
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_motion_params_set_sensitivity (
   Rox_Detection_Motion_Params params, 
   const Rox_Float sensitivity
);

//====== INTERNAL FUNCTIONS =================================================
  
#ifdef __cplusplus
}
#endif // __cplusplus 

#endif // __OPENROX_MOTION_DETECTION_PARAMS__ 
