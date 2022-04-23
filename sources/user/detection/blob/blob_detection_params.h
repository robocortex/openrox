//==============================================================================
//
//    OPENROX   : File blob_detection_params.h
//
//    Contents  : Blob detection parameters module definition
//
//    Author(s) : R&D department directed by Ezio MALIS
// 
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BLOB_DETECTION_PARAMS__
#define __OPENROX_BLOB_DETECTION_PARAMS__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus 

// ===== INCLUDED HEADERS   =================================================
   
#include <inout/system/errors_print.h>

// ===== EXPORTED TYPESDEFS =================================================

//! \ingroup  Detection_Blob
//! \defgroup Detection_Blob_Params Parameters for Blob Detection
//! \brief Parameters for blob detection.

//! \ingroup Detection_Blob_Params
//! \brief 

//! Object 
typedef struct Rox_Detection_Blob_Params_Struct* Rox_Detection_Blob_Params;

// ===== EXPORTED MACROS    =================================================

// ===== INTERNAL MACROS    =================================================

// ===== EXPORTED DATATYPES =================================================

// ===== INTERNAL DATATYPES =================================================

// ===== EXPORTED FUNCTIONS =================================================

//! \ingroup Detection_Blob_Params
//! \brief Create and allocate the blob detection parameters structure.
//! \param[in] params Blob Detection Params object 
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_blob_params_new(Rox_Detection_Blob_Params * params);

//! \ingroup Detection_Blob_Params
//! \brief Delete the blob detection parameters structure.
//! \param[in] params Blob Detection Params object 
//! \return Void
ROX_API Rox_Void rox_detection_blob_params_del(Rox_Detection_Blob_Params * params);

//! \ingroup Detection_Blob_Params
//! \brief Set the bandwidth for target segmentation 
//! \param  [out]  params              Blob Detection Parameters object
//! \param  [in ]  sizu                Bandidth along the u axis
//! \param  [in ]  sizv                Bandidth along the v axis
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_blob_params_set_bandwidth(Rox_Detection_Blob_Params params, const Rox_Sint sizu, const Rox_Sint sizv);

//! \ingroup Detection_Blob_Params
//! \brief Set the sensitivity for blob detection 
//! \param  [out]  params       Blob Detection Parameters object
//! \param  [in ]  sensitivity  Sensitivity of blob detection in [0,1]
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_blob_params_set_sensitivity(Rox_Detection_Blob_Params params, const Rox_Float sensitivity);

// ===== INTERNAL FUNCTIONS =================================================
  
#ifdef __cplusplus
}
#endif // __cplusplus 

#endif // __OPENROX_MOTION_DETECTION_PARAMS__ 
