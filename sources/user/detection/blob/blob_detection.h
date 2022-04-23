//==============================================================================
//
//    OPENROX   : File blob_detection.h
//
//    Contents  : Blob detection module definition
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BLOB_DETECTION__
#define __OPENROX_BLOB_DETECTION__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus 

//=== INCLUDED HEADERS   =======================================================

#include "blob_detection_params.h"
#include <generated/dynvec_rect_sint.h>
#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>

#include <system/memory/datatypes.h>

#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>

#include <inout/system/errors_print.h>

#include <user/detection/motion/cluster.h>

//! \ingroup Vision
//! \defgroup Detection_Blob Blob Detection

//=== EXPORTED TYPESDEFS =======================================================

//! \defgroup Detection_Blob Blob Detection
//! \brief Blob Detection structure and methods.

//! \ingroup Detection_Blob
//! \brief Blob detection object
typedef struct Rox_Detection_Blob_Struct* Rox_Detection_Blob;

//=== EXPORTED MACROS    =======================================================

//=== INTERNAL MACROS    =======================================================

//=== EXPORTED DATATYPES =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== EXPORTED FUNCTIONS =======================================================

//! \addtogroup Detection_Blob
//! @{

//! Create and allocate the blob detection structure.
//! \param [out] blob_detection 	Blob Detection Parameters
//! \param [in]  model  				Image model
//! \return A pointer to the created structure
ROX_API Rox_ErrorCode rox_detection_blob_new(Rox_Detection_Blob * blob_detection, Rox_Image model);

//! Delete the blob detection structure.
//! \param [in] blob_detection 		Blob Detection object
//! \return Void
ROX_API Rox_Void rox_detection_blob_del(Rox_Detection_Blob * blob_detection);

//! Detect blob between the model and the given image
//! \param [in] center_list   		Blob Detection object
//! \param [in] list   					Blob Detection object
//! \param [in] blob_detection   	Blob Detection object
//! \param [in] image  					Image
//! \return A pointer to the list of windows defining the detected ROI
ROX_API Rox_ErrorCode rox_detection_blob_set_window_list (
   Rox_DynVec_Point2D_Double center_list, Rox_DynVec_Rect_Sint list, Rox_Detection_Blob blob_detection, Rox_Image image);

//! Set the image model from an user defined image
//! \param [in] blob_detection 		Blob Detection object
//! \param [in] model  					Image model
//! \return An error code
//! \warning The image size must be the same of the previously defined model
ROX_API Rox_ErrorCode rox_detection_blob_set_model(
   Rox_Detection_Blob blob_detection, 
   const Rox_Image model
);

//! Set the image mask from an user defined mask
//! \param [in] blob_detection 		Blob Detection object
//! \param [in] mask   					Image mask
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_blob_set_imask(
   Rox_Detection_Blob blob_detection, 
   const Rox_Imask mask
);

//! Set the image mask from a window
//! \param [in] blob_detection 		Blob Detection object
//! \param [in] window 					Window
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_blob_set_imask_window (
   Rox_Detection_Blob blob_detection, 
   const Rox_Rect_Sint window
);

//! Set the image mask from a list of windows
//! \param [in] blob_detection 		Blob Detection object
//! \param [in] list    				Window list
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_blob_set_imask_window_list (
   Rox_Detection_Blob blob_detection, 
   const Rox_DynVec_Rect_Sint list
);

//! Set the babdwidth for target segmentation
//! \param [out] blob_detection 		Blob Detection object
//! \param [in]  sizu   				Bandidth along the u axis
//! \param [in]  sizv   				Bandidth along the v axis
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_blob_set_bandwidth (
   Rox_Detection_Blob blob_detection, 
   const Rox_Sint sizu, 
   const Rox_Sint sizv
);

//! Set the sensitivity for blob detection
//! \param [out] blob_detection     Blob Detection object
//! \param [in]  sensitivity 			Sensitivity of blob detection in [0,1]
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_blob_set_sensitivity(
   Rox_Detection_Blob blob_detection, 
   const Rox_Float sensitivity
);

//typedef struct Rox_Matching_3Dto2D_Struct* Rox_Matching_3Dto2D;
//ROX_API Rox_ErrorCode rox_matching_3d_to_2d_points_new(Rox_Matching_3Dto2D * matching, Rox_DynVec_Point3D_Double points3D, Rox_MatUT3 calib);
//ROX_API Rox_ErrorCode rox_matching_3d_to_2d_points_make(Rox_Matching_3Dto2D matching, Rox_DynVec_Point2D_Double points2D);
//ROX_API Rox_ErrorCode rox_matching_3d_to_2d_points_get_pose(Rox_Sint * isidentified, Rox_MatSE3 pose, Rox_Matching_3Dto2D matching);
//ROX_API Rox_ErrorCode rox_matching_3d_to_2d_points_del(Rox_Matching_3Dto2D * matching);
//ROX_API Rox_ErrorCode rox_dynvec_point3d_double_set_data(Rox_DynVec_Point3D_Double points3D, Rox_Double * data_points3D, Rox_Sint numb_points3D);

//====== INTERNAL FUNCTIONS =================================================

//! Create and allocate the blob detection structure.
//! \param [in]  	blob_detection 				Blob Detection Parameters
//! \param [in] 	model  			Image model
//! \param [in] 	params 			Blob Detection Parameters
//! \return A pointer to the created structure
//! \return An error code
ROX_API Rox_ErrorCode rox_detection_blob_new_init_params(
   Rox_Detection_Blob * blob_detection, 
   Rox_Image model, 
   Rox_Detection_Blob_Params params
);

//! @} 

#ifdef __cplusplus
}
#endif // __cplusplus 

#endif // __OPENROX_DETECTION_BLOB__ 
