//==============================================================================
//
//    OPENROX   : File quad_gradientclusterer.h
//
//    Contents  : API of quad_gradientclusterer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QUAD_DETECTION_CLUSTERER__
#define __OPENROX_QUAD_DETECTION_CLUSTERER__

#include "quad_detection.h"

#include <generated/array2d_uint.h>
#include <generated/dynvec_quad.h>
#include <generated/dynvec_orientedimagepoint.h>

#include <core/features/detectors/edges/edgedraw.h>
#include <core/features/detectors/edges/edgepreproc_gray.h>
#include <core/features/detectors/edges/edgepostproc_ac.h>
#include <core/features/detectors/edges/edgepostproc_normal.h>

//! To be commented 
typedef struct LabelEdge_Struct * LabelEdge;

//! To be commented 
typedef struct LabelNode_Struct * LabelNode;

//! To be commented
typedef struct Rox_GradientClusterer_Struct * Rox_GradientClusterer;

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_gradientclusterer_new (
   Rox_GradientClusterer * gradientclusterer, 
   const Rox_Uint iwidth, 
   const Rox_Uint iheight
);

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_gradientclusterer_reset (
   Rox_GradientClusterer gradientclusterer
);

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_gradientclusterer_del (
   Rox_GradientClusterer * gradientclusterer
);

//! Build magnitude and angle of image gradients
//! \param  [out]  Imagval       TEMP buffer
//! \param  [out]  
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! warning The Imagval is a temporary buffer
//! warning The mask is UNUSED
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_gradientclusterer_buildgradients (
   Rox_Array2D_Uint Imagval, 
   Rox_Uint * Imag, 
   Rox_Float * Itheta, 
   Rox_Image image_gray, 
   Rox_Imask mask
);

ROX_API Rox_ErrorCode rox_gradientclusterer_buildgradients_refactor ( 
   Rox_Array2D_Uint Imagval, 
   Rox_Uint * Imag, 
   Rox_Float * Itheta, 
   Rox_Image image_gray, 
   Rox_Imask mask
);

//! To be commented
//! \param  [out]  
//! \return The edge cost
//! \todo   To be tested
//! \todo   To be modified to return An error code
ROX_API Rox_Sint rox_gradientclusterer_edgecost ( 
   Rox_Double theta0, Rox_Uint mag0, 
   Rox_Double theta1, Rox_Uint mag1
);

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_gradientclusterer_edgesort ( 
   Rox_GradientClusterer gradientclusterer 
);

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! \todo  To be tested
ROX_API Rox_ErrorCode rox_gradientclusterer_computeedges (
   Rox_GradientClusterer gradientclusterer
);

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_Uint rox_gradientclusterer_getrepresentative ( 
   LabelNode data, 
   Rox_Uint id
);

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return The node id
//! \todo   To be tested
ROX_API Rox_Uint rox_gradientclusterer_connectnodes (
   LabelNode data, 
   Rox_Uint aid, 
   Rox_Uint bid
);

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_gradientclusterer_cluster (
   Rox_GradientClusterer gradientclusterer
);

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_gradientclusterer_groups ( 
   Rox_GradientClusterer gradientclusterer 
);

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_gradientclusterer_make (
   Rox_GradientClusterer gradientclusterer, 
   Rox_Image image_gray, 
   Rox_Imask mask
);

//! To be commented
//! \param  [out]  gradientclusterer pointer to
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_gradientclusterer_make_ac (
   Rox_GradientClusterer gradientclusterer, 
   Rox_Image image_gray, 
   Rox_Imask mask
);

#endif
