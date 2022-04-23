//==============================================================================
//
//    OPENROX   : File fpsm.h
//
//    Contents  : API of fpsm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FPSM__
#define __OPENROX_FPSM__

#include <generated/array2d_uint.h>
#include <generated/array2d_sint.h>
#include <generated/array2d_point2d_sshort.h>
#include <system/memory/datatypes.h>
#include <core/features/detectors/edges/edgepreproc_gray.h>
#include <core/features/detectors/edges/edgedraw.h>
#include <core/features/detectors/edges/edgepostproc_ac.h>
#include <core/features/descriptors/fpsm/fpsm_feature_struct.h>

//! \ingroup Descriptors
//! \defgroup FPSM FPSM
//! \brief FPSM edges descriptor.

//! \addtogroup FPSM
//! @{

//! Define the pointer of the Rox_Fpsm_Struct 
typedef struct Rox_Fpsm_Struct * Rox_Fpsm;

//! Create fpsm descriptor object
//! \param[out]	obj the pointer to the object
//! \param[in]	width the width
//! \param[in]	height the height
//! \param[in]	nbr_ref_points
//! \param[in]	min_gradient
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_fpsm_new(Rox_Fpsm * obj, Rox_Sint width, Rox_Sint height, Rox_Uint nbr_ref_points, Rox_Uint min_gradient);

//! Delete fpsm  object
//! \param[in]	obj the pointer to the object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_fpsm_del(Rox_Fpsm * obj);

//! Preprocess image
//! \param[in] fpsm							The fpsm object
//! \param[in] source						The image to extract features from
//! \param[in] min_NFA					NFA minimum value to add edge
//! \param [in] nbr_blur_passes		The number of gradientsobel pass (blur) needed 0 (no blur) to n
//! \param[in]	min_segment_size		The minimum size of a segment to take in account
//! \param[in]	straight_edge_only	Only take in account straight edge, no turns allowed
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_fpsm_preprocess(Rox_Fpsm fpsm, Rox_Image source, Rox_Double min_NFA, Rox_Uint nbr_blur_passes, Rox_Uint min_segment_size, Rox_Bool straight_edge_only);

//! Compute feature for a given window
//! \param[out] feat the features
//! \param[in] fpsm the fpsm object
//! \param[in] x the top left coordinate
//! \param[in] y the top left coordinate
//! \param[in] width  Rectangular window width
//! \param[in] height Rectangular window height
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_fpsm_compute(Rox_Fpsm_Feature_Struct * feat, Rox_Fpsm fpsm, Rox_Uint x, Rox_Uint y, Rox_Sint width, Rox_Sint height);

//! @} 

#endif // __OPENROX_FPSM__ 

