//==============================================================================
//
//    OPENROX   : File sraid.h
//
//    Contents  : API of sraid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SRAID__
#define __OPENROX_SRAID__

#include "sraiddesc.h"
#include <generated/dynvec_point2d_float.h>

//! \ingroup Descriptors
//! \defgroup SRAID SRAID
//! \brief SRAID keypoints descriptor.

//! \addtogroup SRAID
//! @{

//! Detect & describe features using SRAID method (inspired by Lowe SIFT)
//! \param [out]  sraid_output         The set of detected features
//! \param [in]   input input          The image to detect in
//! \param [in]   sraid_max_octaves    The maximal number of octaves (-1 if auto)
//! \param [in]   sraid_sigma          The sigma to apply for scale space
//! \param [in]   cutoff               The cutoff applied to gaussian kernel
//! \param [in]   sraid_initial_sigma  The sigma to apply to input images
//! \param [in]   sraid_contr_thresh   Threshold for contrast around feature
//! \param [in]   sraid_curv_thresh    Threshold for curvature in the area
//! \param [in]   sraid_invls          Number of intervals used in dogspace
//! \param [in]   object_id            Id used for convenience.
//! \return Rox_ErrorCode
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraidpipeline_process(Rox_DynVec_SRAID_Feature sraid_output, Rox_Array2D_Float input, Rox_Sint sraid_max_octaves, Rox_Float sraid_sigma, Rox_Float cutoff, Rox_Float sraid_initial_sigma, Rox_Sint sraid_invls, Rox_Float sraid_contr_thresh, Rox_Sint sraid_curv_thresh, Rox_Uint object_id);

//! Convert a list of features to a list of 2d points
//! \param [out] list a list of points 2D
//! \param [in] features a set of features
//! \return Rox_ErrorCode
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraid_populate_pointlist(Rox_DynVec_Point2D_Float list, Rox_DynVec_SRAID_Feature features);

//! Dump a list of features to a file for further processing
//! \param [out] filename the output filename (including the full path)
//! \param [in] features a list of features to save
//! \return Rox_ErrorCode
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraid_dump_to_file(const Rox_Char * filename, Rox_DynVec_SRAID_Feature features);

//! Load a list of features from a file (saved using dump_to_file)
//! \param [out] features a list of features which will contains the features
//! \param [in] filename the input filename (including the full path)
//! \return Rox_ErrorCode
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraid_load_from_file(Rox_DynVec_SRAID_Feature features, const Rox_Char * filename);

//! @} 

#endif
