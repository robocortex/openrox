//==============================================================================
//
//    OPENROX   : File ehid.h
//
//    Contents  : API of ehid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID__
#define __OPENROX_EHID__

#include <stdio.h>

#include "ehid_description.h"
#include <generated/dynvec_ehid_point.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/image/image.h>

//! \defgroup Vision Vision
//! \brief Computer vision structures and methods.

//! \ingroup Vision
//! \defgroup Descriptors Descriptors
//! \brief Image features descriptors.

//! \ingroup Descriptors
//! \defgroup EHID EHID
//! \brief EHID keypoints descriptor.

//! \addtogroup EHID
//! @{

//! Ehid point structure
//typedef struct Rox_Ehid_Point_Struct Rox_Ehid_Point;

//! Compute the descriptor for a list of feature points
//! \param [out] ptr the list of features with 2D coordinates of the points set
//! \param [in] source the image associated with the points to extract description from
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_points_compute(Rox_DynVec_Ehid_Point ptr, Rox_Image source);

//! Warp the position and direction of a feature given an homography matrix
//! \param [in] ptr the list of features to warp
//! \param [in] H the homography matrix
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_points_warp(Rox_DynVec_Ehid_Point ptr, Rox_MatSL3 H);

//! Save a list of feature points to a file in binary.
//! \param [in] filename the output file path
//! \param [in] ptr the feature list to save
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_points_save(char * filename, Rox_DynVec_Ehid_Point ptr);

//! Save a list of feature points to a file in binary.
//! \param [out] output the output file stream
//! \param [in] ptr the feature list to save
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_points_save_stream(FILE * output, Rox_DynVec_Ehid_Point ptr);

//! Load a list of feature points from a file in binary.
//! \param [in] ptr the feature list to load into
//! \param [in] filename the input file path
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_points_load(Rox_DynVec_Ehid_Point ptr, const char * filename);

//! Load a list of feature points from a file in binary.
//! \param [out] ptr the feature list to load into
//! \param [in] input the input file stream
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_points_load_stream(Rox_DynVec_Ehid_Point ptr, FILE * input);

//! Set the target id for all points in this vector
//! \param [out] ptr the feature list to transform
//! \param [in] targetid the id of the target to assign
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_points_set_target(Rox_DynVec_Ehid_Point ptr, Rox_Uint targetid);

//! Compute the matching score between two features (Min 0, max number of bits sets)
//! \param [out] score a pointer to the score variable
//! \param [in] pt1 the first point description
//! \param [in] pt2 the second point description
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_point_match(Rox_Uint * score, Rox_Ehid_Description pt1, Rox_Ehid_Description pt2);

//! Serialize a list of feature points to a buffer.
//! \param [out] ser the serialization buffer
//! \param [in] ptr the feature list to save
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_points_serialize(char* ser, const Rox_DynVec_Ehid_Point ptr);

//! Load a list of feature points from a buffer.
//! \param [out] ptr the feature list to load into
//! \param [in] ser the input buffer
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_points_deserialize(Rox_DynVec_Ehid_Point ptr, const char* ser);

//! Get the octet size for a given list of feature
//! \param [out] ptr the feature list to load into
//! \param [in] size the input buffer
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_points_get_octet_size(Rox_Uint * size, Rox_DynVec_Ehid_Point ptr);

//! Transform a description in integer form to a description in bit form
//! \param [out] bits the output binary description
//! \param [in] ints the input integer description
//! \return Void
//! \todo to be tested
ROX_API Rox_Void rox_ehid_description_from_int_to_bits(Rox_Ehid_Description bits, Rox_Uint ints[64][5]);

//! Transform a description in bit form to a description in integer form
//! \param [out] ints the output integer description
//! \param [in] bits the input binary description
//! \return Void
//! \todo to be tested
ROX_API Rox_Void rox_ehid_description_from_bits_to_int(Rox_Uint ints[64][5], Rox_Ehid_Description bits);

//! @} 

#endif
