//==============================================================================
//
//    OPENROX   : File dynvec_serialize.h
//
//    Contents  : 
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DYNVEC_SERIALIZE__
#define __OPENROX_DYNVEC_SERIALIZE__

#include <generated/dynvec_fpsm_feature.h>
#include <generated/dynvec_fpsm_template.h>
#include <generated/dynvec_point2d_sshort.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_sint.h>
#include <generated/dynvec_rect_sint.h>
#include <generated/dynvec_edgel.h>
#include <generated/dynvec_edgeturn.h>
#include <generated/dynvec_segment_part.h>
#include <stdio.h>

//! Fast binary serialization of a dynvec of sint.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out   The FILE pointer of output container.
//! \param [in]   input The dynvec to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_sint_serialize(FILE * out, Rox_DynVec_Sint input);

//! Fast binary deserialization of a dynvec of sint.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The dynvec to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.

ROX_API Rox_ErrorCode rox_dynvec_sint_deserialize(Rox_DynVec_Sint output, FILE* in);

//! Fast binary serialization of a dynvec of edgel.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out   The FILE pointer of output container.
//! \param [in]   input The dynvec to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_edgel_serialize(FILE * out, Rox_DynVec_Edgel input);

//! Fast binary deserializationof a dynvec of edgel.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The dynvec to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_edgel_deserialize(Rox_DynVec_Edgel output, FILE* in);

//! Fast binary serialization of a dynvec of edgeTurn.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out   The FILE pointer of output container.
//! \param [in]   input The dynvec to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_edgeturn_serialize(FILE * out, Rox_DynVec_EdgeTurn input);

//! Fast binary deserializationof a dynvec of edgeTurn.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The dynvec to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_edgeturn_deserialize(Rox_DynVec_EdgeTurn output, FILE* in);

//! Fast binary serialization of a dynvec of segment parts.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out   		The FILE pointer of output container.
//! \param [in]   input 		The dynvec to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_segment_part_serialize(FILE* out, Rox_DynVec_Segment_Part input);

//! Fast binary deserializationof a dynvec of segment parts.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]	output   The dynvec to deserialize
//! \param [in]		input    The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_segment_part_deserialize(Rox_DynVec_Segment_Part output, FILE* input);

//! Fast binary serialization of a dynvec of point2d of sshort.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out   		The FILE pointer of output container.
//! \param [in]   input 		The dynvec to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_point2d_sshort_serialize(FILE* out, Rox_DynVec_Point2D_Sshort input);

//! Fast binary deserialization of a dynvec of point2d of sshort.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The dynvec to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_point2d_sshort_deserialize(Rox_DynVec_Point2D_Sshort output, FILE* in);

//! Fast binary serialization of a dynvec of rect of int.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out   The FILE pointer of output container.
//! \param [in]   input The dynvec of point2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_rect_sint_serialize(FILE* out, Rox_DynVec_Rect_Sint input);

//! Fast binary deserialization  of a dynvec of rect of int.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The dynvec to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_rect_sint_deserialize(Rox_DynVec_Rect_Sint output, FILE* in);

//! Fast binary serialization of a dynvec of fpsm features.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out   The FILE pointer of output container.
//! \param [in]   input The dynvec of fpsm features to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_fpsm_feature_serialize(FILE* out, Rox_DynVec_Fpsm_Feature input);

//! Fast binary deserialization of a dynvec of fpsm features.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The dynvec of fpsm features to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.

ROX_API Rox_ErrorCode rox_dynvec_fpsm_feature_deserialize(Rox_DynVec_Fpsm_Feature output, FILE* in);

//! Fast binary serialization of a dynvec of fpsm templates.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out   The FILE pointer of output container.
//! \param [in]   input The dynvec to serialize
//! \return An error code.

ROX_API Rox_ErrorCode rox_dynvec_fpsm_template_serialize(FILE* out, Rox_DynVec_Fpsm_Template input);

//! Fast binary deserialization of a dynvec of fpsm templates.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The dynvec to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.

ROX_API Rox_ErrorCode rox_dynvec_fpsm_template_deserialize(Rox_DynVec_Fpsm_Template output, FILE* in);

ROX_API Rox_ErrorCode rox_dynvec_point3d_double_serialize(FILE * out, Rox_DynVec_Point3D_Double input);
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_deserialize(Rox_DynVec_Point3D_Double output, FILE* in);

#endif   //__OPENROX_DYNVEC_SERIALIZE__
