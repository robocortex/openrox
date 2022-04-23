//==============================================================================
//
//    OPENROX   : File objset_dynvec_serialize.h
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

#ifndef __OPENROX_OBJSET_DYNVEC_SERIALIZE__
#define __OPENROX_OBJSET_DYNVEC_SERIALIZE__

#include <generated/objset_dynvec_point2d_sshort.h>
#include <generated/objset_dynvec_fpsm_template.h>
#include <generated/objset_dynvec_sint.h>
#include <generated/objset_dynvec_rect_sint.h>
#include <generated/objset_dynvec_edgel.h>
#include <stdio.h>

//! Fast binary serialization of an objset of dynvec of point2D.
//!  The output is NOT portable at ALL. Meant to save on disk buffers.
//! //! \param [out]  out      The FILE pointer of output container.
//! //! \param [in]   input    The objset of dynvec of point2D to serialize
//! //! \return An error code.
ROX_API Rox_ErrorCode rox_objset_dynvec_point2d_sshort_serialize(FILE* out, Rox_ObjSet_DynVec_Point2D_Sshort input);

//!  Fast binary deserialization of an objset  of dynvec of point2D.
//!  The input is NOT portable at ALL. Meant to load buffers.
//! //! \param [out]     output   The objset of dynvec of point2D
//! //! \param [in]      in       The FILE pointer of input container.
//! //! \return An error code.
ROX_API Rox_ErrorCode rox_objset_dynvec_point2d_sshort_deserialize(Rox_ObjSet_DynVec_Point2D_Sshort output, FILE* in);

//! Fast binary serialization of an objset of dynvec of sint.
//!  The output is NOT portable at ALL. Meant to save on disk buffers.
//! //! \param [out]  out      The FILE pointer of output container.
//! //! \param [in]   input    The objset of dynvec to serialize
//! //! \return An error code.
ROX_API Rox_ErrorCode rox_objset_dynvec_sint_serialize(FILE* out, Rox_ObjSet_DynVec_Sint input);

//!  Fast binary deserialization of an objset  of dynvec of sint.
//!  The input is NOT portable at ALL. Meant to load buffers.
//! //! \param [out]     output   The objset of dynvec to deserialize
//! //! \param [in]      in       The FILE pointer of input container.
//! //! \return An error code.
ROX_API Rox_ErrorCode rox_objset_dynvec_sint_deserialize(Rox_ObjSet_DynVec_Sint output, FILE* in);

//! Fast binary serialization of an objset of dynvec of Rect of Int.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out      The FILE pointer of output container.
//! \param [in]   input    The objset of dynvec of Rect of Int to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_dynvec_rect_sint_serialize(FILE* out, Rox_ObjSet_DynVec_Rect_Sint input);

//! Fast binary deserialization of an objset  of dynvec of Rect of Int.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The objset of dynvec of Rect of Int to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_dynvec_rect_sint_deserialize(Rox_ObjSet_DynVec_Rect_Sint output, FILE* in);

//! Fast binary serialization of an objset of dynvec of edgel.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out      The FILE pointer of output container.
//! \param [in]   input    The objset of dynvec to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_dynvec_edgel_serialize(FILE* out, Rox_ObjSet_DynVec_Edgel input);

//! Fast binary deserialization of an objset of dynvec of edgel.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The objset of dynvec to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_dynvec_edgel_deserialize(Rox_ObjSet_DynVec_Edgel output, FILE* in);

//! Fast binary serialization of an objset of dynvec of fpsm templates.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out      The FILE pointer of output container.
//! \param [in]   input    The objset of dynvec to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_dynvec_fpsm_template_serialize(FILE* out, Rox_ObjSet_DynVec_Fpsm_Template input);

//! Fast binary deserialization of an objset of of fpsm templates.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The objset of dynvec to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_dynvec_fpsm_template_deserialize(Rox_ObjSet_DynVec_Fpsm_Template output, FILE* in);

#endif //__OPENROX_OBJSET_DYNVEC_SERIALIZE__
