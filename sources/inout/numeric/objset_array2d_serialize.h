//============================================================================
//
//    OPENROX   : File objset_array2d_serialize.h
//
//    Contents  : API of array2d_serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OBJSET_ARRAY2D_SERIALIZE__
#define __OBJSET_ARRAY2D_SERIALIZE__

#include <generated/objset_array2d_sshort.h>
#include <generated/objset_array2d_uchar.h>
#include <stdio.h>

//! Fast binary serialization of an objset of array2D.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out] filename the path of output container.
//! \param [in] input the objset of array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_array2d_sshort_serialize_binary(const char * filename, Rox_ObjSet_Array2D_Sshort input);

//! Fast binary serialization of an objset of array2D.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out   The FILE pointer of output container.
//! \param [in]   input The objset of array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_array2d_sshort_serialize(FILE* out, Rox_ObjSet_Array2D_Sshort input);

//! Fast binary deserialization of an objset of array2D.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The objset of array2d to deserialize
//! \param [in]      width    Width of the 2d arrays contained in the output objset
//! \param [in]      height   Height of the 2d arrays contained in the output objset
//! \param [in]      filename The path of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_array2d_sshort_deserialize_binary(Rox_ObjSet_Array2D_Sshort output, Rox_Sint width, Rox_Sint height, const char * filename);

//! Fast binary deserialization of an objset of array2D.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The objset of array2d to deserialize
//! \param [in]      width    Width of the 2d arrays contained in the output objset
//! \param [in]      height   Height of the 2d arrays contained in the output objset
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_array2d_sshort_deserialize(Rox_ObjSet_Array2D_Sshort output, Rox_Sint width, Rox_Sint height, FILE* in);

//! Fast binary serialization of an objset of array2D.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  filename The path of output container.
//! \param [in]   input    The objset of the array2d of Uchar to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_array2d_uchar_serialize_binary(const char * filename, Rox_ObjSet_Array2D_Uchar input);

//! Fast binary serialization of an objset of array2D.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out      The FILE pointer of output container.
//! \param [in]   input    The objset of the array2d of Uchar to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_array2d_uchar_serialize(FILE* out, Rox_ObjSet_Array2D_Uchar input);

//! Fast binary deserialization of an objset of array2D.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The objset of array2d to deserialize
//! \param [in]      width    Width of the 2d arrays contained in the output objset
//! \param [in]      height   Height of the 2d arrays contained in the output objset
//! \param [in]      filename The path of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_array2d_uchar_deserialize_binary(Rox_ObjSet_Array2D_Uchar output, Rox_Sint width, Rox_Sint height, const char * filename);

//! Fast binary deserialization of an objset of array2D.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The objset of array2d to deserialize
//! \param [in]      width    Width of the 2d arrays contained in the output objset
//! \param [in]      height   Height of the 2d arrays contained in the output objset
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_objset_array2d_uchar_deserialize(Rox_ObjSet_Array2D_Uchar output, Rox_Sint width, Rox_Sint height, FILE* in);

#endif   //__OBJSET_ARRAY2D_SERIALIZE__
