//==============================================================================
//
//    OPENROX   : File array2d_serialize.h
//
//    Contents  : API of array2d_serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ARRAY2D_SERIALIZE__
#define __OPENROX_ARRAY2D_SERIALIZE__

#include <generated/array2d_uint.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <generated/array2d_sshort.h>
#include <generated/array2d_point2d_float.h>
#include <generated/array2d_point2d_sshort.h>
#include <stdio.h>

//! Fast binary serialization of an array2D.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out] filename the path of output container.
//! \param [in] input the array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_uint_serialize_binary(const char * filename, Rox_Array2D_Uint input);

//! Fast binary deserialization of an array2D.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out] output the array2d to deserialize
//! \param [in] filename the path of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_uint_deserialize_binary(Rox_Array2D_Uint output, const char * filename);

//! Fast binary serialization of an array2D.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out] filename the path of output container.
//! \param [in] input the array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_uchar_serialize_binary(const char * filename, Rox_Array2D_Uchar input);

//! Fast binary serialization of an array2D to use as a part of another struct serialization for ex.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]   in    FILE pointer of output container.
//! \param [in]    input The array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_uchar_serialize(FILE* out, Rox_Array2D_Uchar input);

//! Fast binary deserialization of an array2D.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out] output the array2d to deserialize
//! \param [in] filename the path of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_uchar_deserialize_binary(Rox_Array2D_Uchar output, const char * filename);

//!  Fast binary deserialization of an array2D to use as part of another struct deserialization
//!  The input is NOT portable at ALL. Meant to load buffers.
//!  \param [out]     output   The array2d to deserialize
//! \param [in]      in       The file pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_uchar_deserialize(Rox_Array2D_Uchar output, FILE* in);

//! Fast binary serialization of an array2D.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out] filename the path of output container.
//! \param [in] input the array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_float_serialize_binary(const char * filename, Rox_Array2D_Float input);

//! Fast binary deserialization of an array2D.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out] output the array2d to deserialize
//! \param [in] filename the path of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_float_deserialize_binary(Rox_Array2D_Float output, const char * filename);

//! Fast binary serialization of an array2D of float.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  filename The path of output container.
//! \param [in]   input    The array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_float_serialize(FILE* out, Rox_Array2D_Float input);

//! Fast binary deserialization of an array2D of float.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]  output   The array2d to deserialize
//! \param [in]   filename The path of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_float_deserialize(Rox_Array2D_Float output, FILE* in);

//! Fast binary serialization of an array2D of double.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  filename The path of output container.
//! \param [in]   input    The array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_double_serialize(FILE* out, Rox_Array2D_Double input);

//! Fast binary deserialization of an array2D of double.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]  output   The array2d to deserialize
//! \param [in]   filename The path of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_double_deserialize(Rox_Array2D_Double output, FILE* in);

//! Fast binary serialization of an array2D of point2D of float.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  filename The path of output container.
//! \param [in]   input    The array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_point2d_float_serialize(FILE* out, Rox_Array2D_Point2D_Float input);

//! Fast binary deserialization of an array2D of point2D of float.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]  output   The array2d to deserialize
//! \param [in]   filename The path of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_point2d_float_deserialize(Rox_Array2D_Point2D_Float output, FILE* in);

//! Fast binary serialization of an array2D of point2D of sshort.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  filename The path of output container.
//! \param [in]   input    The array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_point2d_sshort_serialize(FILE* out, Rox_Array2D_Point2D_Sshort input);

//! Fast binary deserialization of an array2D of point2D of sshort.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]  output   The array2d to deserialize
//! \param [in]   filename The path of input container.
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_point2d_sshort_deserialize(Rox_Array2D_Point2D_Sshort output, FILE* in);

//! Fast binary serialization of an array2D.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  filename the path of output container.
//! \param [in]   input the array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_sshort_serialize_binary(const char * filename, Rox_Array2D_Sshort input);

//! Fast binary serialization of an array2D to use as a part of another struct serialization for ex.
//! The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]   in    FILE pointer of output container.
//! \param [in]    input The array2d to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_sshort_serialize(FILE* out, Rox_Array2D_Sshort input);

//! Fast binary deserialization of an array2D.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out] output the array2d to deserialize
//! \param [in] filename the path of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_sshort_deserialize_binary(Rox_Array2D_Sshort output, const char * filename);

//! Fast binary deserialization of an array2D.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The array2d to deserialize
//! \param [in]      filename The path of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_array2d_sshort_deserialize(Rox_Array2D_Sshort output, FILE* in);

#endif
