//============================================================================
//
//    OPENROX   : File fpsm_serialize.h
//
//    Contents  : API of 
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __FPSM_SERIALIZE__
#define __FPSM_SERIALIZE__

#include <core/features/descriptors/fpsm/fpsm.h>
#include <core/features/descriptors/fpsm/fpsm_index.h>
#include <stdio.h>

//! Fast binary serialization of a fpsm struct.
//! \warning :  - The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]   out   The FILE pointer of output container.
//! \param [in]    input The struct to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_fpsm_serialize(FILE* out, Rox_Fpsm input);

//! Fast binary deserialization of  a fpsm struct.
//! \warning :  - The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]   output   The struct to deserialize
//! \param [in]    in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_fpsm_deserialize(Rox_Fpsm output, FILE* in);

//! Fast binary serialization of a fpsm index struct.
//! \warning :  - The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]   out   The FILE pointer of output container.
//! \param [in]    input The struct to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_fpsm_index_serialize(FILE* out, Rox_Fpsm_Index input);

//! Fast binary deserialization of a fpsm index struct.
//! \warning :  - The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]   output   The struct to deserialize
//! \param [in]    in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_fpsm_index_deserialize(Rox_Fpsm_Index output, FILE* in);

#endif   //__FPSM_SERIALIZE__
