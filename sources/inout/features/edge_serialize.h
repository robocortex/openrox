//============================================================================
//
//    OPENROX   : File edge_serialize.h
//
//    Contents  : API of edge serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_EDGE_SERIALIZE__
#define __OPENROX_EDGE_SERIALIZE__

#include <core/features/detectors/edges/edgepreproc_gray.h>
#include <core/features/detectors/edges/edgedraw.h>
#include <core/features/detectors/edges/edgepostproc_ac.h>
#include <stdio.h>

//! Fast binary serialization of a edgepreproc gray struct.
//! \warning :  - The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]   out   The FILE pointer of output container.
//! \param [in]    input The struct to serialize
//! \return An error code.

ROX_API Rox_ErrorCode rox_edgepreproc_gray_serialize(FILE* out, Rox_EdgePreproc_Gray input);

//! Fast binary deserialization of a edgepreproc gray struct.
//! \warning :  - The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]   output   The struct to deserialize
//! \param [in]    in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_edgepreproc_gray_deserialize(Rox_EdgePreproc_Gray output, FILE* in);


//! Fast binary serialization of a edgedraw struct.
//! \warning :  - The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]   out   The FILE pointer of output container.
//! \param [in]    input The struct to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_edgedraw_serialize(FILE* out, Rox_EdgeDraw input);

//! Fast binary deserialization of a edgedraw struct.
//! \warning :  - The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]   output   The struct to deserialize
//! \param [in]    in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_edgedraw_deserialize(Rox_EdgeDraw output, FILE* in);

//! Fast binary serialization of a edgepostproc ac struct.
//! \warning :  - The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]   out   The FILE pointer of output container.
//! \param [in]    input The struct to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_edgepostproc_ac_serialize(FILE* out, Rox_EdgePostproc_Ac input);

//! Fast binary deserialization of a edgepostproc ac struct.
//! \warning :  - The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]   output   The struct to deserialize
//! \param [in]    in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_edgepostproc_ac_deserialize(Rox_EdgePostproc_Ac output, FILE* in);

#endif //__EDGE_SERIALIZE__
