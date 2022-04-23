//============================================================================
//
//    OPENROX   : File pyramid_npot_serialize.h
//
//    Contents  : API of pyramid npot serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_PYRAMID_NPOT_SERIALIZE__
#define __OPENROX_PYRAMID_NPOT_SERIALIZE__

#include <stdio.h>
#include <baseproc/image/pyramid/pyramid_npot_uchar.h>

//! Fast binary serialization of a pyramid struct.
//! \warning :  - The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param  [out] out               The FILE pointer of output container.
//! \param  [in]  input             The sdwn object to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_pyramid_npot_uchar_serialize(FILE* out, Rox_Pyramid_Npot_Uchar input);

//! Fast binary deserialization of  a pyramid struct.
//! \warning :  - The input is NOT portable at ALL. Meant to load buffers.
//! \param  [out] output            The sdwn object to deserialize
//! \param  [in]  in                The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_pyramid_npot_uchar_deserialize(Rox_Pyramid_Npot_Uchar output, FILE* in);

#endif // __OPENROX_PYRAMID_NPOT_SERIALIZE__
