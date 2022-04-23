//============================================================================
//
//    OPENROX   : File sdwm_object_serialize.h
//
//    Contents  : API of sdwm object serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_SDWM_OBJECT_SERIALIZE__
#define __OPENROX_SDWM_OBJECT_SERIALIZE__

#include <core/features/detectors/shape/sdwm_object.h>
#include <stdio.h>

//! Fast binary serialization of a sdwn object.
//! \warning :  - The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]   out   The FILE pointer of output container.
//! \param [in]    input The sdwn object to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_sdwm_object_serialize(FILE* out, Rox_Sdwm_Object input);

//! Fast binary deserialization of  a sdwn object.
//! \warning :  - The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]   output   The sdwn object to deserialize
//! \param [in]    in       The FILE pointer of input container.
//! \return An error code.
ROX_API Rox_ErrorCode rox_sdwn_object_deserialize(Rox_Sdwm_Object output, FILE* in);

#endif // __OPENROX_SDWM_OBJECT_SERIALIZE__
