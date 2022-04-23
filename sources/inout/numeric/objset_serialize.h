//============================================================================
//
//    OPENROX   : File objset_serialize.h
//
//    Contents  :
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_OBJSET_SERIALIZE__
#define __OPENROX_OBJSET_SERIALIZE__

#include <generated/objset_sdwm_object.h>
#include <stdio.h>

//!  Fast binary serialization of an objset of sdwm objects.
//!  The output is NOT portable at ALL. Meant to save on disk buffers.
//! \param [out]  out   The FILE pointer of output container.
//! \param [in]   input The dynvec of sdwm objects to serialize
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_sdwm_object_serialize(FILE* out, Rox_ObjSet_Sdwm_Object input);

//! Fast binary deserialization of an objset of sdwm objects.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     output   The dynvec of sdwm objects to deserialize
//! \param [in]      in       The FILE pointer of input container.
//! \return An error code.

ROX_API Rox_ErrorCode rox_dynvec_sdwm_object_deserialize(Rox_ObjSet_Sdwm_Object output, FILE* in);

#endif // __OPENROX_OBJSET_SERIALIZE__
