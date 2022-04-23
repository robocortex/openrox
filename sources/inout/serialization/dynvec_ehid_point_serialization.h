//==============================================================================
//
//    OPENROX   : File dynvec_ehid_point_serialization.h
//
//    Contents  : API of dynvec_ehid_point_serialization module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DYNVEC_EHID_POINT_SERIALIZATION__
#define __OPENROX_DYNVEC_EHID_POINT_SERIALIZATION__

#include <generated/dynvec_ehid_point.h>

//! \ingroup EHID
//! \addtogroup Serialization
//! @{

//! Serialize a dynamic vector of Ehid_Point type
//! \param  [out] buffer The serialization output
//! \param  [in]  source The object to serialize
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_ehid_point_serialize(Rox_Char * buffer, Rox_DynVec_Ehid_Point source);

//! Fill a dynamic vector of Ehid_Point type from a buffer
//! \param  [out] output The object to fill
//! \param  [in]  buffer The buffer to deserialize
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_ehid_point_deserialize(Rox_DynVec_Ehid_Point output, Rox_Char * buffer);

//! Get the octet size of a Ehid_Point dynamic vector
//! \param  [out] size The structure size in octets
//! \param  [in]  source The dynamic vector
//! \return An error code
//! \todo  To be tested
ROX_API Rox_ErrorCode rox_dynvec_ehid_point_get_structure_size(Rox_Uint * size, Rox_DynVec_Ehid_Point source);

ROX_API Rox_ErrorCode rox_dynvec_ehid_point_save(const Rox_Char * filename, Rox_DynVec_Ehid_Point source);
ROX_API Rox_ErrorCode rox_dynvec_ehid_point_print(Rox_DynVec_Ehid_Point source);

//! @} 

#endif // __OPENROX_DYNVEC_EHID_POINT_SERIALIZATION__
