//==============================================================================
//
//    OPENROX   : File dynvec_ehid_dbindex_serialization.h
//
//    Contents  : API of dynvec_ehid_dbindex_serialization module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DYNVEC_EHID_DBINDEX_SERIALIZATION__
#define __OPENROX_DYNVEC_EHID_DBINDEX_SERIALIZATION__

#include <generated/dynvec_ehid_dbindex.h>

//! \ingroup EHID
//! \addtogroup Serialization
//! @{

//! Serialize a dynamic vector of Ehid_Dbindex type
//! \param [out] buffer The serialization output
//! \param [in] source The object to serialize
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_dynvec_ehid_dbindex_serialize(Rox_Char *buffer, Rox_DynVec_Ehid_DbIndex source);

//! Fill a dynamic vector of Ehid_Dbindex type from a buffer
//! \param [out] output The object to fill
//! \param [in] buffer The buffer to deserialize
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_dynvec_ehid_dbindex_deserialize(Rox_DynVec_Ehid_DbIndex output, Rox_Char *buffer);

//! Get the octet size of a Ehid_Dbindex dynamic vector
//! \param [out] size The structure size in octets
//! \param [in] source The dynamic vector
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_dynvec_ehid_dbindex_get_structure_size(Rox_Uint *size, Rox_DynVec_Ehid_DbIndex source);

//! @} 

#endif // __OPENROX_DYNVEC_EHID_DBINDEX_SERIALIZATION__
