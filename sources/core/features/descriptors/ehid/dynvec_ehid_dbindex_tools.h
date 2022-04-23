//==============================================================================
//
//    OPENROX   : File dynvec_ehid_dbindex.h
//
//    Contents  : API of ehid_dbindex module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DYNVEC_EHID_DBINDEX_TOOLS__
#define __OPENROX_DYNVEC_EHID_DBINDEX_TOOLS__

#include <generated/dynvec_ehid_dbindex.h>

//! \addtogroup EHID
//! @{

//! Load a list of feature points indices from a file in ascii.
//! \param [in] output the feature indices list to load into
//! \param [in] filename the input file path
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_dynvec_ehid_dbindex_load(Rox_DynVec_Ehid_DbIndex output, const char * filename);

//! Save a list of feature points indices to a file in ascii.
//! \param [in] filename the output file path
//! \param [in] input the feature indices list to save
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_dynvec_ehid_dbindex_save(const char * filename, Rox_DynVec_Ehid_DbIndex input);

//! @} 

#endif
