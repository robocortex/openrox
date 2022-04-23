//==============================================================================
//
//    OPENROX   : File sraid_matchset.h
//
//    Contents  : API of sraid_matchset module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SRAID_MATCHSET__
#define __OPENROX_SRAID_MATCHSET__

#include "sraiddesc.h"
#include "sraid_match.h"
#include <generated/dynvec_sint.h>

//! \addtogroup SRAID
//! @{


//! Get a list of matches for a double set of features.
//! A best match item is defined for each feature of feat1.
//!
//! \param [out] matchassocs : a list of matches (index on feat2, -1 if not matched).
//! \param [in]  feat1       : a reference list of features.
//! \param [in]  feat2       : a second list of features.
//! \return An error code.
ROX_API Rox_ErrorCode rox_sraid_matchset( Rox_DynVec_Sint          matchassocs,
                                  Rox_DynVec_SRAID_Feature feat1,
                                  Rox_DynVec_SRAID_Feature feat2 );

//! Get a list of matches for two sets of features.
//! Matches are exclusives:
//! there are no two features from feat1 matched to a single ( i.e. shared ) feature of feat2
//!
//! \param [out] matchassocs : a list of matches (index on feat2, -1 if not matched).
//! \param [out] matchcount  : the number of valid matches
//! \param [out] matchdists  : the list of matching distances
//! \param [in]  feat1       : the first list of features, to be matched against
//! \param [in]  feat2       : the second list of features
//! \return An error code.
ROX_API Rox_ErrorCode rox_sraid_matchset_exclusive( Rox_DynVec_Sint           matchassocs,
                                            Rox_Sint                 *matchcount,
                                            Rox_DynVec_Sint           matchdists,
                                            Rox_DynVec_SRAID_Feature  feat1,
                                            Rox_DynVec_SRAID_Feature  feat2 );

//! @}


#endif
