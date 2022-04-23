//==============================================================================
//
//    OPENROX   : File sraid_match.h
//
//    Contents  : API of sraid_match module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SRAID_MATCH__
#define __OPENROX_SRAID_MATCH__

#include "sraiddesc.h"

//! \addtogroup SRAID
//! @{

//! Compute the SSD of two 128-vectors
//! \param  [in ]  feat1     first feature descriptor (vector)
//! \param  [in ]  feat2     second feature descriptor (vector)
//! \return the score
//! \todo   To be tested
ROX_API Rox_Uint rox_sraid_match ( Rox_Ushort * feat1, Rox_Ushort * feat2 );

//! @} 

#endif
