//==============================================================================
//
//    OPENROX   : File ehid_computedescriptor.h
//
//    Contents  : API of ehid_computedescriptor module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_COMPUTEDESCRIPTOR__
#define __OPENROX_EHID_COMPUTEDESCRIPTOR__

#include <core/features/descriptors/ehid/ehid.h>

//! \addtogroup EHID
//! @{

//! \param [out] out_mean
//! \param [out] output
//! \param [out] lums
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_point_quantizepatch (
   Rox_Uchar * out_mean, Rox_Uchar output[64], Rox_Double lums[64]);

//! @} 

#endif
