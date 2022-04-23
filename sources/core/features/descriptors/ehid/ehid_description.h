//==============================================================================
//
//    OPENROX   : File ehid_description.h
//
//    Contents  : Opaque object of ehid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_DESCRIPTION__
#define __OPENROX_EHID_DESCRIPTION__

#include <system/memory/datatypes.h>

//! \addtogroup EHID
//!   @{

//! Description for a ehid point. Note that we actually need 5*64 but describe with 6*64 (last 64 being zero) for simd computation
typedef Rox_Int64 Rox_Ehid_Description[6];

//! @}

#endif
