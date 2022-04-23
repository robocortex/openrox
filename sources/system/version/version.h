//==============================================================================
//
//    OPENROX   : File version.h
//
//    Contents  : API of version module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_VERSION__
#define __OPENROX_VERSION__

#include <generated/config.h>
#include <system/errors/errors.h>
#include <system/memory/datatypes.h>

//! \defgroup Version Version
//! \brief Version methods.

//! \addtogroup Version
//! @{

//! Get the version of openrox
//! \param  [out] major          Major version
//! \param  [out] minor          Minor version
//! \param  [out] patch          Patch version
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_get_version(Rox_Uint * major, Rox_Uint * minor, Rox_Uint * patch);

//! @}

#endif // __OPENROX_VERSION__
