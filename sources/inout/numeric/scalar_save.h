//==============================================================================
//
//    OPENROX   : File scalar_save.h
//
//    Contents  : API of scalar save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SCALAR_SAVE__
#define __OPENROX_SCALAR_SAVE__

#include <stdio.h>

#include <system/memory/datatypes.h>
#include <system/errors/errors.h>

//! \addtogroup Scalar
//! @{

//! Save a scalar on ascii file
//! \param  [in] filename        the file name
//! \param  [in] input           the array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_double_save ( const Rox_Char * filename, const Rox_Double input );

//! Append a scalar on ascii file
//! \param  [in] filename        the file name
//! \param  [in] input           the array to save
//! \return An error code
ROX_API Rox_ErrorCode rox_double_save_append ( const Rox_Char * filename, const Rox_Double input );

//! @} 

#endif
