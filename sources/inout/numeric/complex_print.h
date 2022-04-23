//==============================================================================
//
//    OPENROX   : File complex_print.h
//
//    Contents  : API of complex print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_COMPLEX_PRINT__
#define __OPENROX_COMPLEX_PRINT__

#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>
#include <generated/array2d_float.h>
#include <generated/array2d_double.h>

//! \addtogroup Array2D
//! @{

//! Print a complex number on console
//! \param  [in ]  complex 	    The complex number to print
//! \return An error code
ROX_API Rox_ErrorCode rox_complex_print ( Rox_Complex complex );

//! @} 

#endif
