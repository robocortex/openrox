//==============================================================================
//
//    OPENROX   : File ansi_lotinverse.h
//
//    Contents  : API of ansi lotinverse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LOT_INVERSE_ANSI__
#define __OPENROX_LOT_INVERSE_ANSI__

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>

//! \ingroup Matrix
//! \addtogroup Inverse
//! @{

//! Compute the inverse of a symmetric positive definite matrix
//! \param  [out]  dest           The inversed matrix
//! \param  [in ]  input          The symmetric positive definite matrix to inverse
//! \return An error code
ROX_API int rox_ansi_array_float_lotinverse ( float * Li_data, const float * L_data, int n );

//! @}

#endif
