//==============================================================================
//
//    OPENROX   : File ansi_meanvar.h
//
//    Contents  : API of ansi meanvar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_MEANVAR__
#define __OPENROX_ANSI_MEANVAR__

#ifdef __cplusplus
extern "C" {
#endif

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>

//! \ingroup Statistics
//! \addtogroup Meanvar
//!   @{

ROX_API int rox_ansi_array_float_meanvar_mask( float * mean, float * variance, float * input_data, unsigned int * mask_data, int size );
ROX_API int rox_ansi_array_float_meanvar( float * mean, float * variance, float * input_data, int size );

//! @} 

#ifdef __cplusplus
}
#endif

#endif
