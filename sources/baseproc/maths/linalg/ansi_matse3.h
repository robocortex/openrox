//==============================================================================
//
//    OPENROX   : File ansi_matse3.h
//
//    Contents  : API of matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_MATSE3__
#define __OPENROX_ANSI_MATSE3__

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>

//! \ingroup  Linalg
//! \defgroup Lie_Group

//! \ingroup  Lie_Group
//! \addtogroup MatSE3
//! \brief Matrix Lie Group SO3 x R3
//! @{

ROX_API int rox_ansi_array_double_matse3_exponential_algse3 ( double * matse3_data, double * algse3_data );
ROX_API int rox_ansi_matse3_exponential_algse3 ( double ** matse3_data, double ** algse3_data );

ROX_API int rox_ansi_array_float_matse3_exponential_algse3 ( float * matse3_data, float * algse3_data );
ROX_API int rox_ansi_array_float_matse3_update_left ( float * pose, float * vector );

ROX_API int rox_ansi_matse3_float_generate_moving_camera_centered_at_fixed_distance ( float * oTc_centered, float * oTc, float distance, float rx, float ry, float rz );
ROX_API int rox_ansi_matse3_float_inv ( float * Ti, float * T );
ROX_API int rox_ansi_matse3_float_generate_moving_object_constained_axes ( float * cTo_constrained, float * cTo, float tra[3], float rot[3] );

//! @}

#endif // __OPENROX_MATSE3__
