//==============================================================================
//
//    OPENROX   : File ansi_algse3.h
//
//    Contents  : API of ansi algse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_ALGSE3__
#define __OPENROX_ANSI_ALGSE3__

#include <generated/array2d_double.h>


//! \ingroup  Linalg
//! \defgroup Lie_Algebra Lie Algebra
//! \brief Lie Algebra : vector space together with a non-associative multiplication called "Lie bracket"

//! \ingroup  Lie_Algebra
//! \defgroup se3generator se3generator
//! \brief Lie Algebra generator for SE3 group.

//! \addtogroup se3generator
//! @{

ROX_API int rox_ansi_array_double_algse3_set_velocity ( double * alg_data, double * vec_data );
ROX_API int rox_ansi_array_float_algse3_set_velocity ( float * alg_data, float * vec_data );
ROX_API int rox_ansi_array_float_algse3_check_velocity_convergence ( int * convergence, float *v, const float vt_conv_thresh, const float vr_conv_thresh );
ROX_API int rox_ansi_array2d_double_algse3_set_velocity ( double ** alg_data, double ** vec_data );

//! @}

#endif
