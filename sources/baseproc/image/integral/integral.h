//==============================================================================
//
//    OPENROX   : File integral.h
//
//    Contents  : API of image integral module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_CHECKERBOARD__
#define __OPENROX_IDENT_CHECKERBOARD__

#include <generated/array2d_float.h>

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

#include <inout/system/errors_print.h>

//! \addtogroup Image
//! @{

//! 
//! \param  [in ] 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_getValFromIntegral ( 
   Rox_Double * retour, 
   Rox_Double ** integral, 
   const int x, 
   const int y, 
   const int w, 
   const int h 
);

//! 
//! \param  [in ] 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_integral_sqr_double (
   Rox_Matrix I_count_int, 
   Rox_Matrix I_sum_int, 
   Rox_Matrix I_square_int, 
   Rox_Imask I_mask, 
   Rox_Array2D_Float I
);

//! 
//! \param  [in ] 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_integral_sqr_double_nomask (
   Rox_Matrix I_count_int, 
   Rox_Matrix I_sum_int, 
   Rox_Matrix I_square_int, 
   Rox_Array2D_Float I
);

//! @} 

#endif // __OPENROX_IDENT_CHECKERBOARD__
