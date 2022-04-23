//==============================================================================
//
//    OPENROX   : File vvs_tools.c
//
//    Contents  : Implementation of vvs_tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "vvs_tools.h"
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode conv_dist_pixel_to_normalized (
   Rox_Double * dist_norm, 
   Rox_Double dist_pix, 
   Rox_MatUT3 K
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!K || !dist_norm) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size( K, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dK = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dK, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!dK) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double fu = dK[0][0];
   Rox_Double s  = dK[0][1];
   Rox_Double fv = dK[1][1];

   // dist_norm = dist_pix * 1/sqrt(2) * sqrt( (fv-s)*(fv-s)/(fu*fu*fv*fv) + 1/(fv*fv) );
   *dist_norm = dist_pix * 0.5 * ( 1/fu*(1-s/fv) + 1/fv );

function_terminate:
   return error;
}
