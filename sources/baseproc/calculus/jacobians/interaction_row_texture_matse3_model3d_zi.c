//==============================================================================
//
//    OPENROX   : File interaction_row_texture_matse3_model3d_zi.c
//
//    Contents  : Implementation of interaction_row_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "interaction_row_texture_matse3_model3d_zi.h"

#include <float.h>
#include <math.h>

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/array/fill/fillzero.h>
#include <baseproc/array/symmetrise/symmetrise.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_interaction_row_texture_matse3_model3d_zi (
   Rox_Double L_row[6], 
   const Rox_Double ur, 
   const Rox_Double vr, 
   const Rox_Double Iu, 
   const Rox_Double Iv, 
   const Rox_Double zir, 
   const Rox_Double ziur, 
   const Rox_Double zivr,
   const Rox_MatUT3 K, 
   const Rox_Matrix tau 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // NB tau = -inv(cRr)*ctr = rtc 
   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&K_data, K);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double fu = K_data[0][0];
   Rox_Double fv = K_data[1][1];
   Rox_Double cu = K_data[0][2];
   Rox_Double cv = K_data[1][2];

   Rox_Double ** tau_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&tau_data, tau);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double tau1 = tau_data[0][0];
   Rox_Double tau2 = tau_data[1][0];
   Rox_Double tau3 = tau_data[2][0];

   Rox_Double xr = (ur-cu)/fu;
   Rox_Double yr = (vr-cv)/fv;
 
   Rox_Double a1 = - tau1 + tau3 * xr;
   Rox_Double a2 = - tau2 + tau3 * yr;
   
   Rox_Double d1 = fu * ziur;
   Rox_Double d2 = fv * zivr;
   
   Rox_Double s0  = 1 - tau3 * zir;

   if ( fabs(s0) < FLT_MIN ) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double s0i = 1/s0;
   
   Rox_Double xw  = (xr - zir * tau1) * s0i;
   Rox_Double yw  = (yr - zir * tau2) * s0i;
   
   Rox_Double s1 = a1 * d1;
   Rox_Double s2 = a2 * d2;

   if ( fabs(s0 + s1 + s2) < FLT_MIN ) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double s  = 1/(s0 + s1 + s2);
   
   Rox_Double Iu_fus = Iu * fu * s ;
   Rox_Double Iv_fvs = Iv * fv * s ;

   Rox_Double Lpi_11 = s0 + s2;
   Rox_Double Lpi_12 =    - a1 * d2;
   Rox_Double Lpi_21 =    - a2 * d1;
   Rox_Double Lpi_22 = s0 + s1;

   Rox_Double j_11 = + 1;
   Rox_Double j_12 =   0;
   Rox_Double j_13 = - xw;

   Rox_Double j_14 = -  yr * xw;
   Rox_Double j_15 = + 1 + xr * xw;
   Rox_Double j_16 = - yr ;
   
   Rox_Double j_21 =   0;
   Rox_Double j_22 = + 1;
   Rox_Double j_23 = - yw;

   Rox_Double j_24 = - 1 - yr * yw;
   Rox_Double j_25 = + xr * yw;
   Rox_Double j_26 = + xr ;

   Rox_Double vx = Iu_fus * Lpi_11 + Iv_fvs * Lpi_21;
   Rox_Double vy = Iu_fus * Lpi_12 + Iv_fvs * Lpi_22; 

   Rox_Double vx_z = vx * zir;
   Rox_Double vy_z = vy * zir;

   L_row[0] = vx_z;
   L_row[1] = vy_z;
   L_row[2] = vx_z * j_13 + vy_z * j_23;
   L_row[3] = vx   * j_14 + vy   * j_24;
   L_row[4] = vx   * j_15 + vy   * j_25;
   L_row[5] = vx   * j_16 + vy   * j_26; 
  
   ROX_UNUSED(j_11);
   ROX_UNUSED(j_12);
   ROX_UNUSED(j_22);
   ROX_UNUSED(j_21);

function_terminate:
   return error;
}
