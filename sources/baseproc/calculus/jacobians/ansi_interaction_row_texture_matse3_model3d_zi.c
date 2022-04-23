//==============================================================================
//
//    OPENROX   : File ansi_interaction_row_texture_matse3_model3d_zi.c
//
//    Contents  : Implementation of ansi_interaction_row_texture_matse3_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_interaction_row_texture_matse3_model3d_zi.h"
#include <math.h>
#include <float.h>
#include <inout/system/errors_print.h>
#include <system/errors/errors.h>

Rox_ErrorCode rox_ansi_interaction_row_texture_matse3_model3d_zi (
   double * L_row_data, 
   const double ur, 
   const double vr, 
   const double Iu, 
   const double Iv, 
   const double zir, 
   const double ziur, 
   const double zivr,
   double ** K_data, 
   double ** tau_data 
)
{
   Rox_ErrorCode error = 0;

   double fu = K_data[0][0];
   double fv = K_data[1][1];
   double cu = K_data[0][2];
   double cv = K_data[1][2];

   // NB tau = -inv(cRr)*ctr = rtc 
   double tau1 = tau_data[0][0];
   double tau2 = tau_data[1][0];
   double tau3 = tau_data[2][0];

   double xr = (ur-cu)/fu;
   double yr = (vr-cv)/fv;

   double a1 = - tau1 + tau3 * xr;
   double a2 = - tau2 + tau3 * yr;
   
   double d1 = fu * ziur;
   double d2 = fv * zivr;
   
   double s0  = 1 - tau3 * zir;

   if ( fabs(s0) < FLT_MIN ) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   double s0i = 1/s0;
   
   double xw  = (xr - zir * tau1) * s0i;
   double yw  = (yr - zir * tau2) * s0i;
   
   double s1 = a1 * d1;
   double s2 = a2 * d2;

   if ( fabs(s0 + s1 + s2) < FLT_MIN ) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   double s  = 1/(s0 + s1 + s2);
   
   double Iu_fus = Iu * fu * s ;
   double Iv_fvs = Iv * fv * s ;

   double Lpi_11 = s0 + s2;
   double Lpi_12 =    - a1 * d2;
   double Lpi_21 =    - a2 * d1;
   double Lpi_22 = s0 + s1;

   double j_11 = + 1;
   double j_12 =   0;
   double j_13 = - xw;

   double j_14 = -  yr * xw;
   double j_15 = + 1 + xr * xw;
   double j_16 = - yr ;
   
   double j_21 =   0;
   double j_22 = + 1;
   double j_23 = - yw;

   double j_24 = - 1 - yr * yw;
   double j_25 = + xr * yw;
   double j_26 = + xr ;

   double vx = Iu_fus * Lpi_11 + Iv_fvs * Lpi_21;
   double vy = Iu_fus * Lpi_12 + Iv_fvs * Lpi_22; 

   double vx_z = vx * zir;
   double vy_z = vy * zir;

   L_row_data[0] = vx_z;
   L_row_data[1] = vy_z;
   L_row_data[2] = vx_z * j_13 + vy_z * j_23;
   L_row_data[3] = vx   * j_14 + vy   * j_24;
   L_row_data[4] = vx   * j_15 + vy   * j_25;
   L_row_data[5] = vx   * j_16 + vy   * j_26; 
  
   ROX_UNUSED(j_11);
   ROX_UNUSED(j_12);
   ROX_UNUSED(j_22);
   ROX_UNUSED(j_21);

function_terminate:
   return error;
}
