//==============================================================================
//
//    OPENROX   : File complementary_filter.c
//
//    Contents  : Implementation of complementary_filter module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "complementary_filter.h"

#include <math.h>

#include <baseproc/array/scale/scale.h>
#include <baseproc/array/normalize/normalize.h>
#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/array/crossprod/crossprod.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_complementary_filter_make_predictions(Rox_MatSO3 p_R_i_pred, Rox_MatSO3 p_R_i_prev, Rox_Array2D_Double wi, Rox_Double dt)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double wi_dt  = NULL;
   
   if(!p_R_i_pred || !p_R_i_prev || !wi) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Prediction from previous estimation and angular velocity
   // p_R_i_pred = p_R_i_prev * expmSO3(wi * dt);
   error = rox_array2d_double_new(&wi_dt, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale(wi_dt, wi, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matso3_copy(p_R_i_pred, p_R_i_prev);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matso3_update_right(p_R_i_pred, wi_dt);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&wi_dt);
   return error;
}

Rox_ErrorCode rox_complementary_filter_make_corrections(Rox_MatSO3 p_R_i_next, Rox_MatSO3 p_R_i_pred, Rox_MatSO3 p_R_i_meas, Rox_Double k, Rox_Bool update)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double cw  = NULL;
   Rox_Array2D_Double axis_sin_angle  = NULL;
   Rox_MatSO3 p_R_i_diff = NULL;
   
   if(!p_R_i_next || !p_R_i_pred || !p_R_i_meas) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_matso3_new(&p_R_i_diff);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Correction from prediction and measured rotation matrix
   // Rotation "difference" matrix
   // p_R_i_diff = p_R_i_pred' * p_R_i_meas;
   error = rox_array2d_double_mulmattransmat(p_R_i_diff, p_R_i_pred, p_R_i_meas);
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_array2d_double_new(&cw,3,1);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_new(&axis_sin_angle, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   // Correction vector in so3
   // axis_sin_angle = skew(Rt-Rt')/2;
   error = rox_matso3_get_axis_sin_angle(axis_sin_angle, p_R_i_diff);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // cw = k * axis_sin_angle 
   error = rox_array2d_double_scale(cw, axis_sin_angle, k);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // p_R_i_next = p_R_i_pred * expmSO3(cw);
   error = rox_matso3_copy(p_R_i_next, p_R_i_pred);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   if(update)
   {
      error = rox_matso3_update_right(p_R_i_next, cw);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   
function_terminate:
   rox_array2d_double_del(&cw);
   rox_array2d_double_del(&axis_sin_angle);
   rox_matso3_del(&p_R_i_diff);

   return error;
}

Rox_ErrorCode rox_matso3_from_accelerometer_heading(Rox_MatSO3 p_R_i_meas, Rox_Array2D_Double fi, Rox_Double hi)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double r = NULL, gc = NULL, gr = NULL, fi_corr = NULL, mi_corr = NULL;
   Rox_Array2D_Double Rxy = NULL, p_R_i_corr = NULL; 
   Rox_Double gr_data[3] = {0.0,0.0,1.0};
   Rox_Double fi_corr_data[3] = {0.0,0.0,-1.0};

   if(!p_R_i_meas || !fi) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_new_frombuffer(&fi_corr, 3, 1, fi_corr_data);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&mi_corr,3,1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matso3_new(&p_R_i_corr);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matso3_new(&Rxy);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Create the rotation Cayley vector
   error = rox_array2d_double_new(&r, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Create the current gravity vector
   error = rox_array2d_double_new(&gc, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   // Create the reference gravity vector
   error = rox_array2d_double_new_frombuffer(&gr, 3, 1, gr_data);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Set gc as the current gravity 
   error = rox_array2d_double_scale(gc, fi, -1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_cayley_xy_from_vectors(r, gc, gr);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matso3_from_vector_gibbs(Rxy, r);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // mc = [sin(hi); cos(hi); 0];
   error = rox_array2d_double_set_value(mi_corr,0,0,sin(hi));
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(mi_corr,1,0,cos(hi));
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(mi_corr,2,0,0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // specific force correction
   // fc = Rxy * fi = [0;0;1];
   
   // computed corrected rotation 
   error = rox_matso3_from_accelerometer_magnetometer(p_R_i_corr, fi_corr, mi_corr);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // p_R_i_meas = p_R_i_corr * Rxy;
   error = rox_matso3_mulmatmat(p_R_i_meas, p_R_i_corr, Rxy);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:   

   rox_array2d_double_del(&fi_corr);
   rox_array2d_double_del(&mi_corr);

   rox_array2d_double_del(&r);
   rox_array2d_double_del(&gc);
   rox_array2d_double_del(&gr);
   
   rox_matso3_del(&p_R_i_corr);
   rox_matso3_del(&Rxy);

   return error;
}

Rox_ErrorCode rox_matso3_from_accelerometer_magnetometer(Rox_MatSO3 p_R_i_meas, Rox_Array2D_Double fi, Rox_Array2D_Double mi)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double gravity = NULL, north = NULL, temp = NULL;
   
   Rox_Array2D_Double xp = NULL, yp = NULL, zp = NULL;
   
   Rox_Double ** xp_data = NULL;
   Rox_Double ** yp_data = NULL;
   Rox_Double ** zp_data = NULL;
   Rox_Double ** Rm_data = NULL;

   error = rox_array2d_double_new(&gravity,3,1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&north,3,1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&temp,3,1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&xp,3,1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&yp,3,1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&zp,3,1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Normalize vectors
   error = rox_array2d_double_normalize(gravity, fi); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set the z-axis
   error = rox_array2d_double_scale(zp, gravity, -1.0);
   
   error = rox_array2d_double_normalize(north, mi); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_crossprod(temp, north, zp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_normalize(xp, temp); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_crossprod(temp, zp, xp);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_normalize(yp, temp); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer(&xp_data, xp);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer(&yp_data, yp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&zp_data, zp);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_get_data_pointer_to_pointer(&Rm_data, p_R_i_meas);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Transpose the matrix
   Rm_data[0][0] = xp_data[0][0]; Rm_data[0][1] = xp_data[1][0]; Rm_data[0][2] = xp_data[2][0]; 
   Rm_data[1][0] = yp_data[0][0]; Rm_data[1][1] = yp_data[1][0]; Rm_data[1][2] = yp_data[2][0]; 
   Rm_data[2][0] = zp_data[0][0]; Rm_data[2][1] = zp_data[1][0]; Rm_data[2][2] = zp_data[2][0]; 

function_terminate:   

   rox_array2d_double_del(&gravity);
   rox_array2d_double_del(&north);
   rox_array2d_double_del(&temp);
   rox_array2d_double_del(&xp);
   rox_array2d_double_del(&yp);
   rox_array2d_double_del(&zp);
   
   return error;
}