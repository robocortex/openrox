//==============================================================================
//
//    OPENROX   : File interaction_row_point_to_line_matse3.c
//
//    Contents  : Implementation of interaction_row_point_to_line_matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "interaction_row_point_to_line_matse3.h"

#include <float.h>
#include <math.h>

#include <baseproc/geometry/line/line3d_struct.h>
#include <baseproc/geometry/line/line2d_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>


Rox_ErrorCode rox_line3d_get_interaction_matrix_point_to_line_parameters ( Rox_Double * lambda_r, Rox_Double * lambda_t, Rox_Line3D_Planes line3d ) 
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !lambda_r || !lambda_t )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !line3d )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the 3D line parameters in meters
   Rox_Double a1 = line3d->planes[0].a;
   Rox_Double b1 = line3d->planes[0].b;
   Rox_Double c1 = line3d->planes[0].c;
   Rox_Double d1 = line3d->planes[0].d;

   Rox_Double a2 = line3d->planes[1].a;
   Rox_Double b2 = line3d->planes[1].b;
   Rox_Double c2 = line3d->planes[1].c;
   Rox_Double d2 = line3d->planes[1].d;
   
   Rox_Double a = a1*d2-a2*d1;
   Rox_Double b = b1*d2-b2*d1;
   Rox_Double s = a*a + b*b;

   *lambda_t = (a2*b1-a1*b2) / sqrt(s);
   *lambda_r = ((a1*a1+b1*b1)*c2*d2+(a2*a2+b2*b2)*c1*d1-(a1*a2+b1*b2)*(c1*d2+c2*d1)) / s;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ansi_interaction_rho_theta_matse3 (
   double * Lr_data, // The interaction matrix d rho   / dt = Lr * v 
   double * Lt_data, // The interaction matrix d theta / dt = Lt * v 
   double   lr,      // The parameter lambda_rho computed from the 3D line
   double   lt,      // The parameter lambda_theta computed from the 3D line
   double   rho,     // The parameter rho defining the model 2D line in camera frame
   double   ct,      // The parameter cos(theta) defining the model 2D line in camera frame
   double   st       // The parameter cos(theta) defining the model 2D line in camera frame
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   double lk = (1.0 + rho * rho);

   Lr_data[0] = ct * lr;
   Lr_data[1] = st * lr;
   Lr_data[2] = - rho * lr;
   Lr_data[3] =   st * lk;
   Lr_data[4] = - ct * lk;
   Lr_data[5] = 0.0;

   Lt_data[0] = ct * lt;
   Lt_data[1] = st * lt;
   Lt_data[2] = - rho * lt;
   Lt_data[3] = - rho * ct;
   Lt_data[4] = - rho * st;
   Lt_data[5] = - 1.0;

   return error;
}

Rox_ErrorCode rox_ansi_interaction_row_point_to_line_matse3 (
   double * L_row_data, 
   double   lr,         // The parameter lambda_rho computed from the 3D line
   double   lt,         // The parameter lambda_theta computed from the 3D line
   double   rho,        // Rho parameter defining the model 2D line in camera frame
   double   theta,      // Theta parameter defining the model 2D line in camera frame
   double   x,          // The x coordinates (in meters) of the measured point
   double   y           // The y coordinates (in meters) of the measured point
)
{
   Rox_ErrorCode error = 0;

   double st = sin ( theta );
   double ct = cos ( theta );
   
   double Lr[6];
   double Lt[6];

   error = rox_ansi_interaction_rho_theta_matse3 ( Lr, Lt, lr, lt, rho, ct, st );

   double la = x * st - y * ct; // lambda_alpha

   for ( int col = 0; col < 6; col++ )
   {
      L_row_data[col] = Lr[col] + la * Lt[col];
   }

   return error;
}

Rox_ErrorCode rox_interaction_row_point_to_line_matse3 (
   Rox_Matrix L_row, 
   const Rox_Line3D_Planes line3D, 
   const Rox_Line2D_Normal line2D, 
   const Rox_Double x, 
   const Rox_Double y
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !L_row )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !line3D || !line2D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the 3D line parameters in meters
   Rox_Double lambda_r = 0.0;
   Rox_Double lambda_t = 0.0;
   error = rox_line3d_get_interaction_matrix_point_to_line_parameters ( &lambda_r, &lambda_t, line3D );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the 2D line parameters in meters
   Rox_Double rho   = line2D->rho;
   Rox_Double theta = line2D->theta;

   Rox_Double ** L_row_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &L_row_data, L_row );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_interaction_row_point_to_line_matse3 ( L_row_data[0], lambda_r, lambda_t, rho, theta, x, y );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
