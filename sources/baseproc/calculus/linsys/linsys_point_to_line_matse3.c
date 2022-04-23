//==============================================================================
//
//    OPENROX   : File linsys_point_to_line_matse3.c
//
//    Contents  : Implementation of linsys_point_to_line_matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_point_to_line_matse3.h"

#include <float.h>
#include <math.h>

#include <baseproc/calculus/jacobians/interaction_row_point_to_line_matse3.h>

#include <baseproc/geometry/line/line3d_struct.h>
#include <baseproc/geometry/line/line2d_struct.h>

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/array/fill/fillzero.h>
#include <baseproc/array/symmetrise/symmetrise.h>

#include <inout/system/errors_print.h>
#include <baseproc/calculus/jacobians/interaction_row_point_to_line_matse3.h>

// Compute Lte and lower triangular part of LtL
int rox_ansi_linsys_point_to_line_matse3 (
   double ** LtL_data, 
   double ** Lte_data,
   double    lrho,     // The parameter lambda_rho computed from the 3D line
   double    ltheta,   // The parameter lambda_theta computed from the 3D line
   double    rho,      // The parameter rho defining the model 2D line in camera frame
   double    theta,    // The parameter theta defining the model 2D line in camera frame
   double    x,        // The x coordinates (in meters) of the measured point
   double    y,        // The y coordinates (in meters) of the measured point
   double    e         // The signed distance between the measured  2D point and the model 2D line 
)
{
   int error = 0;
   double L_row_data[6];

   error = rox_ansi_interaction_row_point_to_line_matse3 ( L_row_data, lrho, ltheta, rho, theta, x, y );
   if (error) goto function_terminate;

   // e = x * cos(theta) + y * sin (theta) - rho
   // Fill lower part of the symmetric matrix LtL
   for ( int i = 0; i < 6; i++ )
   {
      for ( int j = 0; j <= i; j++ )
      {
         LtL_data[i][j] += L_row_data[i] * L_row_data[j];
      }
      Lte_data[i][0] += L_row_data[i] * e;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_linsys_point_to_line_matse3 (
   Rox_Matrix LtL, 
   Rox_Matrix Lte,
   const Rox_Line3D_Planes line3D, 
   const Rox_Line2D_Normal line2D, 
   const Rox_Double x, 
   const Rox_Double y,
   const Rox_Double e
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Output check
   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Input check
   if ( !line3D || !line2D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_fillzero ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error  );

   error = rox_array2d_double_fillzero ( Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** LtL_data =  NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &LtL_data, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Lte_data =  NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Lte_data, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the 3D line parameters in meters
   Rox_Double lambda_r = 0.0;
   Rox_Double lambda_t = 0.0;
   error = rox_line3d_get_interaction_matrix_point_to_line_parameters ( &lambda_r, &lambda_t, line3D );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the 2D line parameters in meters
   Rox_Double rho   = line2D->rho;
   Rox_Double theta = line2D->theta;

   // Compute Lte and lower triangular part of LtL
   error = rox_ansi_linsys_point_to_line_matse3 ( LtL_data, Lte_data, lambda_r, lambda_t, rho, theta, x, y, e );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Symmetrise lower triangular input
   error = rox_array2d_double_symmetrise_lower ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
