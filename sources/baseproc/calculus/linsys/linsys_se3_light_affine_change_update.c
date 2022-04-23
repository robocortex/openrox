//==============================================================================
//
//    OPENROX   : File linsys_se3_light_affine_change_update.c
//
//    Contents  : Implementation of linsys_se3_light_affine_change_update module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_se3_light_affine_change_update.h"

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_linsys_se3_light_affine_change_update_from_right_to_left (
   Rox_Matrix LtL, 
   Rox_Matrix Lte,  
   const Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Matrix buf88 = NULL, buf81 = NULL, postmul= NULL;

   if ( !LtL || !Lte ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( !pose ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Prepare buffers
   buf88 = 0;
   buf81 = 0;
   postmul = 0;
   error = rox_matrix_new ( &buf81, 8, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new ( &buf88, 8, 8 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new (&postmul, 8, 8 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get pose information as required by jacobian
   Rox_Double ** dP = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dP, postmul);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dT = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dT, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double r11 = dT[0][0]; Rox_Double r12 = dT[0][1]; Rox_Double r13 = dT[0][2]; Rox_Double t1 = dT[0][3];
   Rox_Double r21 = dT[1][0]; Rox_Double r22 = dT[1][1]; Rox_Double r23 = dT[1][2]; Rox_Double t2 = dT[1][3];
   Rox_Double r31 = dT[2][0]; Rox_Double r32 = dT[2][1]; Rox_Double r33 = dT[2][2]; Rox_Double t3 = dT[2][3];

   dP[0][0] = -r11; dP[0][1] = -r21; dP[0][2] = -r31; dP[0][3] = -t2 * r31 + t3 * r21; dP[0][4] = t1 * r31 - t3 * r11; dP[0][5] = -t1 * r21 + t2 * r11; dP[0][6] = 0; dP[0][7] = 0;
   dP[1][0] = -r12; dP[1][1] = -r22; dP[1][2] = -r32; dP[1][3] = -t2 * r32 + t3 * r22; dP[1][4] = t1 * r32 - t3 * r12; dP[1][5] = -t1 * r22 + t2 * r12; dP[1][6] = 0; dP[1][7] = 0;
   dP[2][0] = -r13; dP[2][1] = -r23; dP[2][2] = -r33; dP[2][3] = -t2 * r33 + t3 * r23; dP[2][4] = t1 * r33 - t3 * r13; dP[2][5] = -t1 * r23 + t2 * r13; dP[2][6] = 0; dP[2][7] = 0;
   dP[3][0] =   0; dP[3][1] =   0; dP[3][2] =   0; dP[3][3] = -r33 * r22 + r23 * r32; dP[3][4] = r33 * r12 - r13 * r32; dP[3][5] = -r23 * r12 + r13 * r22; dP[3][6] = 0; dP[3][7] = 0;
   dP[4][0] =   0; dP[4][1] =   0; dP[4][2] =   0; dP[4][3] = -r23 * r31 + r33 * r21; dP[4][4] = r13 * r31 - r33 * r11; dP[4][5] = -r13 * r21 + r23 * r11; dP[4][6] = 0; dP[4][7] = 0;
   dP[5][0] =   0; dP[5][1] =   0; dP[5][2] =   0; dP[5][3] = -r32 * r21 + r22 * r31; dP[5][4] = r32 * r11 - r12 * r31; dP[5][5] = -r22 * r11 + r12 * r21; dP[5][6] = 0; dP[5][7] = 0;
   dP[6][0] =   0; dP[6][1] =   0; dP[6][2] =   0; dP[6][3] = 0; dP[6][4] = 0; dP[6][5] = 0; dP[6][6] = 1; dP[6][7] = 0;
   dP[7][0] =   0; dP[7][1] =   0; dP[7][2] =   0; dP[7][3] = 0; dP[7][4] = 0; dP[7][5] = 0; dP[7][6] = 0; dP[7][7] = 1;

   // Transform jacobian to make an update on the left :
   // T(x)*T = T*T^-1*T(x)*T;
   // dT(x)/dx = dA^-1(T*T(y))/dy * dA^-1(T*T^-1*T(x)*T)/dx where A^-1 is from A(x) to x

   error = rox_array2d_double_mulmatmat ( buf88, LtL, postmul); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmattransmat ( LtL, postmul, buf88); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmattransmat ( buf81, postmul, Lte); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy ( Lte, buf81 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matrix_del(&buf81);
   rox_matrix_del(&buf88);
   rox_matrix_del(&postmul);

   return error;
}