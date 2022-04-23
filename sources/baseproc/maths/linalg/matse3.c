//==============================================================================
//
//    OPENROX   : File matse3.c
//
//    Contents  : Implementation of matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "matse3.h"
#include "ansi_matse3.h"

// Includes from baseproc layer
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/linalg/generators/algse3.h>
#include <baseproc/maths/linalg/matso3.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/logarithm/logmat.h>
#include <baseproc/array/determinant/detgl3.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point3d_tools.h>
#include <baseproc/geometry/transforms/matse3/matse3_from_points3d_sets.h>

// Includes from inout layer
#include <inout/numeric/array2d_save.h>
#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>


Rox_ErrorCode rox_matse3_new ( Rox_MatSE3 *pose )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSE3 ret = NULL;

   if ( !pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   *pose = NULL;

   // An SE3 matrix is a 4 x 4 matrix of doubles
   error = rox_array2d_double_new( &ret, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE( error );

   // An SE3 matrix is set to the Identity by default
   error = rox_array2d_double_fillunit( ret );
   ROX_ERROR_CHECK_TERMINATE( error );

   *pose = ret;

function_terminate:
   if ( error ) rox_array2d_double_del( &ret );
   return error;
}


Rox_ErrorCode rox_matse3_del( Rox_MatSE3 *pose )
{
   return rox_array2d_double_del( pose );
}


Rox_ErrorCode rox_matse3_set_unit( Rox_MatSE3 pose )
{
    return rox_array2d_double_fillunit( pose );
}

// Old generic multiplication that has been replaced with specific SE3 mutiplication
// Rox_ErrorCode rox_matse3_mulmatmat( Rox_MatSE3 result, Rox_MatSE3 input_1, Rox_MatSE3 input_2 )
// {
//    return rox_array2d_double_mulmatmat( result, input_1, input_2 );
// }

Rox_ErrorCode rox_matse3_mulmatmat ( Rox_MatSE3 result, Rox_MatSE3 input_1, Rox_MatSE3 input_2 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !result || !input_1 || !input_2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double ** T  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T, result  );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Double ** T1 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T1, input_1 );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Double ** T2 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T2, input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

   // Multiplication imposing the constraint T[3][0] = 0; T[3][1] = 0; T[3][2] = 0; T[3][3] = 1;
   T[0][0] = T1[0][0]*T2[0][0] + T1[0][1]*T2[1][0] + T1[0][2]*T2[2][0];    T[0][1] = T1[0][0]*T2[0][1] + T1[0][1]*T2[1][1] + T1[0][2]*T2[2][1];    T[0][2] = T1[0][0]*T2[0][2] + T1[0][1]*T2[1][2] + T1[0][2]*T2[2][2];
   T[1][0] = T1[1][0]*T2[0][0] + T1[1][1]*T2[1][0] + T1[1][2]*T2[2][0];    T[1][1] = T1[1][0]*T2[0][1] + T1[1][1]*T2[1][1] + T1[1][2]*T2[2][1];    T[1][2] = T1[1][0]*T2[0][2] + T1[1][1]*T2[1][2] + T1[1][2]*T2[2][2];
   T[2][0] = T1[2][0]*T2[0][0] + T1[2][1]*T2[1][0] + T1[2][2]*T2[2][0];    T[2][1] = T1[2][0]*T2[0][1] + T1[2][1]*T2[1][1] + T1[2][2]*T2[2][1];    T[2][2] = T1[2][0]*T2[0][2] + T1[2][1]*T2[1][2] + T1[2][2]*T2[2][2];
   T[3][0] = 0;                                                            T[3][1] = 0;                                                            T[3][2] = 0;

   T[0][3] = T1[0][3] + T1[0][0]*T2[0][3] + T1[0][1]*T2[1][3] + T1[0][2]*T2[2][3];
   T[1][3] = T1[1][3] + T1[1][0]*T2[0][3] + T1[1][1]*T2[1][3] + T1[1][2]*T2[2][3];
   T[2][3] = T1[2][3] + T1[2][0]*T2[0][3] + T1[2][1]*T2[1][3] + T1[2][2]*T2[2][3];
   T[3][3] = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_inv ( Rox_MatSE3 pose_inverted, const Rox_MatSE3 pose )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose || !pose_inverted )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   // Check that
   error = rox_array2d_double_check_size( pose, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_check_size( pose_inverted, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Double ** T = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Double ** Ti = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Ti, pose_inverted );
   ROX_ERROR_CHECK_TERMINATE( error );

   Rox_Double tx = T[0][3];
   Rox_Double ty = T[1][3];
   Rox_Double tz = T[2][3];

   // R^-1 = R^T
   Ti[0][0] = T[0][0];
   Ti[0][1] = T[1][0];
   Ti[0][2] = T[2][0];
   Ti[1][0] = T[0][1];
   Ti[1][1] = T[1][1];
   Ti[1][2] = T[2][1];
   Ti[2][0] = T[0][2];
   Ti[2][1] = T[1][2];
   Ti[2][2] = T[2][2];

   // t^-1 = -R^T*t
   Ti[0][3] = -Ti[0][0] * tx - Ti[0][1] * ty - Ti[0][2] * tz;
   Ti[1][3] = -Ti[1][0] * tx - Ti[1][1] * ty - Ti[1][2] * tz;
   Ti[2][3] = -Ti[2][0] * tx - Ti[2][1] * ty - Ti[2][2] * tz;

   // Last row
   Ti[3][0] = 0;
   Ti[3][1] = 0;
   Ti[3][2] = 0;
   Ti[3][3] = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_copy( Rox_MatSE3 result, Rox_MatSE3 input )
{
    return rox_array2d_double_copy( result, input );
}

Rox_ErrorCode rox_matse3_set_matso3_r3 ( Rox_MatSE3 pose, Rox_MatSO3 rot, Rox_Matrix tra )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double R = NULL;
   Rox_Array2D_Double t = NULL;

   error = rox_array2d_double_new_subarray2d( &R, pose, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_copy(R, rot);
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_new_subarray2d( &t, pose, 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_copy(t, tra);
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   rox_array2d_double_del( &R );
   rox_array2d_double_del( &t );

   return error;
}

Rox_ErrorCode rox_matse3_set_translation ( Rox_MatSE3 pose, const Rox_Double tra[3] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ** T = NULL;

   if ( !pose || !tra )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer( &T, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   T[0][3] = tra[0];
   T[1][3] = tra[1];
   T[2][3] = tra[2];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_set_rotation ( Rox_MatSE3 pose, const Rox_Double rot[3] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double R = NULL;

   if ( !pose || !rot )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_new_subarray2d( &R, pose, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_expmat_so3_vec( R, rot );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   rox_array2d_double_del( &R );
   return error;
}

Rox_ErrorCode rox_matse3_set_rot_tra ( Rox_MatSE3 pose, const Rox_Double rot[3], const Rox_Double tra[3] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ** T = NULL;

   // Rox_MatSO3 R = NULL;
   Rox_Array2D_Double R = NULL;

   if ( !pose || !rot || !tra )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer( &T, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   T[0][3] = tra[0];
   T[1][3] = tra[1];
   T[2][3] = tra[2];

   //rox_array2d_double_print( pose );

   error = rox_array2d_double_new_subarray2d( &R, pose, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE( error );

   //rox_array2d_double_print( R );

   error = rox_array2d_double_expmat_so3_vec( R, rot );
   ROX_ERROR_CHECK_TERMINATE( error );

   //rox_array2d_double_print( R );

function_terminate:
   rox_array2d_double_del( &R );
   return error;
}


Rox_ErrorCode rox_matse3_set_axis_angle (
   Rox_MatSE3 T, 
   const Rox_Double axis_x, 
   const Rox_Double axis_y, 
   const Rox_Double axis_z, 
   const Rox_Double angle
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 R = NULL;

   if ( !T )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( T );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d ( &R, T, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matso3_set_axis_angle ( R, axis_x, axis_y, axis_z, angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del ( &R );
   return error;
}


Rox_ErrorCode rox_matse3_set_axis_angle_translation (
   Rox_MatSE3 pose,
   Rox_Double axis_x,
   Rox_Double axis_y,
   Rox_Double axis_z,
   Rox_Double angle,
   Rox_Double tx,
   Rox_Double ty,
   Rox_Double tz
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 rotation = NULL;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&rotation, pose, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matso3_set_axis_angle ( rotation, axis_x, axis_y, axis_z, angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** T = NULL;
   error = rox_matse3_get_data_pointer_to_pointer ( &T, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   T[0][3] = tx;
   T[1][3] = ty;
   T[2][3] = tz;

   T[3][0] = 0;
   T[3][1] = 0;
   T[3][2] = 0;
   T[3][3] = 1;

function_terminate:
   rox_array2d_double_del(&rotation);
   return error;
}

Rox_ErrorCode rox_matse3_get_tra( Rox_Double tra[3], const Rox_MatSE3 pose)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ** T = NULL;

   if ( !pose || !tra )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer( &T, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   tra[0] = T[0][3];
   tra[1] = T[1][3];
   tra[2] = T[2][3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_get_rot_tra( Rox_Double rot[3], Rox_Double tra[3], const Rox_MatSE3 pose)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ** T = NULL;

   // Rox_MatSO3 R = NULL;
   Rox_Array2D_Double R = NULL;

   if ( !pose || !rot || !tra )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer( &T, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   tra[0] = T[0][3];
   tra[1] = T[1][3];
   tra[2] = T[2][3];

   //rox_array2d_double_print( pose );

   error = rox_array2d_double_new_subarray2d( &R, pose, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE( error );

   //rox_array2d_double_print( R );
   Rox_Double axis_x = 0.0, axis_y = 0.0, axis_z = 0.0, angle = 0.0;

   error = rox_array2d_double_logmat_so3(&axis_x, &axis_y, &axis_z, &angle, R);
   ROX_ERROR_CHECK_TERMINATE( error );

   rot[0] = axis_x*angle;
   rot[1] = axis_y*angle;
   rot[2] = axis_z*angle;

   //rox_array2d_double_print( R );

function_terminate:
   rox_array2d_double_del( &R );
   return error;
}


Rox_ErrorCode rox_matse3_set_data ( Rox_MatSE3 pose, const Rox_Double data[16] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose || !data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   dp[0][0] = data[0];  dp[0][1] = data[1];  dp[0][2] = data[2];  dp[0][3] = data[3];
   dp[1][0] = data[4];  dp[1][1] = data[5];  dp[1][2] = data[6];  dp[1][3] = data[7];
   dp[2][0] = data[8];  dp[2][1] = data[9];  dp[2][2] = data[10];  dp[2][3] = data[11];
   // The last row must be [0 0 0 1] anyway
   dp[3][0] = 0.0;  dp[3][1] = 0.0;  dp[3][2] = 0.0;  dp[3][3] = 1.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_set_data_float ( Rox_MatSE3 pose, const Rox_Float data[16] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose || !data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   dp[0][0] = (double) data[0];  dp[0][1] = (double) data[1];  dp[0][2] = (double) data[2];  dp[0][3] = (double) data[3];
   dp[1][0] = (double) data[4];  dp[1][1] = (double) data[5];  dp[1][2] = (double) data[6];  dp[1][3] = (double) data[7];
   dp[2][0] = (double) data[8];  dp[2][1] = (double) data[9];  dp[2][2] = (double) data[10];  dp[2][3] = (double) data[11];
   // The last row must be [0 0 0 1] anyway
   dp[3][0] = 0.0;  dp[3][1] = 0.0;  dp[3][2] = 0.0;  dp[3][3] = 1.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_set_data_transform_coordinate_system (
   Rox_MatSE3 pose_out,
   const Rox_Double pose_inp_data[16],
   const Rox_3D_Coordinate_System coordinate_system
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 pose_inp = NULL;

   if ( !pose_out || !pose_inp_data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matse3_new ( &pose_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_set_data ( pose_inp, pose_inp_data );
   ROX_ERROR_CHECK_TERMINATE ( error );

   switch(coordinate_system)
   {
      default :
      {
         error = -1;
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      case Rox_RightHanded_XRight_YDown:
      {
         // We are already in the good coordinate system
         error = rox_matse3_copy ( pose_out, pose_inp );
         break;
      }
      case Rox_RightHanded_XRight_YUp:
      {
         // We must transform the coordinate system : cTo_XRight_YDown = Rx_180 * cTo_XRight_YUp
         Rox_MatSE3 Rx_Pi = NULL;
         Rox_Double r[3] = { ROX_PI, 0.0, 0.0 };

         error = rox_matse3_new ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_set_rotation ( Rx_Pi, r );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_mulmatmat ( pose_out, Rx_Pi, pose_inp );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_del ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         break;
      }
   }

function_terminate:
   rox_matse3_del ( &pose_inp );

   return error;
}

Rox_ErrorCode rox_matse3_set_data_change_coordinate_system (
   Rox_MatSE3 pose_out,
   const Rox_Double pose_inp_data[16],
   const Rox_3D_Coordinate_System coordinate_system
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 pose_inp = NULL;

   if ( !pose_out || !pose_inp_data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matse3_new ( &pose_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_set_data ( pose_inp, pose_inp_data );
   ROX_ERROR_CHECK_TERMINATE ( error );

   switch(coordinate_system)
   {
      default :
      {
         error = -1;
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      case Rox_RightHanded_XRight_YDown:
      {
         // We are already in the good coordinate system
         error = rox_matse3_copy ( pose_out, pose_inp );
         break;
      }
      case Rox_RightHanded_XRight_YUp:
      {
         // We must change the coordinate system : cTo_XRight_YDown = Rx_180 * cTo_XRight_YUp * inv(Rx_180)
         Rox_MatSE3 Rx_Pi = NULL;
         Rox_Double r[3] = { ROX_PI, 0.0, 0.0 };

         error = rox_matse3_new ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_set_rotation ( Rx_Pi, r );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_change_coordinate_system ( pose_out, Rx_Pi, pose_inp );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_del ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         break;
      }
   }

function_terminate:
   rox_matse3_del ( &pose_inp );

   return error;
}

Rox_ErrorCode rox_matse3_get_data_change_coordinate_system (
   Rox_Double pose_out_data[16],
   const Rox_MatSE3 pose_inp,
   const Rox_3D_Coordinate_System coordinate_system
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 pose_out = NULL;

   if ( !pose_inp || !pose_out_data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matse3_new ( &pose_out );
   ROX_ERROR_CHECK_TERMINATE ( error );

   switch(coordinate_system)
   {
      default :
      {
         error = -1;
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      case Rox_RightHanded_XRight_YDown:
      {
         // You are already in the good coordinate system
         error = rox_matse3_copy ( pose_out, pose_inp );
         break;
      }
      case Rox_RightHanded_XRight_YUp:
      {
         // We must change the coordinate system : cTo_XRight_YUp = inv(Rx_180) * cTo_XRight_YDown * Rx_180
         Rox_MatSE3 Rx_Pi = NULL;
         Rox_Double r[3] = { ROX_PI, 0.0, 0.0 };

         error = rox_matse3_new ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_set_rotation ( Rx_Pi, r );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_change_coordinate_system ( pose_out, Rx_Pi, pose_inp );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_del ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         break;
      }
   }

   error = rox_matse3_get_data ( pose_out_data, pose_out );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matse3_del ( &pose_out );

   return error;
}


Rox_ErrorCode rox_matse3_change_predefined_coordinate_system (
   Rox_MatSE3 pose_out,
   const Rox_MatSE3 pose_inp,
   const Rox_3D_Coordinate_System coordinate_system
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose_out || !pose_inp )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   switch(coordinate_system)
   {
      default :
      {
         error = -1;
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      case Rox_RightHanded_XRight_YDown:
      {
         // You are already in the good coordinate system
         error = rox_matse3_copy ( pose_out, pose_inp );
         break;
      }
      case Rox_RightHanded_XRight_YUp:
      {
         // We must change the coordinate system : cTo_XRight_YUp = inv(Rx_180) * cTo_XRight_YDown
         Rox_MatSE3 Rx_Pi = NULL;
         Rox_Double r[3] = { ROX_PI, 0.0, 0.0 };

         error = rox_matse3_new ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_set_rotation ( Rx_Pi, r );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_change_coordinate_system ( pose_out, Rx_Pi, pose_inp );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_del ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         break;
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_matse3_transform_predefined_coordinate_system (
   Rox_MatSE3 pose_out,
   const Rox_MatSE3 pose_inp,
   const Rox_3D_Coordinate_System coordinate_system
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose_out || !pose_inp )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   switch ( coordinate_system )
   {
      default :
      {
         error = -1;
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      case Rox_RightHanded_XRight_YDown:
      {
         // You are already in the good coordinate system
         error = rox_matse3_copy ( pose_out, pose_inp );
         break;
      }
      case Rox_RightHanded_XRight_YUp:
      {
         // We must change the coordinate system : cTo_XRight_YUp = inv(Rx_180) * cTo_XRight_YDown
         Rox_MatSE3 Rx_Pi = NULL;
         Rox_Double r[3] = { ROX_PI, 0.0, 0.0 };

         error = rox_matse3_new ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_set_rotation ( Rx_Pi, r );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_mulmatmat ( pose_out, Rx_Pi, pose_inp );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_del ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         break;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_get_data_transform_coordinate_system (
   Rox_Double pose_out_data[16],
   const Rox_MatSE3 pose_inp,
   const Rox_3D_Coordinate_System coordinate_system
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 pose_out = NULL;

   if ( !pose_inp || !pose_out_data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matse3_new ( &pose_out );
   ROX_ERROR_CHECK_TERMINATE ( error );

   switch(coordinate_system)
   {
      default :
      {
         error = -1;
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      case Rox_RightHanded_XRight_YDown:
      {
         // You are already in the good coordinate system
         error = rox_matse3_copy ( pose_out, pose_inp );
         break;
      }
      case Rox_RightHanded_XRight_YUp:
      {
         // We must change the coordinate system : cTo_XRight_YUp = inv(Rx_180) * cTo_XRight_YDown
         Rox_MatSE3 Rx_Pi = NULL;
         Rox_Double r[3] = { ROX_PI, 0.0, 0.0 };

         error = rox_matse3_new ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_set_rotation ( Rx_Pi, r );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_mulmatmat ( pose_out, Rx_Pi, pose_inp );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_del ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         break;
      }
   }

   error = rox_matse3_get_data ( pose_out_data, pose_out );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matse3_del ( &pose_out );

   return error;
}

Rox_ErrorCode rox_matse3_get_data ( Rox_Double data[16], const Rox_MatSE3 pose )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose || !data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   data[0]  = dp[0][0]; data[1]  = dp[0][1]; data[2]  = dp[0][2]; data[3]  = dp[0][3];
   data[4]  = dp[1][0]; data[5]  = dp[1][1]; data[6]  = dp[1][2]; data[7]  = dp[1][3];
   data[8]  = dp[2][0]; data[9]  = dp[2][1]; data[10] = dp[2][2]; data[11] = dp[2][3];
   data[12] =      0.0; data[13] =      0.0; data[14] =      0.0; data[15] =      1.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_get_data_float (  Rox_Float data[16], const Rox_MatSE3 pose )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose || !data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   data[0]  = (Rox_Float) dp[0][0]; data[1]  = (Rox_Float) dp[0][1]; data[2]  = (Rox_Float) dp[0][2]; data[3]  = (Rox_Float) dp[0][3];
   data[4]  = (Rox_Float) dp[1][0]; data[5]  = (Rox_Float) dp[1][1]; data[6]  = (Rox_Float) dp[1][2]; data[7]  = (Rox_Float) dp[1][3];
   data[8]  = (Rox_Float) dp[2][0]; data[9]  = (Rox_Float) dp[2][1]; data[10] = (Rox_Float) dp[2][2]; data[11] = (Rox_Float) dp[2][3];
   data[12] = (Rox_Float) dp[3][0]; data[13] = (Rox_Float) dp[3][1]; data[14] = (Rox_Float) dp[3][2]; data[15] = (Rox_Float) dp[3][3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_get_data_pointer_to_pointer ( Rox_Double *** rowsptr, Rox_MatSE3 pose )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose || !rowsptr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer(rowsptr, pose);

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_scale_translation( Rox_MatSE3 pose, Rox_Double scale )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dp, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   dp[0][3] = scale * dp[0][3];
   dp[1][3] = scale * dp[1][3];
   dp[2][3] = scale * dp[2][3];

function_terminate:
  return error;
}


Rox_ErrorCode rox_matse3_mulmatinv ( Rox_MatSE3 result, const Rox_MatSE3 input_1, const Rox_MatSE3 input_2 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSE3 inv_input_2 = NULL;

   if ( !result || !input_1 || !input_2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matse3_new( &inv_input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_inv( inv_input_2, input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_mulmatmat( result, input_1, inv_input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   ROX_ERROR_CHECK( rox_matse3_del( &inv_input_2 ) );
   return error;
}


Rox_ErrorCode rox_matse3_mulinvmat( Rox_MatSE3 result, const Rox_MatSE3 input_1, const Rox_MatSE3 input_2 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSE3 inv_input_1 = NULL;

   if ( !result || !input_1 || !input_2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matse3_new( &inv_input_1 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_inv( inv_input_1, input_1 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_mulmatmat( result, inv_input_1, input_2 );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   ROX_ERROR_CHECK( rox_matse3_del( &inv_input_1 ) );
   return error;
}


Rox_ErrorCode rox_matse3_error( Rox_Double * err_tra, Rox_Double * err_rot, const Rox_MatSE3 input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double axis_x = 0.0;
   Rox_Double axis_y = 0.0;
   Rox_Double axis_z = 1.0;
   Rox_Double angle  = 0.0;

   if ( !err_tra || !err_rot || !input )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_logmat( &axis_x, &axis_y, &axis_z, &angle, input );
   ROX_ERROR_CHECK_TERMINATE( error );

   *err_rot = fabs( angle );

   Rox_Double ** input_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &input_data, input );
   ROX_ERROR_CHECK_TERMINATE( error );

   axis_x = input_data[0][3];
   axis_y = input_data[1][3];
   axis_z = input_data[2][3];

   *err_tra = sqrt( axis_x*axis_x + axis_y*axis_y + axis_z*axis_z );

function_terminate:
   return error;
}


Rox_ErrorCode rox_matse3_distance ( Rox_Double * err_tra, Rox_Double * err_rot, const Rox_MatSE3 T1, const Rox_MatSE3 T2 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 T = NULL;

   if ( !err_tra || !err_rot || !T1 || !T2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matse3_new( &T );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_mulinvmat( T, T1, T2 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_error( err_tra, err_rot, T );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   ROX_ERROR_CHECK( rox_matse3_del( &T ) );
   return error;
}


Rox_ErrorCode rox_matse3_print( Rox_MatSE3 pose )
{
    return rox_array2d_double_print_precision( pose, 16 );
}


Rox_ErrorCode rox_matse3_write( FILE* output_file, Rox_MatSE3 input )
{
   return rox_array2d_double_fprint( output_file, input );
}


Rox_ErrorCode rox_matse3_update_right (
   Rox_MatSE3 pose,
   const Rox_Array2D_Double vector
)
{
   Rox_ErrorCode      error = ROX_ERROR_NONE;
   Rox_Array2D_Double algebra = NULL;
   Rox_MatSE3         update = NULL;

   if ( !pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   if ( !vector )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_new ( &algebra, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_new ( &update );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_algse3_set_velocity ( algebra, vector );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_exponential_algse3 ( update, algebra );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_mulmatmat ( algebra, pose, update );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_copy( pose, algebra );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   ROX_ERROR_CHECK( rox_array2d_double_del( &algebra ) );
   ROX_ERROR_CHECK( rox_array2d_double_del( &update ) );
   return error;
}

Rox_ErrorCode rox_matse3_update_algse3_right (
   Rox_MatSE3 pose,
   const Rox_Matrix algse3
)
{
   Rox_ErrorCode      error = ROX_ERROR_NONE;
   Rox_MatSE3         update = NULL;
   Rox_MatSE3         pose_update = NULL;

   if ( !pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   if ( !algse3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matse3_new ( &update );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_new ( &pose_update );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_exponential_algse3 ( update, algse3 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_mulmatmat ( pose_update, pose, update );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_copy( pose, pose_update );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   ROX_ERROR_CHECK( rox_array2d_double_del( &update ) );
   ROX_ERROR_CHECK( rox_array2d_double_del( &pose_update ) );
   return error;
}

Rox_ErrorCode rox_matse3_update_matse3_right (
   Rox_MatSE3 pose,
   const Rox_MatSE3 matse3
)
{
   Rox_ErrorCode      error = ROX_ERROR_NONE;
   Rox_MatSE3         pose_update = NULL;

   if ( !matse3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   if ( !pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matse3_new ( &pose_update );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_mulmatmat ( pose_update, pose, matse3 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_copy( pose, pose_update );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   ROX_ERROR_CHECK( rox_array2d_double_del( &pose_update ) );
   return error;
}

Rox_ErrorCode rox_matse3_update_left ( Rox_MatSE3 pose, Rox_Array2D_Double vector )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algebra = NULL;
   Rox_MatSE3 update = NULL;

   if ( !pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   if ( !vector )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_array2d_double_new ( &algebra, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_new ( &update );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_algse3_set_velocity ( algebra, vector );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_scale_inplace ( algebra, -1.0 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_exponential_algse3 ( update, algebra );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_mulmatmat( algebra, update, pose );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_copy( pose, algebra );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   ROX_ERROR_CHECK( rox_array2d_double_del( &algebra ) );
   ROX_ERROR_CHECK( rox_matse3_del( &update ) );
   return error;
}

Rox_ErrorCode rox_matse3_change_coordinate_system (
   Rox_MatSE3 matse3_2,
   const Rox_MatSE3 matse3_12,
   const Rox_MatSE3 matse3_1
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 matse3_21 = NULL;

   error = rox_matse3_new ( &matse3_21 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_inv ( matse3_21, matse3_12 );
   ROX_ERROR_CHECK_TERMINATE( error );

   // matse3_2 = matse3_21 * matse3_1
   error = rox_matse3_mulmatmat ( matse3_2, matse3_21, matse3_1 );
   ROX_ERROR_CHECK_TERMINATE( error );

   // Use matse3_21 as temporary matrix : matse3_21 = matse3_21 * matse3_1 * matse3_12
   error = rox_matse3_mulmatmat ( matse3_21, matse3_2, matse3_12 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_copy ( matse3_2, matse3_21 );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   rox_matse3_del ( &matse3_21 );
   return error;
}

Rox_ErrorCode rox_matse3_determinant( Rox_Double * determinant, Rox_MatSE3 pose )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double R = NULL;

   error = rox_array2d_double_new_subarray2d( &R, pose, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_array2d_double_detgl3( determinant, R );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   rox_array2d_double_del(&R);

   return error;
}

Rox_ErrorCode rox_matse3_exponential_algse3 (
   Rox_MatSE3 matse3, 
   const Rox_Matrix algse3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!matse3 || !algse3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(algse3, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(matse3, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** matse3_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &matse3_data, matse3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** algse3_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &algse3_data, algse3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_matse3_exponential_algse3 ( matse3_data, algse3_data );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_matse3_check_size ( Rox_MatSE3 pose )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

ROX_API Rox_ErrorCode rox_algse3_adjoint_matse3 (  
   Rox_Matrix adjoint, 
   const Rox_MatSE3 matse3, 
   const Rox_Matrix algse3 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Matrix algse3_tmp = NULL;
   Rox_MatSE3 matse3_inv = NULL;

   error = rox_matrix_new ( &algse3_tmp, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &matse3_inv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_inv ( matse3_inv, matse3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat ( algse3_tmp, algse3, matse3_inv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat ( adjoint, matse3, algse3_tmp );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_matse3_del(&matse3_inv);
   rox_matrix_del(&algse3_tmp);

   return error;
}
Rox_ErrorCode rox_matse3_from_photoframe_vertices_vector (
  Rox_MatSE3 pTo,
  const Rox_Point3D_Double p_points,
  const Rox_Point3D_Double o_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct mean_pt;
   Rox_Point3D_Double_Struct o_n_pts[4];
   Rox_Point3D_Double_Struct p_n_pts[4];

   // o_points contains the vertices of the reference photoframe.
   // o_n_points will contain the normalized vertices of the reference photoframe.
   // The frame of the first photoframe is also the global frame.

   error = rox_vector_point3d_double_mean ( &mean_pt, o_points, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_vector_point3d_double_center_normalize ( o_n_pts, o_points, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_vector_point3d_double_shift ( o_n_pts, 4, &mean_pt );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // p_points contains the vertices of the current photoframe.
   // p_n_points will contain the normalized vertices of the first photoframe.

   error = rox_vector_point3d_double_mean ( &mean_pt, p_points, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_vector_point3d_double_center_normalize ( p_n_pts, p_points, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_vector_point3d_double_shift ( p_n_pts, 4, &mean_pt );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the pose pTo between the photoframe F_p and the global object frame Fo
   error = rox_matse3_from_vector_points3d_double ( pTo, o_n_pts, p_n_pts, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
