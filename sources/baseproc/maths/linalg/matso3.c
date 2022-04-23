//============================================================================
//
//    OPENROX   : File matso3.c
//
//    Contents  : Implementation of matso3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "matso3.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/logarithm/logmat.h>
#include <baseproc/maths/linalg/generators/algso3.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/transpose/transpose.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/array/solve/svd_solve.h>

#include <inout/numeric/array2d_save.h>
#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_matso3_new(Rox_MatSO3 * R)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_MatSO3 ret = NULL;

   if (!R) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
 
   *R = NULL;
   
   // A SO3 matrix is a 3 x 3 matrix of doubles
   error = rox_array2d_double_new(&ret, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   // A SO3 matrix is set to the Identity by default
   error = rox_array2d_double_fillunit(ret);
   ROX_ERROR_CHECK_TERMINATE(error)

   *R = ret;
   
function_terminate:
   if(error) rox_array2d_double_del(&ret);
   return error;
}

Rox_ErrorCode rox_matso3_del(Rox_MatSO3 * R)
{
   return rox_array2d_double_del(R);
}

Rox_ErrorCode rox_matso3_set_unit(Rox_MatSO3 R)
{
    return rox_array2d_double_fillunit(R);
}

Rox_ErrorCode rox_matso3_mulmatmat(Rox_MatSO3 R, const Rox_MatSO3 R1, const Rox_MatSO3 R2)
{
    return rox_array2d_double_mulmatmat(R, R1, R2);
}

Rox_ErrorCode rox_matso3_copy ( Rox_MatSO3 result, const Rox_MatSO3 input )
{
    return rox_array2d_double_copy( result, input );
}

Rox_ErrorCode rox_matso3_inv ( Rox_MatSO3 result, const Rox_MatSO3 input )
{
    return rox_array2d_double_transpose(result, input);
}

Rox_ErrorCode rox_matso3_set_rot ( Rox_MatSO3 R, const Rox_Double rot[3] )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!R || !rot) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_expmat_so3_vec ( R, rot );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_matso3_set_data(Rox_MatSO3 R, const Rox_Double data[9])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!R || !data)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** R_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &R_data, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   R_data[0][0] = data[0]; R_data[0][1] = data[1]; R_data[0][2] = data[2]; 
   R_data[1][0] = data[3]; R_data[1][1] = data[4]; R_data[1][2] = data[5];  
   R_data[2][0] = data[6]; R_data[2][1] = data[7]; R_data[2][2] = data[8];  

function_terminate:
   return error;
}

Rox_ErrorCode rox_matso3_get_data(Rox_Double data[9], const Rox_MatSO3 R)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!R || !data) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double **  R_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &R_data, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   data[0] = R_data[0][0]; data[1] = R_data[0][1]; data[2] = R_data[0][2];  
   data[3] = R_data[1][0]; data[4] = R_data[1][1]; data[5] = R_data[1][2];  
   data[6] = R_data[2][0]; data[7] = R_data[2][1]; data[8] = R_data[2][2];

function_terminate:
   return error;
}

Rox_ErrorCode rox_matso3_mulinvmat(Rox_MatSO3 result, const Rox_MatSO3 input_1, const Rox_MatSO3 input_2)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
    
    Rox_MatSO3 inv_input_1 = NULL;
  
    error = rox_matso3_new(&inv_input_1); 
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_matso3_inv(inv_input_1, input_1);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_double_mulmatmat(result, inv_input_1, input_2);
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

    ROX_ERROR_CHECK( rox_matso3_del(&inv_input_1) )

    return error;
}

Rox_ErrorCode rox_matso3_error(Rox_Double * err_rot, const Rox_MatSO3 input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double axis_x = 0.0;
   Rox_Double axis_y = 0.0;
   Rox_Double axis_z = 1.0;
   Rox_Double angle  = 0.0;
   
   error = rox_array2d_double_logmat(&axis_x, &axis_y, &axis_z, &angle, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *err_rot = fabs(angle);

function_terminate:

   return error;
}

Rox_ErrorCode rox_matso3_distance(Rox_Double * err_rot, const Rox_MatSO3 R1, const Rox_MatSO3 R2)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 R = NULL;

   error = rox_matso3_new(&R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matso3_mulinvmat(R, R1, R2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matso3_error(err_rot, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   ROX_ERROR_CHECK( rox_matso3_del(&R) );
   return error;
}

Rox_ErrorCode rox_matso3_print ( const Rox_MatSO3 R )
{
    return rox_array2d_double_print_precision ( R, 16 );
}

Rox_ErrorCode rox_matso3_write ( FILE * output_file, const Rox_MatSO3 input )
{
   return rox_array2d_double_fprint(output_file, input);
}

Rox_ErrorCode rox_matso3_update_right ( Rox_MatSO3 matso3, const Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algso3 = NULL, update = NULL;

   error = rox_array2d_double_new(&algso3, 3,3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_linalg_so3generator(algso3, vector); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&update, 3,3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_expmat_so3(update, algso3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // We use algso3 as a temporary storage matrix
   
   error = rox_array2d_double_mulmatmat(algso3, matso3, update); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(matso3, algso3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   ROX_ERROR_CHECK( rox_array2d_double_del(&algso3));
   ROX_ERROR_CHECK( rox_array2d_double_del(&update));

   return error;
}

Rox_ErrorCode rox_matso3_update_left(Rox_MatSO3 matso3, const Rox_Array2D_Double vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double algso3 = NULL, update = NULL;

   error = rox_array2d_double_new(&algso3, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&update, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_linalg_so3generator(algso3, vector); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_expmat_so3(update, algso3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(algso3, update, matso3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(matso3, algso3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   ROX_ERROR_CHECK( rox_array2d_double_del(&algso3));
   ROX_ERROR_CHECK( rox_array2d_double_del(&update));

   return error;
}

Rox_ErrorCode rox_matso3_get_data_change_coordinate_system (
   Rox_Double rotation_out_data[16],
   const Rox_MatSO3 rotation_inp,
   const Rox_3D_Coordinate_System coordinate_system
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 rotation_out = NULL;

   if ( !rotation_inp || !rotation_out_data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matso3_new ( &rotation_out );
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
         error = rox_matso3_copy ( rotation_out, rotation_inp );
         break;
      }
      case Rox_RightHanded_XRight_YUp:
      {
         // We must change the coordinate system : cTo_XRight_YUp = inv(Rx_180) * cTo_XRight_YDown * Rx_180
         Rox_MatSE3 Rx_Pi = NULL;
         Rox_Double r[3] = { ROX_PI, 0.0, 0.0 };

         error = rox_matso3_new ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matso3_set_rot ( Rx_Pi, r );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matso3_change_coordinate_system ( rotation_out, Rx_Pi, rotation_inp );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matso3_del ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         break;
      }
   }

   error = rox_matso3_get_data ( rotation_out_data, rotation_out );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matso3_del ( &rotation_out );

   return error;
}

Rox_ErrorCode rox_matso3_set_data_change_coordinate_system (
   Rox_MatSO3 rotation_out,
   const Rox_Double rotation_inp_data[16],
   const Rox_3D_Coordinate_System coordinate_system
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSO3 rotation_inp = NULL;

   if ( !rotation_out || !rotation_inp_data )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matso3_new ( &rotation_inp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matso3_set_data ( rotation_inp, rotation_inp_data );
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
         error = rox_matso3_copy ( rotation_out, rotation_inp );
         break;
      }
      case Rox_RightHanded_XRight_YUp:
      {
         // We must change the coordinate system : cRo_XRight_YDown = Rx_180 * cRo_XRight_YUp * inv(Rx_180)
         Rox_MatSE3 Rx_Pi = NULL;
         Rox_Double r[3] = { ROX_PI, 0.0, 0.0 };

         error = rox_matso3_new ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matso3_set_rot ( Rx_Pi, r );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matso3_change_coordinate_system ( rotation_out, Rx_Pi, rotation_inp );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matso3_del ( &Rx_Pi );
         ROX_ERROR_CHECK_TERMINATE ( error );

         break;
      }
   }

function_terminate:
   rox_matso3_del ( &rotation_inp );

   return error;
}

ROX_API Rox_ErrorCode rox_matso3_change_coordinate_system ( 
   Rox_MatSO3 matso3_2, 
   const Rox_MatSO3 matso3_12, 
   const Rox_MatSO3 matso3_1 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 matso3_21 = NULL;

   error = rox_matso3_new ( &matso3_21 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matso3_inv ( matso3_21, matso3_12 );
   ROX_ERROR_CHECK_TERMINATE( error );

   // matso3_2 = matso3_21 * matso3_1
   error = rox_matso3_mulmatmat ( matso3_2, matso3_21, matso3_1 );
   ROX_ERROR_CHECK_TERMINATE( error );

   // Use matso3_21 as temporary matrix : matso3_21 = matso3_21 * matso3_1 * matso3_12
   error = rox_matso3_mulmatmat ( matso3_21, matso3_2, matso3_12 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matso3_copy ( matso3_2, matso3_21 );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   rox_matso3_del ( &matso3_21 );
   return error;
}

ROX_API Rox_ErrorCode rox_matso3_get_axis_sin_angle(Rox_Array2D_Double axis_sin_angle, const Rox_MatSO3 R)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!axis_sin_angle || !R) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double **  R_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &R_data, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **  axis_sin_angle_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &axis_sin_angle_data, axis_sin_angle);
   ROX_ERROR_CHECK_TERMINATE ( error );

   axis_sin_angle_data[0][0] = 0.5*(R_data[2][1] - R_data[1][2]);
   axis_sin_angle_data[1][0] = 0.5*(R_data[0][2] - R_data[2][0]);
   axis_sin_angle_data[2][0] = 0.5*(R_data[1][0] - R_data[0][1]);

function_terminate:
   return error;
}

Rox_ErrorCode rox_matso3_from_vector_gibbs(Rox_MatSO3 R, const Rox_Array2D_Double cayley)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double A = NULL, Sm = NULL, Sp = NULL, Si = NULL;
   
   // Check the size of the rotation matrix
   error = rox_array2d_double_check_size(R, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check the size of the rotation Cayley vector
   error = rox_array2d_double_check_size(cayley, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create temporary matrices
   error = rox_array2d_double_new(&A, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Sp, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Sm, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Si, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(Sm);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(Sp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // A = skew(cayley)
   error = rox_transformtools_skew_from_vector(A, cayley);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Sp = I + A
   error = rox_array2d_double_add(Sp, Sp, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Sm = I - A
   error = rox_array2d_double_substract(Sm, Sm, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Si = inv(I - A)
   error = rox_array2d_double_mat3x3_inverse(Si, Sm);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // R = inv(I - A)*I + A
   error = rox_array2d_double_mulmatmat(R, Si, Sp);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_array2d_double_del(&Sm);
   rox_array2d_double_del(&Sp);
   rox_array2d_double_del(&Si);
   rox_array2d_double_del(&A);

   return error;
}

Rox_ErrorCode rox_matso3_from_vector_quaternion(Rox_MatSO3 R, const Rox_Array2D_Double quaternion)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   // Check the size of the rotation matrix
   error = rox_array2d_double_check_size(R, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check the size of the rotation quaternion vector
   error = rox_array2d_double_check_size(quaternion, 4, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** qd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &qd, quaternion );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double w = qd[0][0];
   Rox_Double x = qd[1][0];
   Rox_Double y = qd[2][0];
   Rox_Double z = qd[3][0];

   error = rox_transformtools_rotationmatrix_from_quaternion (R, w, x, y, z);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_matso3_set_axis_angle (
   Rox_MatSO3 R, 
   const Rox_Double axisx, 
   const Rox_Double axisy, 
   const Rox_Double axisz, 
   const Rox_Double angle
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !R )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matso3_check_size ( R );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double normaxis = sqrt( axisx * axisx + axisy * axisy + axisz * axisz );
   if (fabs(normaxis - 1.0) > 1e-6)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double cosa = cos(angle);
   Rox_Double sina = sin(angle);

   Rox_Double ** dR = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dR, R );
   ROX_ERROR_CHECK_TERMINATE ( error );

   dR[0][0] = 1.0 - axisz * axisz + axisz * axisz * cosa - axisy * axisy + axisy * axisy * cosa;
   dR[0][1] = -sina * axisz + axisy * axisx - axisy * axisx * cosa;
   dR[0][2] =  sina * axisy + axisz * axisx - axisz * axisx * cosa;
   dR[1][0] =  sina * axisz + axisy * axisx - axisy * axisx * cosa;
   dR[1][1] = 1.0 - axisz * axisz + axisz * axisz * cosa - axisx * axisx + axisx * axisx * cosa;
   dR[1][2] = -sina * axisx + axisz * axisy - axisz * axisy * cosa;
   dR[2][0] = -sina * axisy + axisz * axisx - axisz * axisx * cosa;
   dR[2][1] =  sina * axisx + axisz * axisy - axisz * axisy * cosa;
   dR[2][2] = 1.0 - axisy * axisy + axisy * axisy * cosa - axisx * axisx + axisx * axisx * cosa;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_rotationmatrix_from_quaternion (
   Rox_MatSO3 R, 
   const Rox_Double w, 
   const Rox_Double x, 
   const Rox_Double y, 
   const Rox_Double z
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !R )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size ( R, 3, 3) ;
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Rd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Rd, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rd[0][0] = 1 - 2 * y * y - 2 * z * z ;       
   Rd[0][1] =     2 * x * y - 2 * w * z ;       
   Rd[0][2] =     2 * w * y + 2 * x * z ;
   Rd[1][0] =     2 * w * z + 2 * x * y ;
   Rd[1][1] = 1 - 2 * x * x - 2 * z * z ;
   Rd[1][2] =     2 * y * z - 2 * w * x ;
   Rd[2][0] =     2 * x * z - 2 * w * y ;       
   Rd[2][1] =     2 * w * x + 2 * y * z ; 
   Rd[2][2] = 1 - 2 * x * x - 2 * y * y ;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_rotationmatrix_from_euler_tait_bryan_z_y_x (
  Rox_MatSO3 R,
  const Rox_Double th_z,
  const Rox_Double th_y,
  const Rox_Double th_x 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double cx = cos( th_x );
   Rox_Double cy = cos( th_y );
   Rox_Double cz = cos( th_z );
   Rox_Double sx = sin( th_x );
   Rox_Double sy = sin( th_y );
   Rox_Double sz = sin( th_z );

   if (!R)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_double_get_size( &rows, &cols, R );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( cols < 3 || rows < 3 )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dR = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dR, R );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // See https://en.wikipedia.org/wiki/Euler_angles#Conversion_to_other_orientation_representations
   dR[0][0] = cy*cz; dR[0][1] = cz*sy*sx - cx*sz; dR[0][2] = sz*sx + cz*cx*sy;
   dR[1][0] = cy*sy; dR[1][1] = cz*cx + sz*sy*sx; dR[1][2] = cx*sz*sy - cz*sx;
   dR[2][0] = -sy;   dR[2][1] = cy*sx;            dR[2][2] = cy*cx;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_rotationmatrix_from_euler_tait_bryan_x_y_z (
   Rox_MatSO3 R, 
   const Rox_Double th_x, 
   const Rox_Double th_y, 
   const Rox_Double th_z
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double cx = cos(th_x);
   Rox_Double cy = cos(-th_y);
   Rox_Double cz = cos(-th_z);
   Rox_Double sx = sin(th_x);
   Rox_Double sy = sin(-th_y);
   Rox_Double sz = sin(-th_z);

   if (!R)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_double_get_size(&rows, &cols, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (cols < 3 || rows < 3)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dR = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dR, R );
   ROX_ERROR_CHECK_TERMINATE ( error );

   dR[0][0] =  cy*cz;
   dR[0][1] = -cy*sz;
   dR[0][2] = -sy;
   dR[1][0] = -sx*sy*cz+cx*sz;
   dR[1][1] = sx*sy*sz+cx*cz;
   dR[1][2] = -sx*cy;
   dR[2][0] = cx*sy*cz+sx*sz;
   dR[2][1] = -cx*sy*sz+sx*cz;
   dR[2][2] = cx*cy;

function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_double_expmat_so3(Rox_MatSO3 dest, const Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double th = 0.0;
   Rox_Double th_inv = 0.0;

   Rox_Double ux = 0.0;
   Rox_Double uy = 0.0;
   Rox_Double uz = 0.0;

   if (dest == NULL || input == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(input, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(dest, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dout, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dinp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dinp, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (dout == NULL || dinp == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   th = sqrt(dinp[1][2] * dinp[1][2] + dinp[0][1] * dinp[0][1] + dinp[2][0] * dinp[2][0]);

   // avoid division through 0 
   if (th < DBL_EPSILON)
   {
      ux = dinp[2][1];
      uy = dinp[0][2];
      uz = dinp[1][0];
   }
   else
   {
      th_inv = 1.0 / th;
      ux = dinp[2][1] * th_inv;
      uy = dinp[0][2] * th_inv;
      uz = dinp[1][0] * th_inv;
   }

   dout[0][0] = 1.0 + (1.0 - cos(th)) * (-uz * uz - uy * uy);
   dout[0][1] = -sin(th) * uz + (1.0 - cos(th)) * uy * ux;
   dout[0][2] =  sin(th) * uy + (1.0 - cos(th)) * uz * ux;

   dout[1][0] =  sin(th) * uz + (1.0 - cos(th)) * uy * ux;
   dout[1][1] = 1.0 + (1.0 - cos(th)) * (-uz * uz - ux * ux);
   dout[1][2] = -sin(th) * ux + (1.0 - cos(th)) * uz * uy;

   dout[2][0] = -sin(th) * uy + (1.0 - cos(th)) * uz * ux;
   dout[2][1] =  sin(th) * ux + (1.0 - cos(th)) * uz * uy;
   dout[2][2] = 1.0 + (1.0 - cos(th)) * (-uy * uy - ux * ux);

function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_double_expmat_so3_vec(Rox_MatSO3 matso3, const Rox_Double r[3])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double algso3 = NULL;
   Rox_Array2D_Double v = NULL;

   error = rox_array2d_double_new(&algso3, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&v, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   rox_array2d_double_set_value(v, 0, 0, r[0]);
   rox_array2d_double_set_value(v, 1, 0, r[1]);
   rox_array2d_double_set_value(v, 2, 0, r[2]);

   error = rox_transformtools_skew_from_vector ( algso3, v );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_expmat_so3(matso3, algso3);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   ROX_ERROR_CHECK(rox_array2d_double_del(&algso3))
   ROX_ERROR_CHECK(rox_array2d_double_del(&v))
   
   return error;
}


Rox_ErrorCode rox_transformtools_axisangle_from_rotationmatrix (
   Rox_Double * axis_x, 
   Rox_Double * axis_y, 
   Rox_Double * axis_z, 
   Rox_Double * angle, 
   Rox_Array2D_Double rotation
)
{
   return rox_array2d_double_logmat(axis_x, axis_y, axis_z, angle, rotation);
}


Rox_ErrorCode rox_array2d_double_logmat_so3 (
   Rox_Double * axis_x, 
   Rox_Double * axis_y, 
   Rox_Double * axis_z, 
   Rox_Double * angle, 
   Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input || !axis_x || !axis_y || !axis_z || !angle) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *axis_x = 1;
   *axis_y = 0;
   *axis_z = 0;
   *angle = 0;

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_double_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (cols != 3 || rows != 3) 
   {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dR = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dR, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double cos_theta = 0.5 * (dR[0][0] + dR[1][1] + dR[2][2] - 1.0);

   Rox_Double ax = 0.5 * (dR[2][1] - dR[1][2]);
   Rox_Double ay = 0.5 * (dR[0][2] - dR[2][0]);
   Rox_Double az = 0.5 * (dR[1][0] - dR[0][1]);

   Rox_Double sin_theta = sqrt(ax * ax + ay * ay + az * az);

   if (sin_theta > DBL_EPSILON)
   {
      *axis_x = ax / sin_theta;
      *axis_y = ay / sin_theta;
      *axis_z = az / sin_theta;
      *angle = atan2(sin_theta, cos_theta);
   }
   else
   {
      if (cos_theta > 0)
      {
         *axis_x = 0;
         *axis_y = 0;
         *axis_z = 1;
         *angle = atan2(sin_theta, cos_theta);
      }
      else
      {
         // TODO: TO BE COMPUTED FROM S = (R+R')/2 = I + 2 * skew(u)^2 = I + 2 * (u*u' - I) = 2 * u*u' - I
         // u*u' = (S+I)/2
         // (S+I)/2 * u = u
         // ((S+I)/2 - I) * u = 0
         // (S-I) * u = 0
         Rox_Matrix x = NULL;
         Rox_Matrix A = NULL;

         error = rox_matrix_new(&x, 3, 1);
         error = rox_matrix_new(&A, 3, 3);
         
         Rox_Double ** dA = NULL;
         error = rox_array2d_double_get_data_pointer_to_pointer( &dA, A);
         ROX_ERROR_CHECK_TERMINATE ( error );

         dA[0][0] = 0.5*(dR[0][0]+dR[0][0])-1.0; dA[0][1] =     0.5*(dR[0][1]+dR[1][0]); dA[0][2] =     0.5*(dR[0][2]+dR[2][0]);
         dA[1][0] =     0.5*(dR[0][1]+dR[1][0]); dA[1][1] = 0.5*(dR[1][1]+dR[1][1])-1.0; dA[1][2] =     0.5*(dR[1][2]+dR[2][1]); 
         dA[2][0] =     0.5*(dR[0][2]+dR[2][0]); dA[2][1] =     0.5*(dR[1][2]+dR[2][1]); dA[2][2] = 0.5*(dR[2][2]+dR[2][2])-1.0; 

         error = rox_svd_solve_homogeneous_system ( x,  A );
         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_matrix_get_value(axis_x, x, 0, 0);
         rox_matrix_get_value(axis_y, x, 1, 0);
         rox_matrix_get_value(axis_z, x, 2, 0);

         *angle = ROX_PI;
         rox_matrix_del(&x);
         rox_matrix_del(&A);

      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_matso3_check_size ( const Rox_MatSO3 R )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_check_size ( R, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}