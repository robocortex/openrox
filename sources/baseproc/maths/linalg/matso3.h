//============================================================================
//
//    OPENROX   : File matso3.h
//
//    Contents  : API of matso3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_MATSO3__
#define __OPENROX_MATSO3__

#include <baseproc/geometry/coordinate_systems.h>
#include <generated/array2d_double.h>
#include <stdio.h>

//! \ingroup Lie_Group
//! \addtogroup MatSO3
//! \brief Matrix Lie Group SO3 : orthonormal matrices of size 3x3 such that R'*R = I
//! @{

//! Define the Rox_MatSO3 object
typedef struct _Rox_Array2D_Double * Rox_MatSO3;


//! Create a Rox_MatSO3 object and initialize it to the identity
//! \param  [out]  R              The object to create
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_new ( 
   Rox_MatSO3 * R 
);


//! Delete a Rox_MatSO3 object
//! \param  [out]  R              The object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_del ( 
   Rox_MatSO3 * R 
);


//! Set a Rox_MatSO3 object to the identity
//! \param  [out]  R              The object to set
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_set_unit ( 
   Rox_MatSO3 R 
);


//! Multiply two Rox_MatSO3 matrices
//! \param  [out]  result         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_mulmatmat ( 
   Rox_MatSO3 result, 
   const Rox_MatSO3 input_1, 
   const Rox_MatSO3 input_2 
);


//! Compute the inverse of a Rox_MatSO3 matrix
//! \param  [out]  result         The inversion result
//! \param  [in ]  input          The Rox_MatSO3 to inverse
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_inv ( 
   Rox_MatSO3 result, 
   const Rox_MatSO3 input
);


//! Copy a Rox_MatSO3 matrix
//! \param  [out]  result         The copied matric
//! \param  [in ]  input          The Rox_MatSO3 to copy
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_copy ( 
   Rox_MatSO3 result, const Rox_MatSO3 input );


//! Display the R on the stdout stream
//! \param  [in ]  R              The object to print
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_print ( 
   const Rox_MatSO3 R 
);


//! Set matrix data organized row by row
//! \param  [out]  R              The object to set
//! \param  [in ]  data           Data organized by rows
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_set_data ( 
   Rox_MatSO3 R, 
   const Rox_Double data[9]
);


//! Get matrix data organized row by row
//! \param  [out]  data           Data organized by rows
//! \param  [in ]  R              The object to copy
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_matso3_get_data ( 
   Rox_Double data[9], 
   const Rox_MatSO3 R 
);


//! Error between two Rox_MatSO3 matrices : result = inv(input_1) * input_2
//! \param  [out]  result         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_mulinvmat ( 
   Rox_MatSO3 result, 
   const Rox_MatSO3 input_1, 
   const Rox_MatSO3 input_2 
);


//! Update the rotation on the right given a 3 rows vector : R = R * expmSO3(skew(vector))
//! \param  [out] matso3          The result AND input 3x3 matrix
//! \param  [in ]  vector         The input 3x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_update_right ( 
   Rox_Array2D_Double matso3, 
   const Rox_Array2D_Double vector 
);


//! Update the rotation on the left given a 3 rows vector  : R = expmSO3(skew(vector)) * R
//! \param  [out]  matso3            The result AND input 3x3 matrix
//! \param  [in ]  vector            The input 3x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_update_left ( 
   Rox_Array2D_Double matso3, 
   const Rox_Array2D_Double vector 
);

//! Change of coordinate system : matso3_2 = inv(matso3_12) * matso3_1 * matso3_12
//! \param  [out]  matso3_2       The matso3 in the new coordinate system
//! \param  [in ]  matso3_12      The matso3 defining the change of coordinate system
//! \param  [in ]  matso3_1       The matso3 in the old coordinate system
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_change_coordinate_system ( 
   Rox_MatSO3 matso3_2, 
   const Rox_MatSO3 matso3_12, 
   const Rox_MatSO3 matso3_1 
);

//! \param  [out]  err_rot
//! \param  [in ]  input
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_error ( 
   Rox_Double * err_rot, 
   const Rox_MatSO3 input 
);


//! Compute the distance between two rotation matrices
//! \param  [out]  err_rot
//! \param  [in ]  R1
//! \param  [in ]  R2
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_distance ( 
   Rox_Double * err_rot, 
   const Rox_MatSO3 R1, 
   const Rox_MatSO3 R2 
);


//! \param  [out]  out
//! \param  [in ]  input
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_write ( 
   FILE* out, 
   const Rox_MatSO3 input 
);


//! \param  [out]  R              Rotation matrix
//! \param  [in ]  r              Vector = axis * angle
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_matso3_set_rot ( 
   Rox_MatSO3 R, 
   const Rox_Double r[3] 
);


//! Build a pose (the top left 3*3 subarray) given a rotation axis/angle
//! \param  [out]  pose          the built matrix (only the top left 3*3 submatrix is changed)
//! \param  [in ]  axis_x        the x component of the axis
//! \param  [in ]  axis_y        the y component of the axis
//! \param  [in ]  axis_z        the z component of the axis
//! \param  [in ]  angle         the angle value
//! \return An error code
//! \todo To be tested, to be renamed "rox_matso3_from_axisangle"
ROX_API Rox_ErrorCode rox_matso3_set_axis_angle (
   Rox_MatSO3 R, 
   const Rox_Double axis_x, 
   const Rox_Double axis_y, 
   const Rox_Double axis_z, 
   const Rox_Double angle
);


//! Build a pose (the top left 3*3 subarray) given a rotation quaternion
//! \param  [out]  pose  The built matrix (only the top left 3*3 submatrix is changed)
//! \param  [in ]  aw    The aw component of the quaternion
//! \param  [in ]  bx    The bx component of the quaternion
//! \param  [in ]  by    The by component of the quaternion
//! \param  [in ]  bz    The bz component of the quaternion
//! \return An error code
//! \todo   To be tested, to be renamed "rox_matso3_from_quaternion"
ROX_API Rox_ErrorCode rox_transformtools_rotationmatrix_from_quaternion ( 
   Rox_Array2D_Double pose, 
   const Rox_Double aw, 
   const Rox_Double bx, 
   const Rox_Double by, 
   const Rox_Double bz
);


//! Build a pose (the top left 3*3 subarray) given the euler angles
//! \param  [out]  pose           The built matrix (only the top left 3*3 submatrix is changed)
//! \param  [in ]  th_x           Rotation angle around x axis (in radians)
//! \param  [in ]  th_y           Rotation angle around y axis (in radians)
//! \param  [in ]  th_z           Rotation angle around z axis (in radians)
//! \return An error code
//! \todo   To be tested, to be renamed "rox_matso3_from_euler_tait_bryan_x_y_z"
ROX_API Rox_ErrorCode rox_transformtools_rotationmatrix_from_euler_tait_bryan_x_y_z (
   Rox_MatSO3 pose, 
   const Rox_Double th_x, 
   const Rox_Double th_y, 
   const Rox_Double th_z
);


//! Build a rotation matrix (the top left 3*3 subarray) given the euler angles
//! \param  [out]  pose           The built matrix (only the top left 3*3 submatrix is changed)
//! \param  [in ]  th_z           Rotation angle around z axis (in radians)
//! \param  [in ]  th_y           Rotation angle around y axis (in radians)
//! \param  [in ]  th_x           Rotation angle around x axis (in radians)
//! \return An error code
//! \todo   To be tested, to be renamed "rox_matso3_from_euler_tait_bryan_x_y_z"
ROX_API Rox_ErrorCode rox_transformtools_rotationmatrix_from_euler_tait_bryan_z_y_x ( 
   Rox_MatSO3 pose,
   const Rox_Double th_z,
   const Rox_Double th_y,
   const Rox_Double th_x 
);


//! Build a rotation matrix from the Gibbs vector (r = u*tan(theta/2))
//! \param  [out]  R              The built rotation matrix (only the top left 3*3 submatrix is changed)
//! \param  [in ]  r              The 3x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_from_vector_gibbs ( 
   Rox_MatSO3 rotation, 
   const Rox_Array2D_Double cailey
);

//! Build a rotation matrix from the Quaternion vector (q = [sin(theta/2); u*cos(theta/2)])
//! \param  [out]  R              The built rotation matrix (only the top left 3*3 submatrix is changed)
//! \param  [in ]  q              The 4x1 quaternion vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_from_vector_quaternion ( 
   Rox_MatSO3 rotation, 
   const Rox_Array2D_Double cailey
);



//! Extract the vector : v = vex(R-R')/2 = axis * sin(angle)
//! \param  [out]  v              Vector = axis * sin(angle)
//! \param  [in ]  R              Rotation matrix
ROX_API Rox_ErrorCode rox_matso3_get_axis_sin_angle ( 
   Rox_Array2D_Double axis_sin_angle, 
   const Rox_MatSO3 R 
);


//! Compute the SO3 matrix exponential of an algso3 matrix
//! \param  [out]  matso3          The exponential matso3 = expmSO3(algso3)
//! \param  [in ]  algso3          The skew matrix in so3  
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_expmat_so3(
   Rox_MatSO3 matso3, 
   const Rox_Array2D_Double algso3
);

//! Compute the SO3 matrix exponential of a rotation vector
//! \param  [out]  matso3          The exponential matso3 = expmSO3(skew(r))
//! \param  [in ]  r               The rotation vector
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_expmat_so3_vec(
   Rox_MatSO3 matso3, 
   const Rox_Double r[3]
);

//! Given a pose (the top left 3*3 subarray), compute a rotation axis/angle
//! \param  [out]  axis_x the x component of the axis
//! \param  [out]  axis_y the y component of the axis
//! \param  [out]  axis_z the z component of the axis
//! \param  [out]  angle the angle value
//! \param  [in ]  rotation matrix
//! \return An error code
//! \todo   To be tested, to be renamed "rox_matso3_get_axisangle"
ROX_API Rox_ErrorCode rox_transformtools_axisangle_from_rotationmatrix (
   Rox_Double * axis_x, 
   Rox_Double * axis_y, 
   Rox_Double * axis_z, 
   Rox_Double * angle, 
   const Rox_MatSO3 rotation
);


ROX_API Rox_ErrorCode rox_matso3_set_data_change_coordinate_system (
   Rox_MatSO3 rotation_out,
   const Rox_Double rotation_inp_data[16],
   const Rox_3D_Coordinate_System coordinate_system
);

ROX_API Rox_ErrorCode rox_matso3_get_data_change_coordinate_system (
   Rox_Double rotation_out_data[16],
   const Rox_MatSO3 rotation_inp,
   const Rox_3D_Coordinate_System coordinate_system
);

//! Check that the size of the input MatSO3 matrix is 3x3
//! \param  [in ]  rotation         The input MatSO3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_check_size ( 
   const Rox_MatSO3 rotation 
);

//! Compute the SO3 matrix logarithm from rotation
//! \param  [out]  axis_x   the x component of the axis
//! \param  [out]  axis_y   the y component of the axis
//! \param  [out]  axis_z   the z component of the axis
//! \param  [out]  angle    the angle value
//! \param  [in ]  input    rotation matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_double_logmat_so3 (
   Rox_Double * axis_x, 
   Rox_Double * axis_y, 
   Rox_Double * axis_z, 
   Rox_Double * angle, 
   Rox_Array2D_Double input
);

//! @}

#endif // __OPENROX_MATSO3__
