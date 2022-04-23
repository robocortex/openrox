//==============================================================================
//
//    OPENROX   : File matse3.h
//
//    Contents  : API of matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MATSE3__
#define __OPENROX_MATSE3__

#include <stdio.h>
#include <baseproc/maths/linalg/matso3.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/geometry/coordinate_systems.h>
#include <baseproc/geometry/point/point3d.h>

//! \ingroup  Linalg
//! \defgroup Lie_Group

//! \ingroup  Lie_Group
//! \addtogroup MatSE3
//! \brief Matrix Lie Group SO3 x R3
//! @{

//! Define the Rox_MatSE3 object
typedef struct _Rox_Array2D_Double * Rox_MatSE3;

//! Create a Rox_MatSE3 object and initialize it to the identity
//! \param  [out]  matse3         The object to create
//! \return An error code
ROX_API Rox_ErrorCode rox_matse3_new ( Rox_MatSE3 * matse3);

//! Delete a Rox_MatSE3 object
//! \param  [out]  matse3         The object to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_matse3_del ( Rox_MatSE3 * matse3);

//! Set a Rox_MatSE3 object to the identity
//! \param  [out]  matse3         The object to set
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_set_unit ( Rox_MatSE3 matse3 );

//! Multiply two Rox_MatSE3 matrices
//! \param  [out]  matse3         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_matse3_mulmatmat ( 
   Rox_MatSE3 matse3, 
   const Rox_MatSE3 input_1, 
   const Rox_MatSE3 input_2
);

//! Compute the inverse of a Rox_MatSE3 matrix
//! \param  [out]  matse3_out     The inversion result
//! \param  [in ]  matse3_inp     The Rox_MatSE3 to inverse
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_inv ( 
   Rox_MatSE3 matse3_out, 
   const Rox_MatSE3 matse3_inp 
);

//! Copy a Rox_MatSE3 matrix
//! \param  [out]  matse3         The copy result
//! \param  [in ]  input          The Rox_MatSE3 to copy
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_copy ( 
   Rox_MatSE3 matse3, 
   const Rox_MatSE3 input 
);

//! Display the pose on the stdout stream
//! \param  [in ]  matse3         The Rox_MatSE3 to print
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_print ( 
   const Rox_MatSE3 matse3 
);

//! Set matrix data organized row by row
//! The poses are in the default coordinate system : RightHanded - XRight - YDown
//! \param  [out]  matse3         The object to set
//! \param  [in ]  pose_data      Matrix data organized by rows
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_set_data ( 
   Rox_MatSE3 pose_out, 
   const Rox_Double pose_inp_data[16] 
);

ROX_API Rox_ErrorCode rox_matse3_set_data_float ( 
   Rox_MatSE3 pose_out, 
   const Rox_Float pose_inp_data[16] 
);

//! Set matrix data organized row by row
//! The pose_out is in the default coordinate system : RightHanded - XRight - YDown
//! \param  [out]  pose_out            The MatSE3 matrix in which copy the data
//! \param  [in ]  pose_inp_data       Matrix data organized by rows
//! \param  [in ]  coordinate_system   The coordinate system of the input data
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_set_data_change_coordinate_system ( 
   Rox_MatSE3 pose_out, 
   const Rox_Double pose_inp_data[16],
   const Rox_3D_Coordinate_System coordinate_system
);

//! Get matrix data organized row by row
//! The pose_out_data is in the default coordinate system : RightHanded - XRight - YDown
//! \param  [out]  pose_out_data       The data organized by rows
//! \param  [in ]  pose_inp            The MatSE3 matrix from which copy the data
//! \param  [in ]  coordinate_system   The coordinate system of the input matrix
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_matse3_get_data_change_coordinate_system ( 
   Rox_Double pose_out_data[16], 
   const Rox_MatSE3 pose_inp,
   const Rox_3D_Coordinate_System coordinate_system
);

ROX_API Rox_ErrorCode rox_matse3_set_data_transform_coordinate_system ( 
   Rox_MatSE3 pose_out, 
   const Rox_Double pose_inp_data[16],
   const Rox_3D_Coordinate_System coordinate_system
);

ROX_API Rox_ErrorCode rox_matse3_get_data_transform_coordinate_system ( 
   Rox_Double pose_out_data[16], 
   const Rox_MatSE3 pose_inp, 
   const Rox_3D_Coordinate_System coordinate_system
);

ROX_API Rox_ErrorCode rox_matse3_transform_predefined_coordinate_system (
   Rox_MatSE3 pose_out,
   const Rox_MatSE3 pose_inp,
   const Rox_3D_Coordinate_System coordinate_system
);

//! Get matrix data organized row by row
//! The pose and data are in the default coordinate system : RightHanded - XRight - YDown
//! \param  [out]  pose_out_data  The data organized by rows
//! \param  [in ]  pose_inp       The object to copy
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_matse3_get_data ( 
   Rox_Double pose_out_data[16], 
   const Rox_MatSE3 pose_inp 
);

//! Get matrix data float organized row by row
//! The pose and data are in the default coordinate system : RightHanded - XRight - YDown
//! \param  [out]  data           The data organized by rows
//! \param  [in ]  matse3         The object to copy
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_matse3_get_data_float ( 
   Rox_Float pose_out_data[16], 
   const Rox_MatSE3 pose_inp 
);

//! Scale the translation t.result = scale * t.result;
//! \param  [out]  matse3         The matse3 matrix containing the translation to be scaled
//! \param  [in ]  scale          The scalar factor
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_scale_translation ( 
   Rox_MatSE3 matse3, 
   const Rox_Double scale 
);

//! Error between two Rox_MatSE3 matrices : result = inv(input_1) * input_2
//! \param  [out]  matse3         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_mulinvmat ( 
   Rox_MatSE3 matse3, 
   const Rox_MatSE3 input_1, 
   const Rox_MatSE3 input_2 
);

//! Error between two Rox_MatSE3 matrices : result = input_1 * inv(input_2)
//! \param  [out]  matse3         The multiplication result
//! \param  [in ]  input_1        The left operand
//! \param  [in ]  input_2        The right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_mulmatinv ( 
   Rox_MatSE3 matse3, 
   const Rox_MatSE3 input_1, 
   const Rox_MatSE3 input_2 
);


//! Error on a resulting Rox_MatSE3 matrix : extraction of rotation and translation
//! \param  [out]  err_tra        The error in translation
//! \param  [out]  err_rot        The error in rotation
//! \param  [in ]  input          The matrix containing the errrors
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_error ( 
   Rox_Double * err_tra, 
   Rox_Double * err_rot, 
   const Rox_MatSE3 input
);

//! Error between two Rox_MatSE3 matrices : extraction of rotation and translation
//! \param  [out]  err_tra        The error in translation
//! \param  [out]  err_rot        The error in rotation
//! \param  [in ]  T1             The left operand
//! \param  [in ]  T2             The right operand
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_distance ( 
   Rox_Double * err_tra, 
   Rox_Double * err_rot, 
   const Rox_MatSE3 T1, 
   const Rox_MatSE3 T2
);


//! Write the matrix data in an opened file
//! \param  [out]  out            The opened file to write in
//! \param  [in ]  input          The matrix containing data to write
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_write ( 
   FILE* out, 
   const Rox_MatSE3 input
);


//! Sets the rotation and translation part of an existing matric
//! \param  [out]  pose           The matrix to modify
//! \param  [in ]  rot            The rotation to set (u*theta representation)
//! \param  [in ]  tra            The translation to set 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_set_rot_tra ( 
   Rox_MatSE3 pose, 
   const Rox_Double rot[3], 
   const Rox_Double tra[3]
);


//! Sets the translation part of an existing matse3 matrix
//! \param  [out]  pose           The matrix to modify
//! \param  [in ]  tra            The translation to set 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_set_translation ( 
   Rox_MatSE3 pose, 
   const Rox_Double tra[3]
);


//! Sets the rotation and translation part of an existing matric
//! \param  [out]  pose           The matrix to modify
//! \param  [in ]  rot            The rotation to set (rot = axis * angle) 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_set_rotation ( 
   Rox_MatSE3 pose, 
   const Rox_Double rot[3]
);

ROX_API Rox_ErrorCode rox_matse3_get_tra ( Rox_Double tra[3], const Rox_MatSE3 pose );

//! Sets the rotation and translation part of an existing matric
//! \param  [out]  rot            The rotation to get (u*theta representation)
//! \param  [out]  tra            The translation to set
//! \param  [in ]  pose           The matrix to modify
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_get_rot_tra ( 
   Rox_Double rot[3], 
   Rox_Double tra[3], 
   const Rox_MatSE3 pose
);


//! Build a pose  given a rotation axis/angle and a translation
//! \param  [out]  pose        the built matrix
//! \param  [in ]  axis_x      the x component of the axis
//! \param  [in ]  axis_y      the y component of the axis
//! \param  [in ]  axis_z      the z component of the axis
//! \param  [in ]  angle       the angle value
//! \param  [in ]  tx          the x translation component of the transformation
//! \param  [in ]  ty          the y translation component of the transformation
//! \param  [in ]  tz          the z translation component of the transformation
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_matse3_set_axis_angle_translation (
   Rox_MatSE3 pose, 
   const Rox_Double axis_x, 
   const Rox_Double axis_y, 
   const Rox_Double axis_z, 
   const Rox_Double angle, 
   const Rox_Double tx, 
   const Rox_Double ty, 
   const Rox_Double tz
);

ROX_API Rox_ErrorCode rox_matse3_set_axis_angle (
   Rox_MatSE3 T, 
   const Rox_Double axis_x, 
   const Rox_Double axis_y, 
   const Rox_Double axis_z, 
   const Rox_Double angle
);

//! Sets the rotation and translation part of an existing matrix
//! \param  [out]  pose           The matrix to modify
//! \param  [in ]  rot            The rotation to set (matso3 representation)
//! \param  [in ]  tra            The translation to set 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_set_matso3_r3 ( 
   Rox_MatSE3 pose, 
   const Rox_MatSO3 rot, 
   const Rox_Matrix tra
);


//! Update the pose to the right given a 6 rows vector (se(3) : 3 translation then 3 rotation)
//! \param  [out]  matse3         The result AND input homogeneous 4x4 matrix : T = T * expmSE3 ( algse3(vector) )
//! \param  [in ]  vector         The input 6x1 vector
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_update_right ( 
   Rox_MatSE3 matse3, 
   const Rox_Array2D_Double vector 
);

//! Update the pose to the right given a 4 x 4 matrix in the se3 algebra
//! \param  [out]  matse3         The result AND input homogeneous 4 x 4 matrix in SE3: T = T * expmSE3 ( algse3 )
//! \param  [in ]  algse3         The input 4x4 matrix in se3
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_update_algse3_right (
   Rox_MatSE3 pose,
   const Rox_Matrix algse3
);

//! Update the pose to the right given 4 x 4 matrix in the SE3 group
//! \param  [out]  pose           The result AND input homogeneous 4 x 4 matrix in SE3: T = T * dT
//! \param  [in ]  matse3         The input 4x4 matrix in se3
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_update_matse3_right (
   Rox_MatSE3 pose,
   const Rox_MatSE3 matse3
);


//! Update the pose to the left given a 6 rows vector (se(3) : 3 translation then 3 rotation)
//! T = expmSE3 ( algse3 ( - vector ) ) * T
//! \param  [out]  matse3         The result AND input  
//! \param  [in ]  vector         The input 6x1 vector ( vector = [vt; vr] )
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_update_left ( 
   Rox_MatSE3 matse3, 
   const Rox_Array2D_Double vector 
);


//! Change of coordinate system : matse3_2 = inv(matse3_12) * matse3_1 * matse3_12
//! \param  [out]  matse3_2       The matse3 in the new coordinate system
//! \param  [in ]  matse3_12      The matse3 defining the change of coordinate system
//! \param  [in ]  matse3_1       The matse3 in the old coordinate system
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_change_coordinate_system ( 
   Rox_MatSE3 matse3_2, 
   const Rox_MatSE3 matse3_12, 
   const Rox_MatSE3 matse3_1 
);

ROX_API Rox_ErrorCode rox_matse3_change_predefined_coordinate_system (
   Rox_MatSE3 pose_out,
   const Rox_MatSE3 pose_inp,
   const Rox_3D_Coordinate_System coordinate_system
);

//! Compute the determinant of the MatSE3 matrix (should be 1)
//! \param  [out]  determinant    The determinant of the matrix
//! \param  [in ]  matse3         The input MatSE3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_determinant ( 
   Rox_Double * determinant, 
   const Rox_MatSE3 pose 
);


//! Get the pointer to the matrix data
//! \param  [out]  rowsptr        The pointer to the matrix data
//! \param  [in ]  matse3         The input MatSE3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_get_data_pointer_to_pointer ( 
   Rox_Double *** rowsptr, 
   const Rox_MatSE3 pose 
);

//! Compute the matrix exponential of a matrix in se3 to obtain a matrix in the SE3 group
//! \param  [out]  matse3   The matrix in exponential in SE3
//! \param  [in ]  algse3   The matrix in se3. Note that se3 = so3 x r3
//! \return An error code, rename the function
ROX_API Rox_ErrorCode rox_matse3_exponential_algse3 ( 
   Rox_MatSE3 matse3, 
   const Rox_Matrix algse3 
);

//! Check that the size of the input MatSE3 matrix is 4x4
//! \param  [in ]  matse3         The input MatSE3 matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matse3_check_size ( 
   const Rox_MatSE3 pose 
);


//! Adjoint representation : adjoint = matse3 * algse3 * inv( matse3 )
//! \param  [out]  adjoint        The algse3 adjoint matrix  
//! \param  [in ]  matse3         The matse3 matrix
//! \param  [in ]  algse3         The algse3 matrix
//! \return An error code
//! \todo   To be tested, to be deplaced in the appropriate module
ROX_API Rox_ErrorCode rox_algse3_adjoint_matse3 (  
   Rox_Matrix adjoint, 
   const Rox_MatSE3 matse3, 
   const Rox_Matrix algse3 
);


ROX_API Rox_ErrorCode rox_matse3_from_photoframe_vertices_vector (
  Rox_MatSE3 pTo,
  const Rox_Point3D_Double p_points,
  const Rox_Point3D_Double o_points
);

//! @}

#endif // __OPENROX_MATSE3__
