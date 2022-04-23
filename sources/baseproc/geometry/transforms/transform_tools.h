//==============================================================================
//
//    OPENROX   : File transform_tools.h
//
//    Contents  : API of transform_tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRANSFORM_TOOLS__
#define __OPENROX_TRANSFORM_TOOLS__

#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/plane/plane_transform.h>

#include <baseproc/image/imask/imask.h>

//! \ingroup Geometry
//! \defgroup transform_tools Transform tools
//! @{

//! Decompose a projection matrix into a calibration matrix and a pose (such that P = K*[R t])
//! \param  [out]  calib         the computed calibration matrix
//! \param  [out]  pose          the computed pose
//! \param  [in ]  proj          the input projection matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_decompose_projection ( 
   Rox_MatUT3 calib, 
   Rox_MatSE3 pose, 
   Rox_Matrix proj
);


//! Compute a matrix transformation used for basis change (transform^-1*input*transform)
//! \param  [out]  output        the transformed matrix
//! \param  [in ]  input         the matrix to transform
//! \param  [in ]  transform     the transformation matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_basischange ( 
   Rox_MatSE3 output, 
   Rox_MatSE3 input, 
   Rox_MatSE3 transform
);


//! Compute a matrix transformation used for basis change (transforminv*input*transforminv^-1)
//! \param  [out]  output        the transformed matrix
//! \param  [in ]  input         the matrix to transform
//! \param  [in ]  transforminv  the transformation matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_basischangeinv ( 
   Rox_MatSE3 output, 
   Rox_MatSE3 input, 
   Rox_MatSE3 transforminv
);

//! Precalibrate a transformation matrix on the right (transform = pose * calib^-1)
//! \param  [out]  transform the transformed matrix
//! \param  [in ]  pose the transformation matrix
//! \param  [in ]  calib the calibration matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_pose_precalibright ( 
   Rox_Array2D_Double transform, 
   Rox_Array2D_Double pose, 
   Rox_Array2D_Double calib
);

//! Precalibrate a transformation matrix (transform = calib * pose * calib^-1
//! \param  [out]  transform the transformed matrix
//! \param  [in ]  pose the transformation matrix
//! \param  [in ]  calib the calibration matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_pose_precalibrate (
   Rox_Array2D_Double transform, 
   Rox_Array2D_Double pose, 
   Rox_Array2D_Double calib
);


//! Build a skew symmetric matrix from a 3*1 matrix
//! \param  [out]  A the built matrix
//! \param  [in ]  v a 3*1 vector
//! \return An error code
//! \todo   To be tested, to be renamed "rox_algso3_from_vector"
ROX_API Rox_ErrorCode rox_transformtools_skew_from_vector ( 
   Rox_Array2D_Double A, 
   Rox_Array2D_Double v
);

//! Given a pose T compute logSE3(T)
//! \param  [out]  vector Vector with 6 real values
//! \param  [in ]  pose matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_logse3_from_pose (
   Rox_Array2D_Double vector, 
   Rox_MatSE3 pose
);

//! Given a pose compute a rotation axis/angle and the translation
//! \param  [out]  axis_x the x component of the axis
//! \param  [out]  axis_y the y component of the axis
//! \param  [out]  axis_z the z component of the axis
//! \param  [out]  angle the angle value
//! \param  [out]  tx the x component of the translation
//! \param  [out]  ty the y component of the translation
//! \param  [out]  tz the z component of the translation
//! \param  [in ]  pose matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_axisangle_translation_from_pose (
   Rox_Double * axis_x, 
   Rox_Double * axis_y, 
   Rox_Double * axis_z, 
   Rox_Double * angle, 
   Rox_Double * tx, 
   Rox_Double * ty, 
   Rox_Double * tz, 
   Rox_MatSE3 pose);

//! Given a pose (the top left 3*3 subarray), compute a rotation quaternion
//! \param  [out]  x           the x component of the quaternion
//! \param  [out]  y           the y component of the quaternion
//! \param  [out]  z           the z component of the quaternion
//! \param  [out]  w           the w component of the quaternion
//! \param  [in ]   rotation    the rotation matrix
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_transformtools_quaternion_from_rotationmatrix (
   Rox_Double *x, 
   Rox_Double *y, 
   Rox_Double *z, 
   Rox_Double *w, 
   Rox_MatSO3 rotation
);

//! Build a pose given a vector (the end point being the position of the camera) and a rotation around this vector
//! \param  [out]  pose           The built matrix
//! \param  [in ]  vec            The direction vector (represent a point on a unit sphere around the origin)
//! \param  [in ]  rotation       The rotation angle in degrees
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_pose_from_sphere (
   Rox_Array2D_Double pose, 
   Rox_Point3D_Double vec, 
   const Rox_Double rotation
);

//! Compute relative pose between two poses using fast specific equations
//! \param  [out]  c2Tc1          The relative pose
//! \param  [in ]  c1To           The reference pose
//! \param  [in ]  c2To           The destination pose
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_pose_relative (
   Rox_MatSE3 c2Tc1, 
   Rox_MatSE3 c1To, 
   Rox_MatSE3 c2To
);

//! Build a calibration matrix using 4 classic parameters
//! \param  [out]  calibmatrix    The calibration matrix
//! \param  [in ]  fu             the px parameter
//! \param  [in ]  fv             the py parameter
//! \param  [in ]  cu             the u0 parameter
//! \param  [in ]  cv             the v0 parameter
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_build_calibration_matrix (
   Rox_MatUT3 calibmatrix, 
   const Rox_Double fu, 
   const Rox_Double fv, 
   const Rox_Double cu, 
   const Rox_Double cv
);

//! Build a calibration matrix such that the pixel/meters dimensions are valid
//! calibmatrix is like a camera calibration matrix that allows to transform meters coordinates to pixel coordinates
//! The calibration matrix such that p_k = calibmatrix * q_k
//!   p_k are the coordinates of a square of size height_pixels x width_pixels
//!   q_k are the coordinates of a square of size height_meters x width_meters
//!
//! Where:
//!
//!   p_1 = [-0.5; -0.5; 1]; 
//!   p_2 = [width_pixels-0.5; -0.5; 1]; 
//!   p_3 = [width_pixels-0.5; height_pixels-0.5; 1]; 
//!   p_4 = [-0.5; height_pixels-0.5; 1];
//!
//!   q_1 = [-width_meters/2; -height_meters/2; 1];
//!   q_2 = [ width_meters/2; -height_meters/2; 1];
//!   q_3 = [ width_meters/2;  height_meters/2; 1];
//!   q_4 = [-width_meters/2;  height_meters/2; 1];
//!
//! \param  [out] calibmatrix     The calibration matrix
//! \param  [in ] cols_pixels     Width in pixel
//! \param  [in ] rows_pixels     Height in pixel
//! \param  [in ] cols_meters     Width in meters
//! \param  [in ] rows_meters     Height in meters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_build_calibration_matrix_for_template (
   Rox_MatUT3 calibmatrix, 
   const Rox_Double cols_pixels, 
   const Rox_Double rows_pixels, 
   const Rox_Double cols_meters, 
   const Rox_Double rows_meters
);

ROX_API Rox_ErrorCode rox_transformtools_build_model_to_image_homography_from_4_points (
   Rox_MatSL3 homography, 
   const Rox_Point3D_Double m, 
   const Rox_Point2D_Double p
);

//! Build a calibration matrix such that the pixel/meters dimensions are valid
//! calibmatrix is like a camera calibration matrix that allows to transform meters coordinates to pixel coordinates
//! The calibration matrix such that p_k = calibmatrix * q_k
//!   p_k are the coordinates of a square of size height_pixels x width_pixels
//!   q_k are the coordinates of a square of size height_meters x width_meters
//!
//! Where:
//!
//!   p_1 = [-0.5; -0.5; 1]; 
//!   p_2 = [width_pixels-0.5; -0.5; 1]; 
//!   p_3 = [width_pixels-0.5; height_pixels-0.5; 1]; 
//!   p_4 = [-0.5; height_pixels-0.5; 1];
//!
//!   q_1 = [-width_meters/2; +height_meters/2; 1];
//!   q_2 = [+width_meters/2; +height_meters/2; 1];
//!   q_3 = [+width_meters/2; -height_meters/2; 1];
//!   q_4 = [-width_meters/2; -height_meters/2; 1];
//!
//! \param  [out] calibmatrix     The calibration matrix
//! \param  [in ] cols_pixels     Width in pixel
//! \param  [in ] rows_pixels     Height in pixel
//! \param  [in ] cols_meters     Width in meters
//! \param  [in ] rows_meters     Height in meters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_build_calibration_matrix_for_template_newframe (
   Rox_MatUT3 calibmatrix, 
   const Rox_Double cols_pixels, 
   const Rox_Double rows_pixels, 
   const Rox_Double cols_meters, 
   const Rox_Double rows_meters
);

// Build the (4x4) SA3 matrix cQr = [Kc, 0; 0 1] * cTr * inv ([Kr, 0; 0 1])
ROX_API Rox_ErrorCode rox_transformtools_build_matsa3_matrix (
   Rox_Array2D_Double cQr,
   Rox_MatSE3 cTr, 
   Rox_MatUT3 Kc, 
   Rox_MatUT3 Kr 
);

//! Build a random pose
//! \param  [out]  pose           Randomly created matse3 pose matrix (with valid values)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_pose_random ( Rox_MatSE3 pose );


//! Compute a homography between pts2d the projection of pts3d
ROX_API Rox_ErrorCode rox_transformtools_compute_homography_from_pts2d_projection_pts3d ( 
   Rox_MatSL3 H, 
   const Rox_MatUT3 K, 
   const Rox_MatSE3 cTo, 
   const Rox_Point3D_Double points3d, 
   const Rox_Point2D_Double points2d_ref
);


//! Build a homography in pixel coordinates using the equation H = Kout * ( R - 1/d * t * n^t ) * Kinp^-1
//! Important : we suppose that the input pose has been shifted: T = T * [[eye(3), [0;0;1]]; 0 0 0 1] ?????
//! \param  [out]  homography     the result homography
//! \param  [in ]  pose           the pose
//! \param  [in ]  calib_output
//! \param  [in ]  calib_input
//! \param  [in ]  a
//! \param  [in ]  b
//! \param  [in ]  c
//! \param  [in ]  d
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_build_homography ( 
   Rox_MatSL3 homography, 
   const Rox_MatSE3 pose, 
   const Rox_MatUT3 calib_output, 
   const Rox_MatUT3 calib_input, 
   const Rox_Double a, 
   const Rox_Double b, 
   const Rox_Double c, 
   const Rox_Double d
);

ROX_API Rox_ErrorCode rox_transformtools_build_model_to_image_homography (
   Rox_MatSL3 homography,
   Rox_MatUT3 calib,
   Rox_MatSE3 pose
);

//! Build a pixel homography using the equation H = K_out*([r1 r2 t])*K_inp^-1 (Plane is z=0)
//! \param  [out]  homography        The result homography
//! \param  [in ]  pose              The pose
//! \param  [in ]  calib_output      The input calibration
//! \param  [in ]  calib_input       The output calibration
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_build_homography_intermodel (
   Rox_MatSL3 homography, 
   Rox_MatSE3 pose, 
   Rox_MatUT3 calib_output, 
   Rox_MatUT3 calib_input
);

//! Extract a pose matrix from an model to image homography
//! \param  [out]  pose           The result pose T = [r1 r2 r3 t]
//! \param  [in ]  homography     The model to image homography
//! \param  [in ]  calibration    The camera intrinsic parameters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_build_pose_intermodel (
   Rox_MatSE3 pose, 
   const Rox_MatSL3 homography, 
   const Rox_MatUT3 calibration
);

//! Build an homography (pixel to pixel) for a patch considered parralel to the camera plane with a known depth
//! \brief Given the pose is from c1 to c2 (c2Tc1), the homography will be built from c2 to c1 (c1Hc2).
//! \param  [out]  homography the result homohgraphy
//! \param  [in ]  pose the pose
//! \param  [in ]  calibration_c1 calib of c1
//! \param  [in ]  calibration_c2 calib of c2
//! \param  [in ]  pos_u the u center of the patch in c1
//! \param  [in ]  pos_v the v center of the patch in c1
//! \param  [in ]  Z the depth of the patch in c1
//! \param  [in ]  patchsize the patch size
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_build_homography_pointpatch_front (
   Rox_MatSL3 homography, 
   Rox_MatSE3 pose, 
   Rox_MatUT3 calibration_c1, 
   Rox_MatUT3 calibration_c2, 
   Rox_Double pos_u, 
   Rox_Double pos_v, 
   Rox_Double Z, 
   Rox_Uint patchsize
);

//! Optimize homography shift and destination size such that the forward mapping create as less empty regions as possible
//! \param  [out]  dwidth         Optimal destination width
//! \param  [out]  dheight        Optimal destination height
//! \param  [out]  homography     The input/result homography
//! \param  [in ]  swidth         Source width
//! \param  [in ]  sheight        Source height
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_homography_optimalforward (
   Rox_Sint * dwidth, 
   Rox_Sint * dheight, 
   Rox_MatSL3 homography, 
   const Rox_Sint swidth, 
   const Rox_Sint sheight
);

//! Build an essential matrix from a pose (t[x]R)
//! \param  [out]  essential the result essential
//! \param  [in ]  pose the pose
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_build_essential ( 
   Rox_Matrix essential, 
   const Rox_MatSE3 pose
);

//! Build a Fundamental matrix from an essential matrix and a calibration
//! \param  [out]  fundamental    The result fundamental matrix
//! \param  [in ]  essential      The input essential matrix
//! \param  [in ]  calib          The calibration matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_build_fundamental_from_essential (
   Rox_Matrix fundamental, 
   const Rox_Matrix essential, 
   const Rox_MatUT3 calib
);

//! Build a Fundamental matrix from a pose matrix and a calibration matrix
//! \param  [out]  fundamental    The result fundamental matrix
//! \param  [in ]  pose           The input pose matrix
//! \param  [in ]  calib          The calibration matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_build_fundamental ( 
   Rox_Matrix fundamental, 
   const Rox_MatSE3 pose, 
   const Rox_MatUT3 calib
);

//! Update a calibration to consider a zoom (S = diag(2^level), zoomed = S^-1 * calibration)
//! \param  [out]  zoomed        The updated calibration
//! \param  [in ]  calibration   The initial calibration
//! \param  [in ]  level         The pyramid level used
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_matrix33_left_pyramidzoom (
   Rox_MatUT3 zoomed, 
   const Rox_MatUT3 calibration, 
   const Rox_Uint level
);

//! Update a calibration to consider a zoom (S = diag(2^level), zoomed = S^-1 * calibration), without using pixel central shift
//! \param  [out]  zoomed the updated calibration
//! \param  [in ]  calibration the initial calibration
//! \param  [in ]  level the pyramid level used
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_matrix33_left_pyramidzoom_exact (
   Rox_MatUT3 zoomed, 
   const Rox_MatUT3 calibration, 
   const Rox_Uint level
);

//! Update a calibration to consider a zoom (S = diag(2^level), zoomed = calibration * S)
//! \param  [out]  zoomed the updated calibration
//! \param  [in ]  calibration the initial calibration
//! \param  [in ]  level the pyramid level used
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_matrix33_right_pyramidzoom (
   Rox_MatUT3 zoomed, 
   Rox_MatUT3 calibration, 
   Rox_Uint level
);

//! Update a calibration to consider a zoom (S = diag(2^level), zoomed = calibration * S^-1)
//! \param  [out]  zoomed the updated calibration
//! \param  [in ]  calibration the initial calibration
//! \param  [in ]  level the pyramid level used
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_matrix33_right_pyramidzoominv (
   Rox_MatUT3 zoomed, 
   Rox_MatUT3 calibration, 
   Rox_Uint level
);

//! Given an homography, compute the linearized 2D affine matrix for the point p
//! \param  [out]  affine the output 2x2 affine matrix
//! \param  [in ]  homography the homography matrix
//! \param  [in ]  u the u coordinates of a point p
//! \param  [in ]  v the v coordinates of a point p
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_affine_from_homography (
   Rox_Array2D_Double affine, 
   const Rox_MatSL3 homography, 
   const Rox_Double u, 
   const Rox_Double v
);

//! Shift an homography such that H = H * [1 0 dx; 0 1 dy; 0 0 1]
//! \param  [out]  homography the homography matrix update
//! \param  [in ]  dx the x shift value
//! \param  [in ]  dy the y shift value
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_homography_shift (
   Rox_MatSL3 homography, 
   const Rox_Double dx, 
   const Rox_Double dy
);

//! Shift an homography such that H = [1 0 dx; 0 1 dy; 0 0 1] * H
//! \param  [out]  homography the homography matrix update
//! \param  [in ]  dx the x shift value
//! \param  [in ]  dy the y shift value
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_homography_shiftleft (
   Rox_MatSL3 homography, 
   const Rox_Double dx, 
   const Rox_Double dy
);

//! Given an homography, compute the  linearized scale factor of a point p due to this homography
//! \param  [out]  zoom          A computed scale change
//! \param  [in ]  homography    The homography matrix
//! \param  [in ]  u             The u coordinates of a point p
//! \param  [in ]  v             The v coordinates of a point p
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_homography_get_areazoom (
   Rox_Double * zoom, 
   const Rox_MatSL3 homography, 
   const Rox_Double u, 
   const Rox_Double v
);

//! Compute the homography which rotate an image around its center
//! \param  [out]  homography     The computed homography
//! \param  [in ]  width          The image cols (width)
//! \param  [in ]  height         The image rows (height)
//! \param  [in ]  angle          The rotation angle
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_imagerotation (
   Rox_MatSL3 homography, 
   const Rox_Sint cols, 
   const Rox_Sint rows, 
   const Rox_Double angle
);

//! Change the Z of the reference frame (cTr1 = cTr2 * r2Tr1 such that r2Tr1 = [1 0 0 0; 0 1 0 0; 0 0 1 zref; 0 0 0 1]
//! \param  [out]  updated_pose  the updated pose
//! \param  [out]  pose          the original pose
//! \param  [in ]  zref          the zref in update 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_updateZref ( 
   Rox_MatSE3 updated_pose, 
   Rox_MatSE3 pose, 
   Rox_Double zref 
);

//! Transform a non scaled pose in a multicamera framework
//! \param  [out]  corpose the updated pose (~ cxTc0 * pose * cxTc0^-1)
//! \param  [out]  cxTc0 the transformation pose
//! \param  [out]  pose the original pose
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_transformtools_estimate_relativepose_from_general ( 
   Rox_MatSE3 corpose, 
   Rox_MatSE3 cxTc0, 
   Rox_MatSE3 pose 
);

//! Compute Cayley rotation matrix from two 3D vectors
//! \param   [out] r                 Measured rotation matrix in SO3
//! \param   [in]  v1                First vector in 3D     
//! \param   [in]  v2                Second vector in 3D
//! \return  An error code
//! \warning The angle between the two vectors v1 and v2 must be different from pi and v1(3)+v2(3) > 0
//! \todo    To be tested
ROX_API Rox_ErrorCode rox_cayley_xy_from_vectors ( 
   Rox_Array2D_Double r, 
   Rox_Array2D_Double const v1, 
   Rox_Array2D_Double const v2 
);

ROX_API Rox_ErrorCode convert_depth_ogl_metric ( 
   Rox_Float * Z, 
   const Rox_Float Z_gl, 
   const Rox_Float znear, 
   const Rox_Float zfar 
);

ROX_API Rox_ErrorCode rox_array2d_float_depth_ogl_convert_to_metric ( 
   Rox_Array2D_Float Z, 
   Rox_Imask Z_mask,
   const Rox_Array2D_Float Z_ogl, 
   const Rox_Float Z_near, 
   const Rox_Float Z_far 
);

ROX_API Rox_ErrorCode rox_array2d_float_inverse_depth_ogl_convert_to_metric ( 
   Rox_Array2D_Float Zi, 
   Rox_Imask Zi_mask,
   const Rox_Array2D_Float Zi_ogl, 
   const Rox_Float Z_near, 
   const Rox_Float Z_far 
);

ROX_API Rox_ErrorCode rox_array2d_double_new_from_point2d_float ( Rox_Array2D_Double * array2d, const Rox_Point2D_Float point2d );

//! Compute H = K * R * inv(K)
ROX_API Rox_ErrorCode rox_transformtools_build_matsl3_from_matso3_matut3 (
   Rox_MatSL3 H,
   Rox_MatSO3 R, 
   Rox_MatUT3 K
);


ROX_API Rox_ErrorCode rox_transformtools_build_matsl3_from_pose ( 
   Rox_MatSL3 cHt, 
   const Rox_MatUT3 K, 
   const Rox_MatSE3 cTo, 
   const Rox_MatSE3 pTo,  
   const Rox_Plane3D_Double plane3d_p
);

//! @}

#endif
