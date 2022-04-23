//==============================================================================
//
//    OPENROX   : File transform_tools.c
//
//    Contents  : Implementation of transform_tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "transform_tools.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/flip/flipud.h>
#include <baseproc/array/flip/fliplr.h>
#include <baseproc/array/decomposition/qr.h>
#include <baseproc/array/transpose/transpose.h>
#include <baseproc/array/determinant/detgl3.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/normalize/normalize.h>
#include <baseproc/array/logarithm/logmat.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>

#include <inout/geometry/point/point2d_print.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_transformtools_decompose_projection (
   Rox_Array2D_Double calib,
   Rox_MatSE3 pose,
   Rox_Array2D_Double proj
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double PL = NULL, PR = NULL, R = NULL, T = NULL, buf33_1 = NULL, buf33_2 = NULL, buf33_3 = NULL;
   Rox_Double det, norm;

   if (!calib || !pose || !proj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check input
   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(proj, 3, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve data pointers
   Rox_Double ** dk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dk, calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dt, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&PL, proj, 0, 0, 3, 3);// PL = P(1:3,1:3)
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&PR, proj, 0, 3, 3, 1);// PR = P(1:3,4)
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&R, pose, 0, 0, 3, 3); // R= pose(1:3,1:3)
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&T, pose, 0, 3, 3, 1); // T=pose(1:3,4)
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&buf33_1, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&buf33_2, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&buf33_3, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //
   // We are given a matrix P as input which is a projection matrix.
   // P = K * [R | T]
   //
   // K is a toprigh triangular matrix with components :
   // K = [px sx u0; 0 py v0; 0 0 1]
   // R is an orthogonal matrix with det(R) = 1
   //
   // How can we retrieve K,R,T from P ?
   //
   // We know that a classical [X Y] = QR(PL) decomposition gives us :
   // X which is an orthogonal matrix
   // Y which is a top left triangular matrix
   //
   // How to get a top left triangular matrix from a top right triangular matrix ? :
   // K is a top right
   // K' is a bottom left
   // flipUD(K') is a top left
   //
   // How to get a R*Y instead of Y*R :
   // Transpose ! (Y*R)'=(R'*Y') (R' is still an orthogonal matrix ...)
   //
   // So A = flipud(PL)';
   // [X Y] = qr(A);
   // X contains an orthogonal matrix and Y a top left triangular matrix.
   // K = flipud(fliplr(Y'))
   // R = flipud(X')

   error = rox_array2d_double_copy_flip_ud(buf33_1, PL);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_transpose(buf33_2, buf33_1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_qr(buf33_1, buf33_3, buf33_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve K
   error = rox_array2d_double_transpose(calib, buf33_3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy_flip_lr(calib, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy_flip_ud(calib, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve R
   error = rox_array2d_double_transpose(R, buf33_1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy_flip_ud(R, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // The only condition we ensure is that K is triangular
   // But K is an anisotropic scaling matrix (diagonal is positive)
   // Transform K so that its diagonal is positive, and R so that K*R=P)
   if (dk[0][0] < 0.0)
   {
      dk[0][0] = -dk[0][0];
      dk[1][0] = -dk[1][0];
      dk[2][0] = -dk[2][0];
      dt[0][0] = -dt[0][0];
      dt[0][1] = -dt[0][1];
      dt[0][2] = -dt[0][2];
   }

   if (dk[1][1] < 0.0)
   {
      dk[0][1] = -dk[0][1];
      dk[1][1] = -dk[1][1];
      dk[2][1] = -dk[2][1];
      dt[1][0] = -dt[1][0];
      dt[1][1] = -dt[1][1];
      dt[1][2] = -dt[1][2];
   }

   if (dk[2][2] < 0.0)
   {
      dk[0][2] = -dk[0][2];
      dk[1][2] = -dk[1][2];
      dk[2][2] = -dk[2][2];
      dt[2][0] = -dt[2][0];
      dt[2][1] = -dt[2][1];
      dt[2][2] = -dt[2][2];
   }

   // R is not only orthogonal, but also det(R) = 1, enforce this property
   error = rox_array2d_double_detgl3 ( &det, R );
   ROX_ERROR_CHECK_TERMINATE ( error );

   norm = fabs(det);
   norm = norm * norm * norm;
   if (det < 0.0) norm = -norm;
   norm = 1.0 / norm;
   error = rox_array2d_double_scale ( R, R, norm );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Inverse unnormalized calibration
   error = rox_array2d_double_mat3x3_inverse ( buf33_1, calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Normalize calibration
   error = rox_array2d_double_scale ( calib, calib, 1.0 / dk[2][2] );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // We know K, R and P
   // T is the last unknown
   // Deduce it from the other variables so that P=K*[R | T]
   error = rox_array2d_double_mulmatmat ( T, buf33_1, PR );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mat3x3_inverse(buf33_1, PL);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(buf33_2, R, buf33_1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(buf33_3, calib, buf33_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_detgl3(&det, buf33_3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (det < 0.0) rox_array2d_double_scale ( T, T, -1.0 );

   dt[3][3] = 1.0;

function_terminate:
   rox_array2d_double_del(&PL);
   rox_array2d_double_del(&PR);
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&T);
   rox_array2d_double_del(&buf33_1);
   rox_array2d_double_del(&buf33_2);
   rox_array2d_double_del(&buf33_3);

   return error;
}


Rox_ErrorCode rox_transformtools_basischange (
   Rox_Array2D_Double output,
   Rox_Array2D_Double input,
   Rox_Array2D_Double transform
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double inv = NULL, buffer = NULL;

   if (!output || !input || !transform)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (rox_array2d_double_match_size(output, input) != ROX_ERROR_NONE)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (rox_array2d_double_match_size(output, transform) != ROX_ERROR_NONE)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&inv, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&buffer, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svdinverse(inv, transform);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(buffer, input, transform);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(output, inv, buffer);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&inv);
   rox_array2d_double_del(&buffer);

   return error;
}


Rox_ErrorCode rox_transformtools_basischangeinv (
   Rox_Array2D_Double output,
   Rox_Array2D_Double input,
   Rox_Array2D_Double transforminv
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double inv = NULL, buffer = NULL;

   if (!output || !input || !transforminv)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (rox_array2d_double_match_size(output, input) != ROX_ERROR_NONE)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (rox_array2d_double_match_size(output, transforminv) != ROX_ERROR_NONE)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&inv, rows, cols);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&buffer, rows, cols);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_svdinverse(inv, transforminv);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(buffer, input, inv);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(output, transforminv, buffer);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   rox_array2d_double_del(&inv);
   rox_array2d_double_del(&buffer);

   return error;
}


Rox_ErrorCode rox_transformtools_pose_precalibright (
   Rox_Array2D_Double transform,
   Rox_MatSE3 pose,
   Rox_Array2D_Double calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double **dt, **dp, **dk;
   Rox_Double ipx, ipy, iu0, iv0;

   if (!transform)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!calib)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check input
   error = rox_array2d_double_check_size(transform, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, transform );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute inverse calibration (right)
   ipx = 1.0 / dk[0][0];
   ipy = 1.0 / dk[1][1];
   iu0 = -dk[0][2] * ipx;
   iv0 = -dk[1][2] * ipy;

   // Perform transform
   dt[0][0] = ipx * dp[0][0];
   dt[0][1] = ipy * dp[0][1];
   dt[0][2] = dp[0][2] + iu0 * dp[0][0] + iv0 * dp[0][1];
   dt[0][3] = dp[0][3];
   dt[1][0] = ipx * dp[1][0];
   dt[0][1] = ipy * dp[1][1];
   dt[1][2] = dp[1][2] + iu0 * dp[1][0] + iv0 * dp[1][1];
   dt[1][3] = dp[1][3];
   dt[2][0] = ipx * dp[2][0];
   dt[0][1] = ipy * dp[2][1];
   dt[2][2] = dp[2][2] + iu0 * dp[2][0] + iv0 * dp[2][1];
   dt[2][3] = dp[2][3];

   // Add homogeneous line (we ignore last line of pose, assuming it is already normalized)
   dt[3][0] = 0;
   dt[3][1] = 0;
   dt[3][2] = 0;
   dt[3][3] = 1;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_pose_precalibrate (
   Rox_Array2D_Double transform,
   Rox_MatSE3 pose,
   Rox_Array2D_Double calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double **dt, **dp, **dk;
   Rox_Double ipx, ipy, iu0, iv0;
   Rox_Double px, py, u0, v0;

   if (!transform)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!calib)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check input
   error = rox_array2d_double_check_size(transform, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, transform);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute calibration (left)
   px = dk[0][0];
   py = dk[1][1];
   u0 = dk[0][2];
   v0 = dk[1][2];

   // Compute inverse calibration (right)
   ipx = 1.0 / px;
   ipy = 1.0 / py;
   iu0 = -u0 * ipx;
   iv0 = -v0 * ipy;

   // Perform transform
   dt[0][0] = (px * dp[0][0] + u0 * dp[2][0]) * ipx;
   dt[0][1] = (px * dp[0][1] + u0 * dp[2][1]) * ipy;
   dt[0][2] = (px * dp[0][0] + u0 * dp[2][0]) * iu0 + (px * dp[0][1] + u0 * dp[2][1]) * iv0 + px * dp[0][2] + u0 * dp[2][2];
   dt[0][3] = px * dp[0][3] + u0 * dp[2][3];
   dt[1][0] = (py * dp[1][0] + v0 * dp[2][0]) * ipx;
   dt[1][1] = (py * dp[1][1] + v0 * dp[2][1]) * ipy;
   dt[1][2] = (py * dp[1][0] + v0 * dp[2][0]) * iu0 + (py * dp[1][1] + v0 * dp[2][1]) * iv0 + py * dp[1][2] + v0 * dp[2][2];
   dt[1][3] = py * dp[1][3] + v0 * dp[2][3];
   dt[2][0] = dp[2][0] * ipx;
   dt[2][1] = dp[2][1] * ipy;
   dt[2][2] = dp[2][0] * iu0 + dp[2][1] * iv0 + dp[2][2];
   dt[2][3] = dp[2][3];

   // Add homogeneous line (we ignore last line of pose, assuming it is already normalized)
   dt[3][0] = 0;
   dt[3][1] = 0;
   dt[3][2] = 0;
   dt[3][3] = 1;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_skew_from_vector (
   Rox_Array2D_Double A,
   Rox_Array2D_Double v
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!A || !v)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(A, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(v, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** da = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &da, A );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double *  dv = NULL;
   error = rox_array2d_double_get_data_pointer ( &dv, v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   da[0][0] =    0.0; da[0][1] = -dv[2]; da[0][2] =  dv[1];
   da[1][0] =  dv[2]; da[1][1] =    0.0; da[1][2] = -dv[0];
   da[2][0] = -dv[1]; da[2][1] =  dv[0]; da[2][2] =    0.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_transformtools_logse3_from_pose (
   Rox_Array2D_Double result,
   Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ax = 0.0, ay = 0.0, az = 0.0, angle = 0.0;
   Rox_Array2D_Double R = NULL, T = NULL, M = NULL, iM = NULL, D = NULL;
   Rox_Double ** dm = NULL;
   Rox_Double ** dd = NULL, **dr = NULL;

   if (!pose || !result)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // We work on the top left 3*3 sub matrix, to make it work on pose matrix or rotation matrix
   CHECK_ERROR_TERMINATE(rox_array2d_double_check_size(pose, 4, 4));
   CHECK_ERROR_TERMINATE(rox_array2d_double_check_size(result, 6, 1));
   CHECK_ERROR_TERMINATE(rox_array2d_double_logmat(&ax, &ay, &az, &angle, pose));

   CHECK_ERROR_TERMINATE(rox_array2d_double_new_subarray2d(&R, pose, 0, 0, 3, 3));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new_subarray2d(&T, pose, 0, 3, 3, 1));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&M, 3, 3));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&iM, 3, 3));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&D, 3, 1));

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dm, M );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (fabs(angle)<DBL_EPSILON)
   {
      rox_array2d_double_fillunit(M);
   }
   else
   {
       double mcosa = (1.0 - cos(angle)) / angle;
       double msina = (1.0 - sin(angle) / angle);
       dm[0][0] = 1.0 - msina * az * az - msina * ay * ay;
       dm[0][1] = -mcosa * az + msina * ay * ax;
       dm[0][2] = mcosa * ay + msina * az * ax;
       dm[1][0] = mcosa * az + msina * ay * ax;
       dm[1][1] = 1.0 - msina * az * az - msina * ax * ax;
       dm[1][2] = -mcosa * ax + msina * az * ay;
       dm[2][0] = -mcosa * ay + msina * az * ax;
       dm[2][1] = mcosa * ax + msina * az * ay;
       dm[2][2] = 1.0 - msina * ay * ay - msina * ax * ax;
   }

   error = rox_array2d_double_mat3x3_inverse(iM, M);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(D, iM, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dd, D);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dr, result);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dr[0][0] = dd[0][0];
   dr[1][0] = dd[1][0];
   dr[2][0] = dd[2][0];

   dr[3][0] = ax * angle;
   dr[4][0] = ay * angle;
   dr[5][0] = az * angle;

function_terminate:
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&T);
   rox_array2d_double_del(&M);
   rox_array2d_double_del(&iM);
   rox_array2d_double_del(&D);

   return error;
}


Rox_ErrorCode rox_transformtools_axisangle_translation_from_pose (
   Rox_Double *axis_x,
   Rox_Double *axis_y,
   Rox_Double *axis_z,
   Rox_Double *angle,
   Rox_Double *tx,
   Rox_Double *ty,
   Rox_Double *tz,
   Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose || !axis_x || !axis_y || !axis_z || !angle || !tx || !ty || !tz)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // We work on the top left 3*3 sub matrix, to make it work on pose matrix or rotation matrix
   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Extract rotation parameters
   error = rox_array2d_double_logmat(axis_x, axis_y, axis_z, angle, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *tx = dt[0][3];
   *ty = dt[1][3];
   *tz = dt[2][3];

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_quaternion_from_rotationmatrix (
   Rox_Double *qx,
   Rox_Double *qy,
   Rox_Double *qz,
   Rox_Double *qw,
   Rox_Array2D_Double rotation
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double tr, n4;
   Rox_Double x,y,z,w;

   if (!rotation || !qx || !qy || !qz || !qw)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   *qx = 1;
   *qy = 0;
   *qz = 0;
   *qw = 0;

   // We work on the top left 3*3 sub matrix, to make it work on pose matrix or rotation matrix
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, rotation);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (cols < 3 || rows < 3)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dr = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dr, rotation);
   ROX_ERROR_CHECK_TERMINATE ( error );

   tr = dr[0][0] + dr[1][1] + dr[2][2];

   if (tr > 0.0)
   {
      x = dr[2][1] - dr[1][2];
      y = dr[0][2] - dr[2][0];
      z = dr[1][0] - dr[0][1];
      w = tr + 1.0;
      n4 = w;
   }
   else if ((dr[0][0] > dr[1][1]) && (dr[0][0] > dr[2][2]))
   {
      x = 1.0 + dr[0][0] - dr[1][1] - dr[2][2];
      y = dr[1][0] - dr[0][1];
      z = dr[2][0] - dr[0][2];
      w = dr[2][1] - dr[1][2];
      n4 = x;
   }
   else if ((dr[1][1] > dr[0][0]) && (dr[1][1] > dr[2][2]))
   {
      x = dr[1][0] + dr[0][1];
      y = 1.0 + dr[1][1] - dr[0][0] - dr[2][2];
      z = dr[2][1] + dr[1][2];
      w = dr[0][2] - dr[2][0];
      n4 = y;
   }
   else
   {
      x = dr[2][0] + dr[0][2];
      y = dr[2][1] + dr[1][2];
      z = 1.0 + dr[2][2] - dr[0][0] - dr[1][1];
      w = dr[1][0] - dr[0][2];
      n4 = z;
   }

   Rox_Double scale = 0.5 / sqrt(n4);
   *qx = w * scale;
   *qy = x * scale;
   *qz = y * scale;
   *qw = z * scale;

function_terminate:
   return error;
}

Rox_ErrorCode rox_transformtools_pose_from_sphere (
   Rox_MatSE3 pose,
   Rox_Point3D_Double vec,
   const Rox_Double rotation
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double norm = 0.0;
   Rox_Double vecX[3];
   Rox_Double vecY[3];
   Rox_Double vecZ[3];
   Rox_Double angle = 0.0, ct = 1.0, st = 0.0;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Here we suppose the camera is a point on a half-sphere (z > 0)
   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Renormalize vector
   norm = sqrt(vec->X * vec->X + vec->Y * vec->Y + vec->Z * vec->Z);
   if (norm < DBL_EPSILON)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (fabs(vec->Z) < DBL_EPSILON)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   norm = 1.0 / norm;
   vec->X *= norm;
   vec->Y *= norm;
   vec->Z *= norm;

   // convert the rotation angle from degrees to radians
   angle = rotation * ROX_PI / 180.0;
   ct = cos(angle);
   st = sin(angle);

   // Z vector basis
   vecZ[0] = vec->X;
   vecZ[1] = vec->Y;
   vecZ[2] = vec->Z;

   // X vector basis, try to keep it "parralel" to the image x axis
   norm = sqrt(vecZ[0] * vecZ[0] + vecZ[2] * vecZ[2]);

   if (norm < DBL_EPSILON)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   norm = 1.0 / norm;
   vecX[0] = vecZ[2] * norm;
   vecX[1] = 0;
   vecX[2] = -vecZ[0] * norm;

   // Y vector, computed to be orthogonal to X and Z
   vecY[0] = vecZ[1] * vecX[2] - vecZ[2] * vecX[1];
   vecY[1] = vecZ[2] * vecX[0] - vecZ[0] * vecX[2];
   vecY[2] = vecZ[0] * vecX[1] - vecZ[1] * vecX[0];

   // Rotate vectors with Rz = expmSOe
   dt[0][0] = ct * vecX[0] - st * vecX[1];
   dt[0][1] = ct * vecY[0] - st * vecY[1];
   dt[0][2] = ct * vecZ[0] - st * vecZ[1];
   dt[1][0] = st * vecX[0] + ct * vecX[1];
   dt[1][1] = st * vecY[0] + ct * vecY[1];
   dt[1][2] = st * vecZ[0] + ct * vecZ[1];
   dt[2][0] = vecX[2];
   dt[2][1] = vecY[2];
   dt[2][2] = vecZ[2];

   // Translation
   dt[0][3] = 0;
   dt[1][3] = 0;
   dt[2][3] = 1.0;

   dt[3][0] = 0;
   dt[3][1] = 0;
   dt[3][2] = 0;
   dt[3][3] = 1;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_pose_relative (
   Rox_MatSE3 c2Tc1,
   Rox_MatSE3 c1To,
   Rox_MatSE3 c2To
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!c2Tc1 || !c1To || !c2To)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(c2Tc1, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(c2To, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(c1To, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dr = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dr, c2Tc1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** d1 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &d1, c1To);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** d2 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &d2, c2To);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dr[0][0] = d2[0][0] * d1[0][0] + d2[0][1] * d1[0][1] + d2[0][2] * d1[0][2];
   dr[0][1] = d2[0][0] * d1[1][0] + d2[0][1] * d1[1][1] + d2[0][2] * d1[1][2];
   dr[0][2] = d2[0][0] * d1[2][0] + d2[0][1] * d1[2][1] + d2[0][2] * d1[2][2];
   dr[0][3] = -d2[0][0] * d1[0][0] * d1[0][3] - d2[0][0] * d1[1][0] * d1[1][3] - d2[0][0] * d1[2][0] * d1[2][3] - d2[0][1] * d1[0][1] * d1[0][3] - d2[0][1] * d1[1][1] * d1[1][3] - d2[0][1] * d1[2][1] * d1[2][3] - d2[0][2] * d1[0][2] * d1[0][3] - d2[0][2] * d1[1][2] * d1[1][3] - d2[0][2] * d1[2][2] * d1[2][3] + d2[0][3];
   dr[1][0] = d2[1][0] * d1[0][0] + d2[1][1] * d1[0][1] + d2[1][2] * d1[0][2];
   dr[1][1] = d2[1][0] * d1[1][0] + d2[1][1] * d1[1][1] + d2[1][2] * d1[1][2];
   dr[1][2] = d2[1][0] * d1[2][0] + d2[1][1] * d1[2][1] + d2[1][2] * d1[2][2];
   dr[1][3] = -d2[1][0] * d1[0][0] * d1[0][3] - d2[1][0] * d1[1][0] * d1[1][3] - d2[1][0] * d1[2][0] * d1[2][3] - d2[1][1] * d1[0][1] * d1[0][3] - d2[1][1] * d1[1][1] * d1[1][3] - d2[1][1] * d1[2][1] * d1[2][3] - d2[1][2] * d1[0][2] * d1[0][3] - d2[1][2] * d1[1][2] * d1[1][3] - d2[1][2] * d1[2][2] * d1[2][3] + d2[1][3];
   dr[2][0] = d2[2][0] * d1[0][0] + d2[2][1] * d1[0][1] + d2[2][2] * d1[0][2];
   dr[2][1] = d2[2][0] * d1[1][0] + d2[2][1] * d1[1][1] + d2[2][2] * d1[1][2];
   dr[2][2] = d2[2][0] * d1[2][0] + d2[2][1] * d1[2][1] + d2[2][2] * d1[2][2];
   dr[2][3] = -d2[2][0] * d1[0][0] * d1[0][3] - d2[2][0] * d1[1][0] * d1[1][3] - d2[2][0] * d1[2][0] * d1[2][3] - d2[2][1] * d1[0][1] * d1[0][3] - d2[2][1] * d1[1][1] * d1[1][3] - d2[2][1] * d1[2][1] * d1[2][3] - d2[2][2] * d1[0][2] * d1[0][3] - d2[2][2] * d1[1][2] * d1[1][3] - d2[2][2] * d1[2][2] * d1[2][3] + d2[2][3];
   dr[3][0] = 0;
   dr[3][1] = 0;
   dr[3][2] = 0;
   dr[3][3] = 1;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_build_calibration_matrix (
   Rox_Array2D_Double calibmatrix,
   Rox_Double fu,
   Rox_Double fv,
   Rox_Double cu,
   Rox_Double cv
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!calibmatrix)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(calibmatrix, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dcalib = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dcalib, calibmatrix);

   dcalib[0][0] = fu; dcalib[0][1] =  0; dcalib[0][2] = cu;
   dcalib[1][0] =  0; dcalib[1][1] = fv; dcalib[1][2] = cv;
   dcalib[2][0] =  0; dcalib[2][1] =  0; dcalib[2][2] =  1;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_build_calibration_matrix_for_template (
   Rox_MatUT3 calibmatrix,
   const Rox_Double sizeu_pixels,
   const Rox_Double sizev_pixels,
   const Rox_Double sizex_meters,
   const Rox_Double sizey_meters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!calibmatrix)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(calibmatrix, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dcalib = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dcalib, calibmatrix );

   Rox_Double fu = sizeu_pixels / sizex_meters;
   Rox_Double fv = sizev_pixels / sizey_meters;
   Rox_Double cu = (sizeu_pixels-1.0)/2.0;
   Rox_Double cv = (sizev_pixels-1.0)/2.0;

   dcalib[0][0] = fu;   dcalib[0][1] =  0;   dcalib[0][2] = cu;
   dcalib[1][0] =  0;   dcalib[1][1] = fv;   dcalib[1][2] = cv;
   dcalib[2][0] =  0;   dcalib[2][1] =  0;   dcalib[2][2] =  1;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_build_calibration_matrix_for_template_newframe (
   Rox_MatUT3 calibmatrix,
   const Rox_Double sizeu_pixels,
   const Rox_Double sizev_pixels,
   const Rox_Double sizex_meters,
   const Rox_Double sizey_meters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!calibmatrix)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size ( calibmatrix, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dcalib = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dcalib, calibmatrix );

   Rox_Double fu = + sizeu_pixels / sizex_meters;
   Rox_Double fv = - sizev_pixels / sizey_meters;
   Rox_Double cu = (sizeu_pixels-1.0)/2.0;
   Rox_Double cv = (sizev_pixels-1.0)/2.0;

   dcalib[0][0] = fu;   dcalib[0][1] =  0;   dcalib[0][2] = cu;
   dcalib[1][0] =  0;   dcalib[1][1] = fv;   dcalib[1][2] = cv;
   dcalib[2][0] =  0;   dcalib[2][1] =  0;   dcalib[2][2] =  1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_transformtools_pose_random (
   Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double axisx, axisy, axisz, angle, tx, ty, tz, norme;

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   tx = ROX_RAND(0.0, 100.0);
   ty = ROX_RAND(0.0, 100.0);
   tz = ROX_RAND(0.0, 100.0);

   axisx = ROX_RAND(-1.0, 1.0);
   axisy = ROX_RAND(-1.0, 1.0);
   axisz = ROX_RAND(-1.0, 1.0);

   angle = ROX_RAND(0.0, ROX_PI * 2.0);

   norme = sqrt(axisx * axisx + axisy * axisy + axisz * axisz);

   if (ROX_IS_ZERO_DOUBLE(norme))
   {
      axisx = 1.0;
      axisy = 0.0;
      axisz = 0.0;
   }
   else
   {
      axisx = axisx / norme;
      axisy = axisy / norme;
      axisz = axisz / norme;
   }

   error = rox_matse3_set_axis_angle_translation(pose, axisx, axisy, axisz, angle, tx, ty, tz);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_transformtools_compute_homography_from_pts2d_projection_pts3d (
   Rox_Matrix H,
   const Rox_Matrix K,
   const Rox_MatSE3 cTo,
   const Rox_Point3D_Double points3d,
   const Rox_Point2D_Double points2d_ref
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct points2d_cur[4];
   Rox_Double depths[4];

   error = rox_point2d_double_transform_project(points2d_cur, depths, K, cTo, points3d, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_from_4_points_double(H, points2d_ref, points2d_cur);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_transformtools_build_homography (
   Rox_MatSL3 homography,
   const Rox_MatSE3 pose,
   const Rox_MatUT3 calib_output,
   const Rox_Array2D_Double calib_input,
   const Rox_Double a,
   const Rox_Double b,
   const Rox_Double c,
   const Rox_Double d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !homography )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !pose || !calib_input || !calib_output )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_check_size ( homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( calib_input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( calib_output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (fabs(d) < DBL_EPSILON)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dki = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dki, calib_input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dko = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dko, calib_output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double invd = 1.0 / d;

   Rox_Double pxo = dko[0][0];
   Rox_Double pyo = dko[1][1];
   Rox_Double u0o = dko[0][2];
   Rox_Double v0o = dko[1][2];

   Rox_Double pxi = 1.0 / dki[0][0];
   Rox_Double pyi = 1.0 / dki[1][1];
   Rox_Double u0i = -dki[0][2] * pxi;
   Rox_Double v0i = -dki[1][2] * pyi;

   dh[0][0] = - (a * (pxo * dp[0][3] + u0o * dp[2][3]) * invd - pxo * dp[0][0] - u0o * dp[2][0]) * pxi;
   dh[0][1] = -pyi * (b * (pxo * dp[0][3] + u0o * dp[2][3]) * invd - pxo * dp[0][1] - u0o * dp[2][1]);
   dh[0][2] = - (pxo * dp[0][3] + u0o * dp[2][3]) * (u0i * a + b * v0i + c) * invd + (u0i * dp[0][0] + v0i * dp[0][1] + dp[0][2]) * pxo + u0o * (u0i * dp[2][0] + v0i * dp[2][1] + dp[2][2]);
   dh[1][0] = -pxi * (a * (pyo * dp[1][3] + v0o * dp[2][3]) * invd - pyo * dp[1][0] - v0o * dp[2][0]);
   dh[1][1] = -pyi * (b * (pyo * dp[1][3] + v0o * dp[2][3]) * invd - pyo * dp[1][1] - v0o * dp[2][1]);
   dh[1][2] = - (pyo * dp[1][3] + v0o * dp[2][3]) * (u0i * a + b * v0i + c) * invd + (u0i * dp[1][0] + v0i * dp[1][1] + dp[1][2]) * pyo + v0o * (u0i * dp[2][0] + v0i * dp[2][1] + dp[2][2]);
   dh[2][0] = (dp[2][0] - invd * dp[2][3] * a) * pxi;
   dh[2][1] = (dp[2][1] - invd * dp[2][3] * b) * pyi;
   dh[2][2] = -invd * (u0i * a + b * v0i + c) * dp[2][3] + u0i * dp[2][0] + v0i * dp[2][1] + dp[2][2];

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_build_homography_intermodel (
   Rox_Array2D_Double homography,
   Rox_Array2D_Double pose,
   Rox_Array2D_Double calib_output,
   Rox_Array2D_Double calib_input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double **dh, **dki, **dko, **dt;
   Rox_Double pxo, pyo, u0o, v0o;
   Rox_Double pxi, pyi, u0i, v0i;

   if (!homography || !pose || !calib_input || !calib_output)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib_input, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib_output, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dh, homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dki, calib_input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dko, calib_output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   pxo = dko[0][0];
   pyo = dko[1][1];
   u0o = dko[0][2];
   v0o = dko[1][2];

   pxi = 1.0 / dki[0][0];
   pyi = 1.0 / dki[1][1];
   u0i = -dki[0][2] * pxi;
   v0i = -dki[1][2] * pyi;

   dh[0][0] = (pxo * dt[0][0] + u0o * dt[2][0]) * pxi;
   dh[0][1] = (pxo * dt[0][1] + u0o * dt[2][1]) * pyi;
   dh[0][2] = (pxo * dt[0][0] + u0o * dt[2][0]) * u0i + (pxo * dt[0][1] + u0o * dt[2][1]) * v0i + pxo * dt[0][3] + u0o * dt[2][3];
   dh[1][0] = (pyo * dt[1][0] + v0o * dt[2][0]) * pxi;
   dh[1][1] = (pyo * dt[1][1] + v0o * dt[2][1]) * pyi;
   dh[1][2] = (pyo * dt[1][0] + v0o * dt[2][0]) * u0i + (pyo * dt[1][1] + v0o * dt[2][1]) * v0i + pyo * dt[1][3] + v0o * dt[2][3];
   dh[2][0] = dt[2][0] * pxi;
   dh[2][1] = dt[2][1] * pyi;
   dh[2][2] = dt[2][0] * u0i + dt[2][1] * v0i + dt[2][3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_transformtools_build_model_to_image_homography (
   Rox_MatSL3 homography,
   Rox_MatUT3 calib,
   Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!homography || !pose || !calib)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** H = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &H, homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** T = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double fu = K[0][0];
   Rox_Double fv = K[1][1];
   Rox_Double cu = K[0][2];
   Rox_Double cv = K[1][2];
   Rox_Double su = K[0][1];

   H[0][0] = T[2][0]*cu + T[0][0]*fu + T[1][0]*su;
   H[0][1] = T[2][1]*cu + T[0][1]*fu + T[1][1]*su;
   H[0][2] = T[2][3]*cu + T[0][3]*fu + T[1][3]*su;
   H[1][0] = T[2][0]*cv + T[1][0]*fv;
   H[1][1] = T[2][1]*cv + T[1][1]*fv;
   H[1][2] = T[2][3]*cv + T[1][3]*fv;
   H[2][0] = T[2][0];
   H[2][1] = T[2][1];
   H[2][2] = T[2][3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_transformtools_build_model_to_image_homography_from_4_points (
   Rox_MatSL3 homography,
   const Rox_Point3D_Double m,
   const Rox_Point2D_Double p
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct q[4];

   if (!homography || !m || !p)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   for (Rox_Sint k=0; k<4; k++)
   {
      q[k].u = m[k].X;
      q[k].v = m[k].Y;
   }

   // Compute homography such that p ~ homography * q
   error = rox_matsl3_from_4_points_double ( homography, q, p );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_transformtools_build_homography_pointpatch_front (
   Rox_Array2D_Double homography,
   Rox_Array2D_Double pose,
   Rox_Array2D_Double calibration_c1,
   Rox_Array2D_Double calibration_c2,
   Rox_Double pos_u,
   Rox_Double pos_v,
   Rox_Double Z,
   Rox_Uint patchsize
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double buf;
   Rox_Double **dH;
   Rox_Double u, v, w, iw;
   Rox_Sint decal;

   if (!homography || !pose || !calibration_c1 || !calibration_c2)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calibration_c1, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calibration_c2, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&buf, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Build homography
   error = rox_transformtools_build_homography(buf, pose, calibration_c2, calibration_c1, 0, 0, -1, Z);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimate coordinates of reprojected reference coordinates
   error = rox_array2d_double_get_data_pointer_to_pointer( &dH, buf);
   ROX_ERROR_CHECK_TERMINATE ( error );

   u = dH[0][0] * pos_u + dH[0][1] * pos_v + dH[0][2];
   v = dH[1][0] * pos_u + dH[1][1] * pos_v + dH[1][2];
   w = dH[2][0] * pos_u + dH[2][1] * pos_v + dH[2][2];
   if (fabs(w) < DBL_EPSILON)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Compute rHc
   rox_array2d_double_svdinverse(homography, buf);
   iw = 1.0 / w;
   u = u * iw;
   v = v * iw;

   // Shift homography to make the patch centered on the point of interest
   decal = (((Rox_Sint) patchsize) + 2) / 2;
   rox_transformtools_homography_shift(homography, u - (Rox_Double) decal, v - (Rox_Double) decal);

   rox_array2d_double_del(&buf);

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_build_essential (
   Rox_Array2D_Double essential,
   Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!essential || !pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(essential, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dE = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dE, essential);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dE[0][0] = -dt[2][3] * dt[1][0] + dt[1][3] * dt[2][0];
   dE[0][1] = -dt[2][3] * dt[1][1] + dt[1][3] * dt[2][1];
   dE[0][2] = -dt[2][3] * dt[1][2] + dt[1][3] * dt[2][2];
   dE[1][0] =  dt[2][3] * dt[0][0] - dt[0][3] * dt[2][0];
   dE[1][1] =  dt[2][3] * dt[0][1] - dt[0][3] * dt[2][1];
   dE[1][2] =  dt[2][3] * dt[0][2] - dt[0][3] * dt[2][2];
   dE[2][0] = -dt[1][3] * dt[0][0] + dt[0][3] * dt[1][0];
   dE[2][1] = -dt[1][3] * dt[0][1] + dt[0][3] * dt[1][1];
   dE[2][2] = -dt[1][3] * dt[0][2] + dt[0][3] * dt[1][2];

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_build_fundamental (
   Rox_Array2D_Double fundamental,
   Rox_MatSE3 pose,
   Rox_Array2D_Double calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double essential = NULL;

   error = rox_array2d_double_new(&essential, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_transformtools_build_essential(essential, pose);
   ROX_ERROR_CHECK_TERMINATE(error)

   rox_transformtools_build_fundamental_from_essential(fundamental, essential, calib);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   rox_array2d_double_del(&essential);

   return error;
}


Rox_ErrorCode rox_transformtools_build_fundamental_from_essential (
   Rox_Array2D_Double fundamental,
   Rox_Array2D_Double essential,
   Rox_Array2D_Double calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double **df, **de, **dk;
   Rox_Double px, py, u0, v0;
   Rox_Double ipx, ipy, iu0, iv0;

   error = rox_array2d_double_check_size(fundamental, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(essential, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &df, fundamental);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &de, essential);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   px = dk[0][0];
   py = dk[1][1];
   u0 = dk[0][2];
   v0 = dk[1][2];

   ipx = 1.0 / px;
   ipy = 1.0 / py;
   iu0 = - u0 * ipx;
   iv0 = - v0 * ipy;

   df[0][0] = ipx * ipx * de[0][0];
   df[0][1] = ipx * de[0][1] * ipy;
   df[0][2] = ipx * (iu0 * de[0][0] + de[0][1] * iv0 + de[0][2]);

   df[1][0] = ipy * de[1][0] * ipx;
   df[1][1] = ipy * ipy * de[1][1];
   df[1][2] = ipy * (de[1][0] * iu0 + iv0 * de[1][1] + de[1][2]);

   df[2][0] = (iu0 * de[0][0] + iv0 * de[1][0] + de[2][0]) * ipx;
   df[2][1] = (iu0 * de[0][1] + iv0 * de[1][1] + de[2][1]) * ipy;
   df[2][2] = iu0 * iu0 * de[0][0] + ((de[1][0] + de[0][1]) * iv0 + de[2][0] + de[0][2]) * iu0 + iv0 * iv0 * de[1][1] + (de[2][1] + de[1][2]) * iv0 + de[2][2];

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_homography_optimalforward (
   Rox_Sint *dcols,
   Rox_Sint *drows,
   Rox_Array2D_Double homography,
   Rox_Sint scols,
   Rox_Sint srows
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Float_Struct refpts[4];
   Rox_Point2D_Float_Struct curpts[4];
   Rox_Float minx, maxx, miny, maxy;
   Rox_Double rcols, rrows;

   if (!dcols || !drows || !homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   refpts[0].u = 0;
   refpts[0].v = 0;
   refpts[1].u = (Rox_Float) scols;
   refpts[1].v = 0;
   refpts[2].u = (Rox_Float) scols;
   refpts[2].v = (Rox_Float) srows;
   refpts[3].u = 0;
   refpts[3].v = (Rox_Float) srows;

   // Compute destination properties
   error = rox_point2d_float_homography(curpts, refpts, homography, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   minx = curpts[0].u;
   if (curpts[1].u < minx) minx = curpts[1].u;
   if (curpts[2].u < minx) minx = curpts[2].u;
   if (curpts[3].u < minx) minx = curpts[3].u;
   maxx = curpts[0].u;
   if (curpts[1].u > maxx) maxx = curpts[1].u;
   if (curpts[2].u > maxx) maxx = curpts[2].u;
   if (curpts[3].u > maxx) maxx = curpts[3].u;
   miny = curpts[0].v;
   if (curpts[1].v < miny) miny = curpts[1].v;
   if (curpts[2].v < miny) miny = curpts[2].v;
   if (curpts[3].v < miny) miny = curpts[3].v;
   maxy = curpts[0].v;
   if (curpts[1].v > maxy) maxy = curpts[1].v;
   if (curpts[2].v > maxy) maxy = curpts[2].v;
   if (curpts[3].v > maxy) maxy = curpts[3].v;

   rcols = maxx - minx;
   rrows = maxy - miny;

   *dcols = (Rox_Uint) (rcols + 0.5f);
   *drows = (Rox_Uint) (rrows + 0.5f);

   error = rox_transformtools_homography_shiftleft(homography, -minx, -miny);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_matrix33_left_pyramidzoom (
   Rox_Array2D_Double zoomed,
   const Rox_MatUT3 calibration,
   const Rox_Uint level
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Array2D_Double S  = NULL;
   Rox_Array2D_Double Si = NULL;

   if (!zoomed || !calibration)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(zoomed, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calibration, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double scale = scale = pow(2.0, (int)level);

   error = rox_array2d_double_new(&Si, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dsi = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dsi, Si);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dsi[0][0] = 1.0/scale;  dsi[0][1] = 0.0;        dsi[0][2] = 0;//-0.5 + 0.5 / scale;
   dsi[1][0] = 0.0;        dsi[1][1] = 1.0/scale;  dsi[1][2] = 0;//-0.5 + 0.5 / scale;
   dsi[2][0] = 0.0;        dsi[2][1] = 0.0;        dsi[2][2] = 1.0;

   error = rox_array2d_double_mulmatmat(zoomed, Si, calibration);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   // rox_array2d_double_del(&S);
   rox_array2d_double_del(&Si);

   return error;
}


Rox_ErrorCode rox_transformtools_matrix33_left_pyramidzoom_exact (
   Rox_Array2D_Double zoomed,
   Rox_Array2D_Double calibration,
   Rox_Uint level
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double scale;
   Rox_Double **ds;
   Rox_Array2D_Double S  = NULL;
   Rox_Array2D_Double Si = NULL;

   if (!zoomed || !calibration)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(zoomed, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calibration, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   scale = pow(2.0, (int)level);

   error = rox_array2d_double_new(&S, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Si, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &ds, S);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ds[0][0] = scale;    ds[0][1] = 0.0;   ds[0][2] = 0;
   ds[1][0] = 0.0;      ds[1][1] = scale; ds[1][2] = 0;
   ds[2][0] = 0.0;      ds[2][1] = 0.0;   ds[2][2] = 1.0;

   error = rox_array2d_double_svdinverse(Si, S);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(zoomed, Si, calibration);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&Si);

   return error;
}


Rox_ErrorCode rox_transformtools_matrix33_right_pyramidzoom (
   Rox_Array2D_Double zoomed,
   Rox_Array2D_Double calibration,
   Rox_Uint level
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double S = NULL;

   if (!zoomed || !calibration)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size ( zoomed, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( calibration, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &S, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** ds = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &ds, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double scale = pow(2.0, (int)level);

   ds[0][0] = scale;    ds[0][1] = 0.0;   ds[0][2] = 0; // (scale - 1.0) * 0.5;
   ds[1][0] = 0.0;      ds[1][1] = scale; ds[1][2] = 0; // (scale - 1.0) * 0.5;
   ds[2][0] = 0.0;      ds[2][1] = 0.0;   ds[2][2] = 1.0;

   error = rox_array2d_double_mulmatmat ( zoomed, calibration, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&S);
   return error;
}


Rox_ErrorCode rox_transformtools_matrix33_right_pyramidzoominv (
   Rox_Array2D_Double zoomed,
   Rox_Array2D_Double calibration,
   Rox_Uint level
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double S  = NULL;
   Rox_Array2D_Double Si = NULL;

   if (!zoomed || !calibration)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   error = rox_array2d_double_check_size(zoomed, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calibration, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double scale = pow(2.0, (int)level);

   error = rox_array2d_double_new(&S, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Si, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** ds = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &ds, S);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ds[0][0] = scale;    ds[0][1] = 0.0;   ds[0][2] = 0;//(scale - 1.0) * 0.5;
   ds[1][0] = 0.0;      ds[1][1] = scale; ds[1][2] = 0;//(scale - 1.0) * 0.5;
   ds[2][0] = 0.0;      ds[2][1] = 0.0;   ds[2][2] = 1.0;

   error = rox_array2d_double_svdinverse(Si, S);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(zoomed, calibration, Si);

function_terminate:
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&Si);
   return error;
}


Rox_ErrorCode rox_transformtools_affine_from_homography (
   Rox_Array2D_Double affine,
   Rox_Array2D_Double homography,
   Rox_Double u,
   Rox_Double v
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!affine || !homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(affine, 2, 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, homography);

   Rox_Double ** da = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &da, affine);

   Rox_Double X = dh[0][0] * u + dh[0][1] * v + dh[0][2];
   Rox_Double Y = dh[1][0] * u + dh[1][1] * v + dh[1][2];
   Rox_Double W = dh[2][0] * u + dh[2][1] * v + dh[2][2];

   if (fabs(W) < DBL_EPSILON)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Affine = d(w(H.p))/d(p)
   Rox_Double invWW = 1.0 / (W * W);
   da[0][0] = (dh[0][0] * W - X * dh[2][0]) * invWW;
   da[0][1] = (dh[0][1] * W - X * dh[2][1]) * invWW;
   da[1][0] = (dh[1][0] * W - Y * dh[2][0]) * invWW;
   da[1][1] = (dh[1][1] * W - Y * dh[2][1]) * invWW;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_homography_shift (
   Rox_Array2D_Double homography,
   Rox_Double dx,
   Rox_Double dy
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, homography);

   // homography = homography * [1, 0, dx; 0, 1, dy; 0 0 1]

   dh[0][2] = dh[0][0] * dx + dh[0][1] * dy + dh[0][2];
   dh[1][2] = dh[1][0] * dx + dh[1][1] * dy + dh[1][2];
   dh[2][2] = dh[2][0] * dx + dh[2][1] * dy + dh[2][2];

function_terminate:
   return error;
}

Rox_ErrorCode rox_transformtools_homography_shiftleft (
   Rox_Array2D_Double homography,
   Rox_Double dx,
   Rox_Double dy
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, homography);

   // homography = [1, 0, dx; 0, 1, dy; 0 0 1] * homography

   dh[0][0] = dh[0][0] + dh[2][0] * dx;
   dh[0][1] = dh[0][1] + dh[2][1] * dx;
   dh[0][2] = dh[0][2] + dh[2][2] * dx;

   dh[1][0] = dh[1][0] + dh[2][0] * dy;
   dh[1][1] = dh[1][1] + dh[2][1] * dy;
   dh[1][2] = dh[1][2] + dh[2][2] * dy;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_homography_get_areazoom (
   Rox_Double * zoom,
   Rox_Array2D_Double homography,
   Rox_Double u,
   Rox_Double v
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!zoom || !homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size ( homography, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** h = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &h, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double det = (-h[2][0] * h[0][2] * h[1][1] + h[2][0] * h[0][1] * h[1][2] - h[2][1] * h[0][0] * h[1][2] - h[0][1] * h[1][0] * h[2][2] + h[0][2] * h[2][1] * h[1][0] + h[0][0] * h[2][2] * h[1][1]) * pow(h[2][0] * (u-1) + h[2][1] * (v-1) + h[2][2], -0.3e1);
   *zoom = det;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_build_pose_intermodel (
   Rox_MatSE3 pose,
   Rox_MatSL3 homography,
   Rox_MatUT3 calibration
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double H = NULL;
   Rox_Array2D_Double U = NULL;
   Rox_Array2D_Double S = NULL;
   Rox_Array2D_Double V = NULL;
   Rox_Array2D_Double iK = NULL;

   Rox_Array2D_Double R = NULL;
   Rox_Array2D_Double t = NULL;

   Rox_Double **dH = NULL;
   Rox_Double **dt = NULL;
   Rox_Double **dS = NULL;

   Rox_Double scale = 1.0;
   Rox_Double determinant = 0.0;

   if (!pose || !homography || !calibration )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size (pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size (homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size (calibration, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new (&H, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new (&U, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new (&S, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&V, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &iK, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d ( &R, pose, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d ( &t, pose, 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dH, H );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dt, t );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dS, S );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svdinverse ( iK, calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  The intermodel homography has the form Ge = scale * K * [r1 r2 t]
   //  Compute the normalized intermodel homography He = inv(K)*Ge = scale * [r1 r2 t]
   error = rox_array2d_double_mulmatmat ( H, iK, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  The scaled translation ts = scale*t is the third column of He
   dt[0][0] = dH[0][2];
   dt[1][0] = dH[1][2];
   dt[2][0] = dH[2][2];

   //  Set the third colum to zero before svd decomposition
   dH[0][2] = 0.0;
   dH[1][2] = 0.0;
   dH[2][2] = 0.0;

   //  Now He = scale * [r1 r2 0]
   //  SVD of the matrix He = U*S*V'. Ideally S = [scale, 0, 0; 0, scale, 0; 0, 0, 0]
   error = rox_array2d_double_svd ( U, S, V, H );
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  The matrix Re = U*V' is the estimated rotation
   error = rox_array2d_double_mulmatmattrans ( R, U, V );
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  Recover the scale even if the singular values are not ordered
   scale = 0.5 * (dS[0][0] + dS[1][0] + dS[2][0]);

   error = rox_array2d_double_detgl3(&determinant, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // check if rotation has a negative determinant (left hand frame)
   if(determinant < 0.0)
   {
      // scale the third column to get a rigth hand frame rotation matrix
      error = rox_array2d_double_scale_col ( R, R, -1.0, 2 );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   //  Compute the true unscaled translation
   if (fabs(scale) > 0.0)
   {
      scale = 1.0 / scale;
      dt[0][0] = dt[0][0] * scale;
      dt[1][0] = dt[1][0] * scale;
      dt[2][0] = dt[2][0] * scale;
   }

function_terminate:
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&t);
   rox_array2d_double_del(&H);
   rox_array2d_double_del(&V);
   rox_array2d_double_del(&U);
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&iK);
   return error;
}


Rox_ErrorCode rox_transformtools_imagerotation (
   Rox_Array2D_Double homography,
   Rox_Sint cols,
   Rox_Sint rows,
   Rox_Double angle
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!homography || !angle)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (cols < 2 || rows < 2)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double hh = (double)rows / 2.0;
   Rox_Double hw = (double)cols / 2.0;

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, homography );

   dh[0][0] = cos(angle); dh[0][1] = -sin(angle); dh[0][2] = -cos(angle)*hw + sin(angle)*hh + hw;
   dh[1][0] = sin(angle); dh[1][1] =  cos(angle); dh[1][2] = -sin(angle)*hw - cos(angle)*hh + hh;
   dh[2][0] = 0; dh[2][1] = 0; dh[2][2] = 1;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_updateZref (
   Rox_MatSE3 updated_pose,
   Rox_MatSE3 pose,
   Rox_Double zref
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_check_size ( updated_pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dd, updated_pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dd[0][0] = dt[0][0]; dd[0][1] = dt[0][1]; dd[0][2] = dt[0][2]; dd[0][3] = dt[0][3] + zref*dt[0][2];
   dd[1][0] = dt[1][0]; dd[1][1] = dt[1][1]; dd[1][2] = dt[1][2]; dd[1][3] = dt[1][3] + zref*dt[1][2];
   dd[2][0] = dt[2][0]; dd[2][1] = dt[2][1]; dd[2][2] = dt[2][2]; dd[2][3] = dt[2][3] + zref*dt[2][2];
   dd[3][0] = 0; dd[3][1] = 0; dd[3][2] = 0; dd[3][3] = 1;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_estimate_relativepose_from_general (
   Rox_Array2D_Double corpose,
   Rox_Array2D_Double cxTc0,
   Rox_Array2D_Double pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double bufpose = NULL, bufrot = NULL, E = NULL, cxRc0= NULL, R = NULL, cR= NULL, buf33_1= NULL, buf33_2= NULL, buf33_3= NULL, B= NULL;
   Rox_Double ** dt = NULL, **db = NULL;

   error = rox_array2d_double_check_size ( corpose, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( cxTc0, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( pose, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &E, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &B, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &bufrot, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&buf33_1, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&buf33_2, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&buf33_3, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&bufpose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&cR, corpose, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&R, pose, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&cxRc0, cxTc0, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, corpose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &db, buf33_1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Build reference essential
   error = rox_transformtools_build_essential(E, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_essential(B, cxTc0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimate rotation for this camera
   error = rox_array2d_double_mulmatmattrans(bufrot, R, cxRc0);
   ROX_ERROR_CHECK_TERMINATE ( error ); //RA'

   error = rox_array2d_double_mulmatmat(cR, cxRc0, bufrot);
   ROX_ERROR_CHECK_TERMINATE ( error ); //ARA'

   // Estimate E
   error = rox_array2d_double_mulmatmattrans(buf33_1, R, B);
   ROX_ERROR_CHECK_TERMINATE ( error ); //RB'

   error = rox_array2d_double_mulmatmattrans(buf33_2, E, cxRc0);
   ROX_ERROR_CHECK_TERMINATE ( error ); //EA'

   error = rox_array2d_double_add(buf33_3, buf33_1, buf33_2);
   ROX_ERROR_CHECK_TERMINATE ( error ); //(RB' + EA')

   error = rox_array2d_double_mulmatmat(buf33_1, cxRc0, buf33_3);
   ROX_ERROR_CHECK_TERMINATE ( error ); //A(RB' + EA')

   error = rox_array2d_double_mulmatmat(buf33_2, B, bufrot);
   ROX_ERROR_CHECK_TERMINATE ( error ); //BRA'

   error = rox_array2d_double_add(buf33_3, buf33_1, buf33_2);
   ROX_ERROR_CHECK_TERMINATE ( error ); //A(RB' + EA') + BRA'

   error = rox_array2d_double_mulmatmattrans(buf33_1, buf33_3, cR);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dt[0][3] = db[2][1];
   dt[1][3] = db[0][2];
   dt[2][3] = db[1][0];

function_terminate:

   rox_array2d_double_del ( &bufpose );
   rox_array2d_double_del ( &E );
   rox_array2d_double_del ( &B );
   rox_array2d_double_del ( &R );
   rox_array2d_double_del ( &cxRc0 );
   rox_array2d_double_del ( &bufrot );
   rox_array2d_double_del ( &buf33_1 );
   rox_array2d_double_del ( &buf33_2 );
   rox_array2d_double_del ( &buf33_3 );
   rox_array2d_double_del ( &cR );

   return error;
}

Rox_ErrorCode rox_cayley_xy_from_vectors (
   Rox_Array2D_Double r,
   Rox_Array2D_Double const v1,
   Rox_Array2D_Double const v2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double s[3], d[3];
   Rox_Double * r_data = NULL, * n1_data = NULL, * n2_data = NULL;
   Rox_Array2D_Double n1 = NULL, n2 = NULL;

   if ( !r || !v1 || !v2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new ( &n1, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &n2, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Normalize input vectors
   // n1=v1/norm(v1);
   // n2=v2/norm(v2);

   error = rox_array2d_double_normalize ( n1, v1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_normalize ( n2, v2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer ( &r_data, r );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer ( &n1_data, n1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer ( &n2_data, n2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // From Cayley : r = u*tan(theta/2)
   // We look for u = [ux,uy,0]
   // Then r = [rx,ry,0]
   // R = inv(I-skew(r))*(I+skew(r))
   // R * n1 = n2
   // (I+skew(v))*n1 = (I-skew(r))*n2
   // skew(n1+n2) * r = n1-n2
   // s = n1+n2;
   // d = n1-n2;
   // skew(s)*r = [-sz*ry;sz*rx;sx*ry - sy*rx]
   //
   // rc(1,1) = +d(2,1)/s(3,1);
   // rc(2,1) = -d(1,1)/s(3,1);
   // rc(3,1) = 0.0;

   // s = n1+n2;
   s[0] = n1_data[0]+n2_data[0]; s[1] = n1_data[1]+n2_data[1]; s[2] = n1_data[2]+n2_data[2];
   // d = n1-n2;
   d[0] = n1_data[0]-n2_data[0]; d[1] = n1_data[1]-n2_data[1]; d[2] = n1_data[2]-n2_data[2];

   if(s[2] > 0.0)
   {
      r_data[0] = +d[1]/s[2];
      r_data[1] = -d[0]/s[2];
      r_data[2] = 0.0;
   }
   else
   {
      error = ROX_ERROR_ALGORITHM_FAILURE;
   }

function_terminate:
   rox_array2d_double_del(&n1);
   rox_array2d_double_del(&n2);
   return error;
}

// Z ∈ [n, f]
// z ∈ [-1, 1]
//
// z = ( f + n ) / ( f - n ) + ( 1 / Z ) . ( -2.f.n ) / ( f - n )
//  <=>
// Z = 2.f.n / ( ( f + n ) - ( f - n ).z )
//
Rox_ErrorCode convert_depth_ogl_metric (
   Rox_Float * Z,
   const Rox_Float Z_gl,
   const Rox_Float Z_near,
   const Rox_Float Z_far
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( NULL == Z )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // [0, 1] -> [-1, 1]
   Rox_Float z = (Rox_Float) (( Z_gl - 0.5 ) * 2.0);

   // [-1, 1] -> [n, f]
   *Z = 2*Z_far*Z_near / ( Z_far + Z_near - ( Z_far - Z_near ) * z );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_depth_ogl_convert_to_metric (
   Rox_Array2D_Float Z,
   Rox_Imask Z_mask,
   const Rox_Array2D_Float Z_ogl,
   const Rox_Float Z_near,
   const Rox_Float Z_far
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !Z )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, Z_ogl );

   Rox_Float ** Z_ogl_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Z_ogl_data, Z_ogl );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Z_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Z_data, Z );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** Z_mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &Z_mask_data, Z_mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint v = 0; v < rows; ++v )
   {
      for ( Rox_Sint u = 0; u < cols; ++u )
      {
         if ( fabs(Z_ogl_data[v][u]-1.0) > FLT_EPSILON )
         {
            // [0, 1] -> [-1, 1]
            Rox_Float z = (Rox_Float) (( Z_ogl_data[v][u] - 0.5f ) * 2.0f);

            // [-1, 1] -> [n, f]
            Z_data[rows-1 - v][u] = (2*Z_far*Z_near) / ( Z_far + Z_near - ( Z_far - Z_near ) * z );

            Z_mask_data[ rows-1 - v][u] = ~0;
         }
         else
         {
            Z_data[rows-1 - v][u] = 0.0;

            Z_mask_data[ rows-1 - v][u] = 0;
         }

         // if ((u == 1000) && ( rows-1-v == 600))
         // {
         //    rox_log("Z = %f\n", Z_data[rows-1 - v][u]);
         // }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_inverse_depth_ogl_convert_to_metric (
   Rox_Array2D_Float Zi,
   Rox_Imask Zi_mask,
   const Rox_Array2D_Float Z_ogl,
   const Rox_Float Z_near,
   const Rox_Float Z_far
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !Zi )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, Z_ogl );

   Rox_Float ** Z_ogl_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Z_ogl_data, Z_ogl );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Zi_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Zi_data, Zi );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** Zi_mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &Zi_mask_data, Zi_mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint v = 0; v < rows; ++v )
   {
      for ( Rox_Sint u = 0; u < cols; ++u )
      {
         if ( fabs(Z_ogl_data[v][u]-1.0) > FLT_EPSILON )
         {
            // [0, 1] -> [-1, 1]
            Rox_Float z = (Rox_Float) (( Z_ogl_data[v][u] - 0.5f ) * 2.0f);

            // [-1, 1] -> [n, f]
            Zi_data[rows-1 - v][u] = ( Z_far + Z_near - ( Z_far - Z_near ) * z ) / (2*Z_far*Z_near) ;

            Zi_mask_data[ rows-1 - v][u] = ~0;
         }
         else
         {
            Zi_data[rows-1 - v][u] = 0.0;

            Zi_mask_data[ rows-1 - v][u] = 0;
         }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_transformtools_build_matsa3_matrix (
   Rox_Array2D_Double cQr,
   Rox_MatSE3 cTr,
   Rox_MatUT3 Kc,
   Rox_MatUT3 Kr
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !cQr )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !cTr || !Kc || ! Kr )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** cQr_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &cQr_data, cQr );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Kr_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Kr_data, Kr );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Kc_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Kc_data, Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** cTr_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &cTr_data, cTr);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ifu = 1.0 / Kr_data[0][0];
   Rox_Double ifv = 1.0 / Kr_data[1][1];
   Rox_Double icu = -Kr_data[0][2] * ifu;
   Rox_Double icv = -Kr_data[1][2] * ifv;

   Rox_Double fuo = Kc_data[0][0];
   Rox_Double fvo = Kc_data[1][1];
   Rox_Double cuo = Kc_data[0][2];
   Rox_Double cvo = Kc_data[1][2];

   cQr_data[0][0] = (fuo * cTr_data[0][0] + cuo * cTr_data[2][0]) * ifu;
   cQr_data[0][1] = (fuo * cTr_data[0][1] + cuo * cTr_data[2][1]) * ifv;
   cQr_data[0][2] = (icu * cTr_data[0][0] + icv * cTr_data[0][1] + cTr_data[0][2]) * fuo + cuo * (cTr_data[2][0] * icu + cTr_data[2][1] * icv + cTr_data[2][2]);
   cQr_data[0][3] = fuo * cTr_data[0][3] + cuo * cTr_data[2][3];
   cQr_data[1][0] = (fvo * cTr_data[1][0] + cvo * cTr_data[2][0]) * ifu;
   cQr_data[1][1] = (fvo * cTr_data[1][1] + cvo * cTr_data[2][1]) * ifv;
   cQr_data[1][2] = (icu * cTr_data[1][0] + icv * cTr_data[1][1] + cTr_data[1][2]) * fvo + cvo * (cTr_data[2][0] * icu + cTr_data[2][1] * icv + cTr_data[2][2]);
   cQr_data[1][3] = fvo * cTr_data[1][3] + cvo * cTr_data[2][3];
   cQr_data[2][0] = cTr_data[2][0] * ifu;
   cQr_data[2][1] = cTr_data[2][1] * ifv;
   cQr_data[2][2] = cTr_data[2][0] * icu + cTr_data[2][1] * icv + cTr_data[2][2];
   cQr_data[2][3] = cTr_data[2][3];

   cQr_data[3][0] = 0.0;
   cQr_data[3][1] = 0.0;
   cQr_data[3][2] = 0.0;
   cQr_data[3][3] = 1.0;

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_build_matsl3_from_matso3_matut3 (
   Rox_MatSL3 H,
   Rox_MatSO3 R,
   Rox_MatUT3 K
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !H )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !R || !K )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** H_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &H_data, H );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** R_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &R_data, R );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ifu = 1.0 / K_data[0][0];
   Rox_Double ifv = 1.0 / K_data[1][1];
   Rox_Double icu = -K_data[0][2] * ifu;
   Rox_Double icv = -K_data[1][2] * ifv;

   Rox_Double fuo = K_data[0][0];
   Rox_Double fvo = K_data[1][1];
   Rox_Double cuo = K_data[0][2];
   Rox_Double cvo = K_data[1][2];

   H_data[0][0] = (fuo * R_data[0][0] + cuo * R_data[2][0]) * ifu;
   H_data[0][1] = (fuo * R_data[0][1] + cuo * R_data[2][1]) * ifv;
   H_data[0][2] = (icu * R_data[0][0] + icv * R_data[0][1] + R_data[0][2]) * fuo + cuo * (R_data[2][0] * icu + R_data[2][1] * icv + R_data[2][2]);
   H_data[1][0] = (fvo * R_data[1][0] + cvo * R_data[2][0]) * ifu;
   H_data[1][1] = (fvo * R_data[1][1] + cvo * R_data[2][1]) * ifv;
   H_data[1][2] = (icu * R_data[1][0] + icv * R_data[1][1] + R_data[1][2]) * fvo + cvo * (R_data[2][0] * icu + R_data[2][1] * icv + R_data[2][2]);
   H_data[2][0] = R_data[2][0] * ifu;
   H_data[2][1] = R_data[2][1] * ifv;
   H_data[2][2] = R_data[2][0] * icu + R_data[2][1] * icv + R_data[2][2];


function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_double_new_from_point2d_float ( Rox_Array2D_Double * array2d, const Rox_Point2D_Float point2d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double data[3] = {0.0, 0.0, 1.0};

   error = rox_array2d_double_new ( array2d , 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   data[0] = point2d->u;
   data[1] = point2d->v;

   error = rox_array2d_double_set_buffer_no_stride( *array2d, data );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_transformtools_build_matsl3_from_pose ( 
   Rox_MatSL3 cHt, 
   const Rox_MatUT3 K, 
   const Rox_MatSE3 cTo, 
   const Rox_MatSE3 pTo,  
   const Rox_Plane3D_Double plane3d_p
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_MatSE3 oTp = NULL;
   Rox_MatSE3 cTp = NULL;
   Rox_MatSE3 tTp = NULL;
   Rox_MatSE3 cTt = NULL;

   // =========================================================================

   error = rox_matse3_new ( &oTp ); 
   if(error) goto function_terminate;

   error = rox_matse3_inv ( oTp, pTo ); 
   if(error) goto function_terminate;

   // Compute pose cTp
   error = rox_matse3_new ( &cTp ); 
   if(error) goto function_terminate;

   // Compute cTp = cTo * oTp
   error = rox_matse3_mulmatmat ( cTp, cTo, oTp ); 
   if(error) goto function_terminate;

   // Get translation from cTp
   Rox_Double tra[3] = {0.0,0.0,0.0};

   error = rox_matse3_get_tra ( tra, cTp );
   if(error) goto function_terminate;

   Rox_Double distance = sqrt(tra[0]*tra[0]+tra[1]*tra[1]+tra[2]*tra[2]);

   error = rox_matse3_new ( &tTp ); 
   if(error) goto function_terminate;

   tra[0] = 0.0; tra[1] = 0.0; tra[2] = distance;

   error = rox_matse3_set_translation ( tTp, tra );
   if(error) goto function_terminate;

   // Compute cTt = cTp*pTt
   error = rox_matse3_new ( &cTt ); 
   if(error) goto function_terminate;

   error = rox_matse3_mulmatinv ( cTt, cTp, tTp );
   if(error) goto function_terminate;

   Rox_Plane3D_Double_Struct plane3d_t;
   error = rox_plane3d_transform ( &plane3d_t, tTp, plane3d_p );
   if(error) goto function_terminate;

   // Build homography
   error = rox_transformtools_build_homography ( cHt, cTt, K, K, plane3d_t.a, plane3d_t.b, plane3d_t.c, plane3d_t.d );
   if(error) goto function_terminate;

   // =========================================================================

function_terminate:
   rox_matse3_del ( &cTt ); 
   rox_matse3_del ( &cTp ); 
   rox_matse3_del ( &oTp ); 
   rox_matse3_del ( &tTp ); 

   return error;
}

