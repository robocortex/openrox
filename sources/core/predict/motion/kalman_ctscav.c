//==============================================================================
//
//    OPENROX   : File kalman_ctscav.c
//
//    Contents  : Implementation of checkerboard detect module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "kalman_ctscav.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#define CTSCAV_SIZE 13
#define POSE_SIZE 7

Rox_ErrorCode rox_kalman_ctscav_new(Rox_Kalman_Ctscav *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Kalman_Ctscav ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *obj = NULL;

   ret = (Rox_Kalman_Ctscav) rox_memory_allocate(sizeof(struct Rox_Kalman_Ctscav_Struct), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->state = NULL;
   ret->covariance = NULL;
   ret->State_Innovation = NULL;
   ret->Covariance_Innovation = NULL;
   ret->Jacobian_Prediction = NULL;
   ret->Jacobian_Measure = NULL;
   ret->Covariance_Prediction = NULL;
   ret->Covariance_Measure = NULL;
   ret->Gain = NULL;
   ret->Eye_SizeXSize = NULL;
   ret->Buffer_SizeXSize = NULL;
   ret->Buffer_SizeXSize_2 = NULL;
   ret->Buffer_NXSize = NULL;
   ret->Buffer_NXN = NULL;
   ret->Buffer_3X3 = NULL;
   ret->Buffer_SizeXN = NULL;
   ret->Buffer_SizeX1 = NULL;
   ret->Measure_Predicted = NULL;

   error = rox_array2d_double_new(&ret->state, CTSCAV_SIZE, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->covariance, CTSCAV_SIZE, CTSCAV_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Measure_Predicted, POSE_SIZE, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->State_Innovation, POSE_SIZE, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Covariance_Innovation, POSE_SIZE, POSE_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Jacobian_Prediction, CTSCAV_SIZE, CTSCAV_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Jacobian_Measure, POSE_SIZE, CTSCAV_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Covariance_Prediction, CTSCAV_SIZE, CTSCAV_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Covariance_Measure, POSE_SIZE, POSE_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Gain, CTSCAV_SIZE, POSE_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Eye_SizeXSize, CTSCAV_SIZE, CTSCAV_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Buffer_SizeXSize, CTSCAV_SIZE, CTSCAV_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Buffer_SizeXSize_2, CTSCAV_SIZE, CTSCAV_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Buffer_NXSize, POSE_SIZE, CTSCAV_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Buffer_SizeXN, CTSCAV_SIZE, POSE_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Buffer_SizeX1, CTSCAV_SIZE, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Buffer_NXN, POSE_SIZE, POSE_SIZE);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Buffer_3X3, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(ret->state, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(ret->state, 3, 0, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(ret->covariance, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(ret->Covariance_Prediction);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(ret->Covariance_Measure);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(ret->Eye_SizeXSize);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_kalman_ctscav_del(&ret);
   return error;
}

Rox_ErrorCode rox_kalman_ctscav_del(Rox_Kalman_Ctscav *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Kalman_Ctscav todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_array2d_double_del(&todel->state);
   rox_array2d_double_del(&todel->Measure_Predicted);
   rox_array2d_double_del(&todel->covariance);
   rox_array2d_double_del(&todel->State_Innovation);
   rox_array2d_double_del(&todel->Covariance_Innovation);
   rox_array2d_double_del(&todel->Jacobian_Prediction);
   rox_array2d_double_del(&todel->Jacobian_Measure);
   rox_array2d_double_del(&todel->Covariance_Prediction);
   rox_array2d_double_del(&todel->Covariance_Measure);
   rox_array2d_double_del(&todel->Eye_SizeXSize);
   rox_array2d_double_del(&todel->Buffer_SizeXSize);
   rox_array2d_double_del(&todel->Buffer_SizeXSize_2);
   rox_array2d_double_del(&todel->Buffer_NXSize);
   rox_array2d_double_del(&todel->Buffer_SizeXN);
   rox_array2d_double_del(&todel->Buffer_SizeX1);
   rox_array2d_double_del(&todel->Buffer_NXN);
   rox_array2d_double_del(&todel->Buffer_3X3);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_compose_rotations_quaternion(Rox_Double * q3, Rox_Double *q1, Rox_Double *q2)
{
   Rox_Double norm = 0.0;

   q3[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
   q3[1] = q1[1] * q2[0] + q1[0] * q2[1] - q1[3] * q2[2] + q1[2] * q2[3];
   q3[2] = q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2] - q1[1] * q2[3];
   q3[3] = q1[3] * q2[0] - q1[2] * q2[1] + q1[1] * q2[2] + q1[0] * q2[3];

   norm = sqrt(q3[0]*q3[0] + q3[1]*q3[1] + q3[2]*q3[2]);
   
   q3[0] = q3[0] / norm;
   q3[1] = q3[1] / norm;
   q3[2] = q3[2] / norm;
   q3[3] = q3[3] / norm;

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_compose_rotations_axis_angle(Rox_Double * r3, Rox_Double * pf1, Rox_Double * pf2, Rox_Double * pf3, Rox_Double * pg1, Rox_Double * pg2, Rox_Double * pg3, Rox_Double * r1, Rox_Double * r2)
{
   Rox_Double t1,t2;
   Rox_Double f1,f2,f3;
   Rox_Double g1,g2,g3;
   Rox_Double sinc_g1,sinc_g2;
   Rox_Double cos_g3, sin_g3;
   Rox_Double isinc_g3;

   t1 = sqrt(r1[0] * r1[0] + r1[1] * r1[1] + r1[2] * r1[2]);
   t2 = sqrt(r2[0] * r2[0] + r2[1] * r2[1] + r2[2] * r2[2]);

   g1 = t1 / 2.0;
   if (g1 > DBL_EPSILON)
   {
      sinc_g1 = sin(g1)/g1;
   }
   else
   {
      sinc_g1 = 1;
   }

   g2 = t2 / 2.0;
   if (g2 > DBL_EPSILON)
   {
      sinc_g2 = sin(g2)/g2;
   }
   else
   {
      sinc_g2 = 1;
   }

   cos_g3 = cos(g1) * cos(g2) - sinc_g1 * sinc_g2 * (r1[0]*r2[0] + r1[1]*r2[1] + r1[2]*r2[2]) / 4.0;
   sin_g3 = sqrt(1.0 - cos_g3 * cos_g3);
   g3 = atan2(sin_g3, cos_g3);

   if (g3 > DBL_EPSILON)
   {
      isinc_g3 = g3 / sin_g3;
   }
   else
   {
      isinc_g3 = 1;
   }
   
   f1 = isinc_g3 * cos(g2) * sinc_g1;
   f2 = isinc_g3 * cos(g1) * sinc_g2;
   f3 = isinc_g3 * sinc_g1 * sinc_g2 / 2.0;

   r3[0] = f1 * r1[0] + f2 * r2[0] - f3 * r1[2] * r2[1] + f3 * r1[1] * r2[2];
   r3[1] = f1 * r1[1] + f2 * r2[1] + f3 * r1[2] * r2[0] - f3 * r1[0] * r2[2];
   r3[2] = f1 * r1[2] + f2 * r2[2] - f3 * r1[1] * r2[0] + f3 * r1[0] * r2[1];

   *pf1 = f1;
   *pf2 = f2;
   *pf3 = f3;

   *pg1 = g1;
   *pg2 = g2;
   *pg3 = g3;

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_kalman_ctscav_prediction_jacobian(Rox_Kalman_Ctscav obj, Rox_Double dt)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double * state;
   // Rox_Double *t;
	Rox_Double *v, *r, *w;
   Rox_Double ** J;
   Rox_Double vdt[3], x, wdt[3],nwdt;
   Rox_Double qw[4];
   Rox_Double Jqw_wdt[4][3];
   Rox_Double Jrp_qw[4][4];
   Rox_Uint i,j,k;
   Rox_Double f1,f2;
   Rox_Double M[3][3];
   Rox_Double Mv[3], *m;
   Rox_Double JMv_w[3][3];
   Rox_Double ** R;
   Rox_Double a, b[3];
   Rox_Double JRq_r[3][4];

   error = rox_array2d_double_get_data_pointer ( &state, obj->state);

   // t = state + 0;
   r = state + 3;
   v = state + 7;
   w = state + 10;

   error = rox_array2d_double_get_data_pointer_to_pointer( &J, obj->Jacobian_Prediction);
   error = rox_array2d_double_get_data_pointer_to_pointer( &R, obj->Buffer_3X3);

   rox_array2d_double_fillval(obj->Jacobian_Prediction, 0);

   // Rotation
   wdt[0] = w[0] * dt;
   wdt[1] = w[1] * dt;
   wdt[2] = w[2] * dt;
   nwdt = sqrt(wdt[0]*wdt[0] + wdt[1]*wdt[1] + wdt[2]*wdt[2]);
   if (nwdt > DBL_EPSILON)
   {
      qw[0] = cos(nwdt / 2.0);
      qw[1] = sin(nwdt / 2.0) * wdt[0] / nwdt;
      qw[2] = sin(nwdt / 2.0) * wdt[1] / nwdt;
      qw[3] = sin(nwdt / 2.0) * wdt[2] / nwdt;

      Jqw_wdt[0][0] = -sin(nwdt / 0.2e1) / nwdt * wdt[0] / 0.2e1;
      Jqw_wdt[0][1] = -sin(nwdt / 0.2e1) / nwdt * wdt[1] / 0.2e1;
      Jqw_wdt[0][2] = -sin(nwdt / 0.2e1) / nwdt * wdt[2] / 0.2e1;
      Jqw_wdt[1][0] = sin(nwdt / 0.2e1) / nwdt + pow(nwdt, -0.3e1) * (nwdt * cos(nwdt / 0.2e1) - 0.2e1 * sin(nwdt / 0.2e1)) * pow(wdt[0], 0.2e1) / 0.2e1;
      Jqw_wdt[1][1] = pow(nwdt, -0.3e1) * (nwdt * cos(nwdt / 0.2e1) - 0.2e1 * sin(nwdt / 0.2e1)) * wdt[0] * wdt[1] / 0.2e1;
      Jqw_wdt[1][2] = pow(nwdt, -0.3e1) * (nwdt * cos(nwdt / 0.2e1) - 0.2e1 * sin(nwdt / 0.2e1)) * wdt[0] * wdt[2] / 0.2e1;
      Jqw_wdt[2][0] = pow(nwdt, -0.3e1) * (nwdt * cos(nwdt / 0.2e1) - 0.2e1 * sin(nwdt / 0.2e1)) * wdt[0] * wdt[1] / 0.2e1;
      Jqw_wdt[2][1] = sin(nwdt / 0.2e1) / nwdt + pow(nwdt, -0.3e1) * (nwdt * cos(nwdt / 0.2e1) - 0.2e1 * sin(nwdt / 0.2e1)) * pow(wdt[1], 0.2e1) / 0.2e1;
      Jqw_wdt[2][2] = pow(nwdt, -0.3e1) * (nwdt * cos(nwdt / 0.2e1) - 0.2e1 * sin(nwdt / 0.2e1)) * wdt[1] * wdt[2] / 0.2e1;
      Jqw_wdt[3][0] = pow(nwdt, -0.3e1) * (nwdt * cos(nwdt / 0.2e1) - 0.2e1 * sin(nwdt / 0.2e1)) * wdt[0] * wdt[2] / 0.2e1;
      Jqw_wdt[3][1] = pow(nwdt, -0.3e1) * (nwdt * cos(nwdt / 0.2e1) - 0.2e1 * sin(nwdt / 0.2e1)) * wdt[1] * wdt[2] / 0.2e1;
      Jqw_wdt[3][2] = sin(nwdt / 0.2e1) / nwdt + pow(nwdt, -0.3e1) * (nwdt * cos(nwdt / 0.2e1) - 0.2e1 * sin(nwdt / 0.2e1)) * pow(wdt[2], 0.2e1) / 0.2e1;
   }
   else
   {
      Jqw_wdt[0][0] = 0;
      Jqw_wdt[0][1] = 0;
      Jqw_wdt[0][2] = 0;
      Jqw_wdt[1][0] = 0.5;
      Jqw_wdt[1][1] = 0;
      Jqw_wdt[1][2] = 0;
      Jqw_wdt[2][0] = 0;
      Jqw_wdt[2][1] = 0.5;
      Jqw_wdt[2][2] = 0;
      Jqw_wdt[3][0] = 0;
      Jqw_wdt[3][1] = 0;
      Jqw_wdt[3][2] = 0.5;

      qw[0] = 1;
      qw[1] = 0;
      qw[2] = 0;
      qw[3] = 0;
   }

   Jrp_qw[0][0] = r[0];
   Jrp_qw[0][1] = -r[1];
   Jrp_qw[0][2] = -r[2];
   Jrp_qw[0][3] = -r[3];
   Jrp_qw[1][0] = r[1];
   Jrp_qw[1][1] = r[0];
   Jrp_qw[1][2] = -r[3];
   Jrp_qw[1][3] = r[2];
   Jrp_qw[2][0] = r[2];
   Jrp_qw[2][1] = r[3];
   Jrp_qw[2][2] = r[0];
   Jrp_qw[2][3] = -r[1];
   Jrp_qw[3][0] = r[3];
   Jrp_qw[3][1] = -r[2];
   Jrp_qw[3][2] = r[1];

   for (i = 0; i < 4; i++)
   {
      for (j = 0; j < 3; j++)
      {
         J[3 + i][10 + j] = 0;

         for (k = 0; k < 4; k++)
         {
            J[3 + i][10 + j] += Jrp_qw[i][k] * Jqw_wdt[k][j];
         }

         J[3 + i][10 + j] *= dt;
      }
   }

   J[3][3] = qw[0];
   J[3][4] = -qw[1];
   J[3][5] = -qw[2];
   J[3][6] = -qw[3];
   J[4][3] = qw[1];
   J[4][4] = qw[0];
   J[4][5] = qw[3];
   J[4][6] = -qw[2];
   J[5][3] = qw[2];
   J[5][4] = -qw[3];
   J[5][5] = qw[0];
   J[5][6] = qw[1];
   J[6][3] = qw[3];
   J[6][4] = qw[2];
   J[6][5] = -qw[1];
   J[6][6] = qw[0];

   // Translation
   vdt[0] = v[0] * dt;
   vdt[1] = v[1] * dt;
   vdt[2] = v[2] * dt;

   x = sqrt(wdt[0]*wdt[0]+wdt[1]*wdt[1]+wdt[2]*wdt[2]);
   if (x > DBL_EPSILON)
   {
      f1 = pow(sinc(x / ROX_PI / 0.2e1), 0.2e1) / 0.2e1;
      f2 = (double) (1.0 - sinc(x / ROX_PI)) * pow(x, -0.2e1);

      M[0][0] = 0.1e1 + f2 * (-pow(wdt[2], 0.2e1) - pow(wdt[1], 0.2e1));
      M[0][1] = -f1 * wdt[2] + f2 * wdt[1] * wdt[0];
      M[0][2] = f1 * wdt[1] + f2 * wdt[2] * wdt[0];
      M[1][0] = f1 * wdt[2] + f2 * wdt[1] * wdt[0];
      M[1][1] = 0.1e1 + f2 * (-pow(wdt[2], 0.2e1) - pow(wdt[0], 0.2e1));
      M[1][2] = -f1 * wdt[0] + f2 * wdt[2] * wdt[1];
      M[2][0] = -f1 * wdt[1] + f2 * wdt[2] * wdt[0];
      M[2][1] = f1 * wdt[0] + f2 * wdt[2] * wdt[1];
      M[2][2] = 0.1e1 + f2 * (-pow(wdt[1], 0.2e1) - pow(wdt[0], 0.2e1));

      Mv[0] = M[0][0] * vdt[0] + M[0][1] * vdt[1] + M[0][2] * vdt[2];
      Mv[1] = M[1][0] * vdt[0] + M[1][1] * vdt[1] + M[1][2] * vdt[2];
      Mv[2] = M[2][0] * vdt[0] + M[2][1] * vdt[1] + M[2][2] * vdt[2];

      JMv_w[0][0] = dt * (x * x * wdt[0] * (-vdt[1] * wdt[2] + vdt[2] * wdt[1]) * (x * cos(x / 0.2e1) - 0.2e1 * sin(x / 0.2e1)) * sinc(x / ROX_PI / 0.2e1) - wdt[0] * ((wdt[2] * vdt[2] + wdt[1] * vdt[1]) * wdt[0] - vdt[0] * (pow(wdt[2], 0.2e1) + pow(wdt[1], 0.2e1))) * x * cos(x) + 0.3e1 * wdt[0] * ((wdt[2] * vdt[2] + wdt[1] * vdt[1]) * wdt[0] - vdt[0] * (pow(wdt[2], 0.2e1) + pow(wdt[1], 0.2e1))) * sin(x) + ((-0.2e1 * wdt[2] * vdt[2] - 0.2e1 * wdt[1] * vdt[1]) * pow(wdt[0], 0.2e1) + 0.2e1 * vdt[0] * (pow(wdt[2], 0.2e1) + pow(wdt[1], 0.2e1)) * wdt[0] + pow(x, 0.4e1) * f2 * (wdt[2] * vdt[2] + wdt[1] * vdt[1])) * x) * pow(x, -0.5e1);
      JMv_w[0][1] = (x * x * wdt[1] * (-vdt[1] * wdt[2] + vdt[2] * wdt[1]) * (x * cos(x / 0.2e1) - 0.2e1 * sin(x / 0.2e1)) * sinc(x / ROX_PI / 0.2e1) - wdt[1] * (-pow(wdt[1], 0.2e1) * vdt[0] + wdt[0] * wdt[1] * vdt[1] + wdt[2] * (vdt[2] * wdt[0] - vdt[0] * wdt[2])) * x * cos(x) + 0.3e1 * wdt[1] * (-pow(wdt[1], 0.2e1) * vdt[0] + wdt[0] * wdt[1] * vdt[1] + wdt[2] * (vdt[2] * wdt[0] - vdt[0] * wdt[2])) * sin(x) + (0.2e1 * vdt[0] * pow(wdt[1], 0.3e1) - 0.2e1 * wdt[0] * pow(wdt[1], 0.2e1) * vdt[1] + (-0.2e1 * wdt[2] * vdt[2] * wdt[0] + 0.2e1 * pow(wdt[2], 0.2e1) * vdt[0] - 0.2e1 * f2 * vdt[0] * pow(x, 0.4e1)) * wdt[1] + pow(x, 0.4e1) * (vdt[1] * wdt[0] * f2 + f1 * vdt[2])) * x) * dt * pow(x, -0.5e1);
      JMv_w[0][2] = dt * (x * x * wdt[2] * (-vdt[1] * wdt[2] + vdt[2] * wdt[1]) * (x * cos(x / 0.2e1) - 0.2e1 * sin(x / 0.2e1)) * sinc(x / ROX_PI / 0.2e1) - (-pow(wdt[2], 0.2e1) * vdt[0] + wdt[2] * vdt[2] * wdt[0] + wdt[1] * (vdt[1] * wdt[0] - vdt[0] * wdt[1])) * wdt[2] * x * cos(x) + 0.3e1 * (-pow(wdt[2], 0.2e1) * vdt[0] + wdt[2] * vdt[2] * wdt[0] + wdt[1] * (vdt[1] * wdt[0] - vdt[0] * wdt[1])) * wdt[2] * sin(x) + (0.2e1 * vdt[0] * pow(wdt[2], 0.3e1) - 0.2e1 * wdt[0] * pow(wdt[2], 0.2e1) * vdt[2] + (-0.2e1 * wdt[0] * wdt[1] * vdt[1] - 0.2e1 * f2 * vdt[0] * pow(x, 0.4e1) + 0.2e1 * pow(wdt[1], 0.2e1) * vdt[0]) * wdt[2] + pow(x, 0.4e1) * (vdt[2] * wdt[0] * f2 - f1 * vdt[1])) * x) * pow(x, -0.5e1);
      JMv_w[1][0] = -(x * x * wdt[0] * (vdt[2] * wdt[0] - vdt[0] * wdt[2]) * (x * cos(x / 0.2e1) - 0.2e1 * sin(x / 0.2e1)) * sinc(x / ROX_PI / 0.2e1) + (-vdt[1] * pow(wdt[0], 0.2e1) + wdt[1] * wdt[0] * vdt[0] + wdt[2] * (-vdt[1] * wdt[2] + vdt[2] * wdt[1])) * wdt[0] * x * cos(x) - 0.3e1 * (-vdt[1] * pow(wdt[0], 0.2e1) + wdt[1] * wdt[0] * vdt[0] + wdt[2] * (-vdt[1] * wdt[2] + vdt[2] * wdt[1])) * wdt[0] * sin(x) + (-0.2e1 * vdt[1] * pow(wdt[0], 0.3e1) + 0.2e1 * wdt[1] * pow(wdt[0], 0.2e1) * vdt[0] + (0.2e1 * wdt[1] * vdt[2] * wdt[2] + 0.2e1 * f2 * vdt[1] * pow(x, 0.4e1) - 0.2e1 * vdt[1] * pow(wdt[2], 0.2e1)) * wdt[0] + pow(x, 0.4e1) * (-vdt[0] * wdt[1] * f2 + f1 * vdt[2])) * x) * dt * pow(x, -0.5e1);
      JMv_w[1][1] = dt * (-x * x * wdt[1] * (vdt[2] * wdt[0] - vdt[0] * wdt[2]) * (x * cos(x / 0.2e1) - 0.2e1 * sin(x / 0.2e1)) * sinc(x / ROX_PI / 0.2e1) + wdt[1] * ((-wdt[2] * vdt[2] - wdt[0] * vdt[0]) * wdt[1] + vdt[1] * (pow(wdt[2], 0.2e1) + pow(wdt[0], 0.2e1))) * x * cos(x) - 0.3e1 * wdt[1] * ((-wdt[2] * vdt[2] - wdt[0] * vdt[0]) * wdt[1] + vdt[1] * (pow(wdt[2], 0.2e1) + pow(wdt[0], 0.2e1))) * sin(x) + ((-0.2e1 * wdt[2] * vdt[2] - 0.2e1 * wdt[0] * vdt[0]) * pow(wdt[1], 0.2e1) + 0.2e1 * vdt[1] * (pow(wdt[2], 0.2e1) + pow(wdt[0], 0.2e1)) * wdt[1] + pow(x, 0.4e1) * f2 * (wdt[0] * vdt[0] + wdt[2] * vdt[2])) * x) * pow(x, -0.5e1);
      JMv_w[1][2] = (-x * x * wdt[2] * (vdt[2] * wdt[0] - vdt[0] * wdt[2]) * (x * cos(x / 0.2e1) - 0.2e1 * sin(x / 0.2e1)) * sinc(x / ROX_PI / 0.2e1) - x * wdt[2] * (-vdt[1] * pow(wdt[2], 0.2e1) + wdt[1] * vdt[2] * wdt[2] - vdt[1] * pow(wdt[0], 0.2e1) + wdt[1] * wdt[0] * vdt[0]) * cos(x) + 0.3e1 * wdt[2] * (-vdt[1] * pow(wdt[2], 0.2e1) + wdt[1] * vdt[2] * wdt[2] - vdt[1] * pow(wdt[0], 0.2e1) + wdt[1] * wdt[0] * vdt[0]) * sin(x) + (0.2e1 * vdt[1] * pow(wdt[2], 0.3e1) - 0.2e1 * wdt[1] * pow(wdt[2], 0.2e1) * vdt[2] + (0.2e1 * vdt[1] * pow(wdt[0], 0.2e1) - 0.2e1 * wdt[1] * wdt[0] * vdt[0] - 0.2e1 * f2 * vdt[1] * pow(x, 0.4e1)) * wdt[2] + pow(x, 0.4e1) * (vdt[2] * wdt[1] * f2 + f1 * vdt[0])) * x) * dt * pow(x, -0.5e1);
      JMv_w[2][0] = -0.2e1 * dt * (-x * x * wdt[0] * (vdt[1] * wdt[0] - vdt[0] * wdt[1]) * (x * cos(x / 0.2e1) - 0.2e1 * sin(x / 0.2e1)) * sinc(x / ROX_PI / 0.2e1) / 0.2e1 - wdt[0] * (vdt[2] * pow(wdt[0], 0.2e1) - wdt[2] * wdt[0] * vdt[0] + wdt[1] * (-vdt[1] * wdt[2] + vdt[2] * wdt[1])) * x * cos(x) / 0.2e1 + 0.3e1 / 0.2e1 * wdt[0] * (vdt[2] * pow(wdt[0], 0.2e1) - wdt[2] * wdt[0] * vdt[0] + wdt[1] * (-vdt[1] * wdt[2] + vdt[2] * wdt[1])) * sin(x) + (-vdt[2] * pow(wdt[0], 0.3e1) + wdt[2] * pow(wdt[0], 0.2e1) * vdt[0] + (-vdt[2] * pow(wdt[1], 0.2e1) + vdt[1] * wdt[2] * wdt[1] + f2 * vdt[2] * pow(x, 0.4e1)) * wdt[0] - pow(x, 0.4e1) * (vdt[0] * wdt[2] * f2 + f1 * vdt[1]) / 0.2e1) * x) * pow(x, -0.5e1);
      JMv_w[2][1] = -0.2e1 * dt * (-x * x * wdt[1] * (vdt[1] * wdt[0] - vdt[0] * wdt[1]) * (x * cos(x / 0.2e1) - 0.2e1 * sin(x / 0.2e1)) * sinc(x / ROX_PI / 0.2e1) / 0.2e1 - wdt[1] * (vdt[2] * pow(wdt[1], 0.2e1) - vdt[1] * wdt[2] * wdt[1] + wdt[0] * (vdt[2] * wdt[0] - vdt[0] * wdt[2])) * x * cos(x) / 0.2e1 + 0.3e1 / 0.2e1 * wdt[1] * (vdt[2] * pow(wdt[1], 0.2e1) - vdt[1] * wdt[2] * wdt[1] + wdt[0] * (vdt[2] * wdt[0] - vdt[0] * wdt[2])) * sin(x) + (-vdt[2] * pow(wdt[1], 0.3e1) + wdt[2] * pow(wdt[1], 0.2e1) * vdt[1] + (wdt[2] * wdt[0] * vdt[0] + f2 * vdt[2] * pow(x, 0.4e1) - vdt[2] * pow(wdt[0], 0.2e1)) * wdt[1] - pow(x, 0.4e1) * (vdt[1] * wdt[2] * f2 - f1 * vdt[0]) / 0.2e1) * x) * pow(x, -0.5e1);
      JMv_w[2][2] = dt * (x * x * wdt[2] * (vdt[1] * wdt[0] - vdt[0] * wdt[1]) * (x * cos(x / 0.2e1) - 0.2e1 * sin(x / 0.2e1)) * sinc(x / ROX_PI / 0.2e1) + wdt[2] * ((-wdt[0] * vdt[0] - wdt[1] * vdt[1]) * wdt[2] + vdt[2] * (pow(wdt[0], 0.2e1) + pow(wdt[1], 0.2e1))) * x * cos(x) - 0.3e1 * wdt[2] * ((-wdt[0] * vdt[0] - wdt[1] * vdt[1]) * wdt[2] + vdt[2] * (pow(wdt[0], 0.2e1) + pow(wdt[1], 0.2e1))) * sin(x) + ((-0.2e1 * wdt[0] * vdt[0] - 0.2e1 * wdt[1] * vdt[1]) * pow(wdt[2], 0.2e1) + 0.2e1 * vdt[2] * (pow(wdt[0], 0.2e1) + pow(wdt[1], 0.2e1)) * wdt[2] + pow(x, 0.4e1) * f2 * (wdt[0] * vdt[0] + wdt[1] * vdt[1])) * x) * pow(x, -0.5e1);
   }
   else
   {
      JMv_w[0][0] = 0;
      JMv_w[0][1] = -vdt[2] * (-dt/2);
      JMv_w[0][2] = vdt[1] * (-dt/2);
      JMv_w[1][0] = vdt[2] * (-dt/2);
      JMv_w[1][1] = 0;
      JMv_w[1][2] = -vdt[0] * (-dt/2);
      JMv_w[2][0] = -vdt[1] * (-dt/2);
      JMv_w[2][1] = vdt[0] * (-dt/2);
      JMv_w[2][2] = 0;

      M[0][0] = 1;
      M[0][1] = 0;
      M[0][2] = 0;
      M[1][0] = 0;
      M[1][1] = 1;
      M[1][2] = 0;
      M[2][0] = 0;
      M[2][1] = 0;
      M[2][2] = 1;

      Mv[0] = vdt[0];
      Mv[1] = vdt[1];
      Mv[2] = vdt[2];
   }

   error = rox_transformtools_rotationmatrix_from_quaternion(obj->Buffer_3X3, r[0], r[1], r[2], r[3]);
   ROX_ERROR_CHECK_TERMINATE(error)

   a = r[0];
   b[0] = r[1];
   b[1] = r[2];
   b[2] = r[3];
   m = Mv;

   JRq_r[0][0] = -2 * b[2] * m[1] + 2 * b[1] * m[2];
   JRq_r[0][1] = 2 * b[1] * m[1] + 2 * b[2] * m[2];
   JRq_r[0][2] = 2 * a * m[2] + 2 * b[0] * m[1] - 4 * b[1] * m[0];
   JRq_r[0][3] = -2 * a * m[1] + 2 * b[0] * m[2] - 4 * b[2] * m[0];
   JRq_r[1][0] = 2 * b[2] * m[0] - 2 * b[0] * m[2];
   JRq_r[1][1] = -2 * a * m[2] + 2 * b[1] * m[0] - 4 * b[0] * m[1];
   JRq_r[1][2] = 2 * b[0] * m[0] + 2 * b[2] * m[2];
   JRq_r[1][3] = 2 * a * m[0] + 2 * b[1] * m[2] - 4 * b[2] * m[1];
   JRq_r[2][0] = -2 * b[1] * m[0] + 2 * b[0] * m[1];
   JRq_r[2][1] = 2 * a * m[1] + 2 * b[2] * m[0] - 4 * b[0] * m[2];
   JRq_r[2][2] = -2 * a * m[0] + 2 * b[2] * m[1] - 4 * b[1] * m[2];
   JRq_r[2][3] = 2 * b[0] * m[0] + 2 * b[1] * m[1];

   J[0][0] = 1;
   J[1][1] = 1;
   J[2][2] = 1;

   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 4; j++)
      {
         J[i][3 + j] = JRq_r[i][j];
      }
   }

   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 3; j++)
      {
         J[i][7 + j] = 0;
         J[i][10 + j] = 0;

         for (k = 0; k < 3; k++)
         {
            J[i][7 + j] += R[i][k] * M[k][j];
            J[i][10 + j] += R[i][k] * JMv_w[k][j];
         }

         J[i][7 + j] *= dt;
      }
   }

   // v
   J[7][7] = 1;
   J[8][8] = 1;
   J[9][9] = 1;

   // w
   J[10][10] = 1;
   J[11][11] = 1;
   J[12][12] = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_kalman_ctscav_measurement_jacobian(Rox_Kalman_Ctscav obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_fillunit(obj->Jacobian_Measure);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_kalman_ctscav_prediction_step(Rox_Kalman_Ctscav obj, Rox_Double dt)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double *t = NULL, *v = NULL, *r = NULL, *w = NULL;
   Rox_Double *state = NULL;
   Rox_Double nw;
   Rox_Double whdt[3];
   Rox_Double vhdt[3];
   Rox_Double M[3][3];
   Rox_Double th[3];
   Rox_Double ** R = NULL;
   Rox_Double qw[4];
   Rox_Double qr[4];

   error = rox_array2d_double_get_data_pointer( &state, obj->state);
   t = state + 0;
   r = state + 3;
   v = state + 7;
   w = state + 10;

   vhdt[0] = v[0] * dt;
   vhdt[1] = v[1] * dt;
   vhdt[2] = v[2] * dt;
   whdt[0] = w[0] * dt;
   whdt[1] = w[1] * dt;
   whdt[2] = w[2] * dt;

   error = rox_transformtools_rotationmatrix_from_quaternion(obj->Buffer_3X3, r[0], r[1], r[2], r[3]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &R, obj->Buffer_3X3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   nw = sqrt(whdt[0] * whdt[0] + whdt[1] * whdt[1] + whdt[2] * whdt[2]);
   if (nw > DBL_EPSILON)
   {
      M[0][0] = 0.1e1 + (0.1e1 - sin(nw) / nw) * (-pow(whdt[2], 0.2e1) - pow(whdt[1], 0.2e1));
      M[0][1] = -(0.1e1 - cos(nw)) * pow(nw, -0.2e1) * whdt[2] + (0.1e1 - sin(nw) / nw) * whdt[1] * whdt[0];
      M[0][2] = (0.1e1 - cos(nw)) * pow(nw, -0.2e1) * whdt[1] + (0.1e1 - sin(nw) / nw) * whdt[2] * whdt[0];
      M[1][0] = (0.1e1 - cos(nw)) * pow(nw, -0.2e1) * whdt[2] + (0.1e1 - sin(nw) / nw) * whdt[1] * whdt[0];
      M[1][1] = 0.1e1 + (0.1e1 - sin(nw) / nw) * (-pow(whdt[2], 0.2e1) - pow(whdt[0], 0.2e1));
      M[1][2] = -(0.1e1 - cos(nw)) * pow(nw, -0.2e1) * whdt[0] + (0.1e1 - sin(nw) / nw) * whdt[2] * whdt[1];
      M[2][0] = -(0.1e1 - cos(nw)) * pow(nw, -0.2e1) * whdt[1] + (0.1e1 - sin(nw) / nw) * whdt[2] * whdt[0];
      M[2][1] = (0.1e1 - cos(nw)) * pow(nw, -0.2e1) * whdt[0] + (0.1e1 - sin(nw) / nw) * whdt[2] * whdt[1];
      M[2][2] = 0.1e1 + (0.1e1 - sin(nw) / nw) * (-pow(whdt[1], 0.2e1) - pow(whdt[0], 0.2e1));

      qw[0] = cos(nw / 2.0);
      qw[1] = whdt[0] * sin(nw / 2.0) / nw;
      qw[2] = whdt[1] * sin(nw / 2.0) / nw;
      qw[3] = whdt[2] * sin(nw / 2.0) / nw;
   }
   else
   {
      M[0][0] = 1;
      M[0][1] = 0;
      M[0][2] = 0;
      M[1][0] = 0;
      M[1][1] = 1;
      M[1][2] = 0;
      M[2][0] = 0;
      M[2][1] = 0;
      M[2][2] = 1;

      qw[0] = 1;
      qw[1] = 0;
      qw[2] = 0;
      qw[3] = 0;
   }

   th[0] = M[0][0] * vhdt[0] + M[0][1] * vhdt[1] + M[0][2] * vhdt[2];
   th[1] = M[1][0] * vhdt[0] + M[1][1] * vhdt[1] + M[1][2] * vhdt[2];
   th[2] = M[2][0] * vhdt[0] + M[2][1] * vhdt[1] + M[2][2] * vhdt[2];

   t[0] = t[0] + R[0][0] * th[0] + R[0][1] * th[1] + R[0][2] * th[2];
   t[1] = t[1] + R[1][0] * th[1] + R[1][1] * th[1] + R[1][2] * th[2];
   t[2] = t[2] + R[2][0] * th[2] + R[2][1] * th[1] + R[2][2] * th[2];

   rox_compose_rotations_quaternion(qr, r, qw);

   r[0] = qr[0];
   r[1] = qr[1];
   r[2] = qr[2];
   r[3] = qr[3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_kalman_ctscav_predict(Rox_Kalman_Ctscav obj, Rox_Double dt)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_kalman_ctscav_prediction_jacobian(obj, dt);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_kalman_ctscav_prediction_step(obj, dt);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(obj->Buffer_SizeXSize, obj->Jacobian_Prediction, obj->covariance);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmattrans(obj->covariance, obj->Buffer_SizeXSize, obj->Jacobian_Prediction);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_add(obj->covariance, obj->covariance, obj->Covariance_Prediction);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:   
   return error;
}

Rox_ErrorCode rox_kalman_ctscav_compute_innovation(Rox_Kalman_Ctscav obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_kalman_ctscav_measurement_jacobian(obj);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(obj->Buffer_NXSize, obj->Jacobian_Measure, obj->covariance);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmattrans(obj->Covariance_Innovation, obj->Buffer_NXSize, obj->Jacobian_Measure);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_add(obj->Covariance_Innovation, obj->Covariance_Innovation, obj->Covariance_Measure);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_kalman_ctscav_normalize(Rox_Kalman_Ctscav obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ** J = NULL;
   Rox_Double * state = NULL;
   Rox_Double q1,q2,q3,q4, scale;

   error = rox_array2d_double_fillunit(obj->Buffer_SizeXSize);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_get_data_pointer ( &state, obj->state);
   error = rox_array2d_double_get_data_pointer_to_pointer( &J, obj->Buffer_SizeXSize);

   q1 = state[3];
   q2 = state[4];
   q3 = state[5];
   q4 = state[6];

   state[3] = q1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
   state[4] = q2 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
   state[5] = q3 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
   state[6] = q4 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);

   scale = pow(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4, -0.3e1 / 0.2e1);

   J[3][3] = (q2 * q2 + q3 * q3 + q4 * q4) * scale;
   J[3][4] = (-q1 * q2) * scale;
   J[3][5] = (-q1 * q3) * scale;
   J[3][6] = (-q1 * q4) * scale;
   J[4][3] = (-q1 * q2) * scale;
   J[4][4] = (q1 * q1 + q3 * q3 + q4 * q4) * scale;
   J[4][5] = (-q2 * q3) * scale;
   J[4][6] = (-q2 * q4) * scale;
   J[5][3] = (-q1 * q3) * scale;
   J[5][4] = (-q2 * q3) * scale;
   J[5][5] = (q1 * q1 + q2 * q2 + q4 * q4) * scale;
   J[5][6] = (-q3 * q4) * scale;
   J[6][3] = (-q1 * q4) * scale;
   J[6][4] = (-q2 * q4) * scale;
   J[6][5] = (-q3 * q4) * scale;
   J[6][6] = (q1 * q1 + q2 * q2 + q3 * q3) * scale;

   error = rox_array2d_double_mulmatmattrans(obj->Buffer_SizeXSize_2, obj->covariance, obj->Buffer_SizeXSize);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(obj->covariance, obj->Buffer_SizeXSize, obj->Buffer_SizeXSize_2);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_kalman_ctscav_update(Rox_Kalman_Ctscav obj, Rox_Array2D_Double measure)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double * state = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_get_data_pointer( &state, obj->state);
   rox_array2d_double_set_value(obj->Measure_Predicted, 0, 0, state[0]);
   rox_array2d_double_set_value(obj->Measure_Predicted, 1, 0, state[1]);
   rox_array2d_double_set_value(obj->Measure_Predicted, 2, 0, state[2]);
   rox_array2d_double_set_value(obj->Measure_Predicted, 3, 0, state[3]);
   rox_array2d_double_set_value(obj->Measure_Predicted, 4, 0, state[4]);
   rox_array2d_double_set_value(obj->Measure_Predicted, 5, 0, state[5]);
   rox_array2d_double_set_value(obj->Measure_Predicted, 6, 0, state[6]);

   error = rox_array2d_double_svdinverse(obj->Buffer_NXN, obj->Covariance_Innovation);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmattransmat(obj->Buffer_SizeXN, obj->Jacobian_Measure, obj->Buffer_NXN);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(obj->Gain, obj->covariance, obj->Buffer_SizeXN);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(obj->Buffer_SizeXSize, obj->Gain, obj->Jacobian_Measure);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_substract(obj->Buffer_SizeXSize_2, obj->Eye_SizeXSize, obj->Buffer_SizeXSize);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(obj->Buffer_SizeXSize, obj->Buffer_SizeXSize_2, obj->covariance);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_copy(obj->covariance, obj->Buffer_SizeXSize);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_substract(obj->State_Innovation, measure, obj->Measure_Predicted);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat(obj->Buffer_SizeX1, obj->Gain, obj->State_Innovation);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_add(obj->state, obj->state, obj->Buffer_SizeX1);
   ROX_ERROR_CHECK_TERMINATE(error)

   // error = rox_kalman_ctscav_normalize(obj);
   // ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:   
   return error;
}

Rox_ErrorCode rox_kalman_ctscav_set_prediction_variances(Rox_Kalman_Ctscav obj, Rox_Uint index, Rox_Double variance)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (index >= CTSCAV_SIZE) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dout, obj->Covariance_Prediction);

   dout[index][index] = variance;

function_terminate:
   return error;
}

Rox_ErrorCode rox_kalman_ctscav_set_measurement_variances(Rox_Kalman_Ctscav obj, Rox_Uint index, Rox_Double variance)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (index >= POSE_SIZE) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Double ** dout = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dout, obj->Covariance_Measure);

   dout[index][index] = variance;

function_terminate:
   return error;
}
