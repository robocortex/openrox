//==============================================================================
//
//    OPENROX   : File calibration_generalized.c
//
//    Contents  : Implementation of calibration_generalized module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "calibration_generalized.h"
#include "calibration_generalized_struct.h"

#include <generated/dynvec_point_double_struct.h>
#include <generated/dynvec_point2d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/nonlin/polynomials.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/determinant/detgl3.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_calibration_mono_generalized_new (
   Rox_Calibration_Mono_Generalized * obj,
   Rox_Uint polynomial_order
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Calibration_Mono_Generalized ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Calibration_Mono_Generalized) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->points = NULL;
   ret->points_undistorted = NULL;
   ret->viewposes = NULL;
   ret->coefficients = NULL;
   ret->coefficients_buf = NULL;
   ret->roots = NULL;
   ret->coefficients_inv = NULL;
   ret->map = NULL;
   ret->mapmask = NULL;

   error = rox_objset_dynvec_point_double_new(&ret->points, 10);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_objset_dynvec_point_double_new(&ret->points_undistorted, 10);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_objset_array2d_double_new(&ret->viewposes, 10);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_dynvec_double_new(&ret->coefficients_inv, 10);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&ret->coefficients, polynomial_order + 1, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&ret->coefficients_buf, polynomial_order + 1, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&ret->roots, polynomial_order, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   ret->order = polynomial_order;
   ret->width = 128;
   ret->height = 128;

   *obj = ret;

function_terminate:
   // Delete only if an error occurs
   if (error) rox_calibration_mono_generalized_del(&ret);

   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_del(Rox_Calibration_Mono_Generalized *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_Mono_Generalized todel = NULL;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if(!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_objset_dynvec_point_double_del(&todel->points);
   rox_objset_dynvec_point_double_del(&todel->points_undistorted);
   rox_objset_array2d_double_del(&todel->viewposes);
   rox_array2d_double_del(&todel->coefficients);
   rox_array2d_double_del(&todel->coefficients_buf);
   rox_array2d_double_del(&todel->roots);
   rox_dynvec_double_del(&todel->coefficients_inv);
   rox_array2d_uint_del(&todel->mapmask);
   rox_array2d_point2d_float_del(&todel->map);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_set_imagesize (
   Rox_Calibration_Mono_Generalized obj,
   Rox_Sint width,
   Rox_Sint height
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   obj->width = width;
   obj->height = height;
   obj->image_center.u = width / 2;
   obj->image_center.v = height / 2;

   rox_array2d_point2d_float_del(&obj->map);
   rox_array2d_uint_del(&obj->mapmask);

   error = rox_array2d_point2d_float_new ( &obj->map, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new ( &obj->mapmask, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if(error)
   {
      rox_array2d_point2d_float_del(&obj->map);
      rox_array2d_uint_del(&obj->mapmask);
   }

   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_append_view(
   Rox_Calibration_Mono_Generalized obj,
   Rox_Point_Double points,
   Rox_Uint count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point_Double buf = NULL;

   if (!obj || !points) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   {
      buf = NULL;

      error = rox_dynvec_point_double_new(&buf, 10);
      ROX_ERROR_CHECK_TERMINATE ( error );

      for (Rox_Uint i = 0; i < count; i++)
      {
         error = rox_dynvec_point_double_append(buf, &points[i]);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      error = rox_objset_dynvec_point_double_append(obj->points, buf);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   {
      buf = NULL;

      error = rox_dynvec_point_double_new(&buf, 10);
      ROX_ERROR_CHECK_TERMINATE(error)

      for (Rox_Uint i = 0; i < count; i++)
      {
         error = rox_dynvec_point_double_append(buf, &points[i]);
         ROX_ERROR_CHECK_TERMINATE(error)
      }

      error = rox_objset_dynvec_point_double_append(obj->points_undistorted, buf);
      ROX_ERROR_CHECK_TERMINATE(error)
   }


function_terminate:
   if (error) rox_dynvec_point_double_del(&buf);
   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_compute_parameters_oneview(Rox_Calibration_Mono_Generalized obj, Rox_Uint idview)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double M, U, S, V, R, aR, AtA, Atd, invAtA, vecX;
   Rox_DynVec_Point_Double curvec;
   Rox_Double ** dm, **dv, **dr, **data, **datd, **dx;
   Rox_Uint idpt, idroot, idsign, idsign2;
   Rox_Double uc, vc, X, Y, scale;
   Rox_Double r11, r12, r21, r22, t1, t2, r32, r31;
   Rox_Double MA, MB, MC, MD, rho, rho2;
   Rox_Complex_Struct roots[2];
   Rox_Double coeffs[3];
   Rox_Double sign[2];

   sign[0] = -1.0;
   sign[1] = 1.0;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   M = NULL;
   U = NULL;
   S = NULL;
   V = NULL;
   R = NULL;
   aR = NULL;
   AtA = NULL;
   invAtA = NULL;
   Atd = NULL;
   vecX = NULL;

   if (idview >= obj->points->used) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}
   curvec = obj->points->data[idview];
   if (curvec->used < 6) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_new(&AtA, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&invAtA, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&vecX, 4, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&Atd, 4, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&M, curvec->used, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&U, curvec->used, curvec->used);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&S, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&V, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&R, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dm , M);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dv , V);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dr , R);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&data , AtA);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&datd , Atd);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dx , vecX);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Find
   for (idpt = 0; idpt < curvec->used; idpt++)
   {
      uc = curvec->data[idpt].u - obj->image_center.u;
      vc = curvec->data[idpt].v - obj->image_center.v;
      X = curvec->data[idpt].X;
      Y = curvec->data[idpt].Y;

      dm[idpt][0] = X * vc;
      dm[idpt][1] = Y * vc;
      dm[idpt][2] = -X * uc;
      dm[idpt][3] = -Y * uc;
      dm[idpt][4] = vc;
      dm[idpt][5] = -uc;
   }

   // Solve for the first parameters
   error = rox_array2d_double_svd(U, S, V, M);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_svd_sort(U, S, V);
   ROX_ERROR_CHECK_TERMINATE(error)

   r11 = dv[0][5];
   r12 = dv[1][5];
   r21 = dv[2][5];
   r22 = dv[3][5];
   t1 = dv[4][5];
   t2 = dv[5][5];

   coeffs[0] = 1;
   coeffs[1] = (r12*r12+r22*r22)-(r11*r11+r21*r21);
   coeffs[2] = -((r11*r12)+(r21*r22))*((r11*r12)+(r21*r22));

   error = rox_quadratic_roots(roots, coeffs);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Find possible projection matrices
   for (idroot = 0; idroot < 2; idroot++)
   {
      for (idsign = 0; idsign < 2; idsign++)
      {
         if (roots[idroot].real < 0) continue;

         r32 = sign[idsign] * sqrt(roots[idroot].real);
         r31 = -(r11*r12+r21*r22) / r32;
         scale = 1.0/sqrt(r11*r11+r21*r21+r31*r31);

         for (idsign2 = 0; idsign2 < 2; idsign2++)
         {
            dr[0][0] = sign[idsign2] * scale * r11;
            dr[0][1] = sign[idsign2] * scale * r12;
            dr[1][0] = sign[idsign2] * scale * r21;
            dr[1][1] = sign[idsign2] * scale * r22;
            dr[2][0] = sign[idsign2] * scale * r31;
            dr[2][1] = sign[idsign2] * scale * r32;
            dr[0][2] = sign[idsign2] * scale * t1;
            dr[1][2] = sign[idsign2] * scale * t2;
            dr[2][2] = 0;

            if (ROX_SIGN(dr[0][2]) == ROX_SIGN(curvec->data[0].u - obj->image_center.u) && ROX_SIGN(dr[1][2]) == ROX_SIGN(curvec->data[0].v - obj->image_center.v))
            {
               rox_array2d_double_fillval(AtA, 0);
               rox_array2d_double_fillval(Atd, 0);

               for (idpt = 0; idpt < curvec->used; idpt++)
               {
                  uc = curvec->data[idpt].u - obj->image_center.u;
                  vc = curvec->data[idpt].v - obj->image_center.v;

                  MA = dr[1][0] * curvec->data[idpt].X + dr[1][1] * curvec->data[idpt].Y + dr[1][2];
                  MB = vc * (dr[2][0] * curvec->data[idpt].X + dr[2][1] * curvec->data[idpt].Y);
                  MC = dr[0][0] * curvec->data[idpt].X + dr[0][1] * curvec->data[idpt].Y + dr[0][2];
                  MD = uc * (dr[2][0] * curvec->data[idpt].X + dr[2][1] * curvec->data[idpt].Y);
                  rho2 = (uc*uc + vc*vc);
                  rho = sqrt(rho2);

                  data[0][0] += MA*MA;
                  data[0][1] += MA*MA*rho;
                  data[0][2] += MA*MA*rho2;
                  data[0][3] += MA * -vc;
                  data[1][0] += MA*rho*MA;
                  data[1][1] += MA*rho*MA*rho;
                  data[1][2] += MA*rho*MA*rho2;
                  data[1][3] += MA * rho * -vc;
                  data[2][0] += MA*rho2*MA;
                  data[2][1] += MA*rho2*MA*rho;
                  data[2][2] += MA*rho2*MA*rho2;
                  data[2][3] += MA * rho2 * -vc;
                  data[3][0] += -vc*MA;
                  data[3][1] += -vc*MA*rho;
                  data[3][2] += -vc*MA*rho2;
                  data[3][3] += -vc * -vc;

                  data[0][0] += MC*MC;
                  data[0][1] += MC*MC*rho;
                  data[0][2] += MC*MC*rho2;
                  data[0][3] += MC * -uc;
                  data[1][0] += MC*rho*MC;
                  data[1][1] += MC*rho*MC*rho;
                  data[1][2] += MC*rho*MC*rho2;
                  data[1][3] += MC * rho * -uc;
                  data[2][0] += MC*rho2*MC;
                  data[2][1] += MC*rho2*MC*rho;
                  data[2][2] += MC*rho2*MC*rho2;
                  data[2][3] += MC * rho2 * -uc;
                  data[3][0] += -uc*MC;
                  data[3][1] += -uc*MC*rho;
                  data[3][2] += -uc*MC*rho2;
                  data[3][3] += -uc * -uc;

                  datd[0][0] += MA * MB + MC * MD;
                  datd[1][0] += MA * rho * MB + MC * rho * MD;
                  datd[2][0] += MA * rho2 * MB + MC * rho2 * MD;
                  datd[3][0] += -vc * MB + -uc * MD;
               }

               error = rox_array2d_double_svdinverse(invAtA, AtA);
               ROX_ERROR_CHECK_TERMINATE(error)
               error = rox_array2d_double_mulmatmat(vecX, invAtA, Atd);
               ROX_ERROR_CHECK_TERMINATE(error)

               if (dx[2][0] > 0.0)
               {
                  error = rox_array2d_double_new(&aR, 3, 3);
                  ROX_ERROR_CHECK_TERMINATE(error)
                  error = rox_array2d_double_copy(aR, R);
                  ROX_ERROR_CHECK_TERMINATE(error)
                  error = rox_objset_array2d_double_append(obj->viewposes, aR);
                  ROX_ERROR_CHECK_TERMINATE(error)
                  aR = NULL;

                  error = ROX_ERROR_NONE;
                  goto function_terminate;
               }
            }
         }
      }
   }

function_terminate:
   rox_array2d_double_del(&M);
   rox_array2d_double_del(&AtA);
   rox_array2d_double_del(&invAtA);
   rox_array2d_double_del(&Atd);
   rox_array2d_double_del(&vecX);
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&U);
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&V);
   rox_array2d_double_del(&aR);

   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_compute_parameters_linear(Rox_Calibration_Mono_Generalized obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double A, S, AtA, invAtA, Atd, vecX, R;
   Rox_Uint idview, idpt, order, size, i, j;
   Rox_DynVec_Point_Double curvec;
   Rox_Double **data, **datd, **dr, **dx, **da, **dp, **ds;
   Rox_Double uc, vc, MA, MB, MC, MD;
   Rox_Double rho;
   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   A = NULL;
   AtA = NULL;
   invAtA = NULL;
   Atd = NULL;
   vecX = NULL;

   size = 1 + (obj->order - 1) + obj->points->used;

   error = rox_array2d_double_new(&S, size, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&A, 2, size);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&AtA, size, size);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&invAtA, size, size);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&vecX, size, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Atd, size, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&da , A);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&ds , S);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&data , AtA);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&datd , Atd);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dx , vecX);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dp , obj->coefficients);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(AtA, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Atd, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(S, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (idview = 0; idview < obj->points->used; idview++)
   {
      curvec = obj->points->data[idview];

      R = obj->viewposes->data[idview];
      error = rox_array2d_double_get_data_pointer_to_pointer( &dr, R);

      error = rox_array2d_double_fillval(A, 0);

      for (idpt = 0; idpt < curvec->used; idpt++)
      {
         uc = curvec->data[idpt].u - obj->image_center.u;
         vc = curvec->data[idpt].v - obj->image_center.v;


         MA = dr[1][0] * curvec->data[idpt].X + dr[1][1] * curvec->data[idpt].Y + dr[1][2];
         MB = vc * (dr[2][0] * curvec->data[idpt].X + dr[2][1] * curvec->data[idpt].Y);
         MC = dr[0][0] * curvec->data[idpt].X + dr[0][1] * curvec->data[idpt].Y + dr[0][2];
         MD = uc * (dr[2][0] * curvec->data[idpt].X + dr[2][1] * curvec->data[idpt].Y);

         da[0][0] = MA;
         da[1][0] = MC;

         for (order = 2; order <= obj->order; order++)
         {
            //rho = pow((double)sqrt(uc*uc+vc*vc), (int) order);
            rho = pow(sqrt(uc*uc+vc*vc), (int)order);
            da[0][order - 1] = MA * rho;
            da[1][order - 1] = MC * rho;
         }


         da[0][obj->order + idview] = -vc;
         da[1][obj->order + idview] = -uc;

         for (i = 0; i < size; i++)
         {
            ds[i][0] += da[0][i] * da[0][i] + da[1][i] * da[1][i];

            for (j = 0; j < size; j++)
            {
               data[i][j] += da[0][i] * da[0][j] + da[1][i] * da[1][j];
            }

            datd[i][0] += da[0][i] * MB + da[1][i] * MD;
         }
      }
   }

   for (i = 0; i < size; i++)
   {
      ds[i][0] = 1.0 / sqrt(ds[i][0]);
   }

   for (i = 0; i < size; i++)
   {
      for (j = 0; j < size; j++)
      {
         data[i][j] = data[i][j] * ds[i][0] * ds[j][0];
      }

      datd[i][0] = datd[i][0] * ds[i][0];
   }

   error = rox_array2d_double_svdinverse(invAtA, AtA);
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_mulmatmat(vecX, invAtA, Atd);
   ROX_ERROR_CHECK_TERMINATE(error)

   for (i = 0; i < size; i++)
   {
      dx[i][0] *= ds[i][0];
   }


   dp[0][0] = dx[0][0];
   dp[1][0] = 0.0;

   for (i = 1; i < obj->order; i++)
   {
      dp[i + 1][0] = dx[i][0];
   }

   for (idview = 0; idview < obj->viewposes->used; idview++)
   {
      R = obj->viewposes->data[idview];
      error = rox_array2d_double_get_data_pointer_to_pointer( &dr, R);
      dr[2][2] = dx[obj->order + idview][0];
   }

function_terminate:
   rox_array2d_double_del(&A);
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&AtA);
   rox_array2d_double_del(&invAtA);
   rox_array2d_double_del(&Atd);
   rox_array2d_double_del(&vecX);
   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_compute_parameters(Rox_Calibration_Mono_Generalized obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_objset_array2d_double_reset(obj->viewposes);

   for (Rox_Uint idview = 0; idview < obj->points->used; idview++)
   {
      error = rox_calibration_mono_generalized_compute_parameters_oneview(obj, idview);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (obj->viewposes->used != obj->points->used) {error = ROX_ERROR_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_calibration_mono_generalized_compute_parameters_linear(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_compute_point_error (
   Rox_Double * rx,
   Rox_Double * ry,
   Rox_Calibration_Mono_Generalized obj,
   Rox_Array2D_Double P,
   Rox_Array2D_Double S,
   Rox_Point_Double pt)
{
   Rox_Double X,Y,Z, m, root, x, y;
   Rox_Double **dr, **dc, *dcb, *droots;
   Rox_Sint nbcoefs;
   Rox_Uint nbroots, idroot, countvalid, idcoef;
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !P || !S || !rx || !ry) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer( &dr, P);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dc, S);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_rows(&nbcoefs, obj->coefficients_buf);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer( &dcb, obj->coefficients_buf);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer( &droots, obj->roots);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (idcoef = 0; idcoef < (Rox_Uint) nbcoefs; idcoef++)
   {
      dcb[idcoef] = dc[idcoef][0];
   }

   X = dr[0][0] * pt->X + dr[0][1] * pt->Y + dr[0][2] * pt->Z;
   Y = dr[1][0] * pt->X + dr[1][1] * pt->Y + dr[1][2] * pt->Z;
   Z = dr[2][0] * pt->X + dr[2][1] * pt->Y + dr[2][2] * pt->Z;

   if (X==0.0 && Y  == 0.0 && fabs(Z) == 1.0)
   {
      X = DBL_EPSILON;
      Y = DBL_EPSILON;
   }

   m = Z / sqrt(X*X+Y*Y);

   dcb[1] = dc[1][0] - m;

   error = rox_polynomial_roots_sturm(droots, &nbroots, dcb, nbcoefs-1);
   if (error) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error)}

   countvalid=0;
   for (idroot = 0; idroot < nbroots; idroot++)
   {
      root = droots[idroot];

      if (root < DBL_EPSILON || (obj->image_center.u + root >= obj->height && obj->image_center.v + root >= obj->width)) continue;
      countvalid++;
   }

   if (countvalid != 1) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error)}

   x = root * (X / sqrt(X*X+Y*Y));
   y = root * (Y / sqrt(X*X+Y*Y));

   *rx = x;
   *ry = y;

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_compute_error(Rox_Calibration_Mono_Generalized obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint count;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->mse = 0;
   count = 0;
   for (Rox_Uint idview = 0; idview < obj->viewposes->used; idview++)
   {
      Rox_DynVec_Point_Double curvec = obj->points->data[idview];
      Rox_Array2D_Double R = obj->viewposes->data[idview];

      for (Rox_Uint idpt = 0; idpt < curvec->used; idpt++)
      {
         Rox_Double dx,dy,x,y;

         error = rox_calibration_mono_generalized_compute_point_error(&x, &y, obj, R, obj->coefficients, &curvec->data[idpt]);
         if (error) 
         { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

         dx = curvec->data[idpt].u - obj->image_center.u - x;
         dy = curvec->data[idpt].v - obj->image_center.v - y ;

         obj->mse += dx*dx+dy*dy;
         count++;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_compute_imagecenter(Rox_Calibration_Mono_Generalized obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double xs, xe, ys, ye;
   Rox_Sint idx,idy, idxmin, idymin, iter;
   Rox_Double x,y,sx,sy,min,dx,dy;
   const Rox_Sint xsize = 5;
   const Rox_Sint ysize = 5;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   xs = obj->image_center.u - (((Rox_Double)obj->height) / 4.0);
   ys = obj->image_center.v - (((Rox_Double)obj->width) / 4.0);
   xe = obj->image_center.u + (((Rox_Double)obj->height) / 4.0);
   ye = obj->image_center.v + (((Rox_Double)obj->width) / 4.0);
   sy = (ye-ys)/ysize;
   sx = (xe-xs)/xsize;

   for (iter = 0; iter < 9; iter++)
   {
      min = DBL_MAX;
      idxmin = -1;
      idymin = -1;

      for (idx = 0; idx <= xsize; idx++)
      {
         x = xs + idx * sx;

         for (idy = 0; idy <= ysize; idy++)
         {
            y = ys + idy * sy;
            obj->image_center.u = x;
            obj->image_center.v = y;

            error = rox_calibration_mono_generalized_compute_parameters(obj);
            if (error) continue;

            error = rox_calibration_mono_generalized_compute_error(obj);
            if (error) continue;

            if (min  > obj->mse)
            {
               min = obj->mse;
               idxmin = idx;
               idymin = idy;
            }
         }
      }

      if (idxmin < 0) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

      obj->image_center.u = xs + idxmin * sx;
      obj->image_center.v = ys + idymin * sy;
      dx = fabs(sx);
      dy = fabs(sy);

      xs = obj->image_center.u - dx;
      xe = obj->image_center.u + dx;
      ys = obj->image_center.v - dy;
      ye = obj->image_center.v + dy;
      sy = (ye-ys)/ysize;
      sx = (xe-xs)/xsize;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_refine_extrinsic_view(Rox_Calibration_Mono_Generalized obj, Rox_Uint idview)
{
   Rox_Uint iter, idpt;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point_Double pts;
   Rox_Array2D_Double J, vecd, vecx, invJ, R, P, PM, PP;
   Rox_Double ax, ay, az, angle, nx, ny, nz, xp, xm, yp, ym, tx, ty, tz, sumsq;
   Rox_Double ** dr, **dp, **dpm, **dpp, **dj, *dd, *dx;

   const Rox_Double delta = 1e-9;
   const Rox_Double normalizer = 1.0 / (2*delta);

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (idview >= obj->viewposes->used) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pts = obj->points->data[idview];

   R = NULL;
   PM = NULL;
   PP = NULL;
   J = NULL;
   invJ = NULL;
   vecd = NULL;
   vecx = NULL;

   error = rox_array2d_double_new(&R, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&PM, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&PP, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&J, pts->used * 2, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&vecd, pts->used * 2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&vecx, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&invJ, 6, pts->used * 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   P = obj->viewposes->data[idview];

   error = rox_array2d_double_get_data_pointer_to_pointer( &dj, J);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dr, R);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, P);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dpm, PM);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dpp, PP);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer( &dx, vecx);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer( &dd, vecd);
   ROX_ERROR_CHECK_TERMINATE ( error );

   tx = dp[0][2];
   ty = dp[1][2];
   tz = dp[2][2];

   for (iter = 0; iter < 6; iter++)
   {
	  Rox_Uint i,j;
	  Rox_Double det;

      rox_array2d_double_copy(R, P);
      dr[0][2] = dr[1][0] * dr[2][1] - dr[2][0] * dr[1][1];
      dr[1][2] = dr[2][0] * dr[0][1] - dr[0][0] * dr[2][1];
      dr[2][2] = dr[0][0] * dr[1][1] - dr[1][0] * dr[0][1];

      rox_array2d_double_detgl3(&det, R);

      for (i = 0; i < 3; i++)
      {
         for (j = 0; j < 2; j++)
         {
            dr[i][j] /= pow(det,1.0/3.0);
         }
      }

      error = rox_transformtools_axisangle_from_rotationmatrix(&ax, &ay, &az, &angle, R);
      ROX_ERROR_CHECK_TERMINATE(error)
      ax *= angle;
      ay *= angle;
      az *= angle;

      sumsq = 0;
      for (idpt = 0; idpt < pts->used; idpt++)
      {
         error = rox_calibration_mono_generalized_compute_point_error(&xp, &yp, obj, P, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE(error)

         dd[idpt*2] = pts->data[idpt].u - obj->image_center.u - xp;
         dd[idpt*2+1] = pts->data[idpt].v - obj->image_center.v - yp;
         sumsq += dd[idpt*2]*dd[idpt*2] + dd[idpt*2+1]*dd[idpt*2+1];
      }


      // PARAM1
      nx = ax + delta;
      ny = ay;
      nz = az;
      tx = dp[0][2];
      ty = dp[1][2];
      tz = dp[2][2];
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PP, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpp[0][2] = tx;
      dpp[1][2] = ty;
      dpp[2][2] = tz;

      nx = ax - delta;
      ny = ay;
      nz = az;
      tx = dp[0][2];
      ty = dp[1][2];
      tz = dp[2][2];
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PM, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpm[0][2] = tx;
      dpm[1][2] = ty;
      dpm[2][2] = tz;

      for (idpt = 0; idpt < pts->used; idpt++)
      {
         error = rox_calibration_mono_generalized_compute_point_error(&xp, &yp, obj, PP, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_calibration_mono_generalized_compute_point_error(&xm, &ym, obj, PM, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         dj[idpt*2][0] = (xp - xm) * normalizer;
         dj[idpt*2+1][0] = (yp - ym) * normalizer;
      }

      // PARAM2
      nx = ax;
      ny = ay + delta;
      nz = az;
      tx = dp[0][2];
      ty = dp[1][2];
      tz = dp[2][2];
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PP, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpp[0][2] = tx;
      dpp[1][2] = ty;
      dpp[2][2] = tz;

      nx = ax;
      ny = ay - delta;
      nz = az;
      tx = dp[0][2];
      ty = dp[1][2];
      tz = dp[2][2];
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PM, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpm[0][2] = tx;
      dpm[1][2] = ty;
      dpm[2][2] = tz;

      for (idpt = 0; idpt < pts->used; idpt++)
      {
         error = rox_calibration_mono_generalized_compute_point_error(&xp, &yp, obj, PP, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_calibration_mono_generalized_compute_point_error(&xm, &ym, obj, PM, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         dj[idpt*2][1] = (xp - xm) * normalizer;
         dj[idpt*2+1][1] = (yp - ym) * normalizer;
      }

      // PARAM3
      nx = ax;
      ny = ay;
      nz = az + delta;
      tx = dp[0][2];
      ty = dp[1][2];
      tz = dp[2][2];
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PP, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpp[0][2] = tx;
      dpp[1][2] = ty;
      dpp[2][2] = tz;

      nx = ax;
      ny = ay;
      nz = az - delta;
      tx = dp[0][2];
      ty = dp[1][2];
      tz = dp[2][2];
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PM, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpm[0][2] = tx;
      dpm[1][2] = ty;
      dpm[2][2] = tz;

      for (idpt = 0; idpt < pts->used; idpt++)
      {
         error = rox_calibration_mono_generalized_compute_point_error ( &xp, &yp, obj, PP, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_calibration_mono_generalized_compute_point_error ( &xm, &ym, obj, PM, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         dj[idpt*2][2] = (xp - xm) * normalizer;
         dj[idpt*2+1][2] = (yp - ym) * normalizer;
      }

      // PARAM4
      nx = ax;
      ny = ay;
      nz = az;
      tx = dp[0][2] + delta;
      ty = dp[1][2];
      tz = dp[2][2];
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PP, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpp[0][2] = tx;
      dpp[1][2] = ty;
      dpp[2][2] = tz;

      nx = ax;
      ny = ay;
      nz = az;
      tx = dp[0][2] - delta;
      ty = dp[1][2];
      tz = dp[2][2];
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PM, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpm[0][2] = tx;
      dpm[1][2] = ty;
      dpm[2][2] = tz;

      for (idpt = 0; idpt < pts->used; idpt++)
      {
         error = rox_calibration_mono_generalized_compute_point_error(&xp, &yp, obj, PP, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_calibration_mono_generalized_compute_point_error(&xm, &ym, obj, PM, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         dj[idpt*2][3] = (xp - xm) * normalizer;
         dj[idpt*2+1][3] = (yp - ym) * normalizer;
      }

      // PARAM5
      nx = ax;
      ny = ay;
      nz = az;
      tx = dp[0][2];
      ty = dp[1][2] + delta;
      tz = dp[2][2];
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PP, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpp[0][2] = tx;
      dpp[1][2] = ty;
      dpp[2][2] = tz;

      nx = ax;
      ny = ay;
      nz = az;
      tx = dp[0][2];
      ty = dp[1][2] - delta;
      tz = dp[2][2];
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PM, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpm[0][2] = tx;
      dpm[1][2] = ty;
      dpm[2][2] = tz;

      for (idpt = 0; idpt < pts->used; idpt++)
      {
         error = rox_calibration_mono_generalized_compute_point_error(&xp, &yp, obj, PP, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_calibration_mono_generalized_compute_point_error(&xm, &ym, obj, PM, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         dj[idpt*2][4] = (xp - xm) * normalizer;
         dj[idpt*2+1][4] = (yp - ym) * normalizer;
      }

      // PARAM6
      nx = ax;
      ny = ay;
      nz = az;
      tx = dp[0][2];
      ty = dp[1][2];
      tz = dp[2][2] + delta;
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PP, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpp[0][2] = tx;
      dpp[1][2] = ty;
      dpp[2][2] = tz;

      nx = ax;
      ny = ay;
      nz = az;
      tx = dp[0][2];
      ty = dp[1][2];
      tz = dp[2][2] - delta;
      angle = sqrt(nx*nx+ny*ny+nz*nz);
      if (fabs(angle)<DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      nx /= angle;
      ny /= angle;
      nz /= angle;
      error = rox_matso3_set_axis_angle(PM, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE ( error );
      dpm[0][2] = tx;
      dpm[1][2] = ty;
      dpm[2][2] = tz;

      for (idpt = 0; idpt < pts->used; idpt++)
      {
         error = rox_calibration_mono_generalized_compute_point_error(&xp, &yp, obj, PP, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_calibration_mono_generalized_compute_point_error(&xm, &ym, obj, PM, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         dj[idpt*2][5] = (xp - xm) * normalizer;
         dj[idpt*2+1][5] = (yp - ym) * normalizer;
      }

      error = rox_array2d_double_svdinverse(invJ, J);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_array2d_double_mulmatmat(vecx, invJ, vecd);
      ROX_ERROR_CHECK_TERMINATE(error)

      nx = ax + dx[0];
      ny = ay + dx[1];
      nz = az + dx[2];
      tx = tx + dx[3];
      ty = ty + dx[4];
      tz = tz + dx[5];
      angle = sqrt(nx*nx+ny*ny+nz*nz);


      nx /= angle;
      ny /= angle;
      nz /= angle;

      error = rox_matso3_set_axis_angle(P, nx, ny, nz, angle);
      ROX_ERROR_CHECK_TERMINATE(error)
      dp[0][2] = tx;
      dp[1][2] = ty;
      dp[2][2] = tz;
   }

function_terminate:

   rox_array2d_double_del(&J);
   rox_array2d_double_del(&invJ);
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&PM);
   rox_array2d_double_del(&PP);
   rox_array2d_double_del(&vecx);
   rox_array2d_double_del(&vecd);

   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_refine_intrinsics(Rox_Calibration_Mono_Generalized obj)
{
   Rox_Uint idview, idpt, idparam, idparam2, iter, countmes, pos;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double P, vecd, J, invJ, vecx, coef, coefp, coefm, scoef;
   Rox_DynVec_Point_Double pts;
   Rox_Double * dd, *dcm, *dcp, *drc, *dc, *dx, *dsc;
   Rox_Double ** dj;

   Rox_Double sumsq;
   Rox_Double xp, yp, xm, ym, a, b, c, d, e;

   const Rox_Sint size = obj->order + 1 + 5;
   const Rox_Double delta = 1e-12;
   const Rox_Double normalizer = 1.0 / (2*delta);

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Count total number of measures
   countmes = 0;
   for (idview = 0; idview < obj->points->used; idview++)
   {
      pts = obj->points->data[idview];
      countmes += pts->used;
   }

   vecd = NULL;
   J = NULL;
   scoef = NULL;
   coef = NULL;
   coefp = NULL;
   coefm = NULL;
   vecx = NULL;
   invJ = NULL;

   error = rox_array2d_double_new(&vecd, countmes * 2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&vecx, size, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&J, countmes * 2, size);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&invJ, size, countmes * 2);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&scoef, obj->order + 1, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&coef, size, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&coefp, size, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&coefm, size, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dj, J);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer(&dd, vecd);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer(&dcm, coefm);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer(&dcp, coefp);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer(&dc, coef);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer(&dsc, scoef);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer(&drc, obj->coefficients);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer(&dx, vecx);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(coef, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );


   dc[3] = 0;
   dc[4] = 0;

   for (iter = 0; iter < 5; iter++)
   {
      // Get parameters
      a = dc[0];
      b = dc[1];
      c = dc[2];
      d = dc[3];
      e = dc[4];

      // scale coefficients using x
      for (idparam = 0; idparam < obj->order + 1; idparam++)
      {
         dsc[idparam] = drc[idparam] * dc[idparam + 5];
      }

      // Compute error vector
      sumsq = 0;
      pos = 0;
      for (idview = 0; idview < obj->points->used; idview++)
      {
         pts = obj->points->data[idview];
         P = obj->viewposes->data[idview];

         for (idpt = 0; idpt < pts->used; idpt++)
         {
            error = rox_calibration_mono_generalized_compute_point_error(&xp, &yp, obj, P, scoef, &pts->data[idpt]);
            ROX_ERROR_CHECK_TERMINATE ( error );

            xp = xp * c + yp * d + obj->image_center.u * a;
            yp = xp * e + yp + obj->image_center.v * b;

            dd[pos] = pts->data[idpt].u - xp;
            dd[pos+1] = pts->data[idpt].v - yp;
            sumsq += dd[pos]*dd[pos] + dd[pos+1]*dd[pos+1];

            pos += 2;
         }
      }

      // Compute jacobian
      for (idparam = 0; idparam < (Rox_Uint) size; idparam++)
      {
         rox_array2d_double_copy(coefp, coef);
         dcp[idparam] += delta;
         rox_array2d_double_copy(coefm, coef);
         dcm[idparam] -= delta;

         pos = 0;
         for (idview = 0; idview < obj->points->used; idview++)
         {
            pts = obj->points->data[idview];
            P = obj->viewposes->data[idview];

            for (idpt = 0; idpt < pts->used; idpt++)
            {
               for (idparam2 = 0; idparam2 < obj->order + 1; idparam2++)
               {
                  dsc[idparam2] = drc[idparam2] * dcp[idparam2 + 5];
               }

               error = rox_calibration_mono_generalized_compute_point_error(&xp, &yp, obj, P, scoef, &pts->data[idpt]);
               ROX_ERROR_CHECK_TERMINATE ( error );

               for (idparam2 = 0; idparam2 < obj->order + 1; idparam2++)
               {
                  dsc[idparam2] = drc[idparam2] * dcm[idparam2 + 5];
               }

               error = rox_calibration_mono_generalized_compute_point_error(&xm, &ym, obj, P, scoef, &pts->data[idpt]);
               ROX_ERROR_CHECK_TERMINATE ( error );

               a = dcp[0];
               b = dcp[1];
               c = dcp[2];
               d = dcp[3];
               e = dcp[4];


               xp = xp * c + yp * d + obj->image_center.u * a;
               yp = xp * e + yp + obj->image_center.v * b;

               a = dcm[0];
               b = dcm[1];
               c = dcm[2];
               d = dcm[3];
               e = dcm[4];
               xm = xm * c + ym * d + obj->image_center.u * a;
               ym = xm * e + ym + obj->image_center.v * b;

               dj[pos+0][idparam] = (xp - xm) * normalizer;
               dj[pos+1][idparam] = (yp - ym) * normalizer;

               pos+=2;
            }
         }
      }

      error = rox_array2d_double_svdinverse(invJ, J);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_array2d_double_mulmatmat(vecx, invJ, vecd);
      ROX_ERROR_CHECK_TERMINATE(error)

      for (idparam = 0; idparam < (Rox_Uint) size; idparam++)
      {
         dc[idparam] += dx[idparam];
      }
   }

   // scale coefficients using x
   for (idparam = 0; idparam < obj->order + 1; idparam++)
   {
      drc[idparam] = drc[idparam] * dc[idparam + 5];
   }

   obj->coef_a = dc[0];
   obj->coef_b = dc[1];
   obj->coef_c = dc[2];
   obj->coef_d = dc[3];
   obj->coef_e = dc[4];


   sumsq = 0;
   pos = 0;
   for (idview = 0; idview < obj->points->used; idview++)
   {
      pts = obj->points->data[idview];
      P = obj->viewposes->data[idview];

      for (idpt = 0; idpt < pts->used; idpt++)
      {
         error = rox_calibration_mono_generalized_compute_point_error(&xp, &yp, obj, P, obj->coefficients, &pts->data[idpt]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         xp = xp * obj->coef_c + yp * obj->coef_d + obj->image_center.u * obj->coef_a;
         yp = xp * obj->coef_e + yp + obj->image_center.v * obj->coef_b;


         dd[pos] = pts->data[idpt].u - xp;
         dd[pos+1] = pts->data[idpt].v - yp;
         sumsq += dd[pos]*dd[pos] + dd[pos+1]*dd[pos+1];


         pos += 2;
      }
   }

   obj->image_center.u *= obj->coef_a;
   obj->image_center.v *= obj->coef_b;

   obj->mse = sumsq;

function_terminate:
   rox_array2d_double_del(&vecd);
   rox_array2d_double_del(&J);
   rox_array2d_double_del(&scoef);
   rox_array2d_double_del(&coef);
   rox_array2d_double_del(&coefp);
   rox_array2d_double_del(&coefm);
   rox_array2d_double_del(&invJ);
   rox_array2d_double_del(&vecx);

   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_refine_extrinsics(Rox_Calibration_Mono_Generalized obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   for (Rox_Uint idview = 0; idview < obj->viewposes->used; idview++)
   {
      error = rox_calibration_mono_generalized_refine_extrinsic_view(obj, idview);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_inverse_poly_degree(Rox_Double * error_out, Rox_Calibration_Mono_Generalized obj, Rox_Double radius, Rox_Uint degree)
{
   Rox_Double theta, val, diff, root, result;
   Rox_Double * dcb, * dc, *dres, *droots;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nbroots, idroot, countvalid, idval;
   Rox_DynVec_Point2D_Double tofit;
   Rox_Array2D_Double vecroots;
   Rox_Point2D_Double_Struct pt;

   vecroots = NULL;
   tofit = NULL;

   error = rox_array2d_double_new(&vecroots, degree + 1, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_new(&tofit, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer(&dcb, obj->coefficients_buf);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer(&dc, obj->coefficients);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer(&droots, obj->roots);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer(&dres, vecroots);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(obj->coefficients_buf, obj->coefficients);
   ROX_ERROR_CHECK_TERMINATE ( error );

   theta = -ROX_PI_2;
   while (theta < 1.2)
   {
      pt.u = theta;
      val = tan(theta);
      theta += 0.01;

      dcb[1] = dc[1] - val;
      error = rox_polynomial_roots_sturm(droots, &nbroots, dcb, obj->order);
      if (error) continue;

      countvalid = 0;
      for (idroot = 0; idroot < nbroots; idroot++)
      {
         if (droots[idroot] < 0.0 || droots[idroot] > radius) continue;

         root = droots[idroot];
         countvalid++;
      }
      if (countvalid != 1) continue;

      pt.v = root;
      error = rox_dynvec_point2d_double_append(tofit, &pt);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   error = rox_polynomial_fit(dres, degree, tofit);
   ROX_ERROR_CHECK_TERMINATE(error)

   result = 0;
   for (idval = 0; idval < tofit->used; idval++)
   {
      error = rox_polynomial_eval(&val, dres, degree, tofit->data[idval].u);
      ROX_ERROR_CHECK_TERMINATE(error)

      diff = fabs(tofit->data[idval].v - val);
      if (diff > result) result = diff;
   }

   rox_dynvec_double_reset(obj->coefficients_inv);
   for (idval = 0; idval < degree + 1; idval++)
   {
      rox_dynvec_double_append(obj->coefficients_inv, &dres[idval]);
   }

   *error_out = result;

   error = ROX_ERROR_NONE;

function_terminate:
   rox_dynvec_point2d_double_del(&tofit);
   rox_array2d_double_del(&vecroots);

   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_inverse_poly(Rox_Calibration_Mono_Generalized obj, Rox_Double radius)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Uint N = 1;
   Rox_Double error_out = DBL_MAX;
   while (error > 1e-3)
   {
      N++;

      error = rox_calibration_mono_generalized_inverse_poly_degree(&error_out, obj, radius, N);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_find_scale (
   Rox_Double * scale,
   Rox_Calibration_Mono_Generalized obj,
   Rox_Point2D_Double dest,
   Rox_Point2D_Double pt)
{
   Rox_Double u,v, norm, inorm;
   Rox_Double nxc, nyc;
   Rox_Double resultant;
   Rox_Double * lroots, *lpoly;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint degree, cntroots, iddeg, idroot;

   lroots = NULL;
   lpoly = NULL;

   degree = obj->coefficients_inv->used - 1;
   lroots = (Rox_Double*)rox_memory_allocate(sizeof(Rox_Double), degree);
   lpoly = (Rox_Double*)rox_memory_allocate(sizeof(Rox_Double), degree + 1);
   if (!lroots || !lpoly)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   nxc = obj->image_center.u;
   nyc = obj->image_center.v;
   u = pt->u - nxc;
   v = pt->v - nyc;
   norm = sqrt(u*u + v*v);
   inorm = 1.0 / norm;
   resultant = (dest->u - nxc) / (inorm * (u * obj->coef_c + v * obj->coef_d));

   for (iddeg = 0; iddeg <= degree; iddeg++)
   {
      lpoly[iddeg] = obj->coefficients_inv->data[iddeg];
   }
   lpoly[0] -= resultant;

   error = rox_polynomial_roots_sturm(lroots, &cntroots, lpoly, degree);
   ROX_ERROR_CHECK_TERMINATE(error)

   for (idroot = 0; idroot < cntroots; idroot++)
   {
      Rox_Double inter;
      error = rox_polynomial_eval(&inter, obj->coefficients_inv->data, degree, lroots[idroot]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      *scale = tan(lroots[idroot]) * norm;
   }

function_terminate:
   rox_memory_delete(lroots);
   rox_memory_delete(lpoly);

   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_undistort (
   Rox_Point_Double  dest,
   Rox_Calibration_Mono_Generalized obj,
   Rox_Point_Double pt,
   Rox_Double scale)
{
   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_calibration_mono_generalized_build_lut ( Rox_Calibration_Mono_Generalized obj, Rox_Uint idimage )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint i, j;
   Rox_Uint k;
   Rox_Double u,v, norm, inorm, theta, t, t_i, rho, x, y;
   Rox_Double nxc, nyc, nz;
   Rox_Point2D_Double_Struct ptori, ptdest;
   Rox_Uint idimg, idpt;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Point2D_Float * dm = NULL;
   error = rox_array2d_point2d_float_get_data_pointer_to_pointer ( &dm, obj->map);
   ROX_ERROR_CHECK_TERMINATE ( error );

   nxc = obj->image_center.u;
   nyc = obj->image_center.v;
   nz = - ((Rox_Double)obj->width) ;

   ptdest.u = 0;
   ptdest.v = 0;
   ptori.u = 0;
   ptori.v = 0;

   error = rox_calibration_mono_generalized_find_scale(&nz, obj, &ptdest, &ptori);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (idimg = 0; idimg < obj->points->used; idimg++)
   {
      Rox_DynVec_Point_Double ipts;
      Rox_DynVec_Point_Double opts;

      ipts = obj->points->data[idimg];
      opts = obj->points_undistorted->data[idimg];

      for (idpt = 0; idpt < ipts->used; idpt++)
      {
         error = rox_calibration_mono_generalized_undistort(&opts->data[idpt], obj, &ipts->data[idpt], nz);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

   for (i = 0; i < obj->height; i++)
   {
      for (j = 0; j < obj->width; j++)
      {
         u = ((Rox_Double)j) - nxc;
         v = ((Rox_Double)i) - nyc;

         norm = sqrt(u*u + v*v);
         theta = atan(nz / norm);

         if (fabs(norm) > DBL_EPSILON)
         {
            inorm = 1.0/norm;
            t = theta;
            rho = obj->coefficients_inv->data[0];
            t_i =1;

            for (k = 1; k < obj->coefficients_inv->used; k++)
            {
               t_i *= t;
               rho += t_i * obj->coefficients_inv->data[k];
            }

            x = u * inorm * rho;
            y = v * inorm * rho;

            x = x * obj->coef_c + y * obj->coef_d + obj->image_center.u;
            y = x * obj->coef_e + y + obj->image_center.v;

            dm[i][j].u = (Rox_Float) x;
            dm[i][j].v = (Rox_Float) y;
         }
         else
         {
            dm[i][j].u = (Rox_Float) nxc;
            dm[i][j].v = (Rox_Float) nyc;
         }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_mono_generalized_process(Rox_Calibration_Mono_Generalized obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   // First, compute a coarse projection matrix for all views using linear estimation
   error = rox_calibration_mono_generalized_compute_parameters(obj);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Estimate image center
   error = rox_calibration_mono_generalized_compute_imagecenter(obj);
   ROX_ERROR_CHECK_TERMINATE(error)

   // NL refine of extrinsics
   error = rox_calibration_mono_generalized_refine_extrinsics(obj);
   ROX_ERROR_CHECK_TERMINATE(error)

   // NL refine of intrinsics
   error = rox_calibration_mono_generalized_refine_intrinsics(obj);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Inverse polynom
   error = rox_calibration_mono_generalized_inverse_poly(obj, sqrt((obj->width/2.0)*(obj->width/2.0)+(obj->height/2.0)*(obj->height/2.0)));
   ROX_ERROR_CHECK_TERMINATE(error)

   // Built LUT
   error = rox_calibration_mono_generalized_build_lut(obj, 0);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

