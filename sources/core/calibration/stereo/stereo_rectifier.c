//==============================================================================
//
//    OPENROX   : File stereo_rectifier.c
//
//    Contents  : Implementation of stereo_rectifier module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "stereo_rectifier.h"

#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/geometry/pixelgrid/warp_grid_distortion_matsl3.h>
#include <baseproc/image/remap/remap_bilinear_nomask_uchar_to_uchar/remap_bilinear_nomask_uchar_to_uchar.h>
#include <baseproc/image/remap/remap_bilinear_nomask_uint_to_uint/remap_bilinear_nomask_uint_to_uint.h>
#include <baseproc/image/remap/remap_bilinear_omo_uchar_to_uchar/remap_bilinear_omo_uchar_to_uchar.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/mean/mean.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/crossprod/crossprod.h>
#include <baseproc/geometry/calibration/optimalcalib.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/image/convert/roxrgba_to_roxgray.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_stereorectifier_new(Rox_StereoRectifier * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_StereoRectifier ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_StereoRectifier) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->rectified_calibration = NULL;

   ret->rectified_poseleft = NULL;
   ret->rectified_poseright = NULL;
   ret->rectified_imageleft = NULL;
   ret->rectified_imageright = NULL;
   ret->rectified_leftmask = NULL;
   ret->rectified_rightmask = NULL;
   ret->rectified_rgba_imageleft = NULL;
   ret->rectified_rgba_imageright = NULL;
   ret->rectified_grid_left = NULL;
   ret->rectified_grid_right = NULL;
   ret->rectified_grid_left_fixed = NULL;
   ret->rectified_grid_right_fixed = NULL;
   ret->rectified_homographyleft = NULL;
   ret->rectified_homographyright = NULL;
   ret->rectified_relativepose = NULL;

   error = rox_array2d_double_new(&ret->rectified_calibration, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->rectified_relativepose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->rectified_poseleft, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->rectified_poseright, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->rectified_homographyleft, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->rectified_homographyright, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_stereorectifier_del(&ret);
   return error;
}

Rox_ErrorCode rox_stereorectifier_del(Rox_StereoRectifier * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_StereoRectifier todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_array2d_double_del(&todel->rectified_calibration);

   rox_array2d_double_del(&todel->rectified_poseleft);
   rox_array2d_double_del(&todel->rectified_poseright);
   rox_array2d_double_del(&todel->rectified_relativepose);

   rox_array2d_double_del(&todel->rectified_homographyleft);
   rox_array2d_double_del(&todel->rectified_homographyright);

   rox_array2d_uchar_del(&todel->rectified_imageleft);
   rox_array2d_uchar_del(&todel->rectified_imageright);

   rox_array2d_uint_del(&todel->rectified_rgba_imageleft);
   rox_array2d_uint_del(&todel->rectified_rgba_imageright);

   rox_array2d_uint_del(&todel->rectified_leftmask);
   rox_array2d_uint_del(&todel->rectified_rightmask);

   rox_meshgrid2d_float_del(&todel->rectified_grid_left);
   rox_meshgrid2d_float_del(&todel->rectified_grid_right);

   rox_array2d_point2d_sshort_del(&todel->rectified_grid_left_fixed);
   rox_array2d_point2d_sshort_del(&todel->rectified_grid_right_fixed);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_stereorectifier_prepare (
   Rox_StereoRectifier obj, 
   Rox_Array2D_Double Kl, 
   Rox_Array2D_Double Dl, 
   Rox_Array2D_Double Kr, 
   Rox_Array2D_Double Dr, 
   Rox_Array2D_Double rTl, 
   Rox_Sint width, 
   Rox_Sint height
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double ** dr, **dk;
   Rox_Double * dv1;
   Rox_Double * dv2;
   Rox_Double * dv3;
   Rox_Double * db1;
   Rox_Double ** dh;
   Rox_Double us[4], vs[4], u,v,w,rw,rh,minu,maxu,minv,maxv,destw, desth;
   Rox_Double scale, scaleh, scalew, offsetu, offsetv;
   Rox_Double normv1, normv2, normv3;
   Rox_Float ** src_u;
   Rox_Float ** src_v;
   Rox_Point2D_Sshort * dst;

   Rox_Array2D_Double AR = NULL;
   Rox_Array2D_Double KR = NULL;
   Rox_Array2D_Double iAR = NULL;
   Rox_Array2D_Double R = NULL;
   Rox_Array2D_Double Ro2 = NULL;
   Rox_Array2D_Double A = NULL;
   Rox_Array2D_Double c2 = NULL;
   Rox_Array2D_Double to2 = NULL;
   Rox_Array2D_Double v1 = NULL;
   Rox_Array2D_Double v2 = NULL;
   Rox_Array2D_Double v3 = NULL;
   Rox_Array2D_Double buf31 = NULL;
   Rox_Array2D_Double buf44 = NULL;
   Rox_Array2D_Double Rn1 = NULL;
   Rox_Array2D_Double Rn2 = NULL;
   Rox_Array2D_Double tn1 = NULL;
   Rox_Array2D_Double tn2 = NULL;
   Rox_Array2D_Double Klud = NULL;
   Rox_Array2D_Double Krud = NULL;

   rox_array2d_uchar_del(&obj->rectified_imageleft);
   rox_array2d_uchar_del(&obj->rectified_imageright);
   rox_meshgrid2d_float_del(&obj->rectified_grid_left);
   rox_meshgrid2d_float_del(&obj->rectified_grid_right);
   rox_array2d_point2d_sshort_del(&obj->rectified_grid_left_fixed);
   rox_array2d_point2d_sshort_del(&obj->rectified_grid_right_fixed);

   rox_array2d_uint_del(&obj->rectified_rgba_imageleft);
   rox_array2d_uint_del(&obj->rectified_rgba_imageright);
   rox_array2d_uint_del(&obj->rectified_leftmask);
   rox_array2d_uint_del(&obj->rectified_rightmask);

   error = rox_array2d_double_new(&Klud, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&Krud, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&R, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&AR, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&KR, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&iAR, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&A, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&c2, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&v1, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&buf31, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&buf44, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&v2, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&v3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&Ro2, rTl, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&to2, rTl, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&Rn1, obj->rectified_poseleft, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&tn1, obj->rectified_poseleft, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&Rn2, obj->rectified_poseright, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&tn2, obj->rectified_poseright, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dr, R);

   error = rox_array2d_double_get_data_pointer(&db1, buf31);

   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_get_data_pointer(&dv1, v1);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_get_data_pointer(&dv2, v2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer(&dv3, v3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(obj->rectified_poseleft);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillunit(obj->rectified_poseright);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(obj->rectified_relativepose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error =  rox_calibration_optimalview(Klud, Kl, Dl, width, height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_optimalview(Krud, Kr, Dr, width, height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmattransmat(c2, Ro2, to2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale(c2, c2, -1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(v1, c2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   db1[0] = 0;
   db1[1] = 0;
   db1[2] = 1;

   error = rox_array2d_double_crossprod(v2, buf31, v1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_crossprod(v3, v1, v2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   normv1 = sqrt(dv1[0]*dv1[0] + dv1[1]*dv1[1] + dv1[2]*dv1[2]);
   normv2 = sqrt(dv2[0]*dv2[0] + dv2[1]*dv2[1] + dv2[2]*dv2[2]);
   normv3 = sqrt(dv3[0]*dv3[0] + dv3[1]*dv3[1] + dv3[2]*dv3[2]);

   dr[0][0] = dv1[0] / normv1;
   dr[0][1] = dv1[1] / normv1;
   dr[0][2] = dv1[2] / normv1;
   dr[1][0] = dv2[0] / normv2;
   dr[1][1] = dv2[1] / normv2;
   dr[1][2] = dv2[2] / normv2;
   dr[2][0] = dv3[0] / normv3;
   dr[2][1] = dv3[1] / normv3;
   dr[2][2] = dv3[2] / normv3;

   // Compute rectified calibration
   error = rox_array2d_double_mean(A, Klud, Krud);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute new translations
   error = rox_array2d_double_mulmatmat(tn2, R, c2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale(tn2, tn2, -1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(tn1, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute oldHlnew scale shift
   error = rox_array2d_double_svdinverse(KR, Klud);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(AR, A, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(obj->rectified_homographyleft, AR, KR);
   ROX_ERROR_CHECK_TERMINATE ( error );

   rw = width;
   rh = height;

   error = rox_array2d_double_get_data_pointer_to_pointer(&dh, obj->rectified_homographyleft);
   ROX_ERROR_CHECK_TERMINATE ( error );

   u = dh[0][0] * 0.0 + dh[0][1] * 0.0 + dh[0][2];
   v = dh[1][0] * 0.0 + dh[1][1] * 0.0 + dh[1][2];
   w = dh[2][0] * 0.0 + dh[2][1] * 0.0 + dh[2][2];
   us[0] = u / w;
   vs[0] = v / w;

   u = dh[0][0] * rw + dh[0][1] * 0.0 + dh[0][2];
   v = dh[1][0] * rw + dh[1][1] * 0.0 + dh[1][2];
   w = dh[2][0] * rw + dh[2][1] * 0.0 + dh[2][2];
   us[1] = u / w;
   vs[1] = v / w;

   u = dh[0][0] * rw + dh[0][1] * rh + dh[0][2];
   v = dh[1][0] * rw + dh[1][1] * rh + dh[1][2];
   w = dh[2][0] * rw + dh[2][1] * rh + dh[2][2];
   us[2] = u / w;
   vs[2] = v / w;

   u = dh[0][0] * 0.0 + dh[0][1] * rh + dh[0][2];
   v = dh[1][0] * 0.0 + dh[1][1] * rh + dh[1][2];
   w = dh[2][0] * 0.0 + dh[2][1] * rh + dh[2][2];
   us[3] = u / w;
   vs[3] = v / w;

   minu = us[0];
   if (minu > us[1]) minu = us[1];
   if (minu > us[2]) minu = us[2];
   if (minu > us[3]) minu = us[3];
   minv = vs[0];
   if (minv > vs[1]) minv = vs[1];
   if (minv > vs[2]) minv = vs[2];
   if (minv > vs[3]) minv = vs[3];
   maxu = us[0];
   if (maxu < us[1]) maxu = us[1];
   if (maxu < us[2]) maxu = us[2];
   if (maxu < us[3]) maxu = us[3];
   maxv = vs[0];
   if (maxv < vs[1]) maxv = vs[1];
   if (maxv < vs[2]) maxv = vs[2];
   if (maxv < vs[3]) maxv = vs[3];
   desth = maxv - minv;
   destw = maxu - minu;

   offsetu = minu;
   offsetv = minv;
   scaleh = rh/desth;
   scalew = rw/destw;
   scale = scalew;
   if (scalew > scaleh) scale = scaleh;

   error = rox_array2d_double_get_data_pointer_to_pointer(&dk, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dk[0][0] = scale * dk[0][0];
   dk[1][1] = scale * dk[1][1];
   dk[0][2] = scale * (dk[0][2] - offsetu);
   dk[1][2] = scale * (dk[1][2] - offsetv);

   // Compute oldHlnew
   error = rox_array2d_double_mulmatmat(AR, A, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svdinverse(iAR, AR);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(obj->rectified_homographyleft, Klud, iAR);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute oldHrnew
   error = rox_array2d_double_mulmatmat(KR, Krud, Ro2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(obj->rectified_homographyright, KR, iAR);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Save new calibration
   error = rox_array2d_double_copy(obj->rectified_calibration, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Set new poses
   error = rox_array2d_double_copy(Rn1, R); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_copy(Rn2, R); ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svdinverse(buf44, obj->rectified_poseleft); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(obj->rectified_relativepose, obj->rectified_poseright, buf44); ROX_ERROR_CHECK_TERMINATE ( error );

   //Warping buffers

   error = rox_array2d_uchar_new(&obj->rectified_imageleft, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&obj->rectified_imageright, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&obj->rectified_rgba_imageleft, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&obj->rectified_rgba_imageright, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&obj->rectified_leftmask, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&obj->rectified_rightmask, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&obj->rectified_grid_left, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&obj->rectified_grid_right, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_sshort_new(&obj->rectified_grid_left_fixed, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_sshort_new(&obj->rectified_grid_right_fixed, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_warp_grid_distortion_sl3_float(obj->rectified_grid_left, obj->rectified_homographyleft, Klud, Kl, Dl);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_warp_grid_distortion_sl3_float(obj->rectified_grid_right, obj->rectified_homographyright, Krud, Kr, Dr);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute rectified masks
   error = rox_array2d_uchar_fillval(obj->rectified_imageleft, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_fillval(obj->rectified_imageright, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_remap_bilinear_omo_uchar_to_uchar(obj->rectified_imageleft, obj->rectified_leftmask, obj->rectified_imageleft, obj->rectified_grid_left);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_remap_bilinear_omo_uchar_to_uchar(obj->rectified_imageright, obj->rectified_rightmask, obj->rectified_imageright, obj->rectified_grid_right);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute fixed point
   error = rox_array2d_float_get_data_pointer_to_pointer(&src_u, obj->rectified_grid_left->u);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_get_data_pointer_to_pointer(&src_v, obj->rectified_grid_left->v);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer(&dst, obj->rectified_grid_left_fixed);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < (Rox_Uint) height; i++)
   {
      for (Rox_Uint j = 0; j < (Rox_Uint) width; j++)
      {
         Rox_Float val;

         val = src_u[i][j];
         if (val>= width - 1) val = 0;
         val = (Rox_Float)(val * 16.0);
         if (val < 0.0) val = 0.0;
         dst[i][j].u = (Rox_Sshort)val;

         val = src_v[i][j];
         if (val >= height - 1) val = 0;
         val = (Rox_Float)(val * 16.0);
         if (val < 0.0) val = 0.0;
         dst[i][j].v = (Rox_Sshort)val;
      }
   }

   error = rox_array2d_float_get_data_pointer_to_pointer( &src_u, obj->rectified_grid_right->u);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_get_data_pointer_to_pointer( &src_v, obj->rectified_grid_right->v);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer( &dst, obj->rectified_grid_right_fixed);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < (Rox_Uint) height; i++)
   {
      for (Rox_Uint j = 0; j < (Rox_Uint) width; j++)
      {
         Rox_Float val;

         val = src_u[i][j];
         if (val>= width - 1) val = 0;
         val = (Rox_Float)(val * 16.0);
         if (val < 0.0) val = 0.0;
         dst[i][j].u = (Rox_Sshort)val;

         val = src_v[i][j];
         if (val >= height - 1) val = 0;
         val = (Rox_Float)(val * 16.0);
         if (val < 0.0) val = 0.0;
         dst[i][j].v = (Rox_Sshort)val;
      }
   }

function_terminate:

   rox_array2d_double_del(&AR);
   rox_array2d_double_del(&KR);
   rox_array2d_double_del(&iAR);
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&Ro2);
   rox_array2d_double_del(&A);
   rox_array2d_double_del(&tn1);
   rox_array2d_double_del(&tn2);
   rox_array2d_double_del(&Rn1);
   rox_array2d_double_del(&Rn2);
   rox_array2d_double_del(&to2);
   rox_array2d_double_del(&c2);
   rox_array2d_double_del(&buf31);
   rox_array2d_double_del(&buf44);
   rox_array2d_double_del(&v1);
   rox_array2d_double_del(&v2);
   rox_array2d_double_del(&v3);
   rox_array2d_double_del(&Krud);
   rox_array2d_double_del(&Klud);

   return error;
}

Rox_ErrorCode rox_stereorectifier_apply ( 
   Rox_StereoRectifier obj, 
   Rox_Image left, 
   Rox_Image right
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !left || !right) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   error = rox_remap_bilinear_nomask_uchar_to_uchar_fixed ( obj->rectified_imageleft, left, obj->rectified_grid_left_fixed);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_remap_bilinear_nomask_uchar_to_uchar_fixed ( obj->rectified_imageright, right, obj->rectified_grid_right_fixed);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_stereorectifier_rgba_apply (
   Rox_StereoRectifier obj, 
   Rox_Array2D_Uint rgba_left, 
   Rox_Array2D_Uint rgba_right
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !rgba_left || !rgba_right) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   error = rox_remap_bilinear_nomask_rgba_fixed(obj->rectified_rgba_imageleft, rgba_left, obj->rectified_grid_left_fixed);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_remap_bilinear_nomask_rgba_fixed(obj->rectified_rgba_imageright, rgba_right, obj->rectified_grid_right_fixed);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_roxrgba_to_roxgray(obj->rectified_imageleft, obj->rectified_rgba_imageleft);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_roxrgba_to_roxgray(obj->rectified_imageright, obj->rectified_rgba_imageright);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
