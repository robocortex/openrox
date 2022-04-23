//==============================================================================
//
//    OPENROX   : File objset_edge_ellipse_tools.c
//
//    Contents  : Implementation of module odometry ellipses
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "objset_edge_ellipse_tools.h"

#include <float.h>
#include <generated/dynvec_edge_ellipse_site_struct.h>

#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/ellipse/ellipse3d_struct.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/robust/huber.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/geometry/measures/distance_point_to_ellipse.h>
#include <baseproc/geometry/point/point2d_tools.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_edge_ellipse_build_error (
   Rox_Double ** res_error,
   const Rox_MatUT3 K,
   const Rox_Edge_Ellipse edge_ellipse
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double * ptrRes = NULL;


   if (!res_error || !K || !edge_ellipse)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ptrRes = *res_error;
   if (!ptrRes)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
   {
      Rox_Edge_Ellipse_Site_Struct * site = &edge_ellipse->sites->data[idsite];
      if (site->state) continue;

      // site->coords is a point2d with pixels coordinates

      // Compute the point2d normalized coordinates (from pixel to meters)
      Rox_Point2D_Double_Struct point2d_meters;


      error = rox_point2d_convert_pixel_double_to_meter_double ( &point2d_meters, &(site->coords), K );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Signed distance in normalized coordinates
      error = rox_signed_distance_point2d_to_ellipse2d_algebraic ( ptrRes, &edge_ellipse->ellipse2d_meters, &point2d_meters );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Increment pointer
      ptrRes++;
   }

   *res_error = ptrRes;

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_ellipse_build_interaction_matrix (
   Rox_Matrix L,
   const Rox_MatUT3 K,
   const Rox_Edge_Ellipse edge_ellipse
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Matrix J_ad_ep = NULL;
   Rox_Matrix J_ep_sp = NULL;
   Rox_Matrix L_sp = NULL;
   Rox_Matrix L_ep = NULL;
   Rox_Matrix L_ad = NULL;

   Rox_MatSE3 cTe = NULL;
   Rox_MatSE3 eTc = NULL;

   if (!L || !K || !edge_ellipse)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint valid_ellipse_sites = edge_ellipse->sites->used;

   // error = rox_array2d_double_check_size(L, valid_ellipse_sites, 6);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&J_ad_ep, 1, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&J_ep_sp, 5, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&L_ep, 5, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&L_sp, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&L_ad, 1, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new(&cTe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new(&eTc);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get pointers to data
   Rox_Double ** dL_ad    = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dL_ad, L_ad);
   Rox_Double ** dL_sp    = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dL_sp, L_sp);
   Rox_Double ** dJ_ad_ep = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dJ_ad_ep, J_ad_ep);
   Rox_Double ** dJ_ep_sp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dJ_ep_sp, J_ep_sp);
   Rox_Double ** eTc_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &eTc_data, eTc);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get pointers to data
   Rox_Double ** dL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dL, L);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double fu = 0.0, fv = 0.0, cu = 0.0, cv = 0.0;
   // Get the camera intrinsic parameters
   rox_array2d_double_get_value(&fu, K, 0, 0);
   rox_array2d_double_get_value(&fv, K, 1, 1);
   rox_array2d_double_get_value(&cu, K, 0, 2);
   rox_array2d_double_get_value(&cv, K, 1, 2);

   // Get the inverse camera intrinsic parameters
   Rox_Double ifu = 1.0 / fu;
   Rox_Double ifv = 1.0 / fv;
   Rox_Double icu = - cu / fu;
   Rox_Double icv = - cv / fv;

   // Compute the Jacobian

   // Get the 2D ellipse parameters
   Rox_Double xc = edge_ellipse->ellipse2d_meters.xc;
   Rox_Double yc = edge_ellipse->ellipse2d_meters.yc;
   Rox_Double nxx = edge_ellipse->ellipse2d_meters.nxx;
   Rox_Double nyy = edge_ellipse->ellipse2d_meters.nyy;
   Rox_Double nxy = edge_ellipse->ellipse2d_meters.nxy;

   // Get the 3D ellipse parameters
   Rox_Double a = edge_ellipse->ellipse3d_c->a;
   Rox_Double b = edge_ellipse->ellipse3d_c->b;

   error = rox_matse3_copy(cTe, edge_ellipse->ellipse3d_c->Te);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_inv(eTc, cTe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double r11 = eTc_data[0][0]; Rox_Double r12 = eTc_data[0][1]; Rox_Double r13 = eTc_data[0][2]; Rox_Double t1 = eTc_data[0][3];
   Rox_Double r21 = eTc_data[1][0]; Rox_Double r22 = eTc_data[1][1]; Rox_Double r23 = eTc_data[1][2]; Rox_Double t2 = eTc_data[1][3];
   Rox_Double r31 = eTc_data[2][0]; Rox_Double r32 = eTc_data[2][1]; Rox_Double r33 = eTc_data[2][2]; Rox_Double t3 = eTc_data[2][3];

   Rox_Double k11 = (r11*t3 - r31*t1)/a;
   Rox_Double k12 = (r12*t3 - r32*t1)/a;
   Rox_Double k13 = (r13*t3 - r33*t1)/a;
   Rox_Double k21 = (r21*t3 - r31*t2)/b;
   Rox_Double k22 = (r22*t3 - r32*t2)/b;
   Rox_Double k23 = (r23*t3 - r33*t2)/b;
   Rox_Double k31 = r31;
   Rox_Double k32 = r32;
   Rox_Double k33 = r33;

   Rox_Double k11_2 = k11*k11;
   Rox_Double k12_2 = k12*k12;
   Rox_Double k13_2 = k13*k13;

   Rox_Double k21_2 = k21*k21;
   Rox_Double k22_2 = k22*k22;
   Rox_Double k23_2 = k23*k23;

   Rox_Double k31_2 = k31*k31;
   Rox_Double k32_2 = k32*k32;
   Rox_Double k33_2 = k33*k33;

   Rox_Double h11 = r11/b;
   Rox_Double h12 = r12/b;
   Rox_Double h13 = r13/b;

   Rox_Double h21 = r21/a;
   Rox_Double h22 = r22/a;
   Rox_Double h23 = r23/a;

   Rox_Double sxx = (k11_2 + k21_2 - k31_2);
   Rox_Double syy = (k12_2 + k22_2 - k32_2);
   Rox_Double sxy = (k11*k12 + k21*k22 - k31*k32);
   Rox_Double sx = (k11*k13 + k21*k23 - k31*k33);
   Rox_Double sy = (k12*k13 + k22*k23 - k32*k33);
   Rox_Double sq = (k13_2 + k23_2 - k33_2);

   Rox_Double sx_2 =  sx*sx;
   Rox_Double sy_2 =  sy*sy;

   Rox_Double sxx_2 =  sxx*sxx;
   Rox_Double syy_2 =  syy*syy;

   Rox_Double sxy_2 =  sxy*sxy;
   Rox_Double sxy_3 =  sxy_2*sxy;
   Rox_Double sxy_4 =  sxy_3*sxy;

   Rox_Double g0 = (sxx*syy - sxy_2);
   Rox_Double g1 = (syy*sx_2 - 2*sx*sxy*sy + sxx*sy_2 - sq*g0);
   Rox_Double g2 = sq*(sxx_2*syy_2 - 2*sxx*sxy_2*syy + sxy_4);

   Rox_Double g3 = -(- 2*sx_2*sxx*syy_2 +   sx_2*sxy_2*syy + 4*sx*sxx*sxy*sy*syy - 2*sx*sxy_3*sy -                      sxx_2*sy_2*syy + g2);
   Rox_Double g4 = -(-   sx_2*sxx*syy_2 +                    4*sx*sxx*sxy*sy*syy - 2*sx*sxy_3*sy +   sxx*sxy_2*sy_2 - 2*sxx_2*sy_2*syy + g2);
   Rox_Double g5 = -(-   sx_2*sxx*syy_2 + 3*sx_2*sxy_2*syy                       - 4*sx*sxy_3*sy + 3*sxx*sxy_2*sy_2   - sxx_2*sy_2*syy + g2);

   Rox_Double g6 = (sx*syy - sxy*sy);
   Rox_Double g7 = (sx*sxy - sxx*sy);
   Rox_Double g8 = (sy*sxy_2 - 2*sx*syy*sxy +   sxx*sy*syy);
   Rox_Double g9 = (sx*sxy_2 +   sx*sxx*syy - 2*sxx*sy*sxy);

   Rox_Double g0_2 = g0*g0;
   Rox_Double g1_2 = g1*g1;
   Rox_Double g6_2 = g6*g6;
   Rox_Double g7_2 = g7*g7;

   // Compute the "fixed" part of the Jacobian (i.e. not depending on site point q)
   dJ_ep_sp[0][0] =  syy*g6/g0_2;
   dJ_ep_sp[0][1] =  sxy*g7/g0_2;
   dJ_ep_sp[0][2] =  g8/g0_2;
   dJ_ep_sp[0][3] = -syy/g0;
   dJ_ep_sp[0][4] =  sxy/g0;
   dJ_ep_sp[0][5] =     0.0;

   dJ_ep_sp[1][0] = -sxy*g6/g0_2;
   dJ_ep_sp[1][1] = -sxx*g7/g0_2;
   dJ_ep_sp[1][2] =  g9/g0_2;
   dJ_ep_sp[1][3] =  sxy/g0;
   dJ_ep_sp[1][4] = -sxx/g0;
   dJ_ep_sp[1][5] =     0.0;

   dJ_ep_sp[2][0] =           g3/g1_2;
   dJ_ep_sp[2][1] =     sxx*g7_2/g1_2;
   dJ_ep_sp[2][2] = -2*sxx*g7*g6/g1_2;
   dJ_ep_sp[2][3] = -2*sxx*g0*g6/g1_2;
   dJ_ep_sp[2][4] =  2*sxx*g0*g7/g1_2;
   dJ_ep_sp[2][5] =     sxx*g0_2/g1_2;

   dJ_ep_sp[3][0] =     syy*g6_2/g1_2;
   dJ_ep_sp[3][1] =           g4/g1_2;
   dJ_ep_sp[3][2] = -2*syy*g7*g6/g1_2;
   dJ_ep_sp[3][3] = -2*syy*g0*g6/g1_2;
   dJ_ep_sp[3][4] =  2*syy*g0*g7/g1_2;
   dJ_ep_sp[3][5] =     syy*g0_2/g1_2;

   dJ_ep_sp[4][0] =     sxy*g6_2/g1_2;
   dJ_ep_sp[4][1] =     sxy*g7_2/g1_2;
   dJ_ep_sp[4][2] =           g5/g1_2;
   dJ_ep_sp[4][3] = -2*sxy*g0*g6/g1_2;
   dJ_ep_sp[4][4] =  2*sxy*g0*g7/g1_2;
   dJ_ep_sp[4][5] =     sxy*g0_2/g1_2;

   dL_sp[0][0] = 0.0;
   dL_sp[0][1] = -2*h23*k11 + 2*h13*k21;
   dL_sp[0][2] =  2*h22*k11 - 2*h12*k21;
   dL_sp[0][3] = 0.0;
   dL_sp[0][4] =   -2*sx;
   dL_sp[0][5] =   2*sxy;
   dL_sp[1][0] =  2*h23*k12 - 2*h13*k22;
   dL_sp[1][1] = 0.0;
   dL_sp[1][2] = -2*h21*k12 + 2*h11*k22;
   dL_sp[1][3] =  2*sy;
   dL_sp[1][4] =   0.0;
   dL_sp[1][5] = -2*sxy;
   dL_sp[2][0] =  h23*k11 - h13*k21;
   dL_sp[2][1] = -h23*k12 + h13*k22;
   dL_sp[2][2] = h22*k12 - h21*k11 + h11*k21 - h12*k22;
   dL_sp[2][3] =      sx;
   dL_sp[2][4] =     -sy;
   dL_sp[2][5] = syy - sxx;
   dL_sp[3][0] = - h22*k11 + h12*k21;
   dL_sp[3][1] = h21*k11 - h23*k13 + h13*k23 - h11*k21;
   dL_sp[3][2] =   h22*k13 - h12*k23;
   dL_sp[3][3] =   - sxy;
   dL_sp[3][4] = sxx - sq;
   dL_sp[3][5] =       sy;
   dL_sp[4][0] =  h23*k13 - h22*k12 + h12*k22 - h13*k23;
   dL_sp[4][1] =  h21*k12 - h11*k22;
   dL_sp[4][2] = -h21*k13 + h11*k23;
   dL_sp[4][3] = sq - syy;
   dL_sp[4][4] =     sxy;
   dL_sp[4][5] =     -sx;
   dL_sp[5][0] = - 2*h22*k13 + 2*h12*k23;
   dL_sp[5][1] =   2*h21*k13 - 2*h11*k23;
   dL_sp[5][2] =     0.0;
   dL_sp[5][3] =   -2*sy;
   dL_sp[5][4] =    2*sx;
   dL_sp[5][5] =     0.0;

   // Compute L_ep = J_ep_sp * L_sp
   error = rox_array2d_double_mulmatmat(L_ep, J_ep_sp, L_sp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the "variable" part of the Jacobian (i.e. depending on site point q)
   for (Rox_Sint idsite = 0; idsite < valid_ellipse_sites; idsite++)
   {
      Rox_Edge_Ellipse_Site_Struct * site = &edge_ellipse->sites->data[idsite];
      if (site->state) continue;

      Rox_Double x = ifu * site->coords.u + icu;
      Rox_Double y = ifv * site->coords.v + icv;

      Rox_Double xd = x - xc;
      Rox_Double yd = y - yc;

      dJ_ad_ep[0][0] = - 2*nxy*yd - 2*nxx*xd ;
      dJ_ad_ep[0][1] = - 2*nxy*xd - 2*nyy*yd ;
      dJ_ad_ep[0][2] =     xd*xd ;
      dJ_ad_ep[0][3] =     yd*yd ;
      dJ_ad_ep[0][4] =   2*xd*yd ;

      // Compute L_ad = J_ad_ep * L_ep
      error = rox_array2d_double_mulmatmat(L_ad, J_ad_ep, L_ep);
      ROX_ERROR_CHECK_TERMINATE ( error );

      for (Rox_Sint col = 0; col < 6; col++)
      {
         dL[idsite][col] = dL_ad[0][col];
      }
   }

function_terminate:

   rox_array2d_double_del(&J_ad_ep);
   rox_array2d_double_del(&J_ep_sp);
   rox_array2d_double_del(&L_ep);
   rox_array2d_double_del(&L_sp);
   rox_array2d_double_del(&L_ad);

   rox_matse3_del(&cTe);
   rox_matse3_del(&eTc);

   return error;
}

Rox_ErrorCode rox_edge_ellipse_build_linear_system(Rox_Array2D_Double LtL, Rox_Array2D_Double Lte, const Rox_Array2D_Double K, const Rox_Edge_Ellipse edge_ellipse, Rox_Double * ptrError, Rox_Double * ptrWeight)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double w = 0.0, e = 0.0;

   Rox_Array2D_Double J_de_ep = NULL;
   Rox_Array2D_Double J_ga_ep = NULL;

   Rox_Array2D_Double J_ad_ep = NULL;
   Rox_Array2D_Double J_ep_sp = NULL;
   Rox_Array2D_Double L_sp = NULL;
   Rox_Array2D_Double L_ep = NULL;
   Rox_Array2D_Double L_ad = NULL;

   Rox_MatSE3 cTe = NULL;
   Rox_MatSE3 eTc = NULL;


   if (!LtL || !Lte || !K || !edge_ellipse || !ptrError || !ptrWeight)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(LtL, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(LtL, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Lte, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&J_de_ep, 1, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&J_ga_ep, 1, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&J_ad_ep, 1, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&J_ep_sp, 5, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&L_ep, 5, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&L_sp, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&L_ad, 1, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new(&cTe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new(&eTc);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get pointers to data
   Rox_Double ** dL_ad    = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dL_ad, L_ad);
   Rox_Double ** dL_sp    = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dL_sp, L_sp);
   Rox_Double ** dJ_de_ep = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dJ_de_ep, J_de_ep);
   Rox_Double ** dJ_ga_ep = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dJ_ga_ep, J_ga_ep);
   Rox_Double ** dJ_ad_ep = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dJ_ad_ep, J_ad_ep);
   Rox_Double ** dJ_ep_sp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dJ_ep_sp, J_ep_sp);
   Rox_Double ** eTc_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &eTc_data, eTc);

   // Get pointers to data
   Rox_Double ** dLtL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dLtL, LtL);
   Rox_Double *  dLte = NULL;
   error = rox_array2d_double_get_data_pointer ( &dLte, Lte) ;

   Rox_Double fu, fv, cu, cv;
   // Get the camera intrinsic parameters
   rox_array2d_double_get_value(&fu, K, 0, 0);
   rox_array2d_double_get_value(&fv, K, 1, 1);
   rox_array2d_double_get_value(&cu, K, 0, 2);
   rox_array2d_double_get_value(&cv, K, 1, 2);

   // Get the inverse camera intrinsic parameters
   Rox_Double ifu = 1.0 / fu;
   Rox_Double ifv = 1.0 / fv;
   Rox_Double icu = - cu / fu;
   Rox_Double icv = - cv / fv;

   // Compute the Jacobian

   // Get the 2D ellipse parameters
   Rox_Double xc = edge_ellipse->ellipse2d_meters.xc;
   Rox_Double yc = edge_ellipse->ellipse2d_meters.yc;
   Rox_Double nxx = edge_ellipse->ellipse2d_meters.nxx;
   Rox_Double nyy = edge_ellipse->ellipse2d_meters.nyy;
   Rox_Double nxy = edge_ellipse->ellipse2d_meters.nxy;

   // Get the 3D ellipse parameters
   Rox_Double a = edge_ellipse->ellipse3d_c->a;
   Rox_Double b = edge_ellipse->ellipse3d_c->b;

   error = rox_matse3_copy(cTe, edge_ellipse->ellipse3d_c->Te);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_inv(eTc, cTe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double r11 = eTc_data[0][0]; Rox_Double r12 = eTc_data[0][1]; Rox_Double r13 = eTc_data[0][2]; Rox_Double t1 = eTc_data[0][3];
   Rox_Double r21 = eTc_data[1][0]; Rox_Double r22 = eTc_data[1][1]; Rox_Double r23 = eTc_data[1][2]; Rox_Double t2 = eTc_data[1][3];
   Rox_Double r31 = eTc_data[2][0]; Rox_Double r32 = eTc_data[2][1]; Rox_Double r33 = eTc_data[2][2]; Rox_Double t3 = eTc_data[2][3];

   Rox_Double k11 = (r11*t3 - r31*t1)/a;
   Rox_Double k12 = (r12*t3 - r32*t1)/a;
   Rox_Double k13 = (r13*t3 - r33*t1)/a;
   Rox_Double k21 = (r21*t3 - r31*t2)/b;
   Rox_Double k22 = (r22*t3 - r32*t2)/b;
   Rox_Double k23 = (r23*t3 - r33*t2)/b;
   Rox_Double k31 = r31;
   Rox_Double k32 = r32;
   Rox_Double k33 = r33;

   Rox_Double k11_2 = k11*k11;
   Rox_Double k12_2 = k12*k12;
   Rox_Double k13_2 = k13*k13;

   Rox_Double k21_2 = k21*k21;
   Rox_Double k22_2 = k22*k22;
   Rox_Double k23_2 = k23*k23;

   Rox_Double k31_2 = k31*k31;
   Rox_Double k32_2 = k32*k32;
   Rox_Double k33_2 = k33*k33;

   Rox_Double h11 = r11/b;
   Rox_Double h12 = r12/b;
   Rox_Double h13 = r13/b;

   Rox_Double h21 = r21/a;
   Rox_Double h22 = r22/a;
   Rox_Double h23 = r23/a;

   Rox_Double sxx = (k11_2 + k21_2 - k31_2);
   Rox_Double syy = (k12_2 + k22_2 - k32_2);
   Rox_Double sxy = (k11*k12 + k21*k22 - k31*k32);
   Rox_Double sx = (k11*k13 + k21*k23 - k31*k33);
   Rox_Double sy = (k12*k13 + k22*k23 - k32*k33);
   Rox_Double sq = (k13_2 + k23_2 - k33_2);

   Rox_Double sx_2 =  sx*sx;
   Rox_Double sy_2 =  sy*sy;

   Rox_Double sxx_2 =  sxx*sxx;
   Rox_Double syy_2 =  syy*syy;

   Rox_Double sxy_2 =  sxy*sxy;
   Rox_Double sxy_3 =  sxy_2*sxy;
   Rox_Double sxy_4 =  sxy_3*sxy;

   Rox_Double g0 = (sxx*syy - sxy_2);
   Rox_Double g1 = (syy*sx_2 - 2*sx*sxy*sy + sxx*sy_2 - sq*g0);
   Rox_Double g2 = sq*(sxx_2*syy_2 - 2*sxx*sxy_2*syy + sxy_4);

   Rox_Double g3 = -(- 2*sx_2*sxx*syy_2 +   sx_2*sxy_2*syy + 4*sx*sxx*sxy*sy*syy - 2*sx*sxy_3*sy -                      sxx_2*sy_2*syy + g2);
   Rox_Double g4 = -(-   sx_2*sxx*syy_2 +                    4*sx*sxx*sxy*sy*syy - 2*sx*sxy_3*sy +   sxx*sxy_2*sy_2 - 2*sxx_2*sy_2*syy + g2);
   Rox_Double g5 = -(-   sx_2*sxx*syy_2 + 3*sx_2*sxy_2*syy                       - 4*sx*sxy_3*sy + 3*sxx*sxy_2*sy_2   - sxx_2*sy_2*syy + g2);

   Rox_Double g6 = (sx*syy - sxy*sy);
   Rox_Double g7 = (sx*sxy - sxx*sy);
   Rox_Double g8 = (sy*sxy_2 - 2*sx*syy*sxy +   sxx*sy*syy);
   Rox_Double g9 = (sx*sxy_2 +   sx*sxx*syy - 2*sxx*sy*sxy);

   Rox_Double g0_2 = g0*g0;
   Rox_Double g1_2 = g1*g1;
   Rox_Double g6_2 = g6*g6;
   Rox_Double g7_2 = g7*g7;

   // Compute the "fixed" part of the Jacobian (i.e. not depending on site point q)
   dJ_ep_sp[0][0] =  syy*g6/g0_2;
   dJ_ep_sp[0][1] =  sxy*g7/g0_2;
   dJ_ep_sp[0][2] =  g8/g0_2;
   dJ_ep_sp[0][3] = -syy/g0;
   dJ_ep_sp[0][4] =  sxy/g0;
   dJ_ep_sp[0][5] =     0.0;

   dJ_ep_sp[1][0] = -sxy*g6/g0_2;
   dJ_ep_sp[1][1] = -sxx*g7/g0_2;
   dJ_ep_sp[1][2] =  g9/g0_2;
   dJ_ep_sp[1][3] =  sxy/g0;
   dJ_ep_sp[1][4] = -sxx/g0;
   dJ_ep_sp[1][5] =     0.0;

   dJ_ep_sp[2][0] =           g3/g1_2;
   dJ_ep_sp[2][1] =     sxx*g7_2/g1_2;
   dJ_ep_sp[2][2] = -2*sxx*g7*g6/g1_2;
   dJ_ep_sp[2][3] = -2*sxx*g0*g6/g1_2;
   dJ_ep_sp[2][4] =  2*sxx*g0*g7/g1_2;
   dJ_ep_sp[2][5] =     sxx*g0_2/g1_2;

   dJ_ep_sp[3][0] =     syy*g6_2/g1_2;
   dJ_ep_sp[3][1] =           g4/g1_2;
   dJ_ep_sp[3][2] = -2*syy*g7*g6/g1_2;
   dJ_ep_sp[3][3] = -2*syy*g0*g6/g1_2;
   dJ_ep_sp[3][4] =  2*syy*g0*g7/g1_2;
   dJ_ep_sp[3][5] =     syy*g0_2/g1_2;

   dJ_ep_sp[4][0] =     sxy*g6_2/g1_2;
   dJ_ep_sp[4][1] =     sxy*g7_2/g1_2;
   dJ_ep_sp[4][2] =           g5/g1_2;
   dJ_ep_sp[4][3] = -2*sxy*g0*g6/g1_2;
   dJ_ep_sp[4][4] =  2*sxy*g0*g7/g1_2;
   dJ_ep_sp[4][5] =     sxy*g0_2/g1_2;

   dL_sp[0][0] = 0.0;
   dL_sp[0][1] = -2*h23*k11 + 2*h13*k21;
   dL_sp[0][2] =  2*h22*k11 - 2*h12*k21;
   dL_sp[0][3] = 0.0;
   dL_sp[0][4] =   -2*sx;
   dL_sp[0][5] =   2*sxy;
   dL_sp[1][0] =  2*h23*k12 - 2*h13*k22;
   dL_sp[1][1] = 0.0;
   dL_sp[1][2] = -2*h21*k12 + 2*h11*k22;
   dL_sp[1][3] =  2*sy;
   dL_sp[1][4] =   0.0;
   dL_sp[1][5] = -2*sxy;
   dL_sp[2][0] =  h23*k11 - h13*k21;
   dL_sp[2][1] = -h23*k12 + h13*k22;
   dL_sp[2][2] = h22*k12 - h21*k11 + h11*k21 - h12*k22;
   dL_sp[2][3] =      sx;
   dL_sp[2][4] =     -sy;
   dL_sp[2][5] = syy - sxx;
   dL_sp[3][0] = - h22*k11 + h12*k21;
   dL_sp[3][1] = h21*k11 - h23*k13 + h13*k23 - h11*k21;
   dL_sp[3][2] =   h22*k13 - h12*k23;
   dL_sp[3][3] =   - sxy;
   dL_sp[3][4] = sxx - sq;
   dL_sp[3][5] =       sy;
   dL_sp[4][0] =  h23*k13 - h22*k12 + h12*k22 - h13*k23;
   dL_sp[4][1] =  h21*k12 - h11*k22;
   dL_sp[4][2] = -h21*k13 + h11*k23;
   dL_sp[4][3] = sq - syy;
   dL_sp[4][4] =     sxy;
   dL_sp[4][5] =     -sx;
   dL_sp[5][0] = - 2*h22*k13 + 2*h12*k23;
   dL_sp[5][1] =   2*h21*k13 - 2*h11*k23;
   dL_sp[5][2] =     0.0;
   dL_sp[5][3] =   -2*sy;
   dL_sp[5][4] =    2*sx;
   dL_sp[5][5] =     0.0;

   // Compute L_ep = J_ep_sp * L_sp
   error = rox_array2d_double_mulmatmat(L_ep, J_ep_sp, L_sp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the "variable" part of the Jacobian (i.e. depending on site point q)
   for (Rox_Uint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
   {
      Rox_Edge_Ellipse_Site_Struct * site = &edge_ellipse->sites->data[idsite];
      if (site->state) continue;

      w = *ptrWeight;

      e = *ptrError;
      e = e * w;

      Rox_Double x = ifu * site->coords.u + icu;
      Rox_Double y = ifv * site->coords.v + icv;

      Rox_Double xd = x - xc;
      Rox_Double yd = y - yc;

      Rox_Double delta = xd*xd+yd*yd;
      Rox_Double gamma = nxx*xd*xd + 2*nxy*xd*yd + nyy*yd*yd ;

      Rox_Double k_de = 0.5*(1-1/sqrt(gamma))*(1/sqrt(delta));
      Rox_Double k_ga = 0.5*(sqrt(delta/(gamma*gamma*gamma)));

      // Compute J_ga_ep
      dJ_ga_ep[0][0] = - 2*nxy*yd - 2*nxx*xd ;
      dJ_ga_ep[0][1] = - 2*nxy*xd - 2*nyy*yd ;
      dJ_ga_ep[0][2] =     xd*xd ;
      dJ_ga_ep[0][3] =     yd*yd ;
      dJ_ga_ep[0][4] =   2*xd*yd ;

      // Compute J_de_ep
      dJ_de_ep[0][0] = -2.0 * xd ;
      dJ_de_ep[0][1] = -2.0 * yd ;
      dJ_de_ep[0][2] =  0.0;
      dJ_de_ep[0][3] =  0.0;
      dJ_de_ep[0][4] =  0.0;

      // Compute J_ad_ep
      dJ_ad_ep[0][0] = k_de * dJ_de_ep[0][0] + k_ga * dJ_ga_ep[0][0] ;
      dJ_ad_ep[0][1] = k_de * dJ_de_ep[0][1] + k_ga * dJ_ga_ep[0][1] ;
      dJ_ad_ep[0][2] = k_de * dJ_de_ep[0][2] + k_ga * dJ_ga_ep[0][2] ;
      dJ_ad_ep[0][3] = k_de * dJ_de_ep[0][3] + k_ga * dJ_ga_ep[0][3] ;
      dJ_ad_ep[0][4] = k_de * dJ_de_ep[0][4] + k_ga * dJ_ga_ep[0][4] ;

      // Compute L_ad = J_ad_ep * L_ep
      error = rox_array2d_double_mulmatmat(L_ad, J_ad_ep, L_ep);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Multiply L_ad by the weight w
      for ( Rox_Sint col = 0; col < 6; col++)
      {
         dL_ad[0][col] *= w;
      }

      // Compute LtL and Lte
      for ( Rox_Sint i = 0; i < 6; i++)
      {
         for ( Rox_Sint j = 0; j < 6; j++)
         {
            dLtL[i][j] += dL_ad[0][i] * dL_ad[0][j];
         }

         dLte[i] += dL_ad[0][i] * e;
      }

      ptrError++;
      ptrWeight++;
   }

function_terminate:

   rox_array2d_double_del(&J_de_ep);
   rox_array2d_double_del(&J_ga_ep);

   rox_array2d_double_del(&J_ad_ep);
   rox_array2d_double_del(&J_ep_sp);
   rox_array2d_double_del(&L_ep);
   rox_array2d_double_del(&L_sp);
   rox_array2d_double_del(&L_ad);

   rox_matse3_del(&cTe);
   rox_matse3_del(&eTc);

   return error;
}

Rox_ErrorCode rox_edge_ellipse_get_valid_measures(Rox_Sint * valid_measures, const Rox_Edge_Ellipse edge_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!edge_ellipse || !valid_measures)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *valid_measures = 0;
   for (Rox_Uint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
   {
      if (edge_ellipse->sites->data[idsite].state) continue;

      (*valid_measures)++;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_ellipse_add_ellipse3d(Rox_ObjSet_Edge_Ellipse objset_edge_ellipse, Rox_Ellipse3D ellipse3d, const Rox_Double sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Edge_Ellipse toadd = NULL;


   if (!objset_edge_ellipse || !ellipse3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_edge_ellipse_new(&toadd, (Rox_Sint) sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_edge_ellipse_set_ellipse3d(toadd, ellipse3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_edge_ellipse_append(objset_edge_ellipse, toadd);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (error) rox_edge_ellipse_del(&toadd);
   return error;
}

Rox_ErrorCode rox_objset_edge_ellipse_get_valid_sites(Rox_DynVec_Point2D_Double dynvec_point2d, const Rox_ObjSet_Edge_Ellipse objset_edge_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!objset_edge_ellipse || !dynvec_point2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint id = 0; id < objset_edge_ellipse->used; id++)
   {
      // Rox_Sint valid_measures_ellipse = 0;
      Rox_Edge_Ellipse edge_ellipse = objset_edge_ellipse->data[id];

      error = rox_edge_ellipse_append_valid_sites(dynvec_point2d, edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_ellipse_get_valid_measures(Rox_Sint * valid_measures, const Rox_ObjSet_Edge_Ellipse objset_edge_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!objset_edge_ellipse || !valid_measures)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *valid_measures = 0;
   for (Rox_Uint id = 0; id < objset_edge_ellipse->used; id++)
   {
      Rox_Sint valid_measures_ellipse = 0;
      Rox_Edge_Ellipse edge_ellipse = objset_edge_ellipse->data[id];

      error = rox_edge_ellipse_get_valid_measures(&valid_measures_ellipse, edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );

      *valid_measures += valid_measures_ellipse;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_ellipse_transform_project(Rox_ObjSet_Edge_Ellipse objset_edge_ellipse, const Rox_Array2D_Double K, const Rox_MatSE3 cTo)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!objset_edge_ellipse || !K ||! cTo)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint id = 0; id < objset_edge_ellipse->used; id++)
   {
      Rox_Edge_Ellipse edge_ellipse = objset_edge_ellipse->data[id];

      error = rox_edge_ellipse_transform_project(edge_ellipse, cTo, K);
      ROX_ERROR_CHECK_CONTINUE(error);
   }

   error = ROX_ERROR_NONE;

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_ellipse_build_error(Rox_Double ** res_error, const Rox_Array2D_Double K, const Rox_ObjSet_Edge_Ellipse objset_edge_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   for (Rox_Uint id = 0; id < objset_edge_ellipse->used; id++)
   {
      Rox_Edge_Ellipse edge_ellipse = objset_edge_ellipse->data[id];

      error = rox_edge_ellipse_build_error(res_error, K, edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_ellipse_build_interaction_matrix(Rox_Array2D_Double L, const Rox_Array2D_Double K, const Rox_ObjSet_Edge_Ellipse objset_edge_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint initial_row = 0, initial_col = 0, rows = 0, cols = 6;
   Rox_Array2D_Double L_sub = NULL;

   for (Rox_Uint id = 0; id < objset_edge_ellipse->used; id++)
   {
      Rox_Edge_Ellipse edge_ellipse = objset_edge_ellipse->data[id];

      error = rox_edge_ellipse_get_valid_measures(&rows, edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // If no valid measures are available then continue
      if (rows == 0) continue;

      // Create submatrix of proper size
      error = rox_array2d_double_new_subarray2d(&L_sub, L, initial_row, initial_col, rows, cols);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute contribution of ellipse id to interaction matrix
      error = rox_edge_ellipse_build_interaction_matrix(L_sub, K, edge_ellipse);

      ROX_ERROR_CHECK_TERMINATE ( error );

      // Add actual rows for next ellipse
      initial_row += rows;

      error = rox_array2d_double_del(&L_sub);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   if(error) rox_array2d_double_del(&L_sub);
   return error;
}

Rox_ErrorCode rox_objset_edge_ellipse_build_linear_system(Rox_Array2D_Double JtJ, Rox_Array2D_Double Jte, const Rox_Array2D_Double K, const Rox_ObjSet_Edge_Ellipse objset_edge_ellipse, Rox_Double ** dverr, Rox_Double ** dvw)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double * ptrErr = NULL, * ptrWeight = NULL;
   Rox_Array2D_Double lJtJ = NULL, lJte = NULL;

   error = rox_array2d_double_new(&lJtJ, 6, 6);

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&lJte, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ptrErr = dverr[0];
   ptrWeight = dvw[0];

   for (Rox_Uint id = 0; id < objset_edge_ellipse->used; id++)
   {
      // Compute contribution of ellipse id to linear system
      Rox_Edge_Ellipse edge_ellipse = objset_edge_ellipse->data[id];
      error = rox_edge_ellipse_build_linear_system(lJtJ, lJte, K, edge_ellipse, ptrErr, ptrWeight);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Add contribution of ellipse id to linear system
      rox_array2d_double_add(JtJ, JtJ, lJtJ);
      rox_array2d_double_add(Jte, Jte, lJte);

      Rox_Sint ellipse_sites_measures = edge_ellipse->sites->used;

      error = rox_edge_ellipse_get_valid_measures(&ellipse_sites_measures, edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );

      ptrErr += ellipse_sites_measures;
      ptrWeight += ellipse_sites_measures;
   }

function_terminate:
   rox_array2d_double_del(&lJtJ);
   rox_array2d_double_del(&lJte);
   return error;
}

Rox_ErrorCode rox_objset_edge_ellipse_get_measures (
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse,
   Rox_Tracking_Ellipse tracker,
   const Rox_MatSE3 pose,
   const Rox_MatUT3 calibration,
   const Rox_Image image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // const Rox_Bool log_ellipses_before = 0;


   if (!objset_edge_ellipse || !tracker || !pose || !calibration ||!image)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Uint id = 0; id < objset_edge_ellipse->used; id++)
   {
      // Get the edge_ellipse from the objset stored in odometry_ellipses
      Rox_Edge_Ellipse edge_ellipse = objset_edge_ellipse->data[id];

      // Transform the 3D ellipse in the camera frame and project it in the image
      error = rox_edge_ellipse_transform_project(edge_ellipse, pose, calibration);
      ROX_ERROR_CHECK_CONTINUE(error);

      // Sample the 2D ellipse in pixels
      error = rox_edge_ellipse_sample(edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Initialise tracking if needed
      error = rox_tracking_ellipse_initialize(tracker, image, edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // if (log_ellipses_before)
      // rox_odometry_ellipse_log_ellipses_before_odometry(edge_ellipse, id);

      error = rox_tracking_ellipse_make ( tracker, image, edge_ellipse );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Do not clean sites otherwise we loose count of bad states ???
      error = rox_edge_ellipse_clean(edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = ROX_ERROR_NONE;

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_ellipse_get_score(Rox_Double * score, const Rox_ObjSet_Edge_Ellipse objset_edge_ellipse, const Rox_Double score_max)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!score || !objset_edge_ellipse)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   Rox_Sint nbe = objset_edge_ellipse->used;

   Rox_Double local_score = 0;

   if (nbe == 0)
   { goto function_terminate; }

   for (Rox_Uint id = 0; id < objset_edge_ellipse->used; id++)
   {
      Rox_Edge_Ellipse edge_ellipse = objset_edge_ellipse->data[id];

      Rox_Sint inliers = 0;
      Rox_Sint outliers = 0;

      Rox_Double score_ellipse = 0.0;
      Rox_Sint nbp = edge_ellipse->sites->used;

      // For each ellipse lopp over sites
      for ( Rox_Sint idsite = 0; idsite < nbp; idsite++)
      {
         Rox_Edge_Ellipse_Site_Struct * site = &edge_ellipse->sites->data[idsite];
         if (site->state)
         {
            outliers++;
            continue;
         }
         else
         {
            inliers++;
         }

         Rox_Double signed_distance = 0.0;
         // Signed distance in pixels coordinates
         error = rox_signed_distance_point2d_to_ellipse2d_algebraic(&signed_distance, &edge_ellipse->ellipse2d_pixels, &site->coords);
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Double distance = fabs(signed_distance);
         score_ellipse += distance;
      }

      if ((inliers>0) && (inliers > outliers/2+1))
      {
         local_score += score_ellipse/inliers;
      }
      else
      {
         Rox_Double score_max = 30.0; // score max to be set elsewhere
         local_score += score_max;
      }
   }

   local_score = local_score /nbe;

   *score = local_score;

function_terminate:
   return error;
}
