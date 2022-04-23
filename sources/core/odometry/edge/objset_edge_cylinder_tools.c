//==============================================================================
//
//    OPENROX   : File objset_edge_cylinder_tools.c
//
//    Contents  : Implementation of module odometry cylinders
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "objset_edge_cylinder_tools.h"

#include <float.h>
#include <generated/dynvec_edge_cylinder_site_struct.h>

#include <baseproc/geometry/point/point2d_tools.h>
#include <baseproc/geometry/line/line2d.h>
#include <baseproc/geometry/line/line2d_struct.h>
#include <baseproc/geometry/line/line3d_struct.h>
#include <baseproc/geometry/line/line_from_points.h>
#include <baseproc/geometry/segment/segment2d.h>
#include <baseproc/geometry/segment/segment2d_struct.h>
#include <baseproc/geometry/cylinder/cylinder2d_struct.h>
#include <baseproc/geometry/cylinder/cylinder3d_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/robust/huber.h>
#include <baseproc/array/inverse/svdinverse.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_objset_edge_cylinder_add_cylinder3d (
   Rox_ObjSet_Edge_Cylinder objset_edge_cylinder,
   Rox_Cylinder3D cylinder3d,
   const Rox_Double sampling_step
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Edge_Cylinder toadd = NULL;


   if (!objset_edge_cylinder)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_edge_cylinder_new(&toadd, (Rox_Sint) sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_edge_cylinder_set_cylinder3d(toadd, cylinder3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_edge_cylinder_append(objset_edge_cylinder, toadd);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (error) rox_edge_cylinder_del(&toadd);
   return error;
}

Rox_ErrorCode rox_edge_cylinder_build_error (
   Rox_Double ** res_error,
   const Rox_MatUT3 K,
   const Rox_Edge_Cylinder edge_cylinder
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!res_error || !K || !edge_cylinder)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // TODO
   Rox_Double * ptrRes = NULL;

   // Get the line parameters

   // Segment 1
   Rox_Line2D_Normal_Struct line2d_1_struct;

   error = rox_segment2d_get_line2d_normal(&line2d_1_struct, edge_cylinder->cylinder2d_meters->s1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ptrRes = *res_error;

   if (!ptrRes)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint used_cylinder_sites_segment_1 = edge_cylinder->sites_segment_1->used;

   for ( Rox_Sint idsite = 0; idsite < used_cylinder_sites_segment_1; idsite++)
   {
      Rox_Edge_Cylinder_Site_Struct * site = &edge_cylinder->sites_segment_1->data[idsite];
      if (site->state) continue;

      // Normalized coordinates of the point
      Rox_Point2D_Double_Struct point2d_meters;


      error = rox_point2d_convert_pixel_double_to_meter_double ( &point2d_meters, &(site->coords), K );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Signed distance in normalized coordinates
      error = rox_line2d_normal_signed_distance(ptrRes, &line2d_1_struct, &point2d_meters);
      ROX_ERROR_CHECK_TERMINATE ( error );

      ptrRes++;
   }

   // Segment 2
   Rox_Line2D_Normal_Struct line2d_2_struct;

   error = rox_segment2d_get_line2d_normal(&line2d_2_struct, edge_cylinder->cylinder2d_meters->s2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // ptrRes = *res_error;

   if (!ptrRes)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint used_cylinder_sites_segment_2 = edge_cylinder->sites_segment_2->used;

   for ( Rox_Sint idsite = 0; idsite < used_cylinder_sites_segment_2; idsite++)
   {
      Rox_Edge_Cylinder_Site_Struct * site = &edge_cylinder->sites_segment_2->data[idsite];
      if (site->state) continue;

      // Normalized coordinates of the point
      Rox_Point2D_Double_Struct point2d_meters;


      error = rox_point2d_convert_pixel_double_to_meter_double ( &point2d_meters, &(site->coords), K );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Signed distance in normalized coordinates
      error = rox_line2d_normal_signed_distance(ptrRes, &line2d_2_struct, &point2d_meters);
      ROX_ERROR_CHECK_TERMINATE ( error );

      ptrRes++;
   }

   *res_error = ptrRes;

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_cylinder_build_interaction_matrix(Rox_Array2D_Double L, Rox_Array2D_Double K, Rox_Edge_Cylinder edge_cylinder)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double fu, fv, cu, cv;
   Rox_Double ifu, ifv, icu, icv;
   Rox_Double x, y, rho, theta, sinth, costh, alpha;
   Rox_Double ltheta, lrho;
   Rox_Double A,B,C,D;
   Rox_Double H[2][6];
   Rox_Edge_Cylinder_Site_Struct * site;

   if (!L || !K || !edge_cylinder)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dL, L);
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_array2d_double_get_value(&fu, K, 0, 0);
   rox_array2d_double_get_value(&fv, K, 1, 1);
   rox_array2d_double_get_value(&cu, K, 0, 2);
   rox_array2d_double_get_value(&cv, K, 1, 2);

   ifu = 1.0 / fu;
   ifv = 1.0 / fv;
   icu = - cu / fu;
   icv = - cv / fv;

   // Segment 1
   Rox_Line2D_Normal_Struct line2d_1_struct;

   error = rox_segment2d_get_line2d_normal(&line2d_1_struct, edge_cylinder->cylinder2d_meters->s1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the 2D line parameters in meters
   rho   = line2d_1_struct.rho;
   theta = line2d_1_struct.theta;

   sinth = sin(theta);
   costh = cos(theta);

   // Get the 3D line parameters in meters
   Rox_Line3D_Planes_Struct line3d_1_struct;

   error = rox_line3d_planes_from_segment3d(&line3d_1_struct, &edge_cylinder->tangent_segment3d_1);

   ROX_ERROR_CHECK_TERMINATE ( error );

   A = line3d_1_struct.planes[0].a;
   B = line3d_1_struct.planes[0].b;
   C = line3d_1_struct.planes[0].c;
   D = line3d_1_struct.planes[0].d;

   if (fabs(D) < DBL_EPSILON)
   {
      A = line3d_1_struct.planes[1].a;
      B = line3d_1_struct.planes[1].b;
      C = line3d_1_struct.planes[1].c;
      D = line3d_1_struct.planes[1].d;
   }

   if (fabs(D) < DBL_EPSILON)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; goto function_terminate; }

   ltheta = (A * sinth - B * costh) / D;
   lrho = (C + rho * A * costh + rho * B * sinth) / D;

   H[0][0] = costh * lrho;
   H[0][1] = sinth * lrho;
   H[0][2] = - rho * lrho;
   H[0][3] = sinth * (1.0 + rho * rho);
   H[0][4] = - costh * (1.0 + rho * rho);
   H[0][5] = 0.0;
   H[1][0] = costh * ltheta;
   H[1][1] = sinth * ltheta;
   H[1][2] = - rho * ltheta;
   H[1][3] = - rho * costh;
   H[1][4] = - rho * sinth;
   H[1][5] = - 1.0;

   Rox_Sint used_cylinder_sites_segment_1 = edge_cylinder->sites_segment_1->used;
   Rox_Sint cont = 0;

   for (Rox_Sint idsite = 0; idsite < used_cylinder_sites_segment_1; idsite++)
   {
      site = &edge_cylinder->sites_segment_1->data[idsite];
      if (site->state) continue;

      x = ifu * site->coords.u + icu;
      y = ifv * site->coords.v + icv;

      alpha = x * sinth - y * costh;

      for (Rox_Sint col = 0; col < 6; col++)
      {
         dL[cont][col] = (H[0][col] + alpha * H[1][col]);
      }
      cont++;
   }

   // Segment 2
   Rox_Line2D_Normal_Struct line2d_2_struct;

   error = rox_segment2d_get_line2d_normal(&line2d_2_struct, edge_cylinder->cylinder2d_meters->s2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the 2D line parameters in meters
   rho   = line2d_2_struct.rho;
   theta = line2d_2_struct.theta;

   sinth = sin(theta);
   costh = cos(theta);

   // Get the 3D line parameters in meters

   Rox_Line3D_Planes_Struct line3d_2_struct;

   error = rox_line3d_planes_from_segment3d(&line3d_2_struct, &edge_cylinder->tangent_segment3d_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   A = line3d_2_struct.planes[0].a;
   B = line3d_2_struct.planes[0].b;
   C = line3d_2_struct.planes[0].c;
   D = line3d_2_struct.planes[0].d;

   if (fabs(D) < DBL_EPSILON)
   {
      A = line3d_2_struct.planes[1].a;
      B = line3d_2_struct.planes[1].b;
      C = line3d_2_struct.planes[1].c;
      D = line3d_2_struct.planes[1].d;
   }

   if (fabs(D) < DBL_EPSILON)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; goto function_terminate; }

   ltheta = (A * sinth - B * costh) / D;
   lrho = (C + rho * A * costh + rho * B * sinth) / D;

   H[0][0] = costh * lrho;
   H[0][1] = sinth * lrho;
   H[0][2] = - rho * lrho;
   H[0][3] = sinth * (1.0 + rho * rho);
   H[0][4] = - costh * (1.0 + rho * rho);
   H[0][5] = 0.0;
   H[1][0] = costh * ltheta;
   H[1][1] = sinth * ltheta;
   H[1][2] = - rho * ltheta;
   H[1][3] = - rho * costh;
   H[1][4] = - rho * sinth;
   H[1][5] = - 1.0;

   Rox_Sint used_cylinder_sites_segment_2 = edge_cylinder->sites_segment_2->used;

   for (Rox_Sint idsite = 0; idsite < used_cylinder_sites_segment_2; idsite++)
   {
      site = &edge_cylinder->sites_segment_2->data[idsite];
      if (site->state) continue;

      x = ifu * site->coords.u + icu;
      y = ifv * site->coords.v + icv;

      alpha = x * sinth - y * costh;

      for (Rox_Sint col = 0; col < 6; col++)
      {
         dL[cont][col] = (H[0][col] + alpha * H[1][col]);
      }
      cont++;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_cylinder_build_linear_system(Rox_Array2D_Double LtL, Rox_Array2D_Double Lte, Rox_Array2D_Double K, Rox_Edge_Cylinder edge_cylinder, Rox_Double * ptrError, Rox_Double * ptrWeight)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint col, idsite, i, j;
   Rox_Double fu, fv, cu, cv;
   Rox_Double ifu, ifv, icu, icv;
   Rox_Double x, y, rho, theta, sinth, costh, alpha;
   Rox_Double ltheta, lrho;
   Rox_Double A,B,C,D;
   Rox_Double H[2][6];
   Rox_Double L[6];
   Rox_Edge_Cylinder_Site_Struct * site;

   Rox_Double w, e;

   if (!LtL || !Lte || !K || !edge_cylinder || !ptrError || !ptrWeight)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }


   error = rox_array2d_double_check_size(LtL, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** djtj = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &djtj, LtL);
   Rox_Double * djte = NULL;
   error = rox_array2d_double_get_data_pointer( &djte, Lte );

   rox_array2d_double_get_value(&fu, K, 0, 0);
   rox_array2d_double_get_value(&fv, K, 1, 1);
   rox_array2d_double_get_value(&cu, K, 0, 2);
   rox_array2d_double_get_value(&cv, K, 1, 2);

   error = rox_array2d_double_fillval(LtL, 0);

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Lte, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ifu = 1.0 / fu;
   ifv = 1.0 / fv;
   icu = - cu / fu;
   icv = - cv / fv;

   Rox_Line2D_Normal_Struct line2d_1_struct;

   error = rox_segment2d_get_line2d_normal(&line2d_1_struct, edge_cylinder->cylinder2d_meters->s1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the 2D line parameters in meters
   rho   = line2d_1_struct.rho;
   theta = line2d_1_struct.theta;

   sinth = sin(theta);
   costh = cos(theta);

   // Get the 3D line parameters in meters

   Rox_Line3D_Planes_Struct line3d_1_struct;

   // TODO : implement th following function need to store the tangent segments
   error = rox_line3d_planes_from_segment3d(&line3d_1_struct, &edge_cylinder->tangent_segment3d_1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   A = line3d_1_struct.planes[0].a;
   B = line3d_1_struct.planes[0].b;
   C = line3d_1_struct.planes[0].c;
   D = line3d_1_struct.planes[0].d;

   if (fabs(D) < DBL_EPSILON)
   {
      A = line3d_1_struct.planes[1].a;
      B = line3d_1_struct.planes[1].b;
      C = line3d_1_struct.planes[1].c;
      D = line3d_1_struct.planes[1].d;
   }

   if (fabs(D) < DBL_EPSILON)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; goto function_terminate; }

   ltheta = (A * sinth - B * costh) / D;
   lrho = (C + rho * A * costh + rho * B * sinth) / D;

   H[0][0] = costh * lrho;
   H[0][1] = sinth * lrho;
   H[0][2] = - rho * lrho;
   H[0][3] = sinth * (1.0 + rho * rho);
   H[0][4] = - costh * (1.0 + rho * rho);
   H[0][5] = 0.0;
   H[1][0] = costh * ltheta;
   H[1][1] = sinth * ltheta;
   H[1][2] = - rho * ltheta;
   H[1][3] = - rho * costh;
   H[1][4] = - rho * sinth;
   H[1][5] = - 1.0;

   for (idsite = 0; idsite < edge_cylinder->sites_segment_1->used; idsite++)
   {
      site = &edge_cylinder->sites_segment_1->data[idsite];
      if (site->state) continue;

      w = *ptrWeight;

      e = *ptrError;
      e = e * w;

      x = ifu * site->coords.u + icu;
      y = ifv * site->coords.v + icv;

      alpha = x * sinth - y * costh;

      for (col = 0; col < 6; col++)
      {
         L[col] = w * (H[0][col] + alpha * H[1][col]);
      }

      for (i = 0; i < 6; i++)
      {
         for (j = 0; j < 6; j++)
         {
            djtj[i][j] += L[i] * L[j];
         }

         djte[i] += L[i] * e;
      }

      ptrError++;
      ptrWeight++;
   }

   // TODO

   Rox_Line2D_Normal_Struct line2d_2_struct;

   error = rox_segment2d_get_line2d_normal(&line2d_2_struct, edge_cylinder->cylinder2d_meters->s2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the 2D line parameters in meters
   rho   = line2d_2_struct.rho;
   theta = line2d_2_struct.theta;

   sinth = sin(theta);
   costh = cos(theta);

   // Get the 3D line parameters in meters

   Rox_Line3D_Planes_Struct line3d_2_struct;

   // TODO : implement teh following function need to store the tangent segments
   error = rox_line3d_planes_from_segment3d(&line3d_2_struct, &edge_cylinder->tangent_segment3d_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   A = line3d_2_struct.planes[0].a;
   B = line3d_2_struct.planes[0].b;
   C = line3d_2_struct.planes[0].c;
   D = line3d_2_struct.planes[0].d;

   if (fabs(D) < DBL_EPSILON)
   {
      A = line3d_2_struct.planes[1].a;
      B = line3d_2_struct.planes[1].b;
      C = line3d_2_struct.planes[1].c;
      D = line3d_2_struct.planes[1].d;
   }

   if (fabs(D) < DBL_EPSILON)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; goto function_terminate; }

   ltheta = (A * sinth - B * costh) / D;
   lrho = (C + rho * A * costh + rho * B * sinth) / D;

   H[0][0] = costh * lrho;
   H[0][1] = sinth * lrho;
   H[0][2] = - rho * lrho;
   H[0][3] = sinth * (1.0 + rho * rho);
   H[0][4] = - costh * (1.0 + rho * rho);
   H[0][5] = 0.0;
   H[1][0] = costh * ltheta;
   H[1][1] = sinth * ltheta;
   H[1][2] = - rho * ltheta;
   H[1][3] = - rho * costh;
   H[1][4] = - rho * sinth;
   H[1][5] = - 1.0;

   for (idsite = 0; idsite < edge_cylinder->sites_segment_2->used; idsite++)
   {
      site = &edge_cylinder->sites_segment_2->data[idsite];
      if (site->state) continue;

      w = *ptrWeight;

      e = *ptrError;
      e = e * w;

      x = ifu * site->coords.u + icu;
      y = ifv * site->coords.v + icv;

      alpha = x * sinth - y * costh;

      for (col = 0; col < 6; col++)
      {
         L[col] = w * (H[0][col] + alpha * H[1][col]);
      }

      for (i = 0; i < 6; i++)
      {
         for (j = 0; j < 6; j++)
         {
            djtj[i][j] += L[i] * L[j];
         }

         djte[i] += L[i] * e;
      }

      ptrError++;
      ptrWeight++;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_cylinder_get_valid_measures(Rox_Sint * valid_measures, Rox_ObjSet_Edge_Cylinder objset_edge_cylinder)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!objset_edge_cylinder || !valid_measures)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *valid_measures = 0;

   for (Rox_Uint id = 0; id < objset_edge_cylinder->used; id++)
   {
      Rox_Sint valid_measures_cylinder = 0;
      Rox_Edge_Cylinder edge_cylinder = objset_edge_cylinder->data[id];

      error = rox_edge_cylinder_get_valid_measures(&valid_measures_cylinder, edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );

      *valid_measures += valid_measures_cylinder;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_cylinder_transform_project(Rox_ObjSet_Edge_Cylinder objset_edge_cylinder, Rox_Array2D_Double K, Rox_MatSE3 cTo)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!objset_edge_cylinder || !K ||! cTo)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint id = 0; id < objset_edge_cylinder->used; id++)
   {
      Rox_Edge_Cylinder edge_cylinder = objset_edge_cylinder->data[id];

      error = rox_edge_cylinder_transform_project(edge_cylinder, cTo, K);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_cylinder_build_error(Rox_Double ** res_error, Rox_Array2D_Double K, Rox_ObjSet_Edge_Cylinder objset_edge_cylinder)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   for (Rox_Uint id = 0; id < objset_edge_cylinder->used; id++)
   {
      Rox_Edge_Cylinder edge_cylinder = objset_edge_cylinder->data[id];

      error = rox_edge_cylinder_build_error(res_error, K, edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_cylinder_build_interaction_matrix(Rox_Array2D_Double L, Rox_Array2D_Double K, Rox_ObjSet_Edge_Cylinder objset_edge_cylinder)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint initial_row = 0, initial_col = 0, rows = 0, cols = 6;
   Rox_Array2D_Double L_sub = NULL;

   for (Rox_Uint id = 0; id < objset_edge_cylinder->used; id++)
   {
      Rox_Edge_Cylinder edge_cylinder = objset_edge_cylinder->data[id];

      error = rox_edge_cylinder_get_valid_measures(&rows, edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (rows == 0)continue;
    
      // Create submatrix of proper size
      error = rox_array2d_double_new_subarray2d(&L_sub, L, initial_row, initial_col, rows, cols);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute contribution of cylinder id to linear system
      error = rox_edge_cylinder_build_interaction_matrix(L_sub, K, edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Add actual rows for next cylinder
      initial_row += rows;

      error = rox_array2d_double_del(&L_sub);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   if(error) rox_array2d_double_del(&L_sub);
   return error;
}

Rox_ErrorCode rox_objset_edge_cylinder_build_linear_system(Rox_Array2D_Double A, Rox_Array2D_Double b, Rox_Array2D_Double K, Rox_ObjSet_Edge_Cylinder objset_edge_cylinder, Rox_Double ** dverr, Rox_Double ** dvw)
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

   for (Rox_Uint id = 0; id < objset_edge_cylinder->used; id++)
   {
      // Compute contribution of ellipse id to linear system
      Rox_Edge_Cylinder edge_cylinder = objset_edge_cylinder->data[id];
      error = rox_edge_cylinder_build_linear_system(lJtJ, lJte, K, edge_cylinder, ptrErr, ptrWeight);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Add contribution of ellipse id to linear system
      rox_array2d_double_add(A, A, lJtJ);
      rox_array2d_double_add(b, b, lJte);

      Rox_Sint cylinder_sites_measures = edge_cylinder->sites_segment_1->used + edge_cylinder->sites_segment_2->used;

      error = rox_edge_cylinder_get_valid_measures(&cylinder_sites_measures, edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );

      ptrErr += cylinder_sites_measures;
      ptrWeight += cylinder_sites_measures;
   }

function_terminate:
   rox_array2d_double_del(&lJtJ);
   rox_array2d_double_del(&lJte);
   return error;
}

Rox_ErrorCode rox_objset_edge_cylinder_get_measures (
   Rox_ObjSet_Edge_Cylinder objset_edge_cylinder,
   Rox_Tracking_Cylinder tracker,
   Rox_MatSE3 pose,
   Rox_MatUT3 calibration,
   Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // const Rox_Bool log_cylinders_before = 0;

   if (!objset_edge_cylinder || !tracker || !pose || !calibration ||!image)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Uint id = 0; id < objset_edge_cylinder->used; id++)
   {
      // Get the edge_cylinder from the objset stored in odometry_cylinders
      Rox_Edge_Cylinder edge_cylinder = objset_edge_cylinder->data[id];

      // Transform the 3D cylinder in the camera frame and project it in the image
      error = rox_edge_cylinder_transform_project(edge_cylinder, pose, calibration);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Sample the 2D cylinder in pixels
      error = rox_edge_cylinder_sample(edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Initialise tracking if needed
      error = rox_tracking_cylinder_initialize(tracker, image, edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // if (log_cylinders_before)
      // rox_odometry_cylinder_log_cylinders_before_odometry(edge_cylinder, id);

      error = rox_tracking_cylinder_make(tracker, image, edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Do not clean sites otherwise we loose count of bad states ???
      error = rox_edge_cylinder_clean(edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_edge_cylinder_get_score(Rox_Double * score, Rox_ObjSet_Edge_Cylinder objset_edge_cylinder, const Rox_Double score_max)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!score || !objset_edge_cylinder)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   Rox_Sint nbc = objset_edge_cylinder->used;

   Rox_Double local_score = 0;

   for ( Rox_Sint id = 0; id < nbc; id++)
   {
      Rox_Edge_Cylinder edge_cylinder = objset_edge_cylinder->data[id];

      Rox_Sint inliers = 0;
      Rox_Sint outliers = 0;

      Rox_Double score_cylinder = 0.0;

      Rox_Double distance = 0.0;

      // Segment 1
      Rox_Line2D_Normal_Struct line2d_1_struct;

      error = rox_segment2d_get_line2d_normal(&line2d_1_struct, edge_cylinder->cylinder2d_pixels->s1);

      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Sint used_cylinder_sites_segment_1 = edge_cylinder->sites_segment_1->used;

      for ( Rox_Sint idsite = 0; idsite < used_cylinder_sites_segment_1; idsite++)
      {
         Rox_Edge_Cylinder_Site_Struct * site = &edge_cylinder->sites_segment_1->data[idsite];

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
         error = rox_line2d_normal_signed_distance(&signed_distance, &line2d_1_struct, &site->coords);
         ROX_ERROR_CHECK_TERMINATE ( error );

         distance = fabs(signed_distance);

         score_cylinder += distance;
      }

      // Segment 2
      Rox_Line2D_Normal_Struct line2d_2_struct;

      error = rox_segment2d_get_line2d_normal(&line2d_2_struct, edge_cylinder->cylinder2d_pixels->s2);

      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Sint used_cylinder_sites_segment_2 = edge_cylinder->sites_segment_2->used;

      for ( Rox_Sint idsite = 0; idsite < used_cylinder_sites_segment_2; idsite++)
      {
         Rox_Edge_Cylinder_Site_Struct * site = &edge_cylinder->sites_segment_2->data[idsite];
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
         // Signed distance in normalized coordinates
         error = rox_line2d_normal_signed_distance(&signed_distance, &line2d_2_struct, &site->coords);

         ROX_ERROR_CHECK_TERMINATE ( error );

         distance = fabs(signed_distance);

         score_cylinder += distance;
      }

      Rox_Sint nbp = used_cylinder_sites_segment_1 + used_cylinder_sites_segment_2;

      // rox_log("score ellipse %d : %f \n ", id, score_ellipse/inliers);
      if ((inliers > 0) && (inliers > nbp/2+1))
      {
         local_score += score_cylinder/inliers;
      }
      else
      {
         local_score += score_max;
      }

      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (nbc > 0)
   {
      local_score = local_score/nbc;
   }
   else
   {
      local_score = score_max;
   }

   *score = local_score;

function_terminate:
   return error;
}
