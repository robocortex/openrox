//==============================================================================
//
//    OPENROX   : File search_edge.c
//
//    Contents  : Implementation of search edge module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "search_edge.h"
#include <math.h>

#include <generated/array2d_point2d_double.h>
#include <generated/array2d_double.h>

#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point2d_uint_struct.h>
#include <generated/dynvec_double_struct.h>

#include <system/time/timer.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/base/basemaths.h>
#include <baseproc/image/gradient/gradientsobel.h>
#include <baseproc/image/gradient/gradient_anglenorm.h>
#include <baseproc/array/maxima/maxima.h>

#include <inout//numeric/array2d_save.h>
#include <inout/geometry/point/array2d_point2d_print.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_search_edge_new (
   Rox_Search_Edge * search_edge,
   const Rox_Uint search_range,
   const Rox_Uint norm_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Search_Edge ret = NULL;


   if (!search_edge)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *search_edge = NULL;

   ret = (Rox_Search_Edge) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->_convolution = 0;
   ret->_mask_sign = 1;
   // ret->_sites = NULL;
   // ret->_likelihoods = NULL;
   ret->_search_range = search_range;
   ret->_scale_threshold = norm_threshold * norm_threshold; // scale is norm^2

   //error = rox_dynvec_point2d_double_new(&ret->_sites, ret->_search_range * 2 + 2); // should be +1 not + 2, very strange
   //ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_dynvec_double_new(&ret->_likelihoods, ret->_search_range * 2 + 2); // should be +1 not + 2, very strange
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // // The first  row of the array will be used to store the coordinates u and v for each point on the scanline
   // // The second row of the array will be used to store the scale and angle of the gradient for each point on the scanline
   // error = rox_array2d_point2d_double_new(&ret->_convolutions, ret->_search_range * 2 + 1, 2);

   // ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate 2 arrays of point2d Double. Question : can be Float ?
   // The array will be used to store the coordinates u and v for each point on the scanline
   error = rox_array2d_point2d_double_new ( &ret->_scanline, 1, ret->_search_range * 2 + 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &ret->_scanline_u_row, 1, ret->_search_range * 2 + 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &ret->_scanline_v_row, 1, ret->_search_range * 2 + 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // The scale and angle of the gradient for each point on the scanline
   error = rox_array2d_double_new(&ret->_gradient_scale, 1, ret->_search_range * 2 + 1);

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->_gradient_angle, 1, ret->_search_range * 2 + 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *search_edge = ret;

function_terminate:
   if (error) rox_search_edge_del(&ret);
   return error;
}

Rox_ErrorCode rox_search_edge_del(Rox_Search_Edge * search_edge)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Search_Edge todel = NULL;


   if (!search_edge)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *search_edge;
   *search_edge = NULL;


   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // rox_dynvec_double_del(&todel->_likelihoods);
   // rox_dynvec_point2d_double_del(&todel->_sites);
   rox_array2d_point2d_double_del(&todel->_scanline);
   rox_array2d_double_del(&todel->_scanline_u_row);
   rox_array2d_double_del(&todel->_scanline_v_row);
   rox_array2d_double_del(&todel->_gradient_scale);
   rox_array2d_double_del(&todel->_gradient_angle);

   rox_memory_delete(todel);

function_terminate:
    return error;
}

Rox_ErrorCode rox_search_edge_track (
   Rox_Search_Edge search_edge,
   const Rox_Image image,
   const Rox_Point2D_Double point // The starting point of the search
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Double angle_range = 20 * ROX_PI / 180;

   // There is a difference of pi between Matlab and current implementation
   Rox_Double angle_model = search_edge->_angle;

   // // Define timer to measure performances
   // Rox_Timer timer = NULL;
   // Rox_Double time = 0.0;

   // // Init new timer
   // error = rox_timer_new(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // Check parameters
   if (!search_edge || !image || !point)

   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Compute the normal to the segment
   Rox_Point2D_Double_Struct normal;
   normal.u = cos(angle_model);
   normal.v = sin(angle_model);

   // Compute the scanline from the starting point along the normal direction
   // The size of the scanline is defined into _scanline itself

   // rox_timer_start(timer);

   error = rox_scanline ( search_edge->_scanline, point, &normal );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);

   // rox_log("time to compute a scanline = %f (ms)\n", time);

   // rox_timer_start(timer);

   // Compute scale and angle of image gradient for each point of the
   error = rox_scan_image_scale_angle ( search_edge->_gradient_scale, search_edge->_gradient_angle, search_edge->_scanline, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);

   // rox_log("time to scan scale angle = %f (ms)\n", time);

   //error = rox_find_closest_scale_above_threshold_angle_isinrange (&search_edge->_coords, search_edge->_scanline, search_edge->_gradient_scale, search_edge->_gradient_angle, search_edge->_search_range, angle_model, angle_range);
   //ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_timer_start(timer);

   error = rox_find_closest_scale_peak_above_threshold_angle_isinrange ( &search_edge->_coords, search_edge->_scanline, search_edge->_gradient_scale, search_edge->_gradient_angle, search_edge->_scale_threshold, angle_model, angle_range);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);

   // rox_log("time to find_closest_scale_peak = %f (ms)\n", time);

function_terminate:
   return error;
}

Rox_ErrorCode rox_search_edge_track_gradient (
   Rox_Search_Edge search_edge,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle,
   const Rox_Point2D_Double point // The starting point of the search
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Double angle_range = 20 * ROX_PI / 180;

   // There is a difference of pi between Matlab and current implementation
   Rox_Double angle_model = search_edge->_angle;

   // Define timer to measure performances
   // Rox_Timer timer = NULL;
   // Rox_Double time = 0.0;

   // Init new timer
   // error = rox_timer_new(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // Check parameters
   if ( !search_edge || !gradient_angle || !gradient_scale || !point )

   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Compute the normal to the segment
   Rox_Point2D_Double_Struct normal;
   normal.u = cos(angle_model);
   normal.v = sin(angle_model);

   // Compute the scanline from the starting point along the normal direction
   // The size of the scanline is defined into _scanline itself

   // rox_timer_start(timer);

   //error = rox_scanline ( search_edge->_scanline, point, &normal );
   //ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_array_double_save_append ( "./result_theta.txt", &angle_model, 1 );
   // rox_point2d_double_vector_save_append ( "./result_points2d_mod_pix.txt", point, 1 );
   // rox_array_double_save_append ( "./result_points2d_u_mod_pix", &point->u, 1 );
   // rox_array_double_save_append ( "./result_points2d_v_mod_pix.txt", &point->v, 1 );

   error = rox_scanline_row ( search_edge->_scanline_u_row, search_edge->_scanline_v_row, point, &normal ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);

   // rox_log("time to compute a scanline = %f (ms)\n", time);

   // rox_timer_start(timer);

   // Compute scale and angle of image gradient for each point of the
   //error = rox_scan_scale_angle ( search_edge->_gradient_scale, search_edge->_gradient_angle, search_edge->_scanline, gradient_scale, gradient_angle );
   //ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_scan_scale_angle_matrix ( search_edge->_gradient_scale, search_edge->_gradient_angle, search_edge->_scanline_u_row, search_edge->_scanline_v_row, gradient_scale, gradient_angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_array2d_double_save_append ( "./result_scan_gs.txt", search_edge->_gradient_scale );
   // rox_array2d_double_save_append ( "./result_scan_ga.txt", search_edge->_gradient_angle );


   // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);

   // rox_log("time to scan scale angle = %f (ms)\n", time);

   //error = rox_find_closest_scale_above_threshold_angle_isinrange (&search_edge->_coords, search_edge->_scanline, search_edge->_gradient_scale, search_edge->_gradient_angle, search_edge->_search_range, angle_model, angle_range);
   //ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_timer_start(timer);

   //error = rox_find_closest_scale_peak_above_threshold_angle_isinrange ( &search_edge->_coords, search_edge->_scanline, search_edge->_gradient_scale, search_edge->_gradient_angle, search_edge->_scale_threshold, angle_model, angle_range);
   //ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_find_closest_scale_peak_above_threshold_angle_isinrange_row ( &search_edge->_coords, search_edge->_scanline_u_row, search_edge->_scanline_v_row, search_edge->_gradient_scale, search_edge->_gradient_angle, search_edge->_scale_threshold, angle_model, angle_range );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);

   // rox_log("time to find_closest_scale_peak = %f (ms)\n", time);

function_terminate:
   return error;
}

Rox_ErrorCode rox_scanline (
   Rox_Array2D_Point2D_Double scanline,
   Rox_Point2D_Double point,
   Rox_Point2D_Double normal
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint scanline_size = 0;

   error = rox_array2d_point2d_double_get_cols ( &scanline_size, scanline );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint search_range = (scanline_size-1)/2;

   // Scan scale and angle and store results in scanline
   for (Rox_Sint pos = -search_range; pos <= search_range; pos++)
   {
      Rox_Point2D_Double_Struct poit_cur, point_tmp;

      poit_cur.u = point->u + ((double)pos) * normal->u;
      poit_cur.v = point->v + ((double)pos) * normal->v;

      // Not the best scanline but ok for a first implementation
      // The problem is that there are duplicated pixels
      // See the matlab code for a more precise scanline

      Rox_Sint u = (int) fabs(poit_cur.u + 0.5);
      Rox_Sint v = (int) fabs(poit_cur.v + 0.5);

      // Store results
      point_tmp.u = (double) u;
      point_tmp.v = (double) v;

      error = rox_array2d_point2d_double_set_value ( scanline, 0, pos + search_range, point_tmp );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

  function_terminate:
  return error;
}

Rox_ErrorCode rox_scanline_matrix ( 
   Rox_Array2D_Double scanline_u, 
   Rox_Array2D_Double scanline_v, 
   Rox_Array2D_Double points2d, 
   Rox_Array2D_Double lines2d 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint scanline_size = 0, nbp = 0;
   error = rox_array2d_double_get_size ( &nbp, &scanline_size, scanline_u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** pu = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &pu, scanline_u );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** pv = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &pv, scanline_v );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** points2d_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &points2d_data, points2d );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** lines2d_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &lines2d_data, lines2d );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint search_range = (scanline_size-1)/2;
   // Scan scale and angle and store results in scanline
   for (Rox_Sint i=0;i<nbp;i++)
   {
      Rox_Double theta = lines2d_data[i][1];
      Rox_Double ct = cos(theta);
      Rox_Double st = sin(theta);
      
      for (Rox_Sint pos = -search_range; pos <= search_range; pos++)
      {
         Rox_Point2D_Double_Struct point_cur;

         point_cur.u = points2d_data[i][0] + ((double) pos) * ct;
         point_cur.v = points2d_data[i][1] + ((double) pos) * st;

         // Not the best scanline but ok for a first implementation
         // The problem is that there are duplicated pixels
         // See the matlab code for a more precise scanline

         Rox_Sint u = (int) fabs(point_cur.u + 0.5);
         Rox_Sint v = (int) fabs(point_cur.v + 0.5);

         // Store results
         pu[i][pos + search_range] = (double) u;
         pv[i][pos + search_range] = (double) v;
      }
   }

   function_terminate:
   return error;
}

Rox_ErrorCode rox_scanline_row (
   Rox_Array2D_Double scanline_u,
   Rox_Array2D_Double scanline_v,
   Rox_Point2D_Double point,
   Rox_Point2D_Double normal
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint scanline_size = 0;
   error = rox_array2d_double_get_cols ( &scanline_size, scanline_u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * pu = NULL;
   error = rox_array2d_double_get_data_pointer ( &pu, scanline_u );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double * pv = NULL;
   error = rox_array2d_double_get_data_pointer ( &pv, scanline_v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint search_range = (scanline_size-1)/2;
   // Scan scale and angle and store results in scanline
   for (Rox_Sint pos = -search_range; pos <= search_range; pos++)
   {
      Rox_Point2D_Double_Struct poit_cur;

      poit_cur.u = point->u + ((double) pos) * normal->u;
      poit_cur.v = point->v + ((double) pos) * normal->v;

      // Not the best scanline but ok for a first implementation
      // The problem is that there are duplicated pixels
      // See the matlab code for a more precise scanline

      Rox_Sint u = (int) fabs(poit_cur.u + 0.5);
      Rox_Sint v = (int) fabs(poit_cur.v + 0.5);

      // Store results
      pu[pos + search_range] = (double) u;
      pv[pos + search_range] = (double) v;
   }

  function_terminate:
  return error;
}

Rox_ErrorCode rox_scan_image_scale_angle (
   Rox_Array2D_Double scan_gradient_scale,
   Rox_Array2D_Double scan_gradient_angle,
   const Rox_Array2D_Point2D_Double scanline,
   Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint scanline_size = 0;
   error = rox_array2d_point2d_double_get_cols ( &scanline_size, scanline );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * scan_gradient_scale_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &scan_gradient_scale_data, scan_gradient_scale );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * scan_gradient_angle_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &scan_gradient_angle_data, scan_gradient_angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Scan scale and angle and store results in search_edge->_convolutions
   for (Rox_Sint k = 0; k < scanline_size; k++)
   {
      Rox_Point2D_Sshort_Struct image_gradient;
      Rox_Double gradient_scale_tmp = 0.0, gradient_angle_tmp = 0.0;

      Rox_Point2D_Double_Struct point_cur;

      error = rox_array2d_point2d_double_get_value(&point_cur, scanline, 0, k);
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Sint u = (int) point_cur.u;
      Rox_Sint v = (int) point_cur.v;

      // Test if the scanline point is inside the image
      if ((u < 1 || u >= cols - 1) || (v < 1 || v >= rows - 1))
      {
         gradient_scale_tmp = 0.0;
         gradient_angle_tmp = 0.0;
      }
      else
      {
         // Compute gradient along u and v axes
         error = rox_array2d_uchar_gradientsobel_singlepoint_nomask ( &image_gradient, image, u, v );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_point2d_sshort_angle_scale ( &gradient_scale_tmp, &gradient_angle_tmp, &image_gradient );
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      scan_gradient_scale_data[k] = gradient_scale_tmp;
      scan_gradient_angle_data[k] = gradient_angle_tmp;
   }

  function_terminate:
  return error;
}

Rox_ErrorCode rox_scan_scale_angle (
   Rox_Array2D_Double scan_gradient_scale,
   Rox_Array2D_Double scan_gradient_angle,
   const Rox_Array2D_Point2D_Double scanline,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint scanline_size = 0;
   error = rox_array2d_point2d_double_get_cols ( &scanline_size, scanline );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, gradient_angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** gradient_scale_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &gradient_scale_data, gradient_scale );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** gradient_angle_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &gradient_angle_data, gradient_angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * scan_gradient_scale_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &scan_gradient_scale_data, scan_gradient_scale );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * scan_gradient_angle_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &scan_gradient_angle_data, scan_gradient_angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Scan scale and angle and store results in search_edge->_convolutions
   for (Rox_Sint k = 0; k < scanline_size; k++)
   {
      Rox_Float gradient_scale_tmp = 0.0, gradient_angle_tmp = 0.0;

      Rox_Point2D_Double_Struct point_cur;

      error = rox_array2d_point2d_double_get_value ( &point_cur, scanline, 0, k);
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Sint u = (Rox_Sint) point_cur.u;
      Rox_Sint v = (Rox_Sint) point_cur.v;

      // Test if the scanline point is inside the image
      if ((u < 1 || u >= cols - 1) || (v < 1 || v >= rows - 1))
      {
         gradient_scale_tmp = 0.0;
         gradient_angle_tmp = 0.0;
      }
      else
      {
         // Retrieve gradient scale and angle along u and v axes
         gradient_scale_tmp = (Rox_Float) gradient_scale_data[v][u];
         gradient_angle_tmp = (Rox_Float) gradient_angle_data[v][u];
      }

      scan_gradient_scale_data[k] = gradient_scale_tmp;
      scan_gradient_angle_data[k] = gradient_angle_tmp;
   }

function_terminate:
  return error;
}

Rox_ErrorCode rox_scan_scale_angle_matrix (
   Rox_Array2D_Double scan_gradient_scale,
   Rox_Array2D_Double scan_gradient_angle,
   const Rox_Array2D_Double scanline_u,
   const Rox_Array2D_Double scanline_v,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint scanline_size = 0, nbp = 0;
   error = rox_array2d_double_get_size ( &nbp, &scanline_size, scanline_u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, gradient_angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** gradient_scale_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &gradient_scale_data, gradient_scale );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** gradient_angle_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &gradient_angle_data, gradient_angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** scan_gradient_scale_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &scan_gradient_scale_data, scan_gradient_scale );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** scan_gradient_angle_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &scan_gradient_angle_data, scan_gradient_angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** pu = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &pu, scanline_u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** pv = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &pv, scanline_v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Scan scale and angle and store results in search_edge->_convolutions
   for (Rox_Sint i = 0; i < nbp; i++)
   {
      for (Rox_Sint k = 0; k < scanline_size; k++)
      {
         Rox_Float gradient_scale_tmp = 0.0;
         Rox_Float gradient_angle_tmp = 0.0;
    
         Rox_Sint ui = (Rox_Sint) pu[i][k];
         Rox_Sint vi = (Rox_Sint) pv[i][k];

         // Test if the scanline point is inside the image
         if ((ui < 1 || ui >= cols - 1) || (vi < 1 || vi >= rows - 1))
         {
            gradient_scale_tmp = 0.0;
            gradient_angle_tmp = 0.0;
         }
         else
         {
            // Retrieve gradient scale and angle along u and v axes
            gradient_scale_tmp = (Rox_Double) gradient_scale_data[vi][ui];
            gradient_angle_tmp = (Rox_Double) gradient_angle_data[vi][ui];
        }

         scan_gradient_scale_data[i][k] = gradient_scale_tmp;
         scan_gradient_angle_data[i][k] = gradient_angle_tmp;
      }
   }

   function_terminate:
   return error;
}

Rox_ErrorCode rox_scan_scale (
   Rox_Array2D_Double gradient_scale,
   Rox_Array2D_Point2D_Double scanline,
   Rox_Image image,
   Rox_Double angle_model,
   Rox_Double angle_range
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint scanline_size = 0;
   error = rox_array2d_point2d_double_get_cols(&scanline_size, scanline);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Scan scale and angle and store results in search_edge->_convolutions
   for (Rox_Sint k = 0; k < scanline_size; k++)
   {
      Rox_Point2D_Sshort_Struct image_gradient;
      Rox_Double gradient_scale_tmp = 0.0, gradient_angle_tmp = 0.0;

      Rox_Point2D_Double_Struct point_cur;

      error = rox_array2d_point2d_double_get_value(&point_cur, scanline, 0, k);
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Sint u = (int) point_cur.u;
      Rox_Sint v = (int) point_cur.v;

      // Test if the scanline point is inside the image
      if ((u < 1 || u >= cols - 1) || (v < 1 || v >= rows - 1))
      {
         gradient_scale_tmp = 0.0;
         gradient_angle_tmp = 0.0;
      }
      else
      {
         // Compute gradient along u and v axes
         error = rox_array2d_uchar_gradientsobel_singlepoint_nomask(&image_gradient, image, u, v);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_point2d_sshort_angle_scale(&gradient_scale_tmp, &gradient_angle_tmp, &image_gradient);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      Rox_Sint inrange = 0;

      error = rox_angle_isinrange (&inrange, angle_model, gradient_angle_tmp, angle_range);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (inrange)
      {
         error = rox_array2d_double_set_value(gradient_scale, 0, k, gradient_scale_tmp);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         error = rox_array2d_double_set_value(gradient_scale, 0, k, 0.0);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

  function_terminate:
  return error;
}

Rox_ErrorCode rox_find_closest_scale_above_threshold_angle_isinrange (
   Rox_Point2D_Double point_meas,
   Rox_Array2D_Point2D_Double scanline,
   Rox_Array2D_Double gradient_scale,
   Rox_Array2D_Double gradient_angle,
   Rox_Double scale_threshold,
   Rox_Double angle_model,
   Rox_Double angle_range
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Double angle_threshold = 20 * ROX_PI / 180;

   // Estimate results
   Rox_Double max_scale = 0.0;

   Rox_Sint scanline_size = 0;

   error = rox_array2d_point2d_double_get_cols(&scanline_size, scanline);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint search_range = (scanline_size-1)/2;

   // convert theta to angle
   Rox_Double min_distance = search_range; // Distance from site

   Rox_Point2D_Double_Struct results1;
   Rox_Double scale, angle;

   for (Rox_Sint i = 0; i < search_range; i++)
   {
      Rox_Sint inrange = 0;

      Rox_Double distance = fabs((Rox_Double)i - search_range);

      error = rox_array2d_point2d_double_get_value(&results1, scanline, 0, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&scale, gradient_scale, 0, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&angle, gradient_angle, 0, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_angle_isinrange ( &inrange, angle_model, angle, angle_threshold );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ((distance < min_distance) && (scale > scale_threshold) && (inrange))
      {
         *point_meas = results1;
         min_distance = distance;
         max_scale = scale;
      }
   }

   for (Rox_Sint i = search_range; i < 2 * search_range + 1; i++)
   {
      Rox_Sint inrange = 0;

      Rox_Double distance = fabs((Rox_Double) i - search_range);

      error = rox_array2d_point2d_double_get_value(&results1, scanline, 0, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&scale, gradient_scale, 0, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&angle, gradient_angle, 0, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_angle_isinrange (&inrange, angle_model, angle, angle_threshold);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ((distance < min_distance) && (scale > scale_threshold) && (inrange))
      {
         *point_meas = results1;
         min_distance = distance;
         max_scale = scale;
      }
   }

   if (max_scale < scale_threshold)
   { error = ROX_ERROR_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

Rox_ErrorCode rox_find_closest_scale_above_threshold_angle_isinrange_matrix (
   Rox_Point2D_Double point_meas,
   Rox_Array2D_Double scanline_u,
   Rox_Array2D_Double scanline_v,
   Rox_Array2D_Double gradient_scale,
   Rox_Array2D_Double gradient_angle,
   Rox_Double scale_threshold,
   Rox_Double angle_model,
   Rox_Double angle_range
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Double angle_threshold = 20 * ROX_PI / 180;

   // Estimate results
   Rox_Double max_scale = 0.0;

   Rox_Sint scanline_size = 0;
   error = rox_array2d_double_get_cols ( &scanline_size, scanline_u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * pu = NULL;
   error = rox_array2d_double_get_data_pointer ( &pu, scanline_u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * pv = NULL;
   error = rox_array2d_double_get_data_pointer ( &pv, scanline_v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint search_range = (scanline_size-1)/2;

   // convert theta to angle
   Rox_Double min_distance = search_range; // Distance from site

   Rox_Point2D_Double_Struct results1;
   Rox_Double scale, angle;

   for (Rox_Sint i = 0; i < search_range; i++)
   {
      Rox_Sint inrange = 0;

      Rox_Double distance = fabs((Rox_Double)i - search_range);
      
      results1.u = pu[i]; 
      results1.v = pv[i]; 

      error = rox_array2d_double_get_value(&scale, gradient_scale, 0, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&angle, gradient_angle, 0, i);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_angle_isinrange (&inrange, angle_model, angle, angle_threshold);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ((distance < min_distance) && (scale > scale_threshold) && (inrange))
      {
         *point_meas = results1;
         min_distance = distance;
         max_scale = scale;
      }
   }

   for (Rox_Sint i = search_range; i < 2 * search_range + 1; i++)
   {
      Rox_Sint inrange = 0;

      Rox_Double distance = fabs((Rox_Double) i - search_range);
      
      results1.u = pu[i]; 
      results1.v = pv[i]; 

      error = rox_array2d_double_get_value(&scale, gradient_scale, 0, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&angle, gradient_angle, 0, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_angle_isinrange (&inrange, angle_model, angle, angle_threshold);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ((distance < min_distance) && (scale > scale_threshold) && (inrange))
      {
         *point_meas = results1;
         min_distance = distance;
         max_scale = scale;
      }
   }

   if (max_scale < scale_threshold)
   { error = ROX_ERROR_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

Rox_ErrorCode rox_extract_closest_peak_indexes (
   Rox_Point2D_Uint indexes_meas,
   Rox_Double site_coord,
   Rox_DynVec_Double local_maxima_values,
   Rox_DynVec_Double local_maxima_coords,
   Rox_DynVec_Point2D_Uint local_maxima_indexes
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double distance_min = INFINITY;

   Rox_Point2D_Uint_Struct * data = NULL;
   error = rox_dynvec_point2d_uint_get_data_pointer ( &data, local_maxima_indexes );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set default indexes to optimal coordinates
   indexes_meas->u = (Rox_Uint) site_coord;
   indexes_meas->v = (Rox_Uint) site_coord;

   for (Rox_Uint i=0; i<local_maxima_values->used; i++)
   {
      Rox_Double distance = 0.0;
      Rox_Double local_maxima_coord = 0.0;

      local_maxima_coord = local_maxima_coords->data[i];

      distance = fabs(site_coord - local_maxima_coord);

      if (distance < distance_min)
      {
         distance_min = distance;
         indexes_meas->u = data[i].u;
         indexes_meas->v = data[i].v;
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_find_closest_scale_peak_above_threshold_angle_isinrange (
   Rox_Point2D_Double point_meas,
   const Rox_Array2D_Point2D_Double scanline,
   const Rox_Array2D_Double gradient_scale,
   const Rox_Array2D_Double gradient_angle,
   const Rox_Double scale_threshold,
   const Rox_Double angle_model,
   const Rox_Double angle_range
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_DynVec_Double local_maxima_values = NULL;
   error = rox_dynvec_double_new(&local_maxima_values, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_DynVec_Double local_maxima_coords = NULL;
   error = rox_dynvec_double_new ( &local_maxima_coords, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_DynVec_Point2D_Uint local_maxima_indexes = NULL;
   error = rox_dynvec_point2d_uint_new ( &local_maxima_indexes, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_local_maxima ( local_maxima_values, local_maxima_coords, local_maxima_indexes, gradient_scale, scale_threshold);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_dynvec_double_print(local_maxima_values);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_dynvec_double_print(local_maxima_coords);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_dynvec_point2d_uint_print(local_maxima_indexes);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint scanline_size = 0;

   error = rox_array2d_point2d_double_get_cols ( &scanline_size, scanline);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double site_coord = ((double) scanline_size - 1.0) / 2.0;
   Rox_Point2D_Uint_Struct indexes_meas;

   error = rox_extract_closest_peak_indexes ( &indexes_meas, site_coord, local_maxima_values, local_maxima_coords, local_maxima_indexes );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Double_Struct point_inf;
   Rox_Point2D_Double_Struct point_sup;

   rox_array2d_point2d_double_get_value(&point_inf, scanline, 0, indexes_meas.u);
   rox_array2d_point2d_double_get_value(&point_sup, scanline, 0, indexes_meas.v);

   point_meas->u = (point_inf.u+point_sup.u)/2.0;
   point_meas->v = (point_inf.v+point_sup.v)/2.0;

function_terminate:

   rox_dynvec_double_del ( &local_maxima_values );
   rox_dynvec_double_del ( &local_maxima_coords );
   rox_dynvec_point2d_uint_del ( &local_maxima_indexes );

   return error;
}

// Same function but with scanline in row form
Rox_ErrorCode rox_find_closest_scale_peak_above_threshold_angle_isinrange_row (
   Rox_Point2D_Double point_meas,
   const Rox_Array2D_Double scanline_u,
   const Rox_Array2D_Double scanline_v,
   const Rox_Array2D_Double gradient_scale,
   const Rox_Array2D_Double gradient_angle,
   const Rox_Double scale_threshold,
   const Rox_Double angle_model,
   const Rox_Double angle_range
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_DynVec_Double local_maxima_values = NULL;
   error = rox_dynvec_double_new(&local_maxima_values, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_DynVec_Double local_maxima_coords = NULL;
   error = rox_dynvec_double_new ( &local_maxima_coords, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_DynVec_Point2D_Uint local_maxima_indexes = NULL;
   error = rox_dynvec_point2d_uint_new ( &local_maxima_indexes, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_local_maxima ( local_maxima_values, local_maxima_coords, local_maxima_indexes, gradient_scale, scale_threshold);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sint scanline_size = 0;
   error = rox_array2d_double_get_cols ( &scanline_size, scanline_u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * pu = NULL;
   error = rox_array2d_double_get_data_pointer ( &pu, scanline_u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * pv = NULL;
   error = rox_array2d_double_get_data_pointer ( &pv, scanline_v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double site_coord = ((double) scanline_size - 1.0) / 2.0;
   Rox_Point2D_Uint_Struct indexes_meas;

   error = rox_extract_closest_peak_indexes ( &indexes_meas, site_coord, local_maxima_values, local_maxima_coords, local_maxima_indexes );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Double_Struct point_inf;
   Rox_Point2D_Double_Struct point_sup;

   point_inf.u = pu[indexes_meas.u];
   point_inf.v = pv[indexes_meas.u];

   point_sup.u = pu[indexes_meas.v];
   point_sup.v = pv[indexes_meas.v];

   point_meas->u = (point_inf.u+point_sup.u)/2.0;
   point_meas->v = (point_inf.v+point_sup.v)/2.0;

function_terminate:

   rox_dynvec_double_del ( &local_maxima_values );
   rox_dynvec_double_del ( &local_maxima_coords );
   rox_dynvec_point2d_uint_del ( &local_maxima_indexes );

   return error;
}

// // Same function but with scanline in matrix form
// Rox_ErrorCode rox_find_closest_scale_peak_above_threshold_angle_isinrange_matrix (
//    Rox_Array2D_Double point_meas,
//    const Rox_Array2D_Double scanline_u,
//    const Rox_Array2D_Double scanline_v,
//    const Rox_Array2D_Double gradient_scale,
//    const Rox_Array2D_Double gradient_angle,
//    const Rox_Double angle_model,
//    const Rox_Double angle_range,
//    const Rox_Double scale_threshold
// )
// {
//    Rox_ErrorCode error = ROX_ERROR_NONE;

//    Rox_DynVec_Double local_maxima_values = NULL;
//    error = rox_dynvec_double_new(&local_maxima_values, 1);
//    ROX_ERROR_CHECK_TERMINATE ( error );

//    Rox_DynVec_Double local_maxima_coords = NULL;
//    error = rox_dynvec_double_new ( &local_maxima_coords, 1 );
//    ROX_ERROR_CHECK_TERMINATE ( error );

//    Rox_DynVec_Point2D_Uint local_maxima_indexes = NULL;
//    error = rox_dynvec_point2d_uint_new ( &local_maxima_indexes, 1 );
//    ROX_ERROR_CHECK_TERMINATE ( error );

//    error = rox_array2d_double_local_maxima ( local_maxima_values, local_maxima_coords, local_maxima_indexes, gradient_scale, scale_threshold);
//    ROX_ERROR_CHECK_TERMINATE ( error );

//    // error = rox_dynvec_double_print(local_maxima_values);
//    // ROX_ERROR_CHECK_TERMINATE ( error );

//    // error = rox_dynvec_double_print(local_maxima_coords);
//    // ROX_ERROR_CHECK_TERMINATE ( error );

//    // error = rox_dynvec_point2d_uint_print(local_maxima_indexes);
//    // ROX_ERROR_CHECK_TERMINATE ( error );

//    Rox_Sint scanline_size = 0;
//    error = rox_array2d_double_get_cols ( &scanline_size, scanline_u );
//    ROX_ERROR_CHECK_TERMINATE ( error );

//    Rox_Double * pu = NULL;
//    error = rox_array2d_double_get_data_pointer ( &pu, scanline_u );
//    ROX_ERROR_CHECK_TERMINATE ( error );

//    Rox_Double * pv = NULL;
//    error = rox_array2d_double_get_data_pointer ( &pv, scanline_v );
//    ROX_ERROR_CHECK_TERMINATE ( error );

//    Rox_Double site_coord = ((double) scanline_size - 1.0) / 2.0;
//    Rox_Point2D_Uint_Struct indexes_meas;

//    error = rox_extract_closest_peak_indexes ( &indexes_meas, site_coord, local_maxima_values, local_maxima_coords, local_maxima_indexes );
//    ROX_ERROR_CHECK_TERMINATE ( error );

//    Rox_Point2D_Double_Struct point_inf;
//    Rox_Point2D_Double_Struct point_sup;

//    point_inf.u = pu[indexes_meas.u];
//    point_inf.v = pv[indexes_meas.u];

//    point_sup.u = pu[indexes_meas.v];
//    point_sup.v = pv[indexes_meas.v];

//    point_meas->u = (point_inf.u+point_sup.u)/2.0;
//    point_meas->v = (point_inf.v+point_sup.v)/2.0;

// function_terminate:

//    rox_dynvec_double_del ( &local_maxima_values );
//    rox_dynvec_double_del ( &local_maxima_coords );
//    rox_dynvec_point2d_uint_del ( &local_maxima_indexes );

//    return error;
// }
