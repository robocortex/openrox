//==============================================================================
//
//    OPENROX   : File rectangle_detection.c
//
//    Contents  : Implementation of rectangle detection module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "rectangle_detection.h"

#include <system/time/timer.h>
#include <system/memory/array_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/array/conversion/array2d_double_from_uchar.h>
#include <baseproc/image/filter/median/medianfilter.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_bilinear_nomask_uchar_to_uchar/remap_bilinear_nomask_uchar_to_uchar.h>
#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/meanvar/meanvar.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/image/imask/imask.h>
#include <baseproc/geometry/point/point2d_struct.h>

#include <core/features/detectors/quad/quad_detection.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>
#include <inout/image/pgm/pgmfile.h>
#include <baseproc/image/convert/roxgray_to_roxrgba.h>
#include <baseproc/image/draw/draw_warp_polygon.h>


Rox_QuadDetector quad = NULL;
Rox_Imask mask = NULL;
Rox_MatUT3 Ki = NULL;

Rox_ErrorCode 
rox_rectangle_detector_white(Rox_Sint * is_white, Rox_Image input, Rox_Point2D_Double pts_inp, Rox_Sint idimg, Rox_Sint mean_min, Rox_Sint variance_max);

Rox_ErrorCode 
rox_rectangle_get_ratio_from_homography(Rox_Double *ratio, Rox_MatSL3 H);

Rox_ErrorCode 
rox_rectangle_get_homographies(Rox_MatSL3 H1, Rox_MatSL3 H2, Rox_MatUT3 Ki, Rox_MatSL3 G);

Rox_ErrorCode rox_rectangle_detector_init(Rox_Sint cols, Rox_Sint rows, Rox_Double fu, Rox_Double fv, Rox_Double cu, Rox_Double cv)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_matut3_new ( &Ki );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_transformtools_build_calibration_matrix(Ki, 1.0/fu, 1.0/fv, -cu/fu, -cv/fv);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_quaddetector_new(&quad, cols, rows);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Set the quad color to 0 in order to detect white to black quads 
   error = rox_quaddetector_set_quad_color(quad, 0);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_uint_new(&mask, rows, cols);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_uint_fillval(mask, ~0);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_rectangle_detector_process(
   Rox_Sint * is_detected, 
   Rox_Point2D_Double pts_rect, 
   Rox_Image source, 
   Rox_Double ratio_rect, 
   Rox_Sint method, 
   Rox_Sint mean_min, 
   Rox_Sint variance_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint countq = 0;
   Rox_Sint is_white = 0;
   Rox_Point2D_Double_Struct pts[4];
   Rox_Double score_rect  =  0.0;
   Rox_Double score_best  =   0.0;
   Rox_Double score       =   0.0;
   Rox_MatSL3 G = NULL;
   Rox_MatSL3 H1 = NULL;
   Rox_MatSL3 H2 = NULL;

   if(ratio_rect > 0.0)
   {
      score_rect  =  ratio_rect+1.0/ratio_rect;
   }
   else
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_matsl3_new ( &G );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_new ( &H1 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matsl3_new ( &H2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(method == 0)
   {
      error = rox_quaddetector_process_image_ac(quad, source, mask);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      error = rox_quaddetector_process_image(quad, source, mask);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   
   error = rox_quaddetector_get_quad_count(&countq, quad); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   *is_detected = 0;
   for ( Rox_Sint i = 0; i < countq; i++)
   {         
      Rox_Double ratio1, ratio2;         
      Rox_Double score1, score2;         
        
      error = rox_quaddetector_get_quad_SL3(G, quad, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Get the two possible homographies
      error = rox_rectangle_get_homographies(H1, H2, Ki, G);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_rectangle_get_ratio_from_homography(&ratio1, H1);
      ROX_ERROR_CHECK_TERMINATE ( error );
            
      error = rox_rectangle_get_ratio_from_homography(&ratio2, H2);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      score1 = ratio1+1.0/ratio1;
      score2 = ratio2+1.0/ratio2;
            
      error = rox_quaddetector_get_points(pts, quad, i);
      ROX_ERROR_CHECK_TERMINATE ( error );
         
      error = rox_rectangle_detector_white(&is_white, source, pts, i, mean_min, variance_max);
      ROX_ERROR_CHECK_TERMINATE ( error );
            
      if(is_white==1)
      {               
         //error = rox_image_rgba_draw_polygon(color, pts, 4, ROX_MAKERGBA(0,0,255,255)); // display in blue 
         //ROX_ERROR_CHECK_TERMINATE(error)
         
         // test which score is better
         if (fabs(score1-score_rect) < fabs(score2-score_rect))
         {
            score = score1;
            if (fabs(score-score_rect) < fabs(score_best-score_rect))
            {
               score_best = score;
            }
            
            // Force the longer axis to be the x-axis
            if (ratio1 < 1.0/ratio1)
            {
               pts_rect[0].u = pts[0].u; pts_rect[0].v = pts[0].v;
               pts_rect[1].u = pts[1].u; pts_rect[1].v = pts[1].v;
               pts_rect[2].u = pts[2].u; pts_rect[2].v = pts[2].v;
               pts_rect[3].u = pts[3].u; pts_rect[3].v = pts[3].v;
            }
            else
            {
               // Rotate points
               pts_rect[0].u = pts[1].u; pts_rect[0].v = pts[1].v;
               pts_rect[1].u = pts[2].u; pts_rect[1].v = pts[2].v;
               pts_rect[2].u = pts[3].u; pts_rect[2].v = pts[3].v;
               pts_rect[3].u = pts[0].u; pts_rect[3].v = pts[0].v;
            }
         }
         else
         {
            score = score2;
            if (fabs(score-score_rect) < fabs(score_best-score_rect))
            {
               score_best = score;
            }
            
            if (ratio2 < 1.0/ratio2)
            {
               pts_rect[0].u = pts[0].u; pts_rect[0].v = pts[0].v;
               pts_rect[1].u = pts[1].u; pts_rect[1].v = pts[1].v;
               pts_rect[2].u = pts[2].u; pts_rect[2].v = pts[2].v;
               pts_rect[3].u = pts[3].u; pts_rect[3].v = pts[3].v;
            }
            else
            {
               // Rotate points
               pts_rect[0].u = pts[1].u; pts_rect[0].v = pts[1].v;
               pts_rect[1].u = pts[2].u; pts_rect[1].v = pts[2].v;
               pts_rect[2].u = pts[3].u; pts_rect[2].v = pts[3].v;
               pts_rect[3].u = pts[0].u; pts_rect[3].v = pts[0].v;
            }
         }
      }
   }
   
   
   if(fabs(score_best-score_rect)<0.5) // was 0.5
   {
      *is_detected = 1;
   }   
   
function_terminate:
   rox_matsl3_del(&G);
   rox_matsl3_del(&H1);
   rox_matsl3_del(&H2);
   
   return error;
}

Rox_ErrorCode rox_rectangle_detector_terminate()
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_quaddetector_del(&quad);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_del(&mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_del(&Ki);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
// 
Rox_ErrorCode rox_rectangle_detector_white (
   Rox_Sint * is_white, 
   Rox_Image input, 
   Rox_Point2D_Double pts_inp, 
   Rox_Sint idimg, 
   Rox_Sint mean_min, 
   Rox_Sint variance_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Char filename[FILENAME_MAX];

   Rox_Point2D_Double_Struct pts_out[4];
   Rox_MatSL3 H = NULL;

   // Warping grid 
   Rox_MeshGrid2D_Float grid = NULL;
   
   // Warped image
   Rox_Image warped = NULL;
  
   Rox_Array2D_Double warped_double = NULL;

   Rox_Imask mask = NULL;
   Rox_Double mean, variance;

   error = rox_matsl3_new ( &H );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_meshgrid2d_float_new(&grid, 128, 128); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_new(&warped, 128, 128); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&warped_double, 128, 128); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uint_new(&mask, 128, 128); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_imask_set_ones(mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   pts_out[0].u = 0.0;
   pts_out[0].v = 0.0;
   
   pts_out[1].u = 0.0;
   pts_out[1].v = 127.0;
   
   pts_out[2].u = 127.0;
   pts_out[2].v = 127.0;
   
   pts_out[3].u = 127.0;
   pts_out[3].v = 0.0;
   
   // compute homography such that pts_out = H * pts_inp 
   error = rox_matsl3_from_4_points_double(H, pts_out, pts_inp);
   ROX_ERROR_CHECK_TERMINATE(error)

   // warp input image to warped 
   error = rox_warp_grid_sl3_float(grid, H); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_remap_bilinear_nomask_uchar_to_uchar(warped, input, grid); 
   ROX_ERROR_CHECK_TERMINATE(error)

   // Convert to double in warped_double 
   error = rox_array2d_double_from_uchar(warped_double, warped);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_meanvar(&mean, &variance, warped_double, mask);
   ROX_ERROR_CHECK_TERMINATE(error)

   if((mean > mean_min) && (sqrt(variance) < variance_max))
   {
      *is_white = 1;
   }
   else
   {
      *is_white = 0;
   }

function_terminate:

   rox_matsl3_del(&H);
   rox_array2d_uchar_del(&warped);
   rox_array2d_double_del(&warped_double);
   rox_array2d_uint_del(&mask);
   rox_meshgrid2d_float_del(&grid);

   return error;
}

Rox_ErrorCode rox_rectangle_get_homographies(Rox_MatSL3 H1, Rox_MatSL3 H2, Rox_MatUT3 Ki, Rox_MatSL3 G)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double R_90 = NULL;
   Rox_Double ** data = NULL;

   error = rox_array2d_double_mulmatmat(H1, Ki, G);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_new(&R_90, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_fillunit(R_90);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_get_data_pointer_to_pointer( &data, R_90);
   data[1][0] = -1.0; data[0][1] = +1.0;
   data[0][1] = +0.5; data[1][1] = +0.5;

   error = rox_array2d_double_mulmatmat(H2, H1, R_90);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   rox_array2d_double_del(&R_90);
   return error;
}

Rox_ErrorCode rox_rectangle_get_ratio_from_homography(Rox_Double *ratio, Rox_MatSL3 H)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double n1 = 1.0, n2 = 1.0;
   
   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, H);

   n1 = sqrt(data[0][0]*data[0][0]+data[1][0]*data[1][0]+data[2][0]*data[2][0]);
   n2 = sqrt(data[0][1]*data[0][1]+data[1][1]*data[1][1]+data[2][1]*data[2][1]);

   *ratio = n1/n2;
   
   return error;
}
