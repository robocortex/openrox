//==============================================================================
//
//    OPENROX   : File rox_example_image_undistortion.c
//
//    Contents  : A simple example program for image undistortion.
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== HEADERS   ================================================================

#include <math.h>

#include <system/time/timer.h>

#include <baseproc/array/fill/fillval.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matso3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/image/pyramid/pyramid_float.h>

#include <core/predict/plane_search.h>

#include <inout/image/pgm/pgmfile.h>
#include <inout/image/ppm/ppmfile.h>

#include <baseproc/image/convert/roxrgba_to_roxgray.h>
#include <baseproc/image/draw/draw_warp_polygon.h>
#include <baseproc/image/draw/color.h>

#include <inout/system/errors_print.h>
#include <inout/numeric/array2d_print.h>
#include <inout/numeric/array2d_save.h>

#include <baseproc/image/image.h>
#include <baseproc/image/draw/image_rgba_draw.h>

//#define SYNTHETIC_LOGITECHC920_1280x720
//#define SYNTHETIC_LOGITECHC920
#define SYNTHETIC_SURFACEPRO4

#ifdef SYNTHETIC_SURFACEPRO4
   #define IMAGE_PROCESS ROX_DATA_HOME"/devapps/model_based/tandem/synthetic/pgm/img_%08d.pgm"

   #define FIRST_IMAGE 0
   #define LAST_IMAGE  10

   // Surface Pro 4
   #define FU 1045.7261109469930
   #define FV 1050.6385778690100
   #define CU  638.5931942094010
   #define CV  361.1745588634177
#endif

#ifdef SYNTHETIC_LOGITECHC920_1280x720

   #define IMAGE_PROCESS ROX_DATA_HOME"/../model_based/test_prediction/synthetic_1280x720/pgm/img_%08d.pgm"

   #define FIRST_IMAGE 0
   #define LAST_IMAGE  10

   // Logitech C920
   #define FU           1000.0
   #define FV           1000.0
   #define CU           640.0
   #define CV           360.0
#endif

#ifdef SYNTHETIC_LOGITECHC920

   #define IMAGE_PROCESS ROX_DATA_HOME"/../model_based/test_prediction/synthetic/pgm/img_%08d.pgm"

   #define FIRST_IMAGE 0
   #define LAST_IMAGE  10

   // Logitech C920
   #define FU           536.1661
   #define FV           534.4585
   #define CU           318.8734
   #define CV           237.4206
#endif

#ifdef REAL_LOGITECHC920
   #define IMAGE_PROCESS ROX_DATA_HOME"/../model_based/test_prediction/logitech_c920/image_camera_%08d.ppm"

   #define FIRST_IMAGE 0
   #define LAST_IMAGE  629

   // Logitech C920
   #define FU           536.1661
   #define FV           534.4585
   #define CU           318.8734
   #define CV           237.4206
#endif

int main(int argc, char *argv[])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   
   const Rox_Sint pyramid_max_levels = 10;
   const Rox_Sint pyramid_min_size = 32; // was 64 

   Rox_Sint rows = 0, cols = 0;

   Rox_Sint cols_model = 0;
   Rox_Sint rows_model = 0;

   // Define the shift of the template in the reference image
   Rox_Sint tu_model = 0;
   Rox_Sint tv_model = 0;
   Rox_Sint search_radius = 0;

   Rox_Plane_Search plane_search = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;

   Rox_MatSL3 r_H_t = NULL;
   Rox_MatSL3 c_H_t = NULL;
   Rox_MatSO3 rotation = NULL;

   Rox_Array2D_Double intrinsics_top_level = NULL;

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0;
   
   sprintf(filename, IMAGE_PROCESS, FIRST_IMAGE);
   printf("Read image %s\n", filename);

   // Create and read the uchar image
   error = rox_image_new_read_pgm ( &image_uchar, filename );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_get_size(&rows, &cols, image_uchar);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create the float image
   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );


   // Init -------------------------------------------------------------------------------------------------------------------------------
   // rox_rx_ry_prediction_new(Rox_Rx_Ry_Prediction prediction, Rox_Sint cols, Rox_Sint rows, Rox_Double fu, Rox_Double fv, Rox_Double cu, Rox_Double cv, Rox_Double scaling_factor);
   // TODO : rename scaling_factor -> prediction_range

   // Create a new pyramid
   Rox_Pyramid_Float pyramid = NULL;

   error = rox_pyramid_float_new(&pyramid, cols, rows, pyramid_max_levels, pyramid_min_size);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint nb_levels = 0;
   error = rox_pyramid_float_get_nb_levels(&nb_levels, pyramid);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint top_level_id = nb_levels-1;
   Rox_Double intrinsics_scale = pow(2, top_level_id);

   Rox_Array2D_Float image_top_level = NULL;

   // Get the pointer to the image at the top level of the pyramid
   error = rox_pyramid_float_get_image(&image_top_level, pyramid, top_level_id);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols_top_level = 0;
   Rox_Sint rows_top_level = 0;

   error = rox_array2d_float_get_size(&rows_top_level, &cols_top_level, image_top_level); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create rotation matrix
   error = rox_matso3_new(&rotation);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create camera calibration matrix for the top level of the pyramid
   error = rox_array2d_double_new(&intrinsics_top_level, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Top level of the pyramid is level 3 (4th level), tweak the intrinsics by 2^3
   error = rox_transformtools_build_calibration_matrix(intrinsics_top_level, FU / intrinsics_scale, FV / intrinsics_scale, CU / intrinsics_scale, CV / intrinsics_scale);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Select the half of the image at top level as model
   cols_model = cols_top_level/2;
   rows_model = rows_top_level/2;

   // Define the shift of the template in the reference image
   tu_model = cols_model/2-1;
   tv_model = rows_model/2-1;
   search_radius = cols_model/2;

   // printf("cols_model = %d \n", cols_model);
   // printf("rows_model = %d \n", rows_model);

   // printf("tu_model = %d \n", tu_model);
   // printf("tv_model = %d \n", tv_model);

   // printf("search_radius = %d \n", search_radius);
   // printf("intrinsics_scale = %f \n", intrinsics_scale);

   // getchar();

   error = rox_matsl3_new(&r_H_t);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_homography_shift(r_H_t, tu_model, tv_model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new(&c_H_t);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_homography_shift(c_H_t, tu_model, tv_model);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Define the new search 
   // TODO : rename plane_search -> tu_tv_search
   error = rox_plane_search_new(&plane_search, rows_model, cols_model, search_radius);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // -------------------------------------------------------------------------------------------


   // Add template ------------------------------------------------------------------------------
   // rox_rx_ry_prediction_set(Rox_Rx_Ry_Prediction prediction, Rox_Image image_uchar);

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_pyramid_float_assign(pyramid, image_float);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_plane_search_set_model_warp(plane_search, image_top_level, r_H_t);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // -------------------------------------------------------------------------------------------

   // Reset debug
   FILE* rotation_file = fopen("prediction_rotation.txt", "w");
   fprintf(rotation_file, " ");
   fclose(rotation_file);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for(Rox_Sint i = FIRST_IMAGE+1; i < LAST_IMAGE+1; i++)
   {
      Rox_Double score = 0.0, tu = 0.0, tv = 0.0;

      sprintf(filename, IMAGE_PROCESS, i);

      // Read another uchar image
      error = rox_image_read_pgm(image_uchar, filename);
      //error = rox_image_read_ppm(image_uchar, filename);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //rox_image_save_pgm("image_uchar.pgm", image_uchar);
      
      rox_timer_start(timer);

      // Make -------------------------------------------------------------------------------------------
      // rox_rx_ry_prediction_make(Rox_Rx_Ry_Prediction prediction, Rox_Array2D_Uchar image);

      // Convert image to float
      error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_pyramid_float_assign ( pyramid, image_float );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_pyramid_float_get_image ( &image_top_level, pyramid, top_level_id );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Process current image : put in plug in_process
      error = rox_plane_search_make ( plane_search, image_top_level, c_H_t );
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      // Get results of plane search
      error = rox_plane_search_get_results ( &score, &tu, &tv, plane_search );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute results of so3 prediction
      error = rox_plane_search_update_pose_rotation ( rotation, intrinsics_top_level, plane_search );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // -------------------------------------------------------------------------------------------

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);

      printf("time to compute prediction on a (%d x %d) image = %f (ms)\n", cols, rows, time);

      // Display results
      error = rox_matso3_print(rotation);
      ROX_ERROR_CHECK_TERMINATE ( error );

      printf("score = %.12f\n", score);
      printf("tu = %.2f\n", tu);
      printf("tv = %.2f\n", tv);

      // Write rotation results
      FILE * rotation_file = fopen("prediction_rotation.txt", "a");
      fprintf(rotation_file, "tu=%.2f tv=%.2f\n", tu, tv);
      error = rox_matso3_write(rotation_file, rotation);
      fclose(rotation_file);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Save image
      sprintf(filename, "./image_top_level_%08d.txt", i);
      rox_array2d_float_save(filename, image_top_level);

      // Update the reference template
      error = rox_plane_search_set_model_warp(plane_search, image_top_level, r_H_t);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //getchar();
   }


function_terminate:

   ROX_ERROR_CHECK(rox_image_del(&image_uchar));
   ROX_ERROR_CHECK(rox_array2d_float_del(&image_float));
   ROX_ERROR_CHECK(rox_pyramid_float_del(&pyramid));
   ROX_ERROR_CHECK(rox_matso3_del(&rotation));

   ROX_ERROR_CHECK(rox_array2d_double_del(&intrinsics_top_level));
   ROX_ERROR_CHECK(rox_matsl3_del(&r_H_t));
   ROX_ERROR_CHECK(rox_matsl3_del(&c_H_t));

   // Define the new search : put in plugin_terminate
   ROX_ERROR_CHECK(rox_plane_search_del(&plane_search));

   return error;
}

// Matlab code for image display
/*
for i =1:629
   I = load(sprintf('image_top_level_%08d.txt',i));
   figure(1);clf;imagesc(I); colormap(gray); drawnow
   figure(2);clf;imagesc(I(15:45,20:60)); colormap(gray); drawnow
   %pause;
end
*/