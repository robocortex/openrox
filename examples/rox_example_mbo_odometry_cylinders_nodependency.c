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

#include <string.h>

#include <system/time/timer.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <inout/numeric/array2d_print.h>
#include <inout/image/pgm/pgmfile.h>
#include <inout/image/ppm/ppmfile.h>
#include <inout/system/errors_print.h>
#include <inout/system/memory_print.h>

#include <baseproc/image/draw/color.h>
#include <baseproc/image/draw/draw_polyline.h>
#include <baseproc/image/draw/draw_cylinder.h>
#include <baseproc/image/convert/roxgray_to_roxrgba.h>
#include <baseproc/image/convert/roxrgba_to_roxgray.h>

#include <user/odometry/edge/odometry_cylinders.h>

#include <extern/caofile/caofile_edges.h>

//TODO add other sequences
//#define SEQUENCE_BRIDGING_CENTRALE
#define SEQUENCE_TEST

#ifdef SEQUENCE_BRIDGING_CENTRALE
   //#define MODEL_PATH ROX_DATA_HOME"/devapps/model_based/kc135/models/cao/panier_kc135.cao"
   #define MODEL_PATH ROX_DATA_HOME"/devapps/model_based/kc135/models/cao/panier_kc135_2cercles.cao"
   #define IMAGE_PATH ROX_DATA_HOME"/devapps/model_based/kc135/simulator/approche_contact_central/ppm/kc135_simulator_approche_contact_central_img%04d.ppm"
   #define FIRSTIMAGE 2700
   #define LASTIMAGE  2800
   #define STEPIMAGE  1
   #define IMG_WIDTH  1920 
   #define IMG_HEIGHT 1080
   #define FU         1122
   #define FV         1123
   #define CU          959
   #define CV          539
   // #define WINDOW_WIDTH 960  // not used ?
   // #define WINDOW_HEIGHT 540 // not used ?
   #define POSE_INIT                         \
   { 0.00000, 0.00000,  1.00000,  0.40000    \
   , 0.00000, 1.00000,  0.00000,  0.27000    \
   , -1.0000, 0.00000,  0.00000,  3.80000    \
   , 0.00000, 0.00000,  0.00000,  1.00000    \
   }                                        
#endif

#ifdef SEQUENCE_TEST
   #define MODEL_PATH ROX_DATA_HOME"/devapps/model_based/test_cadmodel/test_cadmodel.cao"   // 2 cylinders
   #define IMAGE_PATH ROX_DATA_HOME"/devapps/model_based/test_cadmodel/test_cadmodel.ppm"
   #define FIRSTIMAGE 0
   #define LASTIMAGE  1
   #define STEPIMAGE  1
   #define IMG_WIDTH  1920 
   #define IMG_HEIGHT 1080
   #define FU         1000
   #define FV         1000
   #define CU          960
   #define CV          540
   // #define WINDOW_WIDTH 960  // not used ?
   // #define WINDOW_HEIGHT 540 // not used ?
   #define POSE_INIT                                                                   \
   {  1.0,  0.0,  0.0,   0.05 \
   ,  0.0,  1.0,  0.0,  -0.02 \
   ,  0.0,  0.0,  1.0,   2.00 \
   ,  0.0,  0.0,  0.0,   1.00 \
   }
   // { 0.984807753012208, -0.173648177666930, 0, 0.050000000000000, 0.173648177666930, 0.984807753012208, 0, -0.000268387495833, 0, 0, 1.000000000000000, 2.000000000000000, 0, 0, 0, 1.000000000000000 }
#endif

// mbo method choice, change MBO_METHOD only to switch
#define MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS   0
#define MBO_METHOD_EXTRACTONCE_SAMPLETRACKVVS   1
#define MBO_METHOD_EXTRACTSAMPLETRACKVVS        2
#define MBO_METHOD                              MBO_METHOD_EXTRACTONCE_SAMPLETRACKVVS // MBO_METHOD_EXTRACTSAMPLETRACKVVS // MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS //  

// COMMON MBO PARAMS
#define SEARCH_RANGE       50
#define SAMPLING_STEP      4
#define NB_ITERATIONS      100
#define MIN_SEGMENT_SIZE   5
#define THRESHOLD          51 // 51 = 4*255*0.05 where 4 = normalization sobel mask, 255 = max grayscale level, 0.05 threshold

int main(int argc, char *argv[])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_CaoFile_Edges caofile = NULL;
   Rox_Image_RGBA resimg = NULL;
   Rox_MatSE3 pose = NULL;
   Rox_Cylinder3D * cylinders = NULL;
   Rox_Uint nb_cylinders = 0;
   Rox_Odometry_Cylinders odometry = NULL;
   Rox_Double ** dt = NULL;
   Rox_Image_RGBA source_color = NULL;
   Rox_Image source_gray = NULL;
   // Rox_Sint color = ROX_MAKERGBA(255, 50, 0);
   Rox_Double pose_init[16] = POSE_INIT;
   Rox_MatUT3 calib = NULL;

   // Prepare iterations
   const Rox_Sint vvs_iterations          = (MBO_METHOD == MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS) ? NB_ITERATIONS : 1;
   const Rox_Sint global_iterations       = (MBO_METHOD == MBO_METHOD_EXTRACTSAMPLETRACKVVS) ? NB_ITERATIONS : 1;
   const Rox_Sint sample_track_iterations = (MBO_METHOD == MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS) ? 1 : NB_ITERATIONS;
   const Rox_Bool draw_before_odometry    = 1;
   const Rox_Bool draw_after_odometry     = 1;
   const Rox_Bool draw_happened           = draw_before_odometry || draw_after_odometry;
   
   // Create a calibration matrix
   error = rox_matut3_new ( &calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_calibration_matrix ( calib, FU, FV, CU, CV );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create images
   error = rox_image_rgba_new ( &resimg, IMG_WIDTH, IMG_HEIGHT );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_rgba_new ( &source_color, IMG_WIDTH, IMG_HEIGHT );
   ROX_ERROR_CHECK_TERMINATE ( error );   
   
   error = rox_image_new ( &source_gray, IMG_WIDTH, IMG_HEIGHT );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create pose
   error = rox_matse3_new ( &pose );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Load object 
   error = rox_caofile_edges_new(&caofile);
   ROX_ERROR_CHECK_TERMINATE ( error );

   sprintf(filename, MODEL_PATH);
   printf("Reading cao file %s \n", filename);
   error = rox_caofile_edges_load_cao(caofile, filename);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Initialize pose
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dt, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   memcpy(dt[0], pose_init, 16 * sizeof(double));
   
   // Prepare odometry
   error = rox_odometry_cylinders_new(&odometry, FU, FV, CU, CV, SEARCH_RANGE, THRESHOLD);
   ROX_ERROR_CHECK_TERMINATE ( error );

   printf("Starting odometry \n");
   for (Rox_Sint id = FIRSTIMAGE; id < LASTIMAGE; id += STEPIMAGE)
   {
      // Load sequence image from file
      //sprintf(filename, IMAGE_PATH, id);
      sprintf(filename, IMAGE_PATH);
      printf("Reading image %s \n", filename);
      error = rox_image_rgba_read_ppm(source_color, filename);
      if (error){ printf("Error %d, skipping frame %d\n", error, id); continue; }

      error = rox_roxrgba_to_roxgray(source_gray, source_color);
      if (error){ printf("Error %d, skipping frame %d\n", error, id); continue; }

      // Create result image
      //sprintf(filename, IMAGE_PATH, id);
      sprintf(filename, IMAGE_PATH);
      error = rox_image_rgba_read_ppm(resimg, filename);
      if (error){ printf("Error %d, skipping frame %d\n", error, id); continue; }

      for (int each_global_iteration = 0; each_global_iteration < global_iterations; ++each_global_iteration)
      {
         // Extract cylinders
         error = rox_caofile_edges_estimate_visible_cylinders(caofile, pose, FU, FV, CU, CV, MIN_SEGMENT_SIZE);
         if (error) { printf("Error %d, skipping frame %d\n", error, id); continue; }

         error = rox_caofile_edges_get_visible_cylinders(&cylinders, &nb_cylinders, caofile);
         if (error) { printf("Error %d, skipping frame %d\n", error, id); continue; }

         //error = rox_cylinder3d_display(cylinders[0]);
         //if (error) { printf("Error %d, skipping frame %d\n", error, id); continue; }

         //error = rox_cylinder3d_display(cylinders[1]);
         //if (error) { printf("Error %d, skipping frame %d\n", error, id); continue; }

         if (draw_before_odometry)
         {
            for(Rox_Sint ide = 0; ide < nb_cylinders; ide++)
            {
               error = rox_image_rgba_draw_cylinder3d(resimg, calib, pose, cylinders[ide], 0xFF00FF00);
               if (error) { printf("Error %d in drawcylinders() before odometry\n", error); } // no consequences 
            }
         }
                  
         // Iterate sampling and vvs resolution
         for (int each_sample_iteration = 0; each_sample_iteration < sample_track_iterations; ++each_sample_iteration)
         {
            // Reset pose
            error = rox_array2d_double_copy(odometry->pose, pose);
            if (error) { printf("Error %d, skipping frame %d\n", error, id); continue; }

            // Reset cylinders
            rox_objset_edge_cylinder_reset(odometry->objset_edge_cylinder);
         
            for (Rox_Sint ide = 0; ide < nb_cylinders; ide++)
            {
               error = rox_odometry_cylinders_add_cylinder(odometry, cylinders[ide], SAMPLING_STEP);
               if (error){ printf("Error %d, quitting frame %d\n", error, id); goto function_terminate; }
            }
            
            // printf("rox_odometry_cylinders_make : \n");

            // Make cylinders odometry
            error = rox_odometry_cylinders_make(odometry, source_gray, vvs_iterations);
            if (error){ printf("Error %d, skipping frame %d\n", error, id); continue; }

            Rox_Double score = -1.0;
            error = rox_odometry_cylinders_get_score(&score, odometry);
            if (error){ printf("Error %d, skipping frame %d\n", error, id); continue; }
            //printf("score = %f \n", score);

            // Get results of odometry
            rox_array2d_double_copy(pose, odometry->pose);
         }
      }

      printf("Pose estimated at image : %d\n", id);
      rox_array2d_double_print(pose);

      if (draw_after_odometry)
      {
         // get visible cylinders after odometry
         error = rox_caofile_edges_estimate_visible_cylinders(caofile, pose, FU, FV, CU, CV, MIN_SEGMENT_SIZE);
         if (error) { printf("Error %d, skipping frame %d\n", error, id); continue; }

         error = rox_caofile_edges_get_visible_cylinders(&cylinders, &nb_cylinders, caofile);
         if (error) { printf("Error %d, skipping frame %d\n", error, id); continue; }

         // draw model after odometry
         for(Rox_Sint ide = 0; ide < nb_cylinders; ide++)
         {
            error = rox_image_rgba_draw_cylinder3d(resimg, calib, pose, cylinders[ide], 0xFF0000FF);
            if (error) { printf("Error %d in drawcylinders() after odometry\n", error); } /*no consequences*/ 
         }
      }

      if (draw_happened)
      {
         //save result image to disk
         sprintf(filename, "result_mbo_odometry_cylinders_%4.4d.ppm", id - FIRSTIMAGE);
         rox_image_rgba_save_ppm(filename, resimg);
      }

      // getchar();
      //rox_memory_print_summary();
   }


function_terminate:
   rox_error_print(error);

   rox_image_rgba_del(&resimg);
   rox_matut3_del(&calib);
   rox_matse3_del(&pose);
   rox_image_del(&source_gray);
   rox_image_rgba_del(&source_color);
   rox_caofile_edges_del(&caofile);
   rox_odometry_cylinders_del(&odometry);
   
   rox_memory_print_summary();

   return error;
}

