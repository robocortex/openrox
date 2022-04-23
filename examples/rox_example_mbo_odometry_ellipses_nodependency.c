//==============================================================================
//
//    OPENROX   : File rox_example_mbo_odometry_ellipses_nodependency.c
//
//    Contents  : Devapp for mbo odometry ellipses module
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
#include <baseproc/geometry/measures/distance_point_to_point.h>

#include <core/tracking/edge/tracking_ellipse.h> // only for debug, can be deleted

#include <inout/numeric/array2d_print.h>
#include <inout/image/pgm/pgmfile.h>
#include <inout/image/ppm/ppmfile.h>

#include <baseproc/image/draw/color.h>
#include <baseproc/image/draw/draw_polyline.h>
#include <baseproc/image/draw/draw_ellipse.h>

#include <baseproc/image/convert/roxgray_to_roxrgba.h>
#include <baseproc/image/convert/roxrgba_to_roxgray.h>
#include <inout/system/memory_print.h>
#include <inout/system/errors_print.h>

#include <baseproc/image/draw/image_rgba_draw.h>
#include <user/odometry/edge/odometry_ellipses.h>

#include <extern/caofile/caofile_edges.h>

//TODO add other sequences
//#define SEQUENCE_BRIDGING_CENTRALE
#define SEQUENCE_TEST

//#define SEQUENCE_VENTANA

#ifdef SEQUENCE_VENTANA
//#define MODEL_PATH ROX_DATA_HOME"/devapps/model_based/ventana/models/cao/circles.cao"
#define MODEL_PATH ROX_DATA_HOME"/devapps/model_based/ventana/models/cao/circles_0600285_guide_unified_stl.cao"

#define FIRSTIMAGE 1
#define LASTIMAGE  200//990
#define STEPIMAGE  1
#define FU 1045.726110946993
#define FV 1050.63857786901
#define CU 638.593194209401
#define CV 361.1745588634177

#define IMG_WIDTH 1280
#define IMG_HEIGHT 720
/*
#define POSE_INIT                         \
    {  0.3419789162942253, -0.9391765846049623, -0.0315873984364761, 62.0000000000006111 \
    , -0.7656266419097623, -0.2589799504710549, -0.5888507709530408, 49.2000000000004292 \
    ,  0.5448543530230877, 0.2255587023011249, -0.8076242974354008, 643.2000000000780346 \
    ,  0.00000, 0.00000,  0.00000,  1.00000    \
    }
*/

// #define IMAGE_PATH ROX_DATA_HOME"/devapps/model_based/ventana/surfacepro4/2017-05-29_08-05-03/ppm/capture%d.ppm"

/*
#define POSE_INIT                         \
    { 0.3324484788296993, -0.9422043607601410, -0.0415806624333127, -17.1999999999999744 \
    ,-0.7861199856757581, -0.2524786557596752, -0.5641541425062245, 5.3999999999999968  \
    , 0.5210502634535032, 0.2202395762581321, -0.8246218236286702, 700.7000000000911086 \
    , 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000 }
*/

// #define IMAGE_PATH ROX_DATA_HOME"/devapps/model_based/ventana/surfacepro4/2017-06-14_23-05-33/ppm/capture_%08d.ppm"

/*
#define POSE_INIT                         \
 { -0.1105696395874618, 0.9938671029157702, 0.0015926529165099, 21.9000000000000412 \
 ,0.8366437155226993, 0.0939429821963013, -0.5396313643335034, 42.8000000000003382  \
 ,-0.5364714792772043, -0.0583343624108388, -0.8418999074024475, 684.3000000000873797 \
 ,0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000}
*/

#define IMAGE_PATH ROX_DATA_HOME"/devapps/model_based/ventana/surfacepro4/WIN_20170615_11_44_32_Pro_ident.ppm"

#define POSE_INIT                            \
   {-0.5206674258286258, -0.8477144706400108, -0.1014179863166225, -3.2000000000000011 \
,-0.5490829090724398, 0.4234522343033860, -0.7205526797035849, 48.7000000000004221     \
,0.6537686063474898, -0.3194814259561840, -0.6859433123979447, 773.0000000000000000    \
,0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000 }

#endif

#ifdef SEQUENCE_BRIDGING_CENTRALE
   //#define MODEL_PATH ROX_DEVAPPS_DATA"/model_based/kc135/models/cao/panier_kc135.cao"
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
   #define POSE_INIT                         \
   { 0.00000, 0.00000,  1.00000,  0.40000    \
   , 0.00000, 1.00000,  0.00000,  0.27000    \
   , -1.0000, 0.00000,  0.00000,  3.80000    \
   , 0.00000, 0.00000,  0.00000,  1.00000    \
   }                                        
#endif

#ifdef SEQUENCE_TEST
   #define MODEL_PATH ROX_DATA_HOME"/devapps/model_based/test_cadmodel/test_ellipses.cao"
   #define IMAGE_PATH ROX_DATA_HOME"/devapps/model_based/test_cadmodel/test_ellipses.ppm"
   #define FIRSTIMAGE 0
   #define LASTIMAGE  1
   #define STEPIMAGE  1
   #define IMG_WIDTH  1920 
   #define IMG_HEIGHT 1080
   #define FU         1000
   #define FV         1000
   #define CU          960
   #define CV          540
   #define POSE_INIT                                                                   \
   {  1.0,  0.0,  0.0,   0.02 \
   ,  0.0,  1.0,  0.0,  -0.02 \
   ,  0.0,  0.0,  1.0,   2.00 \
   ,  0.0,  0.0,  0.0,   1.00 \
   }
   // {  0.984807753012208,  -0.173648177666930, 0, 0.050000000000000, 0.173648177666930, 0.984807753012208, 0,  -0.000268387495833 ,                  0,                   0,   1.000000000000000,   2.000000000000000 ,                  0,                   0,                   0,   1.000000000000000 }
#endif

//mbo method choice, change MBO_METHOD only to switch
#define MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS   0
#define MBO_METHOD_EXTRACTONCE_SAMPLETRACKVVS   1
#define MBO_METHOD_EXTRACTSAMPLETRACKVVS        2
#define MBO_METHOD                              MBO_METHOD_EXTRACTONCE_SAMPLETRACKVVS // MBO_METHOD_EXTRACTONCE_SAMPLETRACKVVS // MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS //    

// COMMON MBO PARAMS

#define SEARCH_RANGE       20
#define SAMPLING_STEP      5
#define NB_ITERATIONS      15
#define MIN_SEGMENT_SIZE   11

#define THRESHOLD          51 // 51 = 4*255*0.05 where 4 = normalization sobel mask, 255 = max grayscale level, 0.05 threshold

int main(int argc, char *argv[])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_CaoFile_Edges caofile = NULL;
   Rox_Image_RGBA resimg = NULL;
   Rox_MatSE3 pose = NULL;
   Rox_Ellipse3D * ellipses = NULL;
   Rox_Uint nb_ellipses = 0;
   Rox_Odometry_Ellipses odometry = NULL;
   Rox_Double ** dt = NULL;
   Rox_Image_RGBA source_color = NULL;
   Rox_Image source_gray = NULL;
   // Rox_Sint color = ROX_MAKERGBA(255, 50, 0,255);
   Rox_Double pose_init[16] = POSE_INIT;
   Rox_MatUT3 calib = NULL;

   // Prepare iterations
   const Rox_Sint vvs_iterations          = (MBO_METHOD == MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS) ? NB_ITERATIONS : 1;
   const Rox_Sint global_iterations       = (MBO_METHOD == MBO_METHOD_EXTRACTSAMPLETRACKVVS) ? NB_ITERATIONS : 1;
   const Rox_Sint sample_track_iterations = (MBO_METHOD == MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS) ? 1 : NB_ITERATIONS;
   const Rox_Bool draw_before_odometry    = 1;
   const Rox_Bool draw_after_odometry     = 1;
   const Rox_Bool draw_happened           = draw_before_odometry || draw_after_odometry;
   
   printf("global_iterations = %d \n ", global_iterations);
   printf("sample_track_iterations = %d \n ", sample_track_iterations);
   printf("vvs_iterations = %d \n ",vvs_iterations);

   // Create a calibration matrix
   error = rox_matut3_new ( &calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_calibration_matrix(calib, FU, FV, CU, CV);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create images
   error = rox_image_rgba_new(&resimg, IMG_WIDTH, IMG_HEIGHT );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_rgba_new(&source_color, IMG_WIDTH, IMG_HEIGHT );
   ROX_ERROR_CHECK_TERMINATE ( error );   
   
   error = rox_image_new(&source_gray, IMG_WIDTH, IMG_HEIGHT );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create pose
   error = rox_array2d_double_new(&pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Load object 
   error = rox_caofile_edges_new(&caofile);
   ROX_ERROR_CHECK_TERMINATE ( error );

   sprintf(filename, MODEL_PATH);
   printf("Reading cao file %s \n", filename);
   error = rox_caofile_edges_load_cao(caofile, filename);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //printf("Press enter when ready\n");
   //getchar();

   // Initialize pose
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dt, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   memcpy(dt[0], pose_init, 16 * sizeof(double));
   
   // Prepare odometry
   error = rox_odometry_ellipses_new(&odometry, FU, FV, CU, CV, SEARCH_RANGE, THRESHOLD);
   ROX_ERROR_CHECK_TERMINATE ( error );

   printf("Starting odometry \n");

   for (Rox_Sint id = FIRSTIMAGE; id < LASTIMAGE; id += STEPIMAGE)
   {
      // Load sequence image from file
      // sprintf(filename, IMAGE_PATH, id);      
      sprintf(filename, IMAGE_PATH);

      printf("Reading image %s \n", filename);
      error = rox_image_rgba_read_ppm(source_color, filename);
      if (error)
      { printf("Error %d, skipping frame %d\n", error, id); continue; }

      error = rox_roxrgba_to_roxgray(source_gray, source_color);
      if (error)
      { printf("Error %d, skipping frame %d\n", error, id); continue; }

      // Create result image
      // sprintf(filename, IMAGE_PATH, id);
      sprintf(filename, IMAGE_PATH);
      error = rox_image_rgba_read_ppm(resimg, filename);
      if (error)
      { printf("Error %d, skipping frame %d\n", error, id); continue; }

      for (int each_global_iteration = 0; each_global_iteration < global_iterations; ++each_global_iteration)
      {
         // Extract ellipses given the camera intrinsic and extrinsic paramaters 
         error = rox_caofile_edges_estimate_visible_ellipses(caofile, pose, FU, FV, CU, CV, MIN_SEGMENT_SIZE);
         if (error) 
         { printf("Error %d, skipping frame %d\n", error, id); continue; }

         // Get the visible ellipses
         error = rox_caofile_edges_get_visible_ellipses(&ellipses, &nb_ellipses, caofile);
         if (error) 
         { printf("Error %d, skipping frame %d\n", error, id); continue; }

         //error = rox_ellipse3d_display(ellipses[0]);
         //if (error) { printf("Error %d, skipping frame %d\n", error, id); continue; }

         //error = rox_ellipse3d_display(ellipses[1]);
         //if (error) { printf("Error %d, skipping frame %d\n", error, id); continue; }

         if (draw_before_odometry)
         {
            for(Rox_Sint ide = 0; ide < nb_ellipses; ide++)
            {
               error = rox_image_rgba_draw_ellipse3d(resimg, calib, pose, ellipses[ide], ROX_MAKERGBA(0,255,0,255));
               if (error) 
               { printf("Error %d in drawellipses() before odometry\n", error); } // no consequences 
            }
         }
                  
         //printf("number of sample iterations = %d \n", sample_track_iterations);

         // Iterate sampling and vvs resolution
         for (int each_sample_iteration = 0; each_sample_iteration < sample_track_iterations; ++each_sample_iteration)
         {
            // Reset pose
            error = rox_array2d_double_copy(odometry->pose, pose);
            if (error) 
            { printf("Error %d, in rox_array2d_double_copy() skipping frame %d\n", error, id); continue; }

            // Reset ellipses
            rox_objset_edge_ellipse_reset(odometry->objset_edge_ellipse);
         
            // Add the ellipses to the odometry
            for (Rox_Sint ide = 0; ide < nb_ellipses; ide++)
            {
               error = rox_odometry_ellipses_add_ellipse(odometry, ellipses[ide], SAMPLING_STEP);
               if (error)
               { printf("Error %d, in rox_odometry_ellipses_add_ellipse() quitting frame %d\n", error, id); goto function_terminate; }
            }
            
            // printf("rox_odometry_ellipses_make : \n");

            // Make ellipses odometry
            error = rox_odometry_ellipses_make(odometry, source_gray, vvs_iterations);
            if (error)
            { printf("Error %d, in rox_odometry_ellipses_make() skipping frame %d\n", error, id); continue; }

            Rox_Double score = -1.0;
            error = rox_odometry_ellipses_get_score(&score, odometry);
            if (error)
            { printf("Error %d, skipping frame %d\n", error, id); continue; }
            //printf("score = %f \n", score);

            // Get results of odometry
            rox_array2d_double_copy(pose, odometry->pose);
         }
      }

      printf("Pose estimated at image : %d\n", id);
      rox_array2d_double_print(pose);

      if (draw_after_odometry)
      {
         // get visible ellipses after odometry
         error = rox_caofile_edges_estimate_visible_ellipses(caofile, pose, FU, FV, CU, CV, MIN_SEGMENT_SIZE);
         if (error) 
         { printf("Error %d, skipping frame %d\n", error, id); continue; }

         error = rox_caofile_edges_get_visible_ellipses(&ellipses, &nb_ellipses, caofile);
         if (error)
         { printf("Error %d, skipping frame %d\n", error, id); continue; }

         // draw model after odometry
         for ( Rox_Sint ide = 0; ide < nb_ellipses; ide++ )
         {
            error = rox_image_rgba_draw_ellipse3d ( resimg, calib, pose, ellipses[ide], ROX_MAKERGBA(255,0,0,255) );
            if (error) 
            { printf("Error %d in drawellipses() after odometry\n", error); } /*no consequences*/ 
         }

         Rox_DynVec_Point2D_Double dynvec_point2d = NULL;
         error = rox_dynvec_point2d_double_new(&dynvec_point2d,100);

         error = rox_odometry_ellipses_get_valid_sites(dynvec_point2d, odometry);
         if (error) 
         { printf("Error %d in rox_odometry_ellipses_get_valid_sites() after odometry\n", error); } 
      
         error = rox_image_rgba_draw_dynvec_point2d_double(resimg, dynvec_point2d, ROX_MAKERGBA(0,0,255,255));
         if (error) 
         { printf("Error %d in rox_image_display_draw_dynvec_point2d_double() after odometry\n", error); } 
      }

      if (draw_happened)
      {
         //save result image to disk
         sprintf(filename, "result_mbo_odometry_ellipses_%4.4d.ppm", id - FIRSTIMAGE);
         error = rox_image_rgba_save_ppm(filename, resimg);
      }

      //if(id == 843) getchar();
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
   rox_odometry_ellipses_del(&odometry);
   
   rox_memory_print_summary();

   return error;
}

