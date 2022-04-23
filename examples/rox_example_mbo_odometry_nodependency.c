//==============================================================================
//
//    OPENROX   : File rox_example_mbo_odometry_nodependency.c
//
//    Contents  : Devapp for mbo odometry module
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
#include <baseproc/image/draw/draw_polyline.h>
#include <baseproc/image/draw/draw_circle.h>
#include <baseproc/image/draw/draw_ellipse.h>
#include <baseproc/image/draw/draw_cylinder.h>
#include <baseproc/image/draw/color.h>
#include <baseproc/image/convert/roxgray_to_roxrgba.h>
#include <baseproc/image/convert/roxrgba_to_roxgray.h>

#include <inout/system/memory_print.h>
#include <inout/system/errors_print.h>
#include <inout/numeric/array2d_print.h>
#include <inout/image/pgm/pgmfile.h>
#include <inout/image/ppm/ppmfile.h>

#include <user/odometry/edge/odometry_cadmodel.h>

#include <extern/caofile/caofile_edges.h>

//TODO add other sequences
#define SEQUENCE_BRIDGING_CENTRALE
// #define SEQUENCE_TEST

#ifdef SEQUENCE_BRIDGING_CENTRALE
   
   // #define MODEL_PATH ROX_DATA_HOME"/devapps/model_based/kc135/models/cao/KC135_GoPro.cao"
   // #define IMAGE_PATH ROX_DATA_HOME"/devapps/model_based/kc135/simulator/approche_contact_central_full_flip/ppm/kc135_simulator_approche_contact_central_img%04d.ppm"
   // #define POSE_INIT { 0.00004, 1.00000, -0.00004, 46.34560, 0.14751, 0.00004, 0.98906, -21.42140    , 0.98906, -0.00004, -0.14751, 108.03323  , 0.00000, 0.00000, 0.00000, 1.00000      }
   
   #define MODEL_PATH ROX_DATA_HOME"/devapps/model_based/kc135/models/cao/MKC_aile_ouverte+L+S+R+Bcomplet_face.cao"
   #define IMAGE_PATH ROX_DATA_HOME"/devapps/model_based/kc135/simulator/approche_contact_central_half_sunset/ppm/kc135_simulator_approche_contact_central_half_sunset_img%04d.ppm"

   #define FIRSTIMAGE   0// 2149 // 0
   #define LASTIMAGE    3000 //2151 // 3000
   #define STEPIMAGE    1
   #define IMG_WIDTH    960
   #define IMG_HEIGHT   540
   #define FU           561
   #define FV           561.5
   #define CU           479.5
   #define CV           269.5
   // Image 0
   #define POSE_INIT                         \
   { 0.00004, 1.00000, -0.00004, 46.34560    \
   , 0.14751, 0.00004, 0.98906, -21.42140    \
   , 0.98906, -0.00004, -0.14751, 108.03323  \
   , 0.00000, 0.00000, 0.00000, 1.00000      \
   }
   // Image 500
   // #define POSE_INIT { -0.0119227825386, 0.9993032691133, -0.0353669845485, 3.6615210970000, 0.1616004817749, 0.0368302033897, 0.9861687585847, -20.7640741500000, 0.9867842375852, 0.0060425539130, -0.1619270082338, 107.4291430000000, 0.0, 0.0, 0.0, 1.0000000000000 }
   // Image 2149
   // #define POSE_INIT { -0.0010045950007900, 0.9999994880424219, -0.0000661486939248,  0.2912852510734719,  0.1567766469463691, 0.0002292169665086,  0.9876334938247519, -3.6202668084078580,  0.9876330082885763, 0.0009872053797999, -0.1567767929250712, 55.2902926250433566,  0.0000000000000000, 0.0000000000000000,  0.0000000000000000,  1.0000000000000000 }
#endif

#ifdef SEQUENCE_TEST
   #define MODEL_PATH ROX_DATA_HOME"/devapps/model_based/test_cadmodel/test_cadmodel.cao"
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
   {  0.984807753012208,  -0.173648177666930,                   0,   0.050000000000000 \
   ,  0.173648177666930,   0.984807753012208,                   0,  -0.000268387495833 \
   ,                  0,                   0,   1.000000000000000,   2.000000000000000 \
   ,                  0,                   0,                   0,   1.000000000000000 \
   }
   // {  1.0,  0.0,  0.0,   0.05,  0.0,  1.0,  0.0,  -0.02,  0.0,  0.0,  1.0,   2.00,  0.0,  0.0,  0.0,   1.00 }
#endif

//mbo method choice, change MBO_METHOD only to switch
#define MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS   0
#define MBO_METHOD_EXTRACTONCE_SAMPLETRACKVVS   1
#define MBO_METHOD_EXTRACTSAMPLETRACKVVS        2
#define MBO_METHOD                              MBO_METHOD_EXTRACTONCE_SAMPLETRACKVVS // MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS //MBO_METHOD_EXTRACTSAMPLETRACKVVS //       

// COMMON MBO PARAMS
#define SEARCH_RANGE       20
#define SAMPLING_STEP      10
#define NB_ITERATIONS      30
#define MIN_SEGMENT_SIZE   10
#define THRESHOLD          51 // 51 = 4*255*0.05 where 4 = normalization sobel mask, 255 = max grayscale level, 0.05 threshold

int main(int argc, char *argv[])
{
   Rox_Char filename[FILENAME_MAX];
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CaoFile_Edges caofile = NULL;
   Rox_Image_RGBA resimg = NULL;
   Rox_Image_RGBA source_color = NULL;
   Rox_Image source_gray = NULL;
   Rox_MatSE3 pose = NULL;
   Rox_MatUT3 calib = NULL;

   Rox_Segment3D_Struct * segments = NULL;
   Rox_Ellipse3D * ellipses = NULL;
   Rox_Cylinder3D * cylinders = NULL;

   Rox_Uint nb_segments  = 0;
   Rox_Uint nb_ellipses  = 0;
   Rox_Uint nb_cylinders = 0;

   Rox_Odometry_CadModel odometry = NULL;
   Rox_Double ** dt = NULL;

   // Rox_Sint color = ROX_MAKERGBA(255, 50, 0);
   Rox_Double pose_init[16] = POSE_INIT;

   // Prepare iterations
   const Rox_Sint vvs_iterations          = (MBO_METHOD == MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS) ? NB_ITERATIONS : 1;
   const Rox_Sint global_iterations       = (MBO_METHOD == MBO_METHOD_EXTRACTSAMPLETRACKVVS) ? NB_ITERATIONS : 1;
   const Rox_Sint sample_track_iterations = (MBO_METHOD == MBO_METHOD_EXTRACTSAMPLETRACKONCE_VVS) ? 1 : NB_ITERATIONS;
   const Rox_Bool draw_before_odometry    = 1;
   const Rox_Bool draw_after_odometry     = 1;
   const Rox_Bool draw_happened = draw_before_odometry || draw_after_odometry;
   
   // Create a calibration matrix
   error = rox_matut3_new ( &calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_calibration_matrix(calib, FU, FV, CU, CV);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //create images
   error = rox_image_rgba_new(&resimg, IMG_WIDTH, IMG_HEIGHT );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_rgba_new(&source_color, IMG_WIDTH, IMG_HEIGHT );
   ROX_ERROR_CHECK_TERMINATE ( error );   
   
   error = rox_image_new(&source_gray, IMG_WIDTH, IMG_HEIGHT );
   ROX_ERROR_CHECK_TERMINATE ( error );

   //create pose
   error = rox_matse3_new ( &pose );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Load object 
   error = rox_caofile_edges_new(&caofile);
   ROX_ERROR_CHECK_TERMINATE ( error );

   sprintf(filename, MODEL_PATH);
   printf("Reading cao file %s \n", filename);
   
   error = rox_caofile_edges_load_cao(caofile, filename);
   ROX_ERROR_CHECK_TERMINATE ( error );

   printf("read coa file \n");

   // Initialize pose
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dt, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   memcpy(dt[0], pose_init, 16 * sizeof(double));
   
   // Prepare odometry
   error = rox_odometry_cadmodel_new ( &odometry, FU, FV, CU, CV, SEARCH_RANGE, THRESHOLD );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   for (Rox_Sint id = FIRSTIMAGE; id < LASTIMAGE; id += STEPIMAGE)
   {
      Rox_Double score = -1.0;

      // Load sequence image from file
      sprintf(filename, IMAGE_PATH, id);
      printf("Reading image %s \n", filename);
      error = rox_image_rgba_read_ppm(source_color, filename);
      if (error) 
      { printf("Error %d after rox_image_rgba_read_ppm, skipping frame %d\n", error, id); continue; }

      error = rox_roxrgba_to_roxgray(source_gray, source_color);
      if (error) 
      { printf("Error %d after rox_roxrgba_to_roxgray, skipping frame %d\n", error, id); continue; }
      
      // Create result image
      sprintf(filename, IMAGE_PATH, id);
      error = rox_image_rgba_read_ppm(resimg, filename);
      if (error) 
      { printf("Error %d after rox_image_rgba_read_ppm, skipping frame %d\n", error, id); continue; }
      
      for (int each_global_iteration = 0; each_global_iteration < global_iterations; ++each_global_iteration)
      {
         // Extract segments
         error = rox_caofile_edges_estimate_segments(caofile, pose, FU, FV, CU, CV, MIN_SEGMENT_SIZE);
         if (error) 
         { printf("Error %d after rox_caofile_edges_estimate_segments, skipping frame %d\n", error, id); continue; }

         error = rox_caofile_edges_get_contours(&segments, &nb_segments, caofile);
         if (error) 
         { printf("Error %d after rox_caofile_edges_get_contours, skipping frame %d\n", error, id); continue; }
                        
         // Extract ellipses
         error = rox_caofile_edges_estimate_visible_ellipses(caofile, pose, FU, FV, CU, CV, MIN_SEGMENT_SIZE);
         if (error) 
         { printf("Error %d after rox_caofile_edges_estimate_visible_ellipses, skipping frame %d\n", error, id); continue; }

         error = rox_caofile_edges_get_visible_ellipses(&ellipses, &nb_ellipses, caofile);
         if (error) 
         { printf("Error %d after rox_caofile_edges_get_visible_ellipses, skipping frame %d\n", error, id); continue; }

         // Extract cylinder
         error = rox_caofile_edges_estimate_visible_cylinders(caofile, pose, FU, FV, CU, CV, MIN_SEGMENT_SIZE);
         if (error) 
         { printf("Error %d after rox_caofile_edges_estimate_visible_cylinders, skipping frame %d\n", error, id); continue; }

         error = rox_caofile_edges_get_visible_cylinders(&cylinders, &nb_cylinders, caofile);
         if (error) 
         { printf("Error %d after rox_caofile_edges_get_visible_cylinders, skipping frame %d\n", error, id); continue; }
         
         //printf("nb_cylinders = %d \n", nb_cylinders);

         if (draw_before_odometry)
         {
            // Draw segments
            error = rox_image_rgba_draw_3d_polyline(resimg, calib, pose, (Rox_Point3D_Double_Struct *) segments, nb_segments * 2, 0xFF00FF00);
            if (error) 
            { printf("Error %d after rox_image_rgba_draw_3d_polyline() before odometry\n", error); } 
            
            // Draw ellipses
            for(Rox_Sint ide = 0; ide < nb_ellipses; ide++)
            {
               error = rox_image_rgba_draw_ellipse3d(resimg, calib, pose, ellipses[ide], 0xFF00FF00);
               if (error) 
               { printf("Error %d after rox_image_rgba_draw_ellipse3d() before odometry\n", error); } // no consequences 
            }

            // Draw cylinders
            for(Rox_Sint ide = 0; ide < nb_cylinders; ide++)
            {
               error = rox_image_rgba_draw_cylinder3d(resimg, calib, pose, cylinders[ide], 0xFF00FF00);
               if (error) 
               { printf("Error %d after rox_image_rgba_draw_cylinder3d() before odometry\n", error); } // no consequences 
            }
         }

         // Iterate sampling and vvs resolution
         for (int each_sample_iteration = 0; each_sample_iteration < sample_track_iterations; ++each_sample_iteration)
         {
            // Reset pose
            error = rox_array2d_double_copy(odometry->pose, pose);
            if (error) 
            { printf("Error %d after rox_array2d_double_copy, skipping frame %d\n", error, id); continue; }

            // Reset segments
            rox_objset_edge_segment_reset(odometry->objset_edge_segment);
            
            // Add segments
            for (Rox_Sint ids = 0; ids < nb_segments; ids++)
            {
               error = rox_odometry_cadmodel_add_segment(odometry, &segments[ids], SAMPLING_STEP);
               if (error) 
               { printf("Error %d after rox_odometry_cadmodel_add_segment, quitting frame %d\n", error, id); goto function_terminate; }
            }

            // Reset ellipses
            rox_objset_edge_ellipse_reset(odometry->objset_edge_ellipse);
            
            // Add ellipses
            for (Rox_Sint ide = 0; ide < nb_ellipses; ide++)
            {
               error = rox_odometry_cadmodel_add_ellipse(odometry, ellipses[ide], SAMPLING_STEP);
               if (error) 
               { printf("Error %d, after rox_odometry_cadmodel_add_ellipse() quitting frame %d\n", error, ide); goto function_terminate; }
            }

            // Reset cylinders
            rox_objset_edge_cylinder_reset(odometry->objset_edge_cylinder);
            
            // Add cylinders
            for (Rox_Sint idc = 0; idc < nb_cylinders; idc++)
            {
               error = rox_odometry_cadmodel_add_cylinder(odometry, cylinders[idc], SAMPLING_STEP);
               if (error) 
               { printf("Error %d in rox_odometry_cadmodel_add_cylinder, quitting frame %d\n", error, idc); goto function_terminate; }
            }
      
            // Make global odometry
            error = rox_odometry_cadmodel_make(odometry, source_gray, vvs_iterations);
            if (error) 
            { printf("Error %d in rox_odometry_cadmodel_make, skipping frame %d\n", error, id); continue; }
            
            // Get results of odometry (score and pose)
            error = rox_odometry_cadmodel_get_results(&score, pose, odometry);
            if (error) 
            { printf("Error %d in rox_odometry_cadmodel_get_results, skipping frame %d\n", error, id); continue; }      
         }

         printf("Pose estimated at image : %d\n\n", id);
         rox_array2d_double_print(pose);
         printf("Score = %f \n",score);
      }

      if (draw_after_odometry)
      {
         // Get segments after odometry
         error = rox_caofile_edges_estimate_segments(caofile, pose, FU, FV, CU, CV, MIN_SEGMENT_SIZE);
         if (error) 
         { printf("Error %d, skipping frame %d\n", error, id); continue; }

         error = rox_caofile_edges_get_contours(&segments, &nb_segments, caofile);
         if (error) 
         { printf("Error %d, skipping frame %d\n", error, id); continue; }

         // Draw model after odometry
         error = rox_image_rgba_draw_3d_polyline(resimg, calib, pose, (Rox_Point3D_Double_Struct *) segments, nb_segments * 2, 0xFF0000FF);
         if (error) 
         { printf("Error %d in drawsegments() after odometry\n", error);  }// no consequences

         // Get visible ellipses after odometry
         error = rox_caofile_edges_estimate_visible_ellipses(caofile, pose, FU, FV, CU, CV, MIN_SEGMENT_SIZE);
         if (error) 
         { printf("Error %d, skipping frame %d\n", error, id); continue; }

         error = rox_caofile_edges_get_visible_ellipses(&ellipses, &nb_ellipses, caofile);
         if (error) 
         { printf("Error %d, skipping frame %d\n", error, id); continue; }

         // draw model after odometry
         for(Rox_Sint ide = 0; ide < nb_ellipses; ide++)
         {
            error = rox_image_rgba_draw_ellipse3d(resimg, calib, pose, ellipses[ide], 0xFF0000FF);
            if (error) 
            { printf("Error %d in drawellipses() after odometry\n", error); } /*no consequences*/ 
         }

         // Get visible cylinders after odometry
         error = rox_caofile_edges_estimate_visible_cylinders(caofile, pose, FU, FV, CU, CV, MIN_SEGMENT_SIZE);
         if (error) 
         { printf("Error %d, skipping frame %d\n", error, id); continue; }

         error = rox_caofile_edges_get_visible_cylinders(&cylinders, &nb_cylinders, caofile);
         if (error)
         { printf("Error %d, skipping frame %d\n", error, id); continue; }

         // draw model after odometry
         for(Rox_Sint ide = 0; ide < nb_cylinders; ide++)
         {
            error = rox_image_rgba_draw_cylinder3d(resimg, calib, pose, cylinders[ide], 0xFF0000FF);
            if (error)
            { printf("Error %d in drawcylinders() after odometry\n", error); } /*no consequences*/ 
         }
      }

      if (draw_happened)
      {
         // Save result image to disk
         sprintf(filename, "result_mbo_odometry_cadmodel_%4.4d.ppm", id - FIRSTIMAGE);
         error = rox_image_rgba_save_ppm(filename, resimg);
      }

      // rox_memory_print_summary();
   }

function_terminate:
   rox_error_print(error);

   rox_image_rgba_del(&source_color);
   rox_image_rgba_del(&resimg);
   rox_matut3_del(&calib);
   rox_matse3_del(&pose);
   rox_image_del(&source_gray);
   
   rox_caofile_edges_del(&caofile);

   rox_odometry_cadmodel_del(&odometry);
   rox_memory_print_summary();

   return error;
}
