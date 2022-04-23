//==============================================================================
//
//    OPENROX   : File rox_example_odometry_single_plane_photoframe.c
//
//    Contents  : A simple example program for odometry relative 
//                to a single plane with photoframe identification.
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =======================================================

#include <api/openrox.h>
#include <stdio.h>

//=== INTERNAL MACROS    =======================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/identification.zip

// Model and database paths
#define mod_file_identify_01 "../seq/identification/photoframe/models/photoframe_1.pgm"
#define mod_file_identify_02 "../seq/identification/photoframe/models/photoframe_2.pgm"
#define mod_file_identify_03 "../seq/identification/photoframe/models/photoframe_3.pgm"
#define mod_file_identify_04 "../seq/identification/photoframe/models/photoframe_4.pgm"

#define mod_file_odometry_01 "../seq/identification/photoframe/models/photoframe_1.pgm"
#define mod_file_odometry_02 "../seq/identification/photoframe/models/photoframe_2.pgm"
#define mod_file_odometry_03 "../seq/identification/photoframe/models/photoframe_3.pgm"
#define mod_file_odometry_04 "../seq/identification/photoframe/models/photoframe_4.pgm"

// File paths
#define res_file    "../res/result_odometry_photoframe_%03d.ppm"
#define seq_file    "../seq/identification/photoframe/images/image_%03d.pgm"

// Define the number of model to be detected
#define nb_models 4
#define nb_images 400

// Define the camera intrinsic parameters
#define FU 617.55 
#define FV 616.98 
#define CU 314.23 
#define CV 246.51

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== MAIN PROGRAM =============================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   // ---------- ALLOCATE GLOBAL DATA -------------------------------------
   Rox_Error error = 0;
   Rox_Char filename[FILENAME_MAX] ;

   Rox_Double score;
   Rox_Uint major = 0, minor = 0, patch = 0;

   Rox_Char mod_file_identify[nb_models][1024] = {mod_file_identify_01, mod_file_identify_02, mod_file_identify_03, mod_file_identify_04};
   Rox_Char mod_file_odometry[nb_models][1024] = {mod_file_odometry_01, mod_file_odometry_02, mod_file_odometry_03, mod_file_odometry_04};

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------
   // Define the image for processing data
   Rox_Camera camera = NULL;

   // Define pose matrix
   Rox_MatSE3 pose[nb_models] = { NULL, NULL, NULL, NULL };

   // ----------- ALLOCATE MODEL DATA -------------------------------------

   // Real size of the target (in meters)
   Rox_Double  model_sizex[nb_models] = {0.2, 0.2, 0.2, 0.2};
   Rox_Double  model_sizey[nb_models] = {0.2, 0.2, 0.2, 0.2};
   Rox_Image model_image_identify[nb_models] = { NULL, NULL, NULL, NULL };
   Rox_Image model_image_odometry[nb_models] = { NULL, NULL, NULL, NULL };

   // Define the 2D model with texture and size in meters
   Rox_Model_Single_Plane model_identify[nb_models] = { NULL, NULL, NULL, NULL };
   Rox_Model_Single_Plane model_odometry[nb_models] = { NULL, NULL, NULL, NULL };

   // Define odometry object
   Rox_Odometry_Single_Plane odometry[nb_models] = { NULL, NULL, NULL, NULL };
   Rox_Odometry_Single_Plane_Params params[nb_models] = { NULL, NULL, NULL, NULL };
   Rox_Sint tracked[nb_models] = {0, 0, 0, 0};

   // Define rox photoframe identification object
   Rox_Ident_PhotoFrame_SE3 ident = NULL;

   // Display
   Rox_Image_RGBA image_rgba = NULL;
   Rox_Uint color[nb_models] = {ROX_MAKERGBA(255, 0, 0, 255), ROX_MAKERGBA(0, 255, 0, 255), ROX_MAKERGBA(0, 255, 255, 255), ROX_MAKERGBA(0, 0, 255, 255)};
   Rox_Matrix K = NULL;
   
   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Define database objects
   error = rox_ident_photoframe_se3_new(&ident, 640, 480);
   if (error) goto function_terminate;

   // Set the type of photoframe
   error = rox_ident_photoframe_se3_set_type(ident, 1);
   if (error) goto function_terminate;
   
   printf("Reading the photoframes \n");
   // Initialize the odometry and ident
   for(Rox_Uint k = 0; k < nb_models; k++)
   { 
      // Crete a pose per model
      error = rox_matse3_new(&pose[k]);
      if (error) goto function_terminate;
     
      // Load model for identification
      error = rox_image_new_read_pgm(&model_image_identify[k], mod_file_identify[k]);
      if (error) goto function_terminate;
      
      // Load model for odometry
      error = rox_image_new_read_pgm(&model_image_odometry[k], mod_file_odometry[k]);
      if (error) goto function_terminate;
      
      // Define the 2D model for identify with texture and size in meters
      error = rox_model_single_plane_new(&model_identify[k]);
      if (error) goto function_terminate;

      error = rox_model_single_plane_set_template_xright_ydown ( model_identify[k], model_image_identify[k], model_sizex[k], model_sizey[k]);
      if (error) goto function_terminate;

      // Add photoframe
      error = rox_ident_photoframe_se3_addframe_model (ident, model_identify[k], 20);
      if (error) goto function_terminate;

      // Define the 2D model for odometry with texture and size in meters
      error = rox_model_single_plane_new(&model_odometry[k]);
      if (error) goto function_terminate;

      error = rox_model_single_plane_set_template_xright_ydown ( model_odometry[k], model_image_odometry[k], model_sizex[k], model_sizey[k]);
      if (error) goto function_terminate;
      
      error = rox_odometry_single_plane_params_new(&params[k]);
      if (error) goto function_terminate;

      // Create odometry
      error = rox_odometry_single_plane_new(&odometry[k], params[k], model_odometry[k]);
      if (error) goto function_terminate;
   }

   // Define the camera containing the first image
   sprintf(filename, seq_file, 0);
   error = rox_camera_new_read_pgm(&camera, filename);
   if (error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, FU, FV, CU, CV);
   if (error) goto function_terminate;

   error = rox_matrix_new(&K, 3, 3);
   if (error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters(K, camera);
   if (error) goto function_terminate;

   error = rox_image_rgba_new_read_pgm(&image_rgba, filename);
   if (error) goto function_terminate;

   for (Rox_Uint i = 0; i < nb_images; i++)
   {
     sprintf(filename, seq_file, i);
      printf("Reading %s\n", filename);

      error = rox_camera_read_pgm(camera, filename);
      if (error) goto function_terminate;

      error = rox_image_rgba_read_pgm(image_rgba, filename);
      if (error) goto function_terminate;

      // Make the detection using the current image
      error = rox_ident_photoframe_se3_make(ident, camera);
      if (error) goto function_terminate;

      for(Rox_Uint k = 0; k < nb_models; k++)
      {
         // Check odometry status
         if(tracked[k] < 1)
         {
             Rox_Sint identified = 0;
             Rox_Double score = 0.0;
             
             error = rox_ident_photoframe_se3_get_result(&identified, &score, pose[k], ident, k);
             if (error) goto function_terminate;

             if(identified)
             {
                 error = rox_odometry_single_plane_set_pose(odometry[k], pose[k]);
                 if (error) goto function_terminate;

                 error = rox_odometry_single_plane_make(odometry[k], camera);
                 if(error)
                 {
                     tracked[k] = 0;
                 }
                 else
                 {
                     tracked[k] = 1;
                 }
             }
         }
         else
         {
             // Make tracking
             error = rox_odometry_single_plane_make(odometry[k], camera);
             if(error)
             {
                 tracked[k] = 0;
             }
             else
             {
                 tracked[k] = 1;
             }
         }

         // Draw results
         if(tracked[k] == 1)
         {
            error = rox_odometry_single_plane_get_score(&score, odometry[k]);
            if (error) goto function_terminate;

            printf("Photoframe %d found, odometry score = %f \n", k+1, score);

            error = rox_odometry_single_plane_get_pose(pose[k], odometry[k]);
            if (error) goto function_terminate;

            error = rox_image_rgba_draw_projection_model_single_plane(image_rgba, K, pose[k], model_odometry[k], color[k]);
            if (error) goto function_terminate;

            error = rox_image_rgba_draw_projection_frame(image_rgba, K, pose[k], .1);
            if (error) goto function_terminate;
         }
      }

      // Save results
      sprintf(filename, res_file, i);
      error = rox_image_rgba_save_ppm(filename, image_rgba);
      if (error) goto function_terminate;
   }

function_terminate:

   // Delete objects and free memory
   if(timer) rox_timer_del(&timer);
   rox_camera_del(&camera);
   rox_matrix_del(&K);
   rox_image_rgba_del(&image_rgba);
   rox_ident_photoframe_se3_del(&ident);

   for(Rox_Uint k = 0; k < nb_models; k++)
   {
       rox_matse3_del(&pose[k]);
       rox_image_del(&model_image_identify[k]);
       rox_image_del(&model_image_odometry[k]);
       rox_model_single_plane_del(&model_identify[k]);
       rox_model_single_plane_del(&model_odometry[k]);
       rox_odometry_single_plane_params_del(&params[k]);
       rox_odometry_single_plane_del(&odometry[k]);
   }

   // Display Error
   rox_error_print(error);

   printf("\n\n Press return to end the program");
   getchar();

   return error;
}
