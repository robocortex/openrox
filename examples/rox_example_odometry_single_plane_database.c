//==============================================================================
//
//    OPENROX   : File rox_example_odometry_single_plane_database.c
//
//    Contents  : A simple example program for single plane odometry
//                with identification from a database of images
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== HEADERS   ================================================================

#include <api/openrox.h>
#include <stdio.h>
#include <stdlib.h>

//=== MACROS    ================================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/identification.zip

// Model and database paths
#define mod_file_01 "../seq/identification/database/models/toy_128x128.pgm"
#define rdi_file_01 "../seq/identification/database/models/toy_512x512.rdi"

#define mod_file_02 "../seq/identification/database/models/gormiti_128x178.pgm"
#define rdi_file_02 "../seq/identification/database/models/gormiti_517x719.rdi"

// File paths
#define res_file    "../res/result_odometry_database_%03d.ppm"
#define seq_file    "../seq/identification/database/images/toytest%03d.pgm"

// Define the number of model to be detected
#define nb_models 2
#define nb_images 44

// Define the camera intrinsic parameters
#define PX 559.279
#define PY 559.279
#define U0 329.890
#define V0 242.164

//=== VARIABLES ================================================================
      
//=== FUNCTIONS ================================================================

//=== MAIN PROGRAM =============================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   // ---------- ALLOCATE GLOBAL DATA -------------------------------------
   Rox_Error error;
   Rox_Char filename[1024] ;

   Rox_Double score = 0.0;
   Rox_Uint major = 0, minor = 0, patch = 0;

   Rox_Char mod_file[nb_models][1024] = {mod_file_01, mod_file_02};
   Rox_Char rdi_file[nb_models][1024] = {rdi_file_01, rdi_file_02};

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------
   // Define the image for processing data
   Rox_Camera camera = NULL;

   // Define pose matrix
   Rox_MatSE3 pose  = NULL;

   // ----------- ALLOCATE MODEL DATA -------------------------------------

   // Real size of the target (in meters)
   Rox_Double  model_sizex[nb_models] = {0.2, 0.14};
   Rox_Double  model_sizey[nb_models] = {0.2, 0.2};
   Rox_Image model_image[nb_models];

   // Define the 2D model with texture and size in meters
   Rox_Model_Single_Plane model[nb_models];

   // Define odometry object
   Rox_Odometry_Single_Plane odometry[nb_models];
   Rox_Odometry_Single_Plane_Params params[nb_models];

   // Define rox database identification object
   Rox_Ident_Database_SE3 ident = NULL;
   Rox_Database_Item item = NULL;
   Rox_Database database = NULL;

   // Display
   Rox_Image_RGBA image_rgba = NULL;
   Rox_Uint color[nb_models] = {ROX_MAKERGBA(255, 0 , 0, 255), ROX_MAKERGBA(0, 255 , 0, 255)};
   Rox_Matrix K = NULL;

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Define database objects
   error = rox_ident_database_se3_new(&ident, nb_models);
   if(error) goto function_terminate;

   error = rox_database_new(&database);
   if(error) goto function_terminate;

   error = rox_database_item_new(&item);
   if(error) goto function_terminate;

   error = rox_matse3_new(&pose);
   if(error) goto function_terminate;

   // Initialize the odometry and ident
   for(Rox_Sint k = 0; k < nb_models; k++)
   {
      // Load model
      error = rox_image_new_read_pgm(&model_image[k], mod_file[k]);
      if(error) goto function_terminate;
   
      // Define the 2D model with texture and size in meters
      error = rox_model_single_plane_new(&model[k]);
      if(error) goto function_terminate;
   
      error = rox_model_single_plane_set_template_xright_ydown ( model[k], model_image[k], model_sizex[k], model_sizey[k] );
      if(error) goto function_terminate;

      // Load db item
      error = rox_database_item_load ( item, rdi_file[k] );
      if(error) goto function_terminate;
 
      // Add item to the database
      error = rox_database_add_item ( database, item, model_sizex[k], model_sizey[k] );
      if(error) goto function_terminate;
   
      error = rox_odometry_single_plane_params_new( &params[k] );
      if(error) goto function_terminate;
   
      // Create odometry
      error = rox_odometry_single_plane_new(&odometry[k], params[k], model[k]);
      if(error) goto function_terminate;
   }

   // Compile database
   error = rox_database_compile(database);
   if(error) goto function_terminate;
   
   error = rox_ident_database_se3_set_database(ident, database);
   if(error) goto function_terminate;

   // Define the camera containing the first image
   sprintf(filename, seq_file, 0);
   error = rox_camera_new_read_pgm(&camera, filename);
   if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, PX, PY, U0, V0);
   if(error) goto function_terminate;

   error = rox_matrix_new(&K, 3, 3);
   if(error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters(K, camera);
   if(error) goto function_terminate;

   error = rox_image_rgba_new_read_pgm(&image_rgba, filename);
   if(error) goto function_terminate;

   for (Rox_Sint i = 0; i < nb_images; i++)
   {
      Rox_Database_Features features = NULL;
      Rox_Point2D_Double_Struct * points = NULL;
      Rox_Uint nbpts = 0;

      sprintf(filename, seq_file, i);
      printf("Reading %s\n", filename);

      error = rox_camera_read_pgm(camera, filename);
      if(error) goto function_terminate;

      error = rox_image_rgba_read_pgm(image_rgba, filename);
      if(error) goto function_terminate;

      // Make the detection using the current image
      error = rox_ident_database_se3_make(ident, camera);
      if(error) goto function_terminate;


      error = rox_database_features_new(&features);
      if(error) goto function_terminate;

      error = rox_ident_database_se3_extract_features(features, ident, camera);
      if(error) goto function_terminate;

      error = rox_database_features_get_points_size(&nbpts, features);
      if(error) goto function_terminate;

      points = (Rox_Point2D_Double_Struct *) malloc(sizeof(*points) * nbpts);

      error = rox_database_features_get_points_copy(points, nbpts, features);
      if(error) goto function_terminate;

      for(Rox_Sint k = 0; k < nb_models; k++)
      {
         Rox_Sint is_identified = 0;

         error = rox_ident_database_se3_getresult(&is_identified, pose, ident, k);
         if(error) goto function_terminate;

         if(is_identified)
         {
            error = rox_odometry_single_plane_set_pose(odometry[k], pose);
            if(error) goto function_terminate;

            error = rox_odometry_single_plane_make(odometry[k], camera);
            if(!error)
            {
               // Draw results
               error = rox_odometry_single_plane_get_score(&score, odometry[k]);
               if(error) goto function_terminate;

               printf("Template %d found, odometry score = %f \n", k+1, score);

               error = rox_odometry_single_plane_get_pose(pose, odometry[k]);
               if(error) goto function_terminate;
            
               error = rox_image_rgba_draw_projection_model_single_plane(image_rgba, K, pose, model[k], color[k]);
               if(error) goto function_terminate;

               error = rox_image_rgba_draw_projection_frame(image_rgba, K, pose, 0.1);
               if(error) goto function_terminate;
             }
         }
      }

      // Write points      
      error = rox_image_rgba_draw_2d_points(image_rgba, points, nbpts, ROX_MAKERGBA(255, 0, 0, 255));
      if(error) goto function_terminate;

      // Save results
      sprintf(filename, res_file, i);
      error = rox_image_rgba_save_ppm(filename, image_rgba);
      if(error) goto function_terminate;

      rox_database_features_del(&features);
      free(points);
   }

function_terminate:

   // Delete objects and free memory
   rox_timer_del(&timer);
   rox_camera_del(&camera);
   rox_matse3_del(&pose);
   rox_matrix_del(&K);
   rox_image_rgba_del(&image_rgba);
   rox_ident_database_se3_del(&ident);
   rox_database_item_del(&item);
   rox_database_del(&database);
   
   for(Rox_Sint k = 0; k < nb_models; k++)
   {
       rox_image_del(&model_image[k]);
       rox_model_single_plane_del(&model[k]);
       rox_odometry_single_plane_params_del(&params[k]);
       rox_odometry_single_plane_del(&odometry[k]);
   }

   // Display Error
   rox_error_print(error);

   printf("\nPress Enter to end the program\n");
   getchar();

   return error;
}

