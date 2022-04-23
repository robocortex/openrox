//==============================================================================
//
//    OPENROX   : File rox_example_tracking_photoframe.c
//
//    Contents  : A simple example program for tracking relative 
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
#define mod_file_01 "../seq/identification/photoframe/models/photoframe_1.pgm"
#define mod_file_02 "../seq/identification/photoframe/models/photoframe_2.pgm"
#define mod_file_03 "../seq/identification/photoframe/models/photoframe_3.pgm"
#define mod_file_04 "../seq/identification/photoframe/models/photoframe_4.pgm"

// File paths
#define res_file    "../res/result_tracking_photoframe_%03d.ppm"
#define seq_file    "../seq/identification/photoframe/images/image_%03d.pgm"

// Define the number of model to be detected
#define nb_models 4
#define nb_images 400

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== MAIN PROGRAM =============================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   // ---------- ALLOCATE GLOBAL DATA -------------------------------------
   Rox_Error error = 0;
   Rox_Char filename[1024] ;

   Rox_Sint cols = 0, rows = 0;
   Rox_Double score = 0.0;
   Rox_Uint major = 0, minor = 0, patch = 0;

   Rox_Char mod_file[nb_models][1024] = {mod_file_01, mod_file_02, mod_file_03, mod_file_04};

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------
   
   // Define the image for processing data
   Rox_Image image = NULL;

   // Define pose matrix
   Rox_MatSL3 homography[nb_models] = {NULL, NULL, NULL, NULL};

   // ----------- ALLOCATE MODEL DATA -------------------------------------

   // Define the model images
   Rox_Image model_image[nb_models] = {NULL, NULL, NULL, NULL};

   // Define tracking object
   Rox_Tracking tracking[nb_models];
   Rox_Tracking_Params params[nb_models];
   Rox_Sint tracked[nb_models] = {0, 0, 0, 0};

   // Define rox photoframe identification object
   Rox_Ident_PhotoFrame_SL3 ident = 0;

   // Display
   Rox_Image_RGBA image_rgba = NULL;
   Rox_Uint color[nb_models] = {ROX_MAKERGBA(255, 0, 0, 255), ROX_MAKERGBA(0, 255, 0, 255), ROX_MAKERGBA(0, 255, 255, 255), ROX_MAKERGBA(0, 0, 255, 255)};
   Rox_Rect_Sint_Struct rectangle[nb_models];

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Define database objects
   error = rox_ident_photoframe_sl3_new(&ident, 640, 480);
   if(error) goto function_terminate;

   printf("Reading the photoframes \n");
   // Initialize the odometry and ident
   for(Rox_Sint k = 0; k < nb_models; k++)
   { 
      // Crete a pose per model
      error = rox_matsl3_new(&homography[k]);
      if(error) goto function_terminate;

      // Load model
      error = rox_image_new_read_pgm(&model_image[k], mod_file[k]);
      if(error) goto function_terminate;
   
      error = rox_image_get_cols(&cols, model_image[k]); 
      if(error) goto function_terminate;
      error = rox_image_get_rows(&rows, model_image[k]); 
      if(error) goto function_terminate;
   
      rectangle[k].x = 0;
      rectangle[k].y = 0;
      rectangle[k].width = cols;
      rectangle[k].height = rows;
   
      // Add photoframe
      error = rox_ident_photoframe_sl3_addframe(ident, model_image[k], 20);
      if(error) goto function_terminate;

      error = rox_tracking_params_new(&params[k]);
      if(error) goto function_terminate;

      // Create odometry
      error = rox_tracking_new(&tracking[k], params[k], model_image[k]);
      if(error) goto function_terminate;
   }

   // Define the first image
   sprintf(filename, seq_file, 0);
   error = rox_image_new_read_pgm(&image, filename);
   if(error) goto function_terminate;

   error = rox_image_rgba_new_read_pgm(&image_rgba, filename);
   if(error) goto function_terminate;

   for (Rox_Sint i = 0; i < nb_images; i++)
   {
      sprintf(filename, seq_file, i);
      printf("Reading %s\n", filename);

      error = rox_image_read_pgm(image, filename);
      if(error) goto function_terminate;

      error = rox_image_rgba_read_pgm(image_rgba, filename);
      if(error) goto function_terminate;

      // Make the detection using the current image
      error = rox_ident_photoframe_sl3_make(ident, image);
      if(error) goto function_terminate;

      for(Rox_Sint k = 0; k < nb_models; k++)
      {
         // Check odometry status
         if(tracked[k] < 1)
         {
            Rox_Sint identified = 0;

            error = rox_ident_photoframe_sl3_getresult(&identified, homography[k], ident, k);
            if(error) goto function_terminate;

            if(identified)
            {
               error = rox_tracking_set_homography(tracking[k], homography[k]);
               if(error) goto function_terminate;

               error = rox_tracking_make(tracking[k], image);
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
            error = rox_tracking_make(tracking[k], image);
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
            error = rox_tracking_get_score(&score, tracking[k]);
            if(error) goto function_terminate;

            printf("Photoframe %d found, tracking score = %f \n", k+1, score);

            error = rox_tracking_get_homography(homography[k], tracking[k]);
            if(error) goto function_terminate;

            error = rox_image_rgba_draw_warp_rectangle(image_rgba, homography[k], &rectangle[k], color[k]);
            if(error) goto function_terminate;
         }
      }

      // Save results
      sprintf(filename, res_file, i);
      error = rox_image_rgba_save_ppm(filename, image_rgba);
      if(error) goto function_terminate;
   }

function_terminate:

   // Delete objects and free memory
   rox_timer_del(&timer);
   rox_image_del(&image);
   rox_image_rgba_del(&image_rgba);
   rox_ident_photoframe_sl3_del(&ident);

   for(Rox_Sint k = 0; k < nb_models; k++)
   {
      rox_matsl3_del(&homography[k]);
      rox_image_del(&model_image[k]);
      rox_tracking_params_del(&params[k]);
      rox_tracking_del(&tracking[k]);
   }

   // Display Error
   rox_error_print(error);

   printf("\n\n Press return to end the program");
   getchar();

   return error;
}
