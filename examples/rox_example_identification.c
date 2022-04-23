//==============================================================================
//
//    OPENROX   : File rox_example_identification.c
//
//    Contents  : A simple example program for image identification.
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

// ===== MACROS       ==========================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/identification.zip

#define seq_file "../seq/plane/image_plane3D099.pgm"
#define mod_file "../seq/plane/model.pgm"
#define res_file "../res/result_identification.ppm"

// ===== VARIABLES    ==========================================================

// ===== FUNCTIONS    ==========================================================

// ===== MAIN PROGRAM ==========================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   Rox_Error error;

   Rox_Uint major = 0, minor = 0, patch = 0;
   Rox_Timer  timer = 0;
   Rox_Image  model    = 0;
   Rox_Image  current  = 0;
   Rox_Image_RGBA display = 0;
   Rox_Identification  identifier = 0;
   Rox_MatSL3 homography = 0;
   Rox_Uint red = ROX_MAKERGBA(255, 0 , 0, 255);
   Rox_Double time;
   Rox_Rect_Sint_Struct window;
   Rox_Sint cols, rows;

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Create Rox objects
   error = rox_matsl3_new(&homography);
   if(error) goto function_terminate;

   error = rox_timer_new(&timer);
   if(error) goto function_terminate;

   printf("ROX Reading the model image...\n");
   error = rox_image_new_read_pgm(&model, mod_file);
   if(error) goto function_terminate;

   error = rox_image_get_cols(&cols, model);
   if(error) goto function_terminate;

   error = rox_image_get_rows(&rows, model);
   if(error) goto function_terminate;

   printf("ROX Reading the search image...\n");
   error = rox_image_new_read_pgm(&current, seq_file);
   if(error) goto function_terminate;

   error = rox_image_rgba_new_read_pgm(&display, seq_file);
   if(error) goto function_terminate;

   error = rox_identification_new(&identifier, model);
   if(error) goto function_terminate;

   printf("ROX starting identification...\n");

   // Identification and display elapsed time
   error = rox_timer_start(timer);
   if(error) goto function_terminate;

   error = rox_identification_make(identifier, current);
   if(error) goto function_terminate;

   error = rox_timer_stop(timer);
   if(error) goto function_terminate;

   error = rox_timer_get_elapsed_ms(&time, timer);
   if(error) goto function_terminate;

   printf("Time to identification %f\n", time);

   // Draw results
   error = rox_identification_get_homography(homography, identifier);
   if(error) goto function_terminate;

   printf("Target identified, saving the result in identification.ppm \n");

   // Define model window
   window.x = 0; window.y = 0;
   window.width = cols; window.height = rows;

   error = rox_image_rgba_draw_warp_rectangle(display, homography, &window, red);
   if(error) goto function_terminate;

   error = rox_image_rgba_save_ppm(res_file, display);
   if(error) goto function_terminate;

function_terminate:

   // Delete objects and free memory
   rox_timer_del(&timer);
   rox_image_del(&model);
   rox_image_del(&current);
   rox_image_rgba_del(&display);
   rox_identification_del(&identifier);
   rox_matsl3_del(&homography);

   // Display Error
   rox_error_print(error);

   printf("\nPress Enter to end the program\n");
   getchar();

   return error;
}

