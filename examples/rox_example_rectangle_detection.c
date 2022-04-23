//==============================================================================
//
//    OPENROX   : File rox_example_rectangle_detection.c
//
//    Contents  : A simple example program for rectangle detection.
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

// ===== MACROS       ======================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/rectangle_detection.zip

#define img_file "../seq/rectangle_detection/img_rectangle_detection_01.pgm"

#define fu 1200
#define fv 1200
#define cu  816
#define cv  612

// ===== VARIABLES    ======================================================

// ===== FUNCTIONS    ======================================================

// ===== MAIN PROGRAM ======================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   Rox_Error error = 0;

   Rox_Image_RGBA  image_rgba = NULL;
   Rox_Image  image = NULL;
   Rox_Timer  timer = NULL;
   Rox_Double time;
   Rox_Sint rows = 0;
   Rox_Sint cols = 0;
   Rox_Double ratio_rect = 29.7/21.0;
   Rox_Sint is_detected = 0;
   Rox_Point2D_Double_Struct pts_rect[4];
   Rox_Uint mean_min = 220; 
   Rox_Uint variance_max = 30;
   Rox_Uint major = 0, minor = 0, patch = 0;

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Create Rox objects
   error = rox_timer_new(&timer);
   if(error) goto function_terminate;

   printf("ROX Reading the image ...\n");
   error = rox_image_new_read_pgm(&image, img_file);
   if(error) goto function_terminate;
   
   error = rox_image_rgba_new_read_pgm(&image_rgba, img_file);
   if(error) goto function_terminate;
   
   error = rox_image_get_rows(&rows, image);
   if(error) goto function_terminate;
    
   error = rox_image_get_cols(&cols, image);
   if(error) goto function_terminate;
 
   error = rox_rectangle_detector_init(cols, rows, fu, fv, cu, cv);
   if (error) goto function_terminate;
   
   printf("ROX starts rectangle detection...\n");
   error = rox_timer_start(timer);
   if(error) goto function_terminate;

   error = rox_rectangle_detector_process(&is_detected, pts_rect, image, ratio_rect, 0,  mean_min, variance_max);
   if (error) goto function_terminate;
         
   error = rox_timer_stop(timer);
   if(error) goto function_terminate;

   error = rox_timer_get_elapsed_ms(&time, timer);
   if(error) goto function_terminate;

   printf("Time needed to detect rectangle %f\n", time);
   
   if(is_detected)
   {
      printf("detected rectangle:\n");
      printf("%f %f %f %f\n", pts_rect[0].u, pts_rect[1].u, pts_rect[2].u, pts_rect[3].u); 
      printf("%f %f %f %f\n", pts_rect[0].v, pts_rect[1].v, pts_rect[2].v, pts_rect[3].v); 
               
      error = rox_image_rgba_draw_2d_polygon(image_rgba, pts_rect, 4, ROX_MAKERGBA(255, 0, 0, 255)); // display in red
      if (error) goto function_terminate;
   }
         
   // Save results
   error = rox_image_rgba_save_ppm("../res/result_rectangle_detection.ppm", image_rgba);
   if(error) goto function_terminate;
         
function_terminate:

   // Delete objects and free memory
   rox_rectangle_detector_terminate();
   
   rox_timer_del(&timer);
   rox_image_del(&image);
   rox_image_rgba_del(&image_rgba);

   // Display Error
   rox_error_print(error);

   printf("\nPress Enter to end the program\n");
   getchar();

   return error;
}

