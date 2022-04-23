//==============================================================================
//
//    OPENROX   : File rox_example_plane_detection_from_rectangles.c
//
//    Contents  : An example for plane detection from two rectangles
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

// ===== HEADERS      ==========================================================

#include <api/openrox.h>
#include <stdio.h>

// ===== MACROS       ==========================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/rectangle_detection.zip

#define img1_file "../seq/rectangle_detection/img_rectangle_detection_01.pgm"
#define img2_file "../seq/rectangle_detection/img_rectangle_detection_02.pgm"

#define fu 1200
#define fv 1200
#define cu  816
#define cv  612

// ===== VARIABLES    ==========================================================

// ===== FUNCTIONS    ==========================================================

// ===== MAIN PROGRAM ==========================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   Rox_Error error = 0;
   
   Rox_Image  image1 = NULL;
   Rox_Image  image2 = NULL;
   
   Rox_Imask  imask1 = NULL;
   Rox_Imask  imask2 = NULL;
   
   Rox_Sint rows = 0;
   Rox_Sint cols = 0;
   
   Rox_Double ratio_rect = 29.7/21.0;
   
   Rox_Sint is_detected1 = 0;
   Rox_Sint is_detected2 = 0;
    
   Rox_Point2D_Double_Struct pts_rect_1[4];
   Rox_Point2D_Double_Struct pts_rect_2[4];
   
   Rox_Sint mean_min = 220; 
   Rox_Sint variance_max = 30;

   Rox_Uint major = 0, minor = 0, patch = 0;

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Create Rox objects
   printf("ROX Reading the images ...\n");
   error = rox_image_new_read_pgm(&image1, img1_file);
   if(error) goto function_terminate;
   
   error = rox_image_new_read_pgm(&image2, img2_file);
   if(error) goto function_terminate;
   
   error = rox_image_get_rows(&rows, image1);
   if(error) goto function_terminate;
    
   error = rox_image_get_cols(&cols, image1);
   if(error) goto function_terminate;
 
   error = rox_imask_new(&imask1, cols, rows);
   if (error) goto function_terminate;
   
   error = rox_imask_new(&imask2, cols, rows);
   if (error) goto function_terminate;
      
   error = rox_rectangle_detector_init(cols, rows, fu, fv, cu, cv);
   if (error) goto function_terminate;
   
   printf("ROX starts rectangles detection...\n");
   error = rox_rectangle_detector_process(&is_detected1, pts_rect_1, image1, ratio_rect, 0,  mean_min, variance_max);
   if (error) goto function_terminate;
           
   error = rox_rectangle_detector_process(&is_detected2, pts_rect_2, image2, ratio_rect, 0,  mean_min, variance_max);
   if (error) goto function_terminate;
       
   printf("ROX starts plane detection...\n");
   if((is_detected1 == 1) && (is_detected1 == 1))
   {
      error = rox_plane_detector_from_rectangles(imask1, imask2, pts_rect_1, pts_rect_2, image1, image2, 20);
      if (error) goto function_terminate;

      // Save results
      error = rox_imask_save_pgm("../res/result_plane_detection1.pgm", imask1);
      if (error) goto function_terminate;
    
      error = rox_imask_save_pgm("../res/result_plane_detection2.pgm", imask2);
      if (error) goto function_terminate;
   }
        
function_terminate:

   // Delete objects and free memory
   rox_rectangle_detector_terminate();
   
   rox_image_del(&image1);
   rox_image_del(&image2);
   
   rox_imask_del(&imask1);
   rox_imask_del(&imask2);
   
   // Display Error
   rox_error_print(error);

   printf("\nPress Enter to end the program\n");
   getchar();

   return error;
}

