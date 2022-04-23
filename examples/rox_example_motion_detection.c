//==============================================================================
//
//    OPENROX   : File rox_example_motion_detection.c
//
//    Contents  : Test for motion detection module implementation
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

// ===== INTERNAL MACROS    =================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/motion_detection.zip

#define seq_file "../seq/motion_detection/img_motion_detection_%02d.ppm"
#define res_file "../res/result_motion_detection_%02d.ppm" 

#define ROX_MAXFILENAME 1024

// ===== INTERNAL TYPESDEFS =================================================

// ===== INTERNAL DATATYPES =================================================

// ===== INTERNAL VARIABLES =================================================

// ===== INTERNAL FUNCTDEFS =================================================

// ===== INTERNAL FUNCTIONS =================================================

// ===== EXPORTED FUNCTIONS =================================================

Rox_Sint main(Rox_Void)
{
   Rox_Error error = 0;
   Rox_Char filename[ROX_MAXFILENAME];

   Rox_Sint i = 0;
   Rox_Sint nbi = 100;
   Rox_Sint ini =   0;
   Rox_Uint major = 0, minor = 0, patch = 0;

   // Define a timer to measure computation time 
   Rox_Timer timer = 0; 
   Rox_Double time_ms = 0.0;

   // Define the motion detection object
   Rox_Detection_Motion detection = 0;

   // Define the motion detection object
   Rox_Image image = 0;
   
   // Define the image in which dispaly the detection results
   Rox_Image_RGBA image_rgba = 0;

   // Define the window in which perform the detection
   Rox_Rect_Sint_Struct window_detection;
   
   // Define the list of bounding boxes around the moving objects
   Rox_DynVec_Rect_Sint window_list_moving_objects = 0;

   Rox_Uint red = ROX_MAKERGBA(255,0,0,255);
   Rox_Uint green = ROX_MAKERGBA(0,255,0,255);

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Define a timer to measure detection delay 
   error = rox_timer_new(&timer);
   if(error!=0) goto function_terminate;

   // Read the model image from disk for detection 
   sprintf(filename, seq_file, ini);
   error = rox_image_new_read_ppm(&image, filename);
   if(error!=0) goto function_terminate;

   // Read the model image from disk for display 
   error = rox_image_rgba_new_read_ppm(&image_rgba, filename);
   if(error!=0) goto function_terminate;

   // Define a motion detection object for a given image model
   error = rox_detection_motion_new(&detection, image);
   if(error!=0) goto function_terminate;

   // Define the approximated size of the object to be detected 
   rox_detection_motion_set_bandwidth(detection, 80, 80);

   // Define a window to set the rectangular region of detection 
   window_detection.x = 100;
   window_detection.y = 100;
   window_detection.width = 300;
   window_detection.height = 200;
   
   // Set a rectangular region of interest for the detection 
   error = rox_detection_motion_set_imask_window(detection, &window_detection);
   if(error!=0) goto function_terminate;

   // Allocate list of windows
   error = rox_dynvec_rect_sint_new(&window_list_moving_objects, 10);
   if(error!=0) goto function_terminate;

   for (i=ini; i<ini+nbi; ++i)
   {
      // Read current image from disk for detection 
      sprintf(filename, seq_file, i);
      rox_image_read_ppm(image, filename);
      rox_image_rgba_read_ppm(image_rgba, filename);
      printf("reading image %s\n", filename);

      // Start timer
      rox_timer_start(timer);

      // Perform motion detection
      error = rox_detection_motion_set_window_list(window_list_moving_objects, detection, image);
      if(error!=0) goto function_terminate;

      error = rox_timer_stop(timer);
      if(error!=0) goto function_terminate;
      
      error = rox_timer_get_elapsed_ms(&time_ms, timer);
      if(error!=0) goto function_terminate;
     
      // Display time elapsed for detection
      printf("Time to detect = %f ms \n", time_ms);

      // Draw the region of detection in green
      error = rox_image_rgba_draw_rectangle(image_rgba, &window_detection, green);
      if(error!=0) goto function_terminate;

      // Draw detection results in red on the image
      error = rox_image_rgba_draw_dynvec_rectangle(image_rgba, window_list_moving_objects, red);
      if(error!=0) goto function_terminate;   

      // Save results on disk
      sprintf (filename, res_file, i);
      error = rox_image_rgba_save_ppm(filename, image_rgba);
      if(error!=0) goto function_terminate;
   }

   // Delete windows list
   rox_dynvec_rect_sint_del(&window_list_moving_objects);
      
   // ----------- FREE MEMORY ----------------------------------------------

function_terminate:

   rox_error_print(error);

   rox_timer_del(&timer);
   rox_image_del(&image);
   rox_image_rgba_del(&image_rgba);
   rox_detection_motion_del(&detection);

   printf("\n\n Press return to end the program");
   getchar();
   return(1);
}
