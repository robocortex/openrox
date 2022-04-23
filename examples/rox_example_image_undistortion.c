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

//=== HEADERS      =============================================================

#include <api/openrox.h>
#include <stdio.h>

//=== MACROS       =============================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/calibration.zip
#define seq_file "../seq/calibration/distorted/Image%d.pgm"
#define res_file "../res/result_undistorted_image_%d.pgm"

//=== VARIABLES    =============================================================

//=== FUNCTIONS    =============================================================

//=== MAIN PROGRAM =============================================================

Rox_Sint main (Rox_Sint argc, Rox_Char *argv[])
{
   Rox_Error error;
   Rox_Char filename[1024];

   Rox_Camera camera = 0;
   Rox_Matrix K = 0;
   Rox_Matrix radial = 0;
   Rox_Matrix tangential = 0;
   Rox_Uint major = 0, minor = 0, patch = 0;

   Rox_Uint id;
   
   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Create ROX objects
   error = rox_matrix_new(&K, 3, 3);
   if(error) goto function_terminate;

   error = rox_matrix_new(&radial, 3, 1);
   if(error) goto function_terminate;

   error = rox_matrix_new(&tangential, 2, 1);
   if(error) goto function_terminate;

   // Set distortion parameters
   error = rox_matrix_set_value(radial, 0, 0, -0.26425);
   if(error) goto function_terminate;
   error = rox_matrix_set_value(radial, 1, 0, 0.22645);
   if(error) goto function_terminate;
   error = rox_matrix_set_value(radial, 2, 0, 0.0);
   if(error) goto function_terminate;

   error = rox_matrix_set_value(tangential, 0, 0, 0.0020);
   if(error) goto function_terminate;
   error = rox_matrix_set_value(tangential, 1, 0, 0.0023);
   if(error) goto function_terminate;

   // Define the camera
   error = rox_camera_new(&camera, 640, 480);
   if(error) goto function_terminate;

   // Set intrinsic parameters
   error = rox_camera_set_pinhole_params(camera, 671.12759, 680.77186, 319.50, 239.5);
   if(error) goto function_terminate;

   // Get a copy of intrinsic parameters
   error = rox_camera_get_intrinsic_parameters(K, camera);
   if(error) goto function_terminate;

   // Prepare calibration parameters once for all images of the same camera*/
   error = rox_camera_set_params_undistort(camera, K, radial, tangential);
   if(error) goto function_terminate;

   for(id = 1; id <= 20; id++)
   {
      printf("Undistorting image %d \n", id);
      sprintf(filename, seq_file, id);

      error = rox_camera_read_pgm(camera, filename);
      if(error) goto function_terminate;

      error = rox_camera_undistort_image(camera);
      if(error) goto function_terminate;

      sprintf(filename, res_file, id);
      error = rox_camera_save_pgm(filename, camera);
      if(error) goto function_terminate;
   }

function_terminate:

   // Delete objects and free memory
   rox_camera_del(&camera);
   rox_matrix_del(&K);
   rox_matrix_del(&radial);
   rox_matrix_del(&tangential);

   // Display Error
   rox_error_print(error);

   printf("\nPress Enter to end the program\n");
   getchar();

   return error;
}

