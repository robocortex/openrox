//==============================================================================
//
//    OPENROX   : File rox_example_camera_calibration.c
//
//    Contents  : A simple example program for camera calibration.
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

//=== MACROS    ================================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/calibration.zip

#define seq_file "../seq/calibration/texture/image_plane3D_calib_%02d.pgm"
#define mod_file "../seq/calibration/texture/model_256x256.pgm"

//=== VARIABLES ================================================================

Rox_Sint i = 0;

//=== FUNCTIONS ================================================================

// This function is used to simulate an image grabber
Rox_Error grab_image_buffer(Rox_Uchar * buffer, Rox_Sint * cols, Rox_Sint * rows);

// This function is used to simulate an image reader
Rox_Error read_image_model(Rox_Uchar * buffer, Rox_Sint * cols, Rox_Sint * rows);

Rox_Error grab_image_buffer(Rox_Uchar * buffer, Rox_Sint * cols, Rox_Sint * rows)
{
   Rox_Char filename[1024];
   Rox_Image image = 0;
   Rox_Error error;

   sprintf(filename, seq_file, i+1);

   error = rox_image_new_read_pgm(&image, filename); 
   if(error) goto function_terminate;
   error = rox_image_get_data(buffer, image); 
   if(error) goto function_terminate;
   error = rox_image_get_cols(cols, image); 
   if(error) goto function_terminate;
   error = rox_image_get_rows(rows, image); 
   if(error) goto function_terminate;
   
   printf ("ROX simulation of grabbing image %d\n", i);

function_terminate:
   rox_image_del(&image);
   return error;
}

Rox_Error read_image_model(Rox_Uchar * buffer, Rox_Sint * cols, Rox_Sint * rows)
{
   Rox_Image image = 0;
   Rox_Error error;

   error = rox_image_new_read_pgm(&image, mod_file); 
   if(error) goto function_terminate;
   error = rox_image_get_data(buffer, image); 
   if(error) goto function_terminate;
   error = rox_image_get_cols(cols, image); 
   if(error) goto function_terminate;
   error = rox_image_get_rows(rows, image); 
   if(error) goto function_terminate;
   printf ("ROX simulation of reading model\n");

function_terminate:
   rox_image_del(&image);
   return error;
}

//=== MAIN PROGRAM =====================================================

Rox_Sint main (Rox_Void)
{
   Rox_Error error;
   Rox_Sint nbimg = 10;
   Rox_Sint method = 5; // 5 paramaters fu, fv, cu, cv, s
   Rox_Uint major = 0, minor = 0, patch = 0;

   // Define the result object
   Rox_Matrix Ke = 0;

   // Allocate the buffer containing the image data
   // In this example the maximum resolution is 640 x 480 pixels
   Rox_Uchar buffer[640*480];

   // Define objects for the current image
   Rox_Sint rows, cols;
   Rox_Image current = 0;

   // Define the 2D model
   Rox_Model_Single_Plane model = NULL;
   Rox_Image Im = NULL;
   Rox_Double size_x = 0.279;
   Rox_Double size_y = 0.280;

   // Define the calibration object
   Rox_Camera_Calibration calibration = 0;

   // Define the camera object
   Rox_Camera camera = 0;
   
   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Read the image model from disk
   // Replace this function with your own function to fill the buffer
   error = read_image_model(buffer, &cols, &rows); 
   if(error) goto function_terminate;

   // Define the camera with a given image resolution
   error = rox_camera_new(&camera, cols, rows);
   if(error) goto function_terminate;

   // Define the image model
   error = rox_image_new(&Im, cols, rows);
   if(error) goto function_terminate;

   // Fill the Rox_Image with the data in the buffer
   error = rox_image_set_data(Im, buffer, cols, Rox_Image_Format_Grays);
   if(error) goto function_terminate;

   // Replace this function with your own function to fill the buffer
   error = grab_image_buffer(buffer, &cols, &rows);
   if(error) goto function_terminate;

   // Create new image
   error = rox_image_new(&current, cols, rows);
   if(error) goto function_terminate;

   // Fill the Rox_Image with the data in the buffer
   error = rox_image_set_data(current, buffer, cols, Rox_Image_Format_Grays);
   if(error) goto function_terminate;

   // Define the 2D model
   error = rox_model_single_plane_new(&model);
   if(error) goto function_terminate;

   error =  rox_model_single_plane_set_template_xright_ydown ( model, Im, size_x, size_y );
   if(error) goto function_terminate;

   // Define the calibration object
   error = rox_camera_calibration_new ( &calibration, model );
   if(error) goto function_terminate;

   // Create the intrinsic matrix
   error = rox_matrix_new(&Ke, 3, 3);
   if(error) goto function_terminate;

   // Add calibration images to a set
   for(i = 0; i < nbimg; i++)
   {
      // Read the current image
      // Replace this function with your own function to fill the buffer
      error = grab_image_buffer ( buffer, &cols, &rows );
      if(error) goto function_terminate;

      // Fill the Rox_Image with the data in the buffer
      error = rox_image_set_data ( current, buffer, cols, Rox_Image_Format_Grays );
      if(error) goto function_terminate;

      error = rox_camera_calibration_add_image(calibration, current);
      if(error)
      {
	     printf("The given image cannot be used for camera calibration \n");
      }
      else
      {
	     printf("Added image %02d to calibration set \n", i);
      }
   }

   error = rox_camera_calibration_make(calibration, method);
   if(error) goto function_terminate;

   error = rox_camera_calibration_get_intrinsics(Ke, calibration);
   if(error) goto function_terminate;

   // Set the camera intrisic parameters to a given camera
   error = rox_camera_set_intrinsic_parameters(camera, Ke);
   if(error) goto function_terminate;

   printf("The estimated intrinsic parameters are: \n");
   error = rox_matrix_print(Ke);
   if(error) goto function_terminate;

   printf("The expected  intrinsic parameters are: \n");
   printf("Matrix (3x3) \n");
   printf("1233.0915 0.0374 332.9738 \n");
   printf("0.0000 1236.0150 244.2956 \n");
   printf("0.0000 0.0000 1.0000 \n");

function_terminate:

   // Delete objects and free memory
   rox_camera_del(&camera);
   rox_matrix_del(&Ke);
   rox_image_del(&Im);
   rox_image_del(&current);
   rox_camera_calibration_del(&calibration);

   // Display Error
   rox_error_print(error);

   printf("\nPress return to end the program\n");
   getchar();

   return error;
}
