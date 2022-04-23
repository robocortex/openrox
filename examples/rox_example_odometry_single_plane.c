//==============================================================================
//
//    OPENROX   : File rox_example_odometry_single_plane.c
//
//    Contents  : A simple example program for single plane odometry.
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

#define seq_path "../seq/plane/image_plane3D%03d.pgm"
#define mod_file "../seq/plane/model.pgm"
#define res_file "../res/result_odometry_%03d.ppm"

//=== VARIABLES ================================================================

Rox_Sint i = 0;

//=== FUNCTIONS ================================================================

// This function is used to simulate an image grabber
Rox_Error grab_image_buffer(Rox_Uchar* buffer, Rox_Sint *cols, Rox_Sint *rows);

// This function is used to simulate an image reader
Rox_Error read_image_model(Rox_Uchar* buffer, Rox_Sint *cols, Rox_Sint *rows);

Rox_Error grab_image_buffer(Rox_Uchar* buffer, Rox_Sint *cols, Rox_Sint *rows)
{
   Rox_Char filename[1024];
   Rox_Image image = 0;
   Rox_Error error;

   sprintf(filename, seq_path, i);

   error = rox_image_new_read_pgm(&image, filename); if(error) goto function_terminate;
   error = rox_image_get_data(buffer, image); if(error) goto function_terminate;
   error = rox_image_get_cols(cols, image); if(error) goto function_terminate;
   error = rox_image_get_rows(rows, image); if(error) goto function_terminate;
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

//=== MAIN PROGRAM ======================================================

Rox_Sint main (Rox_Void)
{
   Rox_Error error;
   Rox_Double score;
   Rox_Char filename[1024];

   Rox_Uint major = 0, minor = 0, patch = 0;

   Rox_Uint red = ROX_MAKERGBA(255, 0 , 0, 255);

   // Real size of the target (in meters)
   Rox_Double  model_sizex = 0.2;
   Rox_Double  model_sizey = 0.2;

   // Define the 2D model with texture and size in meters
   Rox_Model_Single_Plane model = 0;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------
 
   // Define objects for the 2d model
   Rox_Image    Im = 0;

   // Define objects for the current image
   Rox_Camera camera  = 0;
   Rox_Matrix K = 0;
   Rox_Image image = 0;
   Rox_Image_RGBA image_rgba = 0;

   // Define pose object
   Rox_MatSE3 pose = 0;

   // Define identification object
   Rox_Ident_Texture_SE3 identifier = 0;

   // Define odometry parameters
   Rox_Odometry_Single_Plane_Params params = 0;

   // Define odometry object
   Rox_Odometry_Single_Plane odometry = 0;

   // Define tracking results
   Rox_Sint tracked = 0;

   // Define timer object
   Rox_Timer timer = 0;
   Rox_Double time;

   // Define intrinsic parameters
   Rox_Double PX = 2560.0;
   Rox_Double PY = 2560.0;
   Rox_Double U0 = 255.5;
   Rox_Double V0 = 255.5;

   // Define the resolution : 512 x 512 pixels
   Rox_Sint cols = 512;
   Rox_Sint rows = 512;

   // Define the buffer containing the image data
   Rox_Uchar buffer[512*512];

   Rox_Sint is_identified = 0;

   // ---------- INIT VISUAL ODOMETRY -------------------------------------

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Read the image model from disk
   // Replace this function with your own function to fill the buffer
   error = read_image_model(buffer, &cols, &rows);
   if(error) goto function_terminate;

   // Define the image model
   error = rox_image_new(&Im, cols, rows);
   if(error) goto function_terminate;

   // Fill the Rox_Image with the data in the buffer
   error = rox_image_set_data(Im, buffer, cols, Rox_Image_Format_Grays);
   if(error) goto function_terminate;

   // Define the 2D model with texture and size in meters :
   // in this case 0.2 m x 0.2 m
   error = rox_model_single_plane_new(&model);
   if(error) goto function_terminate;

   error = rox_model_single_plane_set_template_xright_ydown ( model, Im, model_sizex, model_sizey );
   if(error) goto function_terminate;

   // Define the ident object
   error = rox_ident_texture_se3_new(&identifier);
   if(error) goto function_terminate;

   error = rox_ident_texture_se3_set_model(identifier, model);
   if(error) goto function_terminate;

   // Define the odometry parameters
   error = rox_odometry_single_plane_params_new(&params);
   if(error) goto function_terminate;

   // Define the odometry object
   error = rox_matse3_new(&pose);
   if(error) goto function_terminate;

   error = rox_odometry_single_plane_new(&odometry, params, model);
   if(error)
   {
      printf ("Type return to exit the program\n");
      getchar();
      return error;
   }
   else
   {
      printf ("ROX Odometry structure ready\n");
   }

   // Replace this function with your own function to fill the buffer
   error = grab_image_buffer(buffer, &cols, &rows);
   if(error) goto function_terminate;

   // Create new image
   error = rox_image_new(&image, cols, rows);
   if(error) goto function_terminate;

   // Fill the Rox_Image with the data in the buffer
   error = rox_image_set_data(image, buffer, cols, Rox_Image_Format_Grays);
   if(error) goto function_terminate;

   // Define the camera with a given image resolution
   error = rox_camera_new(&camera, cols, rows);
   if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, PX, PY, U0, V0);
   if(error) goto function_terminate;

   // Define the intrinsic parameters
   error = rox_matrix_new(&K, 3, 3);
   if(error) goto function_terminate;

   error = rox_camera_get_intrinsic_parameters(K, camera);
   if(error) goto function_terminate;

   // Get the pointer to the image of the camera
   error = rox_camera_set_image(camera, image);
   if(error) goto function_terminate;

   // Define timer object
   error = rox_timer_new(&timer);
   if(error) goto function_terminate;

   // Define the display object
   error = rox_image_rgba_new(&image_rgba, cols, rows);
   if(error) goto function_terminate;

   // Fill the Rox_Image with the data in the buffer
   error = rox_image_rgba_set_data(image_rgba, buffer, cols, Rox_Image_Format_Grays);
   if(error) goto function_terminate;

   // ----------- START MAIN LOOP   ---------------------------------------

   for (i = 0; i < 100; i++)
   {
      // Replace this function with your own function to fill the buffer
      error = grab_image_buffer(buffer, &cols, &rows);
      if(error) goto function_terminate;

      // Fill the Rox_Image with the data in the buffer
      error = rox_image_set_data(image, buffer, cols, Rox_Image_Format_Grays);
      if(error) goto function_terminate;

      // Fill the Rox_Image with the data in the buffer
      error = rox_image_rgba_set_data(image_rgba, buffer, cols, Rox_Image_Format_Grays);
      if(error) goto function_terminate;

      // Get the pointer to the image of the camera
      error = rox_camera_set_image(camera, image);
      if(error) goto function_terminate;

      error = rox_timer_start(timer);
      if(error) goto function_terminate;

      if(tracked == 1)
      {
         // Tracking the target
         error = rox_odometry_single_plane_make(odometry, camera);
         if(error) tracked = 0;
         else tracked = 1;
      }
      else
      {
         // Identify the target
         error = rox_ident_texture_se3_make(&is_identified, pose, identifier, camera);
         if(error == 0)
         {
            error = rox_odometry_single_plane_set_pose(odometry, pose);
            if(error) goto function_terminate;

            error = rox_odometry_single_plane_make(odometry, camera);
            if(error) tracked = 0;
            else tracked = 1;
         }
      }
      error = rox_timer_stop(timer);
      if(error) goto function_terminate;

      error = rox_timer_get_elapsed_ms(&time, timer);
      if(error) goto function_terminate;

      // Get a copy of the pose (position and orientation) of the camera 
      error = rox_odometry_single_plane_get_pose(pose, odometry);
      if(error) goto function_terminate;

      // Display quality measure
      error = rox_odometry_single_plane_get_score(&score, odometry);
      if(error) goto function_terminate;

      printf("Quality score = %f \n", score);
      printf("time to track: %f ms\n", time);

      if(tracked == 1)
      {
         printf("Pose: \n");
         error = rox_matse3_print(pose);
         if(error) goto function_terminate;
         
         // Draw and save results
         error = rox_image_rgba_draw_projection_model_single_plane(image_rgba, K, pose, model, red);
         if(error) goto function_terminate;

         error = rox_image_rgba_draw_projection_frame(image_rgba, K, pose, 0.1);
         if(error) goto function_terminate;
      }

      sprintf(filename, res_file, i);
      error = rox_image_rgba_save_ppm(filename, image_rgba);
      if(error) goto function_terminate;
   }

function_terminate:

   // Delete objects and free Memory
   rox_image_del(&Im);
   rox_image_rgba_del(&image_rgba);
   rox_image_del(&image);
   rox_camera_del(&camera);
   rox_matrix_del(&K);
   rox_matse3_del(&pose);
   rox_odometry_single_plane_del(&odometry);
   rox_odometry_single_plane_params_del(&params);
   rox_ident_texture_se3_del(&identifier);
   rox_model_single_plane_del(&model);
   rox_timer_del(&timer);

   // Display Error
   rox_error_print(error);

   printf("\n\n Press return to end the program");
   getchar();

   return error;
}
