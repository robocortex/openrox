//==============================================================================
//
//    OPENROX   : File rox_example_odometry_multi_plane.c
//
//    Contents  : A simple example program for multi plane odometry.
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
// http://public.robocortex.com/download/sequences/cube.zip

#define seq_file "../seq/cube/images/cube%4.4d.pgm"
#define res_file "../res/result_cube_%4.4d.ppm"

// Define the paths for the model textures
#define mod_file_L "../seq/cube/models/left.pgm"
#define mod_file_F "../seq/cube/models/front.pgm"
#define mod_file_T "../seq/cube/models/top.pgm"
#define mod_file_B "../seq/cube/models/back.pgm"
#define mod_file_R "../seq/cube/models/right.pgm"

#define PX 559.279
#define PY 559.279
#define U0 329.890
#define V0 242.164

//=== VARIABLES ================================================================

//=== FUNCTIONS ================================================================

//=== MAIN PROGRAM =============================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   Rox_Error error;
   Rox_Char filename[1024];
   Rox_Uint major = 0, minor = 0, patch = 0;

   Rox_Image source = 0;
   Rox_Image_RGBA image_rgba = 0;
   Rox_MatSE3 pose = 0;
   Rox_Matrix K = 0;

   Rox_Model_Multi_Plane model = 0;
   Rox_Camera camera = 0;

   Rox_Ident_Multi_Plane identifier = NULL;
   Rox_Odometry_Multi_Plane odometry = NULL;
   Rox_Odometry_Multi_Plane_Params params = NULL;
   Rox_Timer timer = 0;

   Rox_Uint tracked, idtemplate, idimg;
   Rox_Double time;
   Rox_Uint red = ROX_MAKERGBA(255, 0 , 0, 255);

   const Rox_Uint count_templates = 5;

   Rox_Point3D_Double_Struct vertices[][4] =
   {
      { {-1, -1,  1}, {-1, -1, -1}, {-1,  1, -1}, {-1,  1,  1} },
      { {-1, -1, -1}, { 1, -1, -1}, { 1,  1, -1}, {-1,  1, -1} },
      { {-1, -1,  1}, { 1, -1,  1}, { 1, -1, -1}, {-1, -1, -1} },
      { { 1, -1,  1}, {-1, -1,  1}, {-1,  1,  1}, { 1,  1,  1} },
      { { 1, -1, -1}, { 1, -1,  1}, { 1,  1,  1}, { 1,  1, -1} }
   };

   const char * names[] =
   {
       mod_file_L, mod_file_F, mod_file_T, mod_file_B, mod_file_R
   };

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Creating model
   error = rox_model_multi_plane_new(&model);
   if (error) goto function_terminate;

   error = rox_ident_multi_plane_new(&identifier);
   if (error) goto function_terminate;

   for (idtemplate = 0; idtemplate < count_templates; idtemplate++)
   {
      error = rox_image_new_read_pgm(&source, names[idtemplate]);
      if (error) goto function_terminate;

      error = rox_model_multi_plane_append_plane(model, source, vertices[idtemplate]);
      if (error) goto function_terminate;

      rox_image_del(&source);
   }

   error = rox_ident_multi_plane_set_model(identifier, model);
   if (error) goto function_terminate;

   error = rox_matse3_new(&pose);
   if (error) goto function_terminate;

   error = rox_odometry_multi_plane_params_new(&params);
   if (error) goto function_terminate;

   error = rox_odometry_multi_plane_new(&odometry, params, model);
   if (error) goto function_terminate;

   error = rox_image_rgba_new(&image_rgba, 640, 480);
   if (error) goto function_terminate;

   error = rox_camera_new(&camera, 640, 480);
   if (error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, PX, PY, U0, V0);
   if (error) goto function_terminate;

   error = rox_matrix_new(&K, 3, 3);
   if (error) goto function_terminate;

   error = rox_matrix_build_calibration_matrix(K, PX, PY, U0, V0);
   if (error) goto function_terminate;

   error = rox_timer_new(&timer);
   if (error) goto function_terminate;

   tracked = 0;

   for (idimg = 0; idimg < 400; idimg++)
   {
      // Read image from sequence
      sprintf(filename, seq_file, idimg);
      error = rox_camera_read_pgm(camera, filename);
      if (error) goto function_terminate;

      error = rox_timer_start(timer);
      if(error) goto function_terminate;

      // Identification
      if (!tracked)
      {
          error = rox_ident_multi_plane_make ( identifier, model, camera );
          if(!error)
          {
             error = rox_ident_multi_plane_get_pose(pose, identifier);
             if (error) goto function_terminate;

             error = rox_odometry_multi_plane_set_pose(odometry, pose);
             if (error) goto function_terminate;
	     
             tracked = 1;
          }
      }

      // Odometry
      if (tracked)
      {
         error = rox_odometry_multi_plane_make ( odometry, model, camera );
         if (!error)
         {
            error = rox_odometry_multi_plane_get_pose ( pose, odometry );
            if (error) goto function_terminate;

            tracked = 1;
         }
         else
         {
             tracked = 0;
         }
      }

      error = rox_timer_stop(timer);
      if(error) goto function_terminate;

      error = rox_timer_get_elapsed_ms(&time, timer);
      if(error) goto function_terminate;

      printf("Time to track: %f ms \n", time);
      // Display results
      if (tracked)
      {
         error = rox_image_rgba_read_pgm(image_rgba, filename);
         if (error) goto function_terminate;

         error = rox_image_rgba_draw_projection_model_multi_plane(image_rgba, K, pose, model, red);
         if (error) goto function_terminate;

         // Save image
         sprintf(filename, res_file, idimg);
         error = rox_image_rgba_save_ppm(filename, image_rgba);
         if (error) goto function_terminate;
      }
   }

function_terminate:

   // Delete objects and free Memory
   rox_model_multi_plane_del(&model);
   rox_camera_del(&camera);

   rox_ident_multi_plane_del(&identifier);
   rox_odometry_multi_plane_del(&odometry);

   rox_matse3_del(&pose);
   rox_matrix_del(&K);

   rox_image_del(&source);
   rox_image_rgba_del(&image_rgba);

   // Display Error
   rox_error_print(error);

   printf("\n\n Press return to end the program");
   getchar();

   return error;
}

