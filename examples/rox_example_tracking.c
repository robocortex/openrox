//==============================================================================
//
//    OPENROX   : File rox_example_tracking.c
//
//    Contents  : A simple example program for using the Rox Tracking SDK
//                This example show how to fill an image with a buffer
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

// ===== MACROS    ================================================

#define seq_file "../seq/plane/image_plane3D%03d.pgm"
#define mod_file "../seq/plane/model.pgm"
#define res_file "../res/result_tracking_%03d.ppm" 
#define res_path "../res/" 

// ===== VARIABLES ================================================

int main(int argc, char *argv[])
{
   Rox_Error error = 0;
   Rox_Char filename[FILENAME_MAX];

   // Declaration of tracking object
   Rox_Tracking tracker = 0;
   // Declaration of tracking parameters object
   Rox_Tracking_Params params = 0;

   Rox_Ident_Texture_SL3 identifier = 0;

   Rox_Image image = 0;
   Rox_Image model = 0;

   Rox_MatSL3 homography = 0;
   Rox_Image_RGBA image_rgba = 0;
   Rox_Rect_Sint_Struct rectangle;

   Rox_Sint model_cols, model_rows;
   Rox_Sint cols, rows;
   Rox_Sint tracked = 0;
   Rox_Sint identified = 0;
   Rox_Double score = 0;
   Rox_Uint major = 0, minor = 0, patch = 0;

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Load model
   sprintf(filename, mod_file);
   //printf("Reading %s image\n", filename);
   error = rox_image_new_read_pgm(&model, filename); 
   if(error) goto function_terminate;

   // Load current
   sprintf(filename, seq_file, 0);
   //printf("Reading %s image\n", filename);
   error = rox_image_new_read_pgm(&image, filename); 
   if(error) goto function_terminate;

   // Create buffers
   error = rox_matsl3_new(&homography); 
   if(error) goto function_terminate;

   // Tracking params
   error = rox_tracking_params_new(&params); 
   if(error) goto function_terminate;

   // Tracking creation
   error = rox_tracking_new(&tracker, params, model); 
   if(error) goto function_terminate;

   // Identification
   error = rox_ident_texture_sl3_new(&identifier); 
   if(error) goto function_terminate;
   
   error = rox_ident_texture_sl3_set_model(identifier, model); 
   if(error) goto function_terminate;

   // Visualisation
   error = rox_image_get_cols(&model_cols, model); 
   if(error) goto function_terminate;
   
   error = rox_image_get_rows(&model_rows, model); 
   if(error) goto function_terminate;

   rectangle.x = 0;
   rectangle.y = 0;
   rectangle.width = model_cols;
   rectangle.height = model_rows;

   error = rox_image_get_cols(&cols, image); 
   if(error) goto function_terminate;
   
   error = rox_image_get_rows(&rows, image); 
   if(error) goto function_terminate;
   
   error = rox_image_rgba_new(&image_rgba, rows, cols); 
   if(error) goto function_terminate;

   for(Rox_Sint i = 0; i < 100; i++)
   {
      sprintf(filename, seq_file, i);
      printf("Reading %s image\n", filename);

      error = rox_image_read_pgm(image, filename); 
      if(error) goto function_terminate;
      
      error = rox_image_rgba_read_pgm(image_rgba, filename); 
      if(error) goto function_terminate;

      if(!tracked)
      {
          printf("Start detection \n");
          error = rox_ident_texture_sl3_make(&identified, homography, identifier, image);
          if(error) goto function_terminate;
      }

      if(identified || tracked)
      {
         printf("Start tracking \n");
         
         error = rox_tracking_set_homography(tracker, homography);
         if(error) goto function_terminate;
          
         error = rox_tracking_make(tracker, image);
         if (!error)
         {
            printf("Tracked\n");
            tracked = 1;
         }
         else
         {
            printf("Lost\n");
            tracked = 0;
         }
         
         error = rox_tracking_get_homography(homography, tracker); 
         if(error) goto function_terminate;

         error = rox_tracking_get_score(&score, tracker); 
         if(error) goto function_terminate;
      
         printf("Quality score %f\n", score);

         error = rox_image_rgba_draw_warp_rectangle(image_rgba, homography, &rectangle, 0xff0000ff); 
         if(error) goto function_terminate;
      
         sprintf(filename, "../res/res_tracking_%03d.ppm", i);
         error = rox_image_rgba_save_ppm(filename, image_rgba);
         if(error) goto function_terminate;
      }
   }

function_terminate:
   printf("error =%d\n",error);

   rox_tracking_params_del(&params);
   rox_tracking_del(&tracker);
   rox_ident_texture_sl3_del(&identifier);

   rox_image_del(&image);
   rox_image_del(&model);
   
   rox_image_rgba_del(&image_rgba);

   rox_matsl3_del(&homography);

   return error;
}
