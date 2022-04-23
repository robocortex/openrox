//==============================================================================
//
//    OPENROX   : File rox_example_image_score.c
//
//    Contents  : A simple example program for image score.
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

// ===== HEADERS      ======================================================

#include <api/openrox.h>
#include <stdio.h>

// ===== MACROS       ======================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/identification.zip

#define mod_file "../seq/plane/model.pgm"

// ===== VARIABLES    ======================================================

// ===== FUNCTIONS    ======================================================

// ===== MAIN PROGRAM ======================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   Rox_Error error;

   Rox_Timer  timer = 0;
   Rox_Image  model = 0;
   Rox_Double time;
   Rox_Double score = 0.0;
   Rox_Uint major = 0, minor = 0, patch = 0;

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Create Rox objects
   error = rox_timer_new(&timer);
   if(error) goto function_terminate;

   printf("ROX Reading the model image...\n");
   error = rox_image_new_read_pgm(&model, mod_file);
   if(error) goto function_terminate;

   printf("ROX starts computing fast score...\n");
   error = rox_timer_start(timer);
   if(error) goto function_terminate;

   error = rox_image_get_quality_score(&score, model);
   if(error) goto function_terminate;

   error = rox_timer_stop(timer);
   if(error) goto function_terminate;

   error = rox_timer_get_elapsed_ms(&time, timer);
   if(error) goto function_terminate;

   printf("Time needed to compute fast score %f\n", time);

   printf("fast score = %f\n", score);

   printf("ROX starts computing precise score...\n");
   error = rox_timer_start(timer);
   if(error) goto function_terminate;

   error = rox_image_get_quality_score_precise(&score, model); 
   if(error) goto function_terminate;

   error = rox_timer_stop(timer);
   if(error) goto function_terminate;

   error = rox_timer_get_elapsed_ms(&time, timer);
   if(error) goto function_terminate;

   printf("Time needed to compute precise score %f\n", time);
   printf("precise score = %f\n", score);

function_terminate:

   // Delete objects and free memory
   rox_timer_del(&timer);
   rox_image_del(&model);

   // Display Error
   rox_error_print(error);

   printf("\nPress Enter to end the program\n");
   getchar();

   return error;
}

