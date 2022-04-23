//==============================================================================
//
//		Filename  : test_image_flip.cpp
//
//		Contents  : Tests for image_flip.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =======================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <system/time/timer.h>
   #include <baseproc/image/image.h>
   #include <inout/system/print.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(image_flip)

// Image 512  x  512 =  262144 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"

// Image 1280 x  720 =  921600 pixels
// #define IMAGE_PATH "model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
// #define IMAGE_PATH "database/test_database_1280x0720.pgm"

// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH "model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// #define IMAGE_PATH "database/test_database_1920x1080.pgm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH "projects/apra/img005.ppm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_flip)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0, total_time = 0.0;

   Rox_Image image = NULL;
   Rox_Image image_flip = NULL;
   Rox_Sint cols = 0, rows = 0;
   Rox_Uint nbflip = 1000;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);
   
   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_cols(&cols, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_rows(&rows, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_new(&image_flip, cols, rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   total_time = 0.0;
   for (Rox_Uint i = 0; i < nbflip; i++)
   {
      rox_timer_start(timer);

      error = rox_image_copy_flip(image_flip, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to copy and flip a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbflip);

   total_time = 0.0;
   for (Rox_Uint i = 0; i < nbflip; i++)
   {
      rox_timer_start(timer);

      error = rox_image_flip(image_flip);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to flip a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbflip);
   
   // On Iris computer    
   // tt_4096x2160 = 2.391513 ms
   // tt_1920x1080 = 0.234260 ms
   // tt_1280x0720 = 0.114074 ms

   // ratio time : 0.234260 / 0.114074  = 2.0536
   // ration size : 1920x1080 / 1280x0720 = 2.2500

   // ratio time : 2.391513 / 0.114074  = 9.3029
   // ration size : 4096x2160 / 1280x0720 = 9.6000

   // On Neptune computer
   // tt_1920x1080 = 0.129076 ms
   // tt_1280x0720 = 0.052858 ms
   // tt_0512x0512 = 0.020094 ms

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image_flip);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
