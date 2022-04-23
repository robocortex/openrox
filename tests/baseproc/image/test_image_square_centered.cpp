//==============================================================================
//
//		Filename  : test_image_square_centered.cpp
//
//		Contents  : Tests for image.c
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

ROX_TEST_SUITE_BEGIN(image_square_centered)

// Image 517  x  719
#define IMAGE_517x719_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_517x719.pgm"

// Image 640  x  480
#define IMAGE_640x480_PATH ROX_DATA_HOME"/regression_tests/openrox/calibration/texture/image_plane3D_calib_01.pgm"

// Image 256  x  256
#define IMAGE_256x256_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/model_corkes_256x256.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_square_centered)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   Rox_Image image = NULL;
   Rox_Image image_square_centered = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Sint cols = 0, rows = 0, nbi = 1;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test image with rows > cols
   sprintf(filename, "%s", IMAGE_517x719_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size(&rows, &cols, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   total_time = 0.0;
   for (Rox_Sint i = 0; i < nbi; i++)
   {
      rox_timer_start(timer);

      error = rox_image_new_square_centered(&image_square_centered, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to center a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbi);

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image_square_centered);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test image with cols > rows
   sprintf(filename, "%s", IMAGE_640x480_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size(&rows, &cols, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   total_time = 0.0;
   for (Rox_Sint i = 0; i < nbi; i++)
   {
      rox_timer_start(timer);

      error = rox_image_new_square_centered(&image_square_centered, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;


   }
   rox_log("mean time to center a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbi);

   error = rox_image_del(&image_square_centered);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test image with cols = rows
   sprintf(filename, "%s", IMAGE_256x256_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size(&rows, &cols, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   total_time = 0.0;
   for (Rox_Sint i = 0; i < nbi; i++)
   {
      rox_timer_start(timer);

      error = rox_image_new_square_centered(&image_square_centered, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to center a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbi);

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image_square_centered);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_timer_del(&timer);

}

ROX_TEST_SUITE_END()
