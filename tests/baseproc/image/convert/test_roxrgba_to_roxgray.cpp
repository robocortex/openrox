//==============================================================================
//
//    OPENROX   : File test_roxrgba_to_roxgray.cpp
//
//    Contents  : Tests for roxrgba_to_roxgray.c
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
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/image/ppm/ppmfile.h>
   #include <baseproc/image/convert/roxrgba_to_roxgray.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(roxrgba_to_roxgray)

// Image 640  x  480 =  307200 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/motion_detection/img_motion_detection_00.ppm"
// Image 1280 x  720 =  921600 pixels
//#define IMAGE_PATH "model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH "model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
//#define IMAGE_PATH "projects/apra/img005.ppm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_roxrgba_to_roxgray)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   Rox_Image image_gray = NULL;
   Rox_Image_RGBA image_rgba = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Sint cols = 0, rows = 0, nbconv = 100;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_rgba_new_read_ppm ( &image_rgba, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_get_size(&rows, &cols, image_rgba);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_new(&image_gray, cols, rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint i = 0; i < nbconv; i++)
   {
      rox_timer_start(timer);

      error = rox_roxrgba_to_roxgray_approx(image_gray, image_rgba);
      // error = rox_roxrgba_to_roxgray(image_gray, image_rgba);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to convert a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbconv);

   //error = rox_image_save_pgm("./result_test_roxrgba_to_roxgray.pgm", image_gray);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // On Ezio's computer
   // tt_4096x2160 = 67.879562 ms
   // tt_1920x1080 = 16.383383 ms
   // tt_1280x0720 =  7.267900 ms

   // ratio time : 16.383383 / 7.267900 = 2.2542
   // ration size : 1920x1080 / 1280x0720 = 2.2500

   // ratio time : 67.879562 / 7.267900 = 9.3396
   // ration size : 4096x2160 / 1280x0720 = 9.6000

   // On Ezio's computer approx
   // tt_4096x2160 = 63.434970 ms
   // tt_1920x1080 = 15.018909 ms
   // tt_1280x0720 =  7.244956 ms

   error = rox_image_rgba_del(&image_rgba);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image_gray);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_roxrgba_flip_to_roxgray)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   Rox_Image image_gray = NULL;
   Rox_Image_RGBA image_rgba = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Sint cols = 0, rows = 0, nbconv = 100;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_rgba_new_read_ppm(&image_rgba, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_get_size(&rows, &cols, image_rgba);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_new ( &image_gray, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint i = 0; i < nbconv; i++)
   {
      rox_timer_start(timer);

      error = rox_roxrgba_flip_to_roxgray_approx(image_gray, image_rgba);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to flip and convert a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbconv);

   //error = rox_image_save_pgm("./result_test_roxrgba_flip_to_roxgray.pgm", image_gray);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // On Ezio's computer approx
   // tt_4096x2160 = 68.828764 ms
   // tt_1920x1080 = 15.199602 ms
   // tt_1280x0720 =  6.549482 ms

   // ratio time : 15.199602 / 6.549482 = 2.3207
   // ratio size : 1920x1080 / 1280x0720 = 2.2500

   // ratio time : 68.828764 / 6.549482 = 10.509
   // ratio size : 4096x2160 / 1280x0720 = 9.6000

   error = rox_image_rgba_del(&image_rgba);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image_gray);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
