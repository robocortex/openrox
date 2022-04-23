//==============================================================================
//
//    OPENROX   : File test_ppm.cpp
//
//    Contents  : Tests for ppm.c
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
   #include <inout/image/ppm/ppmfile.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(ppm)

// Image 640  x  480 =  307200 pixels
 #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/motion_detection/img_motion_detection_00.ppm"
// Image 1280 x  720 =  921600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/projects/apra/deformation/img005.ppm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_rgba_read_ppm)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   Rox_Image_RGBA image_rgba = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Sint cols = 0, rows = 0, nb_read = 100;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Create and read the image
   error = rox_image_rgba_new_read_ppm(&image_rgba, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_get_size(&rows, &cols, image_rgba);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint i = 0; i < nb_read; i++)
   {
      // Start timer
      rox_timer_start(timer);

      // Read image from ppm file
      error = rox_image_rgba_read_ppm(image_rgba, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to read a (%d x %d) ppm image = %f (ms)\n", cols, rows, total_time/nb_read);

   // On computer "Iris" : Intel(R) Core(TM) i7 CPU Q 820 @ 1.73GHz
   // tt_4096 x 2160 = 109.706466 ms
   // tt_1920 x 1080 =  26.953568 ms
   // tt_1280 x 0720 =  12.425859 ms
   // tt_0640 x 0480 =   5.260344 ms

   // ratio time : 109.706466 / 26.953568 = 4.0702
   // ratio size : 4096x2160 / 1920x1080  = 4.2667

   // ratio time : 26.953568 /12.425859   = 2.1692
   // ration size : 1920x1080 / 1280x0720 = 2.2500

   // ratio time :  12.425859 /  5.260344 = 2.3622
   // ration size : 1280x0720 / 0640x0480 = 3.0000

   rox_image_rgba_del(&image_rgba);
   rox_timer_del(&timer);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_rgba_save_ppm)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   Rox_Image_RGBA image_rgba = NULL;
   Rox_Double time = 0.0, total_time = 0.0, max_time = 0.0;
   Rox_Sint cols = 0, rows = 0, nb_save = 100;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Create and read the image
   error = rox_image_rgba_new_read_ppm(&image_rgba, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_get_size(&rows, &cols, image_rgba);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint i = 0; i < nb_save; i++)
   {
      // Start timer
      rox_timer_start(timer);

      sprintf(filename, "test_result_%04d.ppm", i);

      // Save image from ppm file
      //error = rox_image_rgba_save_ppm(filename, image_rgba);
      //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
      if (time > max_time) max_time = time;

      rox_log("save file %s in %f (ms) \n", filename, time);

   }
   rox_log("mean time to save a (%d x %d) ppm image = %f (ms)\n", cols, rows, total_time/nb_save);
   rox_log("max time to save a (%d x %d) ppm image = %f (ms)\n", cols, rows, max_time);
   // On computer "Iris" : Intel(R) Core(TM) i7 CPU Q 820 @ 1.73GHz
   // tt_4096 x 2160 = 109.706466 ms
   // tt_1920 x 1080 =  26.953568 ms
   // tt_1280 x 0720 =  12.425859 ms
   // tt_0640 x 0480 =   5.260344 ms

   // ratio time : 109.706466 / 26.953568 = 4.0702
   // ratio size : 4096x2160 / 1920x1080  = 4.2667

   // ratio time : 26.953568 /12.425859   = 2.1692
   // ration size : 1920x1080 / 1280x0720 = 2.2500

   // ratio time :  12.425859 /  5.260344 = 2.3622
   // ration size : 1280x0720 / 0640x0480 = 3.0000

   rox_image_rgba_del(&image_rgba);
   rox_timer_del(&timer);
}

ROX_TEST_SUITE_END()
