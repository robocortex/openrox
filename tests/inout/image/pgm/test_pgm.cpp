//==============================================================================
//
//    OPENROX   : File test_pgm.cpp
//
//    Contents  : Tests for pgm.c
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
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(pgm)

// Image 640  x  480 =  307200 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/calibration/texture/image_plane3D_calib_01.pgm"
// Image 1280 x  720 =  921600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/pgm/switch_shield_seq1_img0001.pgm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/lumia/sequence_1/pgm/switch_lumia_seq1_img0001.pgm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/projects/apra/deformation/img005.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_read_ppm)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   Rox_Image image_gray = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Sint cols = 0, rows = 0, nb_read = 1000;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Create and read the image
   error = rox_image_new_read_pgm(&image_gray, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size(&rows, &cols, image_gray);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint i = 0; i < nb_read; i++)
   {
      // Start timer
      rox_timer_start(timer);

      // Read image from ppm file
      error = rox_image_read_pgm(image_gray, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to read a (%d x %d) pgm image = %f (ms)\n", cols, rows, total_time/nb_read);

   // On computer "Iris" : Intel(R) Core(TM) i7 CPU Q 820 @ 1.73GHz
   // tt_4096 x 2160 =   3.671118 ms
   // tt_1920 x 1080 =   1.021429 ms
   // tt_1280 x 0720 =   0.486151 ms
   // tt_0640 x 0480 =   0.250778 ms

   // ratio time : 109.706466 / 26.953568 = 4.0702
   // ratio size : 4096x2160 / 1920x1080  = 4.2667

   // ratio time : 26.953568 /12.425859   = 2.1692
   // ration size : 1920x1080 / 1280x0720 = 2.2500

   // ratio time :  12.425859 /  5.260344 = 2.3622
   // ration size : 1280x0720 / 0640x0480 = 3.0000

   rox_image_del(&image_gray);
   rox_timer_del(&timer);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uint_rgba_save_pgm)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   Rox_Image image_gray = NULL;
   Rox_Double time = 0.0, total_time = 0.0, max_time = 0.0;
   Rox_Sint cols = 0, rows = 0, nb_save = 100;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Create and read the image
   error = rox_image_new_read_pgm(&image_gray, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size(&rows, &cols, image_gray);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint i = 0; i < nb_save; i++)
   {
      // Start timer
      rox_timer_start(timer);

      sprintf(filename, "test_result_%04d.pgm", i);

      // Save image from ppm file
      //error = rox_image_save_pgm(filename, image_gray);
      //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
      if (time > max_time) max_time = time;

      rox_log("save file %s in %f (ms) \n", filename, time);
      rox_log("max time to save a (%d x %d) pgm image = %f (ms)\n", cols, rows, max_time);
   }
   rox_log("mean time to save a (%d x %d) pgm image = %f (ms)\n", cols, rows, total_time/nb_save);
   rox_log("max time to save a (%d x %d) ppm image = %f (ms)\n", cols, rows, max_time);

   // On computer "Iris" : Intel(R) Core(TM) i7 CPU Q 820 @ 1.73GHz
   // tt_4096 x 2160 = t1 ms
   // tt_1920 x 1080 = t2 ms
   // tt_1280 x 0720 = t3 ms
   // tt_0640 x 0480 = t4 ms

   // ratio time : t1 / t2 = r12
   // ratio size : 4096x2160 / 1920x1080  = 4.2667

   // ratio time : t2 / t3 = r23
   // ration size : 1920x1080 / 1280x0720 = 2.2500

   // ratio time : t3 / t4 = r34
   // ration size : 1280x0720 / 0640x0480 = 3.0000

   rox_image_del(&image_gray);
   rox_timer_del(&timer);
}
ROX_TEST_SUITE_END()
