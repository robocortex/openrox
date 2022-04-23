//==============================================================================
//
//    OPENROX   : File test_camera.cpp
//
//    Contents  : Tests for camera.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license 
//
//==============================================================================

//====== INCLUDED HEADERS   ====================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <system/time/timer.h>

   #include <baseproc/image/image.h>
   #include <baseproc/image/image_rgba.h>
   
   #include <inout/image/ppm/ppmfile.h>
   #include <inout/system/print.h>
   
   #include <user/sensor/camera/camera.h>
}

//====== INTERNAL MACROS    ====================================================

ROX_TEST_SUITE_BEGIN(camera)

// Image 640  x  480 =  307200 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/motion_detection/img_motion_detection_00.ppm"
// Image 1280 x  720 =  921600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/projects/apra/img005.ppm"

//====== INTERNAL TYPESDEFS ====================================================

//====== INTERNAL DATATYPES ====================================================

//====== INTERNAL VARIABLES ====================================================

//====== INTERNAL FUNCTDEFS ====================================================

//====== INTERNAL FUNCTIONS ====================================================

//====== EXPORTED FUNCTIONS ====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_camera_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols = 1920, rows = 1080;
   Rox_Camera camera = NULL;

   error = rox_camera_new ( &camera, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_del ( &camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   camera = NULL;

   error = rox_camera_del ( &camera );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NULL_POINTER );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Camera camera = NULL;
   Rox_Image_RGBA image = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Sint cols = 0, rows = 0, nbflip = 100;
   Rox_Uint ** data;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);

   error = rox_image_rgba_new_read_ppm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_get_cols(&cols, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_get_rows(&rows, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_new ( &camera, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_get_data_pointer_to_pointer(&data, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   total_time = 0.0;
   for (Rox_Sint i = 0; i < nbflip; i++)
   {
      rox_timer_start(timer);

      error = rox_camera_set_image_data(camera, (Rox_Uchar *) data[0], 4*cols, Rox_Image_Format_RGBA_FlippedUpsideDown);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to set and flip a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nbflip);

   // Save 
   // error = rox_camera_save_pgm("test.pgm", camera);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // On Ezio's computer
   // tt_4096x2160 = 83.721349 ms
   // tt_1920x1080 = 19.509934 ms
   // tt_1280x0720 = 10.617649 ms

   // ratio time : 19.509934 / 10.617649  = 1.8375
   // ration size : 1920x1080 / 1280x0720 = 2.2500

   // ratio time : 83.721349 / 10.617649  = 7.8851
   // ration size : 4096x2160 / 1280x0720 = 9.6000

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_del(&camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
