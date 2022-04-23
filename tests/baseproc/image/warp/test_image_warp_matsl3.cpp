//==============================================================================
//
//    OPENROX   : File test_image_warp_matsl3.cpp
//
//    Contents  : Tests for image.c
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
   #include <inout/numeric/array2d_save.h>
   #include <inout/numeric/array2d_print.h>
   #include <baseproc/image/image.h>
   #include <baseproc/image/warp/image_warp_matsl3.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(image)

// Image  512 x  512 =  262144 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
// Image 1280 x  720 =  921600 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
//#define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/pgm/switch_shield_seq1_img0001.pgm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/projects/apra/deformation/img005.ppm"
// Image  517 x  719 =  371723 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_517x719.pgm"

//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/debug/earth_4000x3000.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

// const type * pointer : is a pointer to a constant (i.e. we cannot do *pointer = NULL)

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_warp_matsl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Image image = NULL;
   Rox_Image warped = NULL;
   Rox_Sint rows = 128, cols = 128;
   Rox_MatSL3 homography = NULL;
   Rox_Double data[9] = {1.0, 0.0, 1600.0, 0.0, 1.0,  1200.0, 0.0, 0.0, 1.0};
   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0; // , total_time = 0.0;

   Rox_Char filename[FILENAME_MAX];

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);
   
   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   ROX_TEST_MESSAGE("read file %s\n", filename);

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_new(&warped, cols, rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_new(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_set_data(homography, data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_timer_start(timer);

   error = rox_image_warp_matsl3(warped, image, homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Display elapsed time
   rox_timer_stop(timer);
   rox_timer_get_elapsed_ms(&time, timer);

   ROX_TEST_MESSAGE("mean time to warp a (%d x %d) image = %f (ms)\n", cols, rows, time);

   //error = rox_image_save_pgm("test_image_warp_matsl3.pgm", warped);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&warped);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}


ROX_TEST_SUITE_END()
