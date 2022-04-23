//==============================================================================
//
//    OPENROX   : File test_pyramid_uchar.cpp
//
//    Contents  : Tests for pyramid_uchar.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =====================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <system/time/timer.h>
   #include <inout/image/pgm/pgmfile.h>
   #include <baseproc/image/pyramid/pyramid_uchar.h>
   #include <inout/system/print.h>   
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(pyramid_uchar)

// Image  512  x   512 =    262 144 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
// Image  640  x   480 =    307 200 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_640x480.pgm"
// Image 1280  x   720 =    921 600 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1280x720.pgm"
// Image 1920  x  1080 =  2 073 600 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1920x1080.pgm"
// Image 3840  x  2160 =  8 294 400 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_3840x2160.pgm"
// Image 4096  x  2160 =  8 847 360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_4096x2160.pgm"
// Image 5120  x  3200 = 16 384 000 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_5120x3200.pgm"

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_pyramid_uchar_new)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_pyramid_uchar_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_pyramid_uchar_assign)
{
   const Rox_Sint max_levels = 4;
   const Rox_Sint min_size = 32;

   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Pyramid_Uchar pyramid = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;

   Rox_Sint cols = 0;
   Rox_Sint rows = 0;

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0; //, total_time = 0.0;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("Read file %s\n", filename);
   
   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   //error = rox_image_new_read_ppm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_pyramid_uchar_new(&pyramid, cols, rows, max_levels, min_size);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_timer_start(timer);

   // error = rox_pyramid_float_assign_nofiltering(pyramid, image_float);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
         
   error = rox_pyramid_uchar_assign(pyramid, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_pyramid_float_assign_gaussian(pyramid, image_float, 1.0);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Display elapsed time
   rox_timer_stop(timer);
   rox_timer_get_elapsed_ms(&time, timer);

   rox_log("time to compute a %d level pyramid of a (%d x %d) image = %f (ms)\n", max_levels, cols, rows, time);

   for (Rox_Sint level = 0; level < max_levels; level++)
   {
      Rox_Array2D_Uchar image_level = NULL;

      // Get pointer to image at desired level
      error = rox_pyramid_uchar_get_image(&image_level, pyramid, level);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_pyramid_uchar_del(&pyramid);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_pyramid_uchar_assign_gaussian)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
