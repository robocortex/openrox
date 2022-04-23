//==============================================================================
//
//    OPENROX   : File test_pyramid_float.cpp
//
//    Contents  : Tests for pyramid_float.c
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
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/image/pyramid/pyramid_float.h>
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/numeric/array2d_print.h>
   #include <generated/array2d_uchar.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(pyramid_float)

// Image  512  x   512 =    262 144 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
// Image  640  x   480 =    307 200 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_640x480.pgm"
// Image 1280  x   720 =    921 600 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1280x720.pgm"
// Image 1920  x  1080 =  2 073 600 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1920x1080.pgm"
// Image 3840  x  2160 =  8 294 400 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_3840x2160.pgm"
// Image 4096  x  2160 =  8 847 360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_4096x2160.pgm"
// Image 5120  x  3200 = 16 384 000 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_5120x3200.pgm"

// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/debug/america_1920x1080.pgm"

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_pyramid_float_new_del)
{
   const Rox_Sint max_levels = 4;
   const Rox_Sint min_size = 32;

   Rox_ErrorCode error = ROX_ERROR_NONE;
   //Rox_Char filename[FILENAME_MAX];

   Rox_Pyramid_Float pyramid = NULL;

   Rox_Sint cols = 640;
   Rox_Sint rows = 480;
   
   error = rox_pyramid_float_new(&pyramid, cols, rows, max_levels, min_size);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_pyramid_float_del(&pyramid);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_pyramid_float_assign_nofiltering)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE("This test has not been implemented yet !!! \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_pyramid_float_assign)
{
   const Rox_Sint max_levels = 4;
   const Rox_Sint min_size = 32;

   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Pyramid_Float pyramid = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;

   Rox_Sint cols = 0;
   Rox_Sint rows = 0;

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time_ms = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 1000;
#endif

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);
   
   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   ROX_TEST_MESSAGE("Read file %s\n", filename);
   
   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   //error = rox_image_new_read_ppm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the float image
   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_pyramid_float_new(&pyramid, cols, rows, max_levels, min_size);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint i = 0; i < nb_tests; ++i)
   {  
      rox_timer_start(timer);

      // error = rox_pyramid_float_assign_nofiltering(pyramid, image_float);
      // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
         
      error = rox_pyramid_float_assign ( pyramid, image_float );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // error = rox_pyramid_float_assign_gaussian(pyramid, image_float, 1.0);
      // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time_ms, timer);
      total_time += time_ms;
   }

   ROX_TEST_MESSAGE("time to compute a %d level pyramid of a (%d x %d) image = %f (ms)\n", max_levels, cols, rows, total_time/nb_tests);

   for (Rox_Sint level = 0; level < max_levels; level++)
   {
      Rox_Array2D_Float image_level = NULL;

      // Get pointer to image at desired level
      error = rox_pyramid_float_get_image(&image_level, pyramid, level);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Save image
      // sprintf(filename, "./test_pyramid_image_level_%d.txt", level);
      // error = rox_array2d_float_save(filename, image_level);
      // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_pyramid_float_del(&pyramid);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_pyramid_float_assign_gaussian)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE("This test has not been implemented yet !!! \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
