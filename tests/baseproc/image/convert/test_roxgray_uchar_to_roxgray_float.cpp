//==============================================================================
//
//    OPENROX   : File test_roxgray_uchar_to_roxgray_float.cpp
//
//    Contents  : Tests for roxgray_uchar_to_roxgray_float.c
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
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/image/ppm/ppmfile.h>
   #include <baseproc/image/convert/roxgray_to_gray.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(roxgray_uchar_to_roxgray_float)

// Image 4096 x 2160 = 8 847 360 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_4096x2160.pgm"
// Image 553 x 768 =  424 704 pixels
//#define IMAGE_PATH "/home/emalis/Github/demos/rox_gui/desktop/generic/build/test.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_roxgray_uchar_to_gray)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   Rox_Image image_gray_uchar = NULL;
   Rox_Image_Float image_gray_float = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Sint cols = 0, rows = 0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 100;
#endif

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_image_new_read_pgm(&image_gray_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size(&rows, &cols, image_gray_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &image_gray_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint i = 0; i < nb_tests; i++)
   {
      rox_timer_start(timer);

      error = rox_array2d_float_from_uchar ( image_gray_float, image_gray_uchar );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to convert a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nb_tests);

   error = rox_image_del ( &image_gray_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &image_gray_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
