//==============================================================================
//
//    OPENROX   : File test_gradient_sobel.cpp
//
//    Contents  : Tests for gradient_sobel.c
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
   #include <generated/array2d_point2d_sshort.h>
   #include <system/time/timer.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/image/gradient/gradientsobel.h>
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(gradient_sobel)

// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
// #define TEST_IMG ROX_DATA_HOME"/devapps/model_based/engine/surfacepro4/2017-03-16_10-27-22/engine_surfacepro4_2017-03-16_10-27-22_img0027.pgm" 
// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/debug/earth_4000x3000.pgm"
// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/debug/america_1920x1080.pgm"
// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_3840x2160.pgm"
// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1920x1080.pgm"
#define TEST_IMG ROX_DATA_HOME"/regression_tests/rox_opencl/mbo/capture1.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_gradientsobel_nomask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Timer timer = NULL ;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;

   Rox_Array2D_Float image_gradient_u = NULL;
   Rox_Array2D_Float image_gradient_v = NULL;

   Rox_Sint rows = 0, cols = 0;
   Rox_Double time = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 100;
#endif

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&image_gradient_u, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&image_gradient_v, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test the image gradient without mask
   for(Rox_Sint i = 0; i < nb_tests; ++i)
   {
      rox_timer_start(timer);

      error = rox_array2d_float_gradientsobel_nomask ( image_gradient_u, image_gradient_v, image_float );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to compute the sobel gradient of a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nb_tests);

   rox_array2d_float_save ("./gu.txt", image_gradient_u );
   rox_array2d_float_save ("./gv.txt", image_gradient_v );

   // Test the image gradient with mask
   // To be implemented

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_gradient_u);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_gradient_v);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uchar_gradientsobel_nomask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Timer timer = NULL ;
   Rox_Array2D_Uchar image_uchar = NULL;

   Rox_Array2D_Point2D_Sshort image_gradient = NULL;

   Rox_Sint rows = 0, cols = 0;
   Rox_Double time = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 100;
#endif

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_point2d_sshort_new(&image_gradient, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test the image gradient without mask

   for(Rox_Sint i = 0; i < nb_tests; ++i)
   {
      rox_timer_start(timer);

      error = rox_array2d_uchar_gradientsobel_nomask(image_gradient, image_uchar);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to compute the sobel gradient of a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nb_tests);

   //rox_igrad_savetxt_deru ("./res/deru.txt", G);
   //rox_igrad_savetxt_derv ("./res/derv.txt", G);

   // Test the image gradient with mask
   // To be implemented

   //rox_image_del(I);
   //rox_igrad_del(G);

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_point2d_sshort_del(&image_gradient);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_sint_gradientsobel_nomask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Timer timer = NULL ;
   Rox_Array2D_Uchar image_uchar = NULL;

   Rox_Array2D_Sint image_gradient_u = NULL;
   Rox_Array2D_Sint image_gradient_v = NULL;

   Rox_Sint rows = 0, cols = 0;
   Rox_Double time = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 100;
#endif

   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", TEST_IMG);
   rox_log("read file %s\n", filename);

   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_sint_new(&image_gradient_u, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_sint_new(&image_gradient_v, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test the image gradient without mask

   for(Rox_Sint i = 0; i < nb_tests; ++i)
   {
      rox_timer_start(timer);

      error = rox_array2d_sint_gradientsobel_nomask(image_gradient_u, image_gradient_v, image_uchar);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to compute the sobel gradient of a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nb_tests);

   // Test the image gradient with mask
   // To be implemented

   //rox_image_del(I);
   //rox_igrad_del(G);

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_sint_del(&image_gradient_u);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_sint_del(&image_gradient_v);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
