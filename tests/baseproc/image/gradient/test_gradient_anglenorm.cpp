//==============================================================================
//
//    OPENROX   : File test_gradient_anglenorm.cpp
//
//  	Contents  : Tests for gradient_anglenorm.c
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
   #include <string.h>
   #include <system/time/timer.h>
   #include <baseproc/maths/maths_macros.h>
	#include <baseproc/image/gradient/gradient_anglenorm.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/image/gradient/gradientsobel.h>
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(gradient_anglenorm)

// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/cube/images/cube0104.pgm"
// #define TEST_IMG ROX_DATA_HOME"/devapps/model_based/engine/surfacepro4/2017-03-16_10-27-22/engine_surfacepro4_2017-03-16_10-27-22_img0027.pgm" 
// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/rectangle_detection/img_rectangle_detection_01.pgm" 

// #define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_3840x2160.pgm"

#define TEST_IMG ROX_DATA_HOME"/regression_tests/rox_opencl/mbo/capture1.pgm"

//#define TEST_IMG ROX_DATA_HOME"/regression_tests/openrox/image/test_image_random_1920x1080.pgm"

#define TEST_UNIT_IMG ROX_DATA_HOME"/regression_tests/openrox/image/gradient/image_gradient_normal_angle_%d.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_gradient_angle_norm_nomask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Timer timer = NULL ;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;

   Rox_Array2D_Float image_gradient_u = NULL;
   Rox_Array2D_Float image_gradient_v = NULL;

   Rox_Array2D_Float angle = NULL;
   Rox_Array2D_Float norm = NULL;

   Rox_Sint rows = 0, cols = 0;
   Rox_Double time = 0.0, total_time = 0.0;

   Rox_Float norm_threshold = 1.0;

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

   error = rox_array2d_float_new(&angle, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&norm, rows, cols);
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

      error = rox_array2d_float_gradient_angle_norm_nomask ( angle, norm, image_gradient_u, image_gradient_v, norm_threshold );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to compute the angle/norm of images gradient of a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nb_tests);

   // Test the image gradient with mask
   // To be implemented

   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &image_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &image_gradient_u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &image_gradient_v );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &angle );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &norm );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &image_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_gradient_angle_scale_nomask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Timer timer = NULL ;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;

   Rox_Array2D_Float image_gradient_u = NULL;
   Rox_Array2D_Float image_gradient_v = NULL;

   Rox_Array2D_Float angle = NULL;
   Rox_Array2D_Float scale = NULL;

   Rox_Sint rows = 0, cols = 0;
   Rox_Double time = 0.0, total_time = 0.0;

   Rox_Float scale_threshold = 1.0;

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

   error = rox_array2d_float_new(&angle, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new(&scale, rows, cols);
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

      error = rox_array2d_float_gradient_angle_scale_nomask ( angle, scale, image_gradient_u, image_gradient_v, scale_threshold );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to compute the angle/scale of images gradient of a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nb_tests);

   rox_array2d_float_save ("./ga.txt", angle );
   rox_array2d_float_save ("./gs.txt", scale );

   // Test the image gradient with mask
   // To be implemented

   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &image_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &image_gradient_u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &image_gradient_v );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &angle );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &scale );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &image_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_image_gradient_sobel_angle_scale_nomask_buffers)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Timer timer = NULL ;
   Rox_Array2D_Uchar image_uchar = NULL;
 
   Rox_Float *angle = NULL;
   Rox_Uint  *scale = NULL;

   Rox_Sint rows = 0, cols = 0;
   Rox_Double time = 0.0, total_time = 0.0;

   Rox_Uint scale_threshold = 120*120;

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

   angle = (Rox_Float *) rox_memory_allocate(sizeof(*angle), rows * cols);
   memset(angle, 0, sizeof(*angle) * rows * cols);

   scale = (Rox_Uint *) rox_memory_allocate(sizeof(*scale), rows * cols);
   memset(scale, 0, sizeof(*scale) * rows * cols);

   for(Rox_Sint i = 0; i < nb_tests; ++i)
   {
      rox_timer_start(timer);
      
      error = rox_image_gradient_sobel_angle_scale_nomask_buffers (angle, scale, image_uchar, scale_threshold);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   rox_log("mean time to compute the angle/norm of images gradient of a (%d x %d) image = %f (ms)\n", cols, rows, total_time/nb_tests);

   // Test the image gradient with mask
   // To be implemented

   //rox_image_del(I);
   //rox_igrad_del(G);
   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &image_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_memory_delete(angle);
   rox_memory_delete(scale);
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_image_gradient_sobel_angle_scale_orientation )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float angle = NULL; 
   Rox_Array2D_Uint  scale = NULL;
   Rox_Image image_gray = NULL;
   Rox_Sint scale_threshold = 0;
   Rox_Char filename[FILENAME_MAX];

   error = rox_array2d_uint_new ( &scale, 128, 128 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &angle, 128, 128 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_new ( &image_gray, 128, 128 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint a = 180; a > -180; a-=45)
   {
      sprintf(filename, TEST_UNIT_IMG, a);
      rox_log("read file %s\n", filename);

      error = rox_image_read_pgm ( image_gray, filename );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_image_gradient_sobel_angle_scale_nomask ( angle, scale, image_gray, scale_threshold );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
      Rox_Float ** da = NULL;
      error = rox_array2d_float_get_data_pointer_to_pointer ( &da, angle );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Check the measured angle
      Rox_Float angle_val = da[64][64];
      ROX_TEST_CHECK_CLOSE ( angle_val*180/ROX_PI, a, 1e-03 );

      rox_log("a     = %d \n", a);
      rox_log("angle = %f \n", angle_val*180/ROX_PI);
   }

   error = rox_image_del ( &image_gray );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del ( &scale );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &angle );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
