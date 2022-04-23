//==============================================================================
//
//    OPENROX   : File test_remap_bilinear_omo.cpp
//
//    Contents  : Tests for remap_bilinear_omo
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
   #include <baseproc/image/remap/remap_bilinear_omo_float_to_float/remap_bilinear_omo_float_to_float.h>
   #include <baseproc/image/remap/remap_bilinear_omo_uchar_to_uchar/remap_bilinear_omo_uchar_to_uchar.h>   
   #include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/array/fill/fillval.h>
   #include <baseproc/maths/linalg/matsl3.h>
   #include <baseproc/array/error/l2_error.h>
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/system/errors_print.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( remap_bilinear_omo )

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
   #define PRECISION 1e-4

#else
   #define RESULT_PATH "./"
   #define PRECISION 1e-12   
#endif


// Image 512 x  512
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
// Image 1280 x  720 =  921600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/projects/apra/img005.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_remap_bilinear_omo_float_to_float )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float l2_error = 0.0;
   Rox_Sint rows = 3, cols = 3;
   //Rox_Float image_inp_float_data[9] = {1,2,3,4,5,6,7,8,9};
   Rox_Float image_inp_float_data[9] = {  12, 23, 43,
                                          24, 35, 26,
                                          17, 38, 59 };
   // Rox_Float image_grt_float_data[9] = { 7,0,0,0,0,0,0,0,0};
   Rox_Float image_grt_float_data[9] = {  31.7500000000000000, 34.5000000000000000, 0.0000000000000000, 
                                          39.5000000000000000, 42.5000000000000000, 0.0000000000000000, 
                                          48.5000000000000000, 59.0000000000000000, 0.0000000000000000 };


   Rox_Array2D_Float image_inp_float = NULL;
   Rox_Array2D_Float image_out_float = NULL;
   Rox_Array2D_Uint mask_out = NULL;
   Rox_MeshGrid2D_Float grid = NULL;
   Rox_Array2D_Float image_grt_float = NULL;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   Rox_MatSL3 homography = NULL;
   error = rox_matsl3_new ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value ( homography, 0, 2, 1.5 ); // 1.5 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value ( homography, 1, 2, 0.5 ); // 1.5 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New current mask
   error = rox_array2d_uint_new ( &mask_out, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // New grt float image
   error = rox_array2d_float_new ( &image_grt_float, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( image_grt_float, image_grt_float_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New out float image
   error = rox_array2d_float_new ( &image_out_float, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New inp float image
   error = rox_array2d_float_new ( &image_inp_float, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( image_inp_float, image_inp_float_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New grid
   error = rox_meshgrid2d_float_new ( &grid, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Warp coordinates
   error = rox_warp_grid_sl3_float ( grid, homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_remap_bilinear_omo_float_to_float ( image_out_float, mask_out, image_inp_float, grid );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
   error = rox_array2d_float_difference_l2_norm_mask ( &l2_error, image_grt_float, image_out_float, mask_out );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error image_out = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, PRECISION);
   
   rox_array2d_float_print(image_out_float);
   rox_array2d_uint_print(mask_out);

   rox_array2d_float_save ( RESULT_PATH"/image_out_float_omo.txt", image_out_float );
   rox_array2d_uint_save ( RESULT_PATH"/mask_out_float_omo.txt", mask_out );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_remap_bilinear_omo_uchar_to_uchar )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float l2_error = 0.0;
   Rox_Sint rows = 3, cols = 3;
   //Rox_Uchar image_inp_uchar_data[9] = {1,2,3,4,5,6,7,8,9};
   Rox_Uchar image_inp_uchar_data[9] = {  12, 23, 43,
                                          24, 35, 26,
                                          17, 38, 59 };

   // Rox_Uchar image_grt_uchar_data[9] = {7,0,0,0,0,0,0,0,0};
   Rox_Uchar image_grt_uchar_data[9] = {  32, 35, 0, 
                                          40, 43, 0, 
                                          49, 59, 0 };

   Rox_Array2D_Uchar image_inp_uchar = NULL;
   Rox_Array2D_Uchar image_out_uchar = NULL;
   Rox_Array2D_Uint mask_out = NULL;
   Rox_MeshGrid2D_Float grid = NULL;
   Rox_Array2D_Uchar image_grt_uchar = NULL;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   Rox_MatSL3 homography = NULL;
   error = rox_matsl3_new ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value ( homography, 0, 2, 1.5);//1.5 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value ( homography, 1, 2, 0.5);//1.5 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matsl3_print ( homography );

   // New current mask
   error = rox_array2d_uint_new ( &mask_out, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // New grt uchar image
   error = rox_array2d_uchar_new ( &image_grt_uchar, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride ( image_grt_uchar, image_grt_uchar_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New out uchar image
   error = rox_array2d_uchar_new ( &image_out_uchar, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New inp uchar image
   error = rox_array2d_uchar_new ( &image_inp_uchar, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride ( image_inp_uchar, image_inp_uchar_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New grid
   error = rox_meshgrid2d_float_new ( &grid, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Warp coordinates
   error = rox_warp_grid_sl3_float ( grid, homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_remap_bilinear_omo_uchar_to_uchar ( image_out_uchar, mask_out, image_inp_uchar, grid );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
   error = rox_array2d_uchar_difference_l2_norm_mask ( &l2_error, image_grt_uchar, image_out_uchar, mask_out );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error image_out = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, PRECISION);
   
   rox_array2d_uchar_print(image_out_uchar);
   rox_array2d_uint_print(mask_out);

   rox_array2d_uchar_save ( RESULT_PATH"/image_out_uchar_omo.txt", image_out_uchar );
   rox_array2d_uint_save ( RESULT_PATH"/mask_out_uchar_omo.txt", mask_out );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_perf_remap_bilinear_omo_float_to_float )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Sint cols = 0, rows = 0;
   Rox_Double time = 0.0, total_time = 0.0;

   Rox_Sint rows_template = 256;
   Rox_Sint cols_template = 256;
   
#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 1000;
#endif

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   // Declare the source image
   Rox_Array2D_Uchar image_uchar = NULL;

   // Declare the source image
   Rox_Array2D_Float image_float = NULL;

   // Declare the current template
   Rox_Array2D_Float current = NULL;

   // Declare the remap grid
   Rox_MeshGrid2D_Float grid = NULL;

   // Declare the current mask
   Rox_Array2D_Uint current_mask = NULL;

   // Declare the homography matrix
   Rox_MatSL3 homography = NULL;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New homography
   error = rox_matsl3_new(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value(homography, 0, 2, 700);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value(homography, 1, 2, 200);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value(homography, 0, 0, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value(homography, 1, 1, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value(homography, 2, 2, 1/9);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   ROX_TEST_MESSAGE("read file %s\n", filename);

   // Read source image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New float image
   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New current template
   error = rox_array2d_float_new(&current, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New current mask
   error = rox_array2d_uint_new(&current_mask, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(current_mask, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New grid
   error = rox_meshgrid2d_float_new(&grid, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Warp coordinates
   error = rox_warp_grid_sl3_float(grid, homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   total_time = 0.0;
   for (Rox_Sint i = 0; i < nb_tests; i++)
   {
      rox_timer_start(timer);

      // Remap with bilinear interpolation
      error = rox_remap_bilinear_omo_float_to_float(current, current_mask, image_float, grid);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   ROX_TEST_MESSAGE("mean time to remap a (%d x %d) float image = %f (ms)\n", rows_template, cols_template, total_time/nb_tests);

   //error = rox_array2d_float_save("test_warp_image_float.txt", current);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&current);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uint_del(&current_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_meshgrid2d_float_del(&grid);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Sint rows_template = 256;
   Rox_Sint cols_template = 256;

   Rox_Double time = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 1000;
#endif

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   // Declare the source image
   Rox_Array2D_Uchar image_uchar = NULL;

   // Declare the current template
   Rox_Array2D_Uchar current = NULL;

   // Declare the remap grid
   Rox_MeshGrid2D_Float grid = NULL;

   // Declare the current mask
   Rox_Array2D_Uint current_mask = NULL;

   // Declare the homography matrix
   Rox_MatSL3 homography = NULL;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New homography
   error = rox_matsl3_new(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_double_set_value(homography, 0, 2, 700);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_double_set_value(homography, 1, 2, 200);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_double_set_value(homography, 0, 0, 3);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_double_set_value(homography, 1, 1, 3);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_double_set_value(homography, 2, 2, 1/9);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Read source image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New current template
   error = rox_array2d_uchar_new(&current, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New current mask
   error = rox_array2d_uint_new(&current_mask, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(current_mask, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New grid
   error = rox_meshgrid2d_float_new(&grid, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Warp coordinates
   error = rox_warp_grid_sl3_float(grid, homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   total_time = 0.0;
   for (Rox_Sint i = 0; i < nb_tests; i++)
   {
      rox_timer_start(timer);

      // Remap with bilinear interpolation
      error = rox_remap_bilinear_omo_uchar_to_uchar(current, current_mask, image_uchar, grid);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   ROX_TEST_MESSAGE("mean time to remap a (%d x %d) uchar image = %f (ms)\n", rows_template, cols_template, total_time/nb_tests);

   //error = rox_array2d_uchar_save("test_warp_image_uchar.txt", current);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &image_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uchar_del ( &current );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&current_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_meshgrid2d_float_del(&grid);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_perf_remap_bilinear_omo_uchar_to_uchar )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Sint rows_template = 512;
   Rox_Sint cols_template = 512;

   Rox_Double time = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 1000;
#endif

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   // Declare the source image
   Rox_Array2D_Uchar image_uchar = NULL;

   // Declare the current template
   Rox_Array2D_Uchar current = NULL;

   // Declare the remap grid
   Rox_MeshGrid2D_Float grid = NULL;

   // Declare the current mask
   Rox_Array2D_Uint current_mask = NULL;

   // Declare the homography matrix
   Rox_MatSL3 homography = NULL;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);
   
   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New homography set to identity
   error = rox_matsl3_new(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   ROX_TEST_MESSAGE("read file %s\n", filename);

   // Read source image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_array2d_uchar_save("test_warp_image_uchar_ident_source.txt", image_uchar);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New current template
   error = rox_array2d_uchar_new(&current, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New current mask
   error = rox_array2d_uint_new(&current_mask, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(current_mask, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New grid
   error = rox_meshgrid2d_float_new(&grid, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Warp coordinates
   error = rox_warp_grid_sl3_float(grid, homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   total_time = 0.0;
   for (Rox_Sint i = 0; i < nb_tests; i++)
   {
      rox_timer_start(timer);

      // Remap with bilinear interpolation
      error = rox_remap_bilinear_omo_uchar_to_uchar(current, current_mask, image_uchar, grid);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }
   ROX_TEST_MESSAGE("mean time to remap a (%d x %d) uchar image = %f (ms)\n", rows_template, cols_template, total_time/nb_tests);

   //error = rox_array2d_uchar_save("test_warp_image_uchar_ident.txt", current);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uchar_del(&current);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&current_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_meshgrid2d_float_del(&grid);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
