//=============================================================================
//
//    OPENROX   : File test_region_zncc_search_mask_template_mask.cpp
//
//    Contents  : Tests for region_zncc_search_mask_template_mask.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//=============================================================================

//=== INCLUDED HEADERS   ======================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <generated/array2d_uchar.h>

   #include <system/time/timer.h>

   #include <baseproc/array/fill/fillval.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>

   #include <inout/image/pgm/pgmfile.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/system/print.h>

   #include <core/templatesearch/region_zncc_search_mask_template_mask.h>
   #include <core/templatesearch/region_zebc_search_mask_template_mask.h>
}

//=== INTERNAL MACROS    ======================================================

ROX_TEST_SUITE_BEGIN(region_zncc_search_mask_template_mask)

// Image 512  x  512 =  262144 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/rox_framework/odometry/edges/references/mbo/switch_lumia_seq1_img0001.pgm"

//=== INTERNAL TYPESDEFS ======================================================

//=== INTERNAL DATATYPES ======================================================

//=== INTERNAL VARIABLES ======================================================

//=== INTERNAL FUNCTDEFS ======================================================

//=== INTERNAL FUNCTIONS ======================================================

//=== EXPORTED FUNCTIONS ======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_region_zncc_search_mask_template_mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Float res_score = 0.0;
   Rox_Sint res_topleft_x = 0, res_topleft_y = 0;

   Rox_Array2D_Float image_search = NULL;
   Rox_Array2D_Uint image_search_mask = NULL;
   Rox_Array2D_Float image_template = NULL;
   Rox_Array2D_Uint image_template_mask = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;

   Rox_Sint cols_template = 80, rows_template = 40;
   Rox_Sint cols_search = 160, rows_search = 80;
   // Rox_Sint cols_search = 320, rows_search = 160;
   // Rox_Sint cols_search = 640, rows_search = 320;

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Sint nbt = 1;
   
   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   ROX_TEST_MESSAGE("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("rows = %d\n", rows);
   ROX_TEST_MESSAGE("cols = %d\n", cols);

   // Create the float image
   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the search image
   error = rox_array2d_float_new_subarray2d(&image_search, image_float, 100, 100, rows_search, cols_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&image_search_mask, rows_search, cols_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(image_search_mask, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new_subarray2d(&image_template, image_search, 20, 40, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_array2d_float_save("test_image_template.txt", image_template);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_array2d_float_save("test_image_search.txt", image_search);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_float_new(image_template, rows, cols);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&image_template_mask, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(image_template_mask, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for(Rox_Sint i = 0; i < nbt; i++)
   {
      rox_timer_start(timer);

      error = rox_array2d_float_region_zncc_search_mask_template_mask(&res_score, &res_topleft_x, &res_topleft_y, image_search, image_search_mask, image_template, image_template_mask);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }

   ROX_TEST_MESSAGE("time to search a (%d x %d) template in a (%d x %d) image with zncc = %f (ms)\n", cols_template, rows_template, cols_search, rows_search, total_time/nbt);

   ROX_TEST_MESSAGE("res_score = %f\n", res_score);
   ROX_TEST_MESSAGE("res_topleft_x = %d\n", res_topleft_x);
   ROX_TEST_MESSAGE("res_topleft_y = %d\n", res_topleft_y);

   ROX_TEST_CHECK_CLOSE (res_score, 1.0, 1e-12);

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&image_template_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&image_search_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_search);
   error = rox_array2d_float_del(&image_template);

}


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_uchar_region_zncc_search_mask_template_mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Float res_score = 0.0;
   Rox_Sint res_topleft_x = 0, res_topleft_y = 0;

   Rox_Array2D_Uchar image_search = NULL;
   Rox_Array2D_Uint image_search_mask = NULL;
   Rox_Array2D_Uchar image_template = NULL;
   Rox_Array2D_Uint image_template_mask = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;

   Rox_Sint cols_template = 80, rows_template = 40;
   Rox_Sint cols_search = 160, rows_search = 80;
   // Rox_Sint cols_search = 320, rows_search = 160;
   // Rox_Sint cols_search = 640, rows_search = 320;

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Sint nbt = 1;
   
   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   ROX_TEST_MESSAGE("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("rows = %d\n", rows);
   ROX_TEST_MESSAGE("cols = %d\n", cols);

   // Create the search image
   error = rox_array2d_uchar_new_subarray2d(&image_search, image_uchar, 100, 100, rows_search, cols_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&image_search_mask, rows_search, cols_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(image_search_mask, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new_subarray2d(&image_template, image_search, 20, 40, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_array2d_float_save("test_image_template.txt", image_template);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_array2d_float_save("test_image_search.txt", image_search);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_float_new(image_template, rows, cols);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&image_template_mask, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(image_template_mask, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for(Rox_Sint i = 0; i < nbt; i++)
   {
      rox_timer_start(timer);

      error = rox_array2d_uchar_region_zncc_search_mask_template_mask(&res_score, &res_topleft_x, &res_topleft_y, image_search, image_search_mask, image_template, image_template_mask);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }

   ROX_TEST_MESSAGE("time to search a (%d x %d) template in a (%d x %d) image with zncc = %f (ms)\n", cols_template, rows_template, cols_search, rows_search, total_time/nbt);

   ROX_TEST_MESSAGE("res_score = %f\n", res_score);
   ROX_TEST_MESSAGE("res_topleft_x = %d\n", res_topleft_x);
   ROX_TEST_MESSAGE("res_topleft_y = %d\n", res_topleft_y);

   ROX_TEST_CHECK_CLOSE (res_score, 1.0, 1e-12);

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );


   error = rox_array2d_uint_del(&image_template_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&image_search_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_region_zebc_search_mask_template_mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];
   Rox_Float res_score = 0.0, zncc_threshold = 0.9;
   Rox_Sint res_topleft_x = 0, res_topleft_y = 0;

   Rox_Array2D_Float image_search = NULL;
   Rox_Array2D_Uint image_search_mask = NULL;
   Rox_Array2D_Float image_template = NULL;
   Rox_Array2D_Uint image_template_mask = NULL;
   Rox_Array2D_Uchar image_uchar = NULL;
   Rox_Array2D_Float image_float = NULL;

   Rox_Sint cols_template = 80, rows_template = 40;
   Rox_Sint cols_search = 160, rows_search = 80;
   // Rox_Sint cols_search = 320, rows_search = 160;
   // Rox_Sint cols_search = 640, rows_search = 320;

   // Define timer to measure performances
   Rox_Timer timer = NULL;
   Rox_Double time = 0.0, total_time = 0.0;
   Rox_Sint nbt = 1;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   ROX_TEST_MESSAGE("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image_uchar, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("rows = %d\n", rows);
   ROX_TEST_MESSAGE("cols = %d\n", cols);

   // Create the float image
   error = rox_array2d_float_new(&image_float, rows, cols);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // convert image to float
   error = rox_array2d_float_from_uchar_normalize(image_float, image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the search image
   error = rox_array2d_float_new_subarray2d(&image_search, image_float, 100, 100, rows_search, cols_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&image_search_mask, rows_search, cols_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(image_search_mask, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new_subarray2d(&image_template, image_search, 20, 40, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_float_save("test.txt", image_template);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_array2d_float_new(image_template, rows, cols);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new(&image_template_mask, rows_template, cols_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_fillval(image_template_mask, ~0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for(Rox_Sint i = 0; i < nbt; i++)
   {
      rox_timer_start(timer);

      error = rox_array2d_float_region_zebc_search_mask_template_mask ( &res_score, &res_topleft_x, &res_topleft_y, image_search, image_search_mask, image_template, image_template_mask, zncc_threshold);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time, timer);
      total_time += time;
   }

   ROX_TEST_MESSAGE("time to search a (%d x %d) template in a (%d x %d) image with zebc = %f (ms)\n", cols_template, rows_template, cols_search, rows_search, total_time/nbt);

   ROX_TEST_MESSAGE("res_score = %f\n", res_score);
   ROX_TEST_MESSAGE("res_topleft_x = %d\n", res_topleft_x);
   ROX_TEST_MESSAGE("res_topleft_y = %d\n", res_topleft_y);
   
   ROX_TEST_CHECK_CLOSE (res_score, 1.0, 1e-12);

   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del(&image_uchar);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_float);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&image_template_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del(&image_search);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&image_search_mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()

// blocksize  1 : 10.102383 (ms)
// blocksize  2 :  6.907996 (ms)
// blocksize  3 :  6.040787 (ms)
// blocksize  4 :  5.578322 (ms)
// blocksize  5 :  5.377304 (ms)
// blocksize  6 :  5.391598 (ms)
// blocksize  7 :  5.188751 (ms)
// blocksize  8 :  5.314936 (ms)
// blocksize  9 :  7.529428 (ms); problem res_score = 0.991266
// blocksize 10 :  5.467258 (ms)
// blocksize 11 :  5.453492 (ms)
// blocksize 12 :  7.806251 (ms); problem res_score = 0.983819
// blocksize 13 :  7.843951 (ms); problem res_score = 0.991266
// blocksize 14 :  9.884916 (ms); problem res_score = 0.975484
// blocksize 15 :  6.529047 (ms);
// blocksize 16 :  9.441424 (ms); problem res_score = 0.991266
// blocksize 17 : 11.326278 (ms); problem res_score = 0.983819
// blocksize 18 :  7.564587 (ms);
// blocksize 19 : 14.565908 (ms); problem res_score = 0.000000
// blocksize 20 : problem
// blocksize 25 :  6.611893 (ms)
// blocksize 40 :  9.073418 (ms);
