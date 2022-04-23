//==============================================================================
//
//    OPENROX   : File test_remap_box_halved.cpp
//
//    Contents  : Tests for remap_box_halved.c
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
	#include <baseproc/image/remap/remap_box_halved/remap_box_halved.h>
   #include <baseproc/array/conversion/array2d_float_from_uchar.h>
   #include <baseproc/array/error/l2_error.h>
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/system/print.h>
   #include <system/time/timer.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( remap_box_halved )

//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/debug/america_1920x1080.pgm"
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/debug/earth_4000x3000.pgm"
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_array2d_uchar_remap_halved_box )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float l2_error = 0.0;

   Rox_Uchar source_data[8*8] = {   228,   208,    90,    98,   146,    43,    59,    28,
                                    245,    63,   212,   145,   120,   154,   233,   246,
                                    140,   237,   150,    20,     4,    68,    39,     2,
                                     36,    90,   141,    14,    86,   167,   211,   198,
                                     39,    51,   234,   136,    42,   176,   138,   209,
                                     66,    65,    73,   199,   203,   191,   255,   222,
                                    215,   158,   194,   239,    80,   115,    20,    22,
                                     65,   121,   193,    34,   135,    22,   113,   102};

   Rox_Uchar halved_grt_data[4*4] = {  186, 136, 115, 141, 
                                       125,  81,  81, 112, 
                                        55, 160, 153, 206, 
                                       139, 165,  88,  64};

   Rox_Sint rows = 8;
   Rox_Sint cols = 8;

   Rox_Array2D_Uchar source = NULL;
   Rox_Array2D_Uchar halved = NULL;
   Rox_Array2D_Uchar halved_grt = NULL;

   error = rox_array2d_uchar_new ( &source, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride ( source, source_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new ( &halved, rows/2, cols/2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_new ( &halved_grt, rows/2, cols/2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_set_buffer_no_stride ( halved_grt, halved_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_remap_box_nomask_uchar_to_uchar_halved ( halved, source );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_uchar_print ( halved );

   error = rox_array2d_uchar_difference_l2_norm ( &l2_error, halved_grt, halved );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   rox_log("l2_error halved = %0.12f \n", l2_error);

   error = rox_array2d_uchar_del ( &source );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &halved );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &halved_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_perf_array2d_uchar_remap_halved_box )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Uchar dest = NULL;
   Rox_Array2D_Uchar source = NULL;
   Rox_Char filename[FILENAME_MAX];
   Rox_Sint rows = 0, cols = 0;
   Rox_Double time_ms = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 1000;
#endif

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Read source image
   error = rox_array2d_uchar_new_pgm ( &source, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_get_size ( &rows, &cols, source );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New float image
   error = rox_array2d_uchar_new ( &dest, rows/2, cols/2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   for (Rox_Sint i = 0; i < nb_tests; ++i)
   {  
      rox_timer_start(timer);

      error = rox_remap_box_nomask_uchar_to_uchar_halved(dest, source);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time_ms, timer);
      total_time += time_ms;
   }

   rox_log("mean time for halving a (%d x %d) uchar image = %f (ms)\n", cols, rows, total_time/nb_tests);

   error = rox_array2d_uchar_del ( &source );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &dest );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_array2d_float_remap_halved_box )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float l2_error = 0.0;

   Rox_Float source_data[8*8] = {   228,   208,    90,    98,   146,    43,    59,    28,
                                    245,    63,   212,   145,   120,   154,   233,   246,
                                    140,   237,   150,    20,     4,    68,    39,     2,
                                     36,    90,   141,    14,    86,   167,   211,   198,
                                     39,    51,   234,   136,    42,   176,   138,   209,
                                     66,    65,    73,   199,   203,   191,   255,   222,
                                    215,   158,   194,   239,    80,   115,    20,    22,
                                     65,   121,   193,    34,   135,    22,   113,   102};

   Rox_Float halved_grt_data[4*4] = {  186.00, 136.25, 115.75, 141.50, 
                                       125.75,  81.25,  81.25, 112.50, 
                                        55.25, 160.50, 153.00,  206.0, 
                                       139.75, 165.00,  88.00,  64.25};

   Rox_Sint rows = 8;
   Rox_Sint cols = 8;

   Rox_Array2D_Float source = NULL;
   Rox_Array2D_Float halved = NULL;
   Rox_Array2D_Float halved_grt = NULL;

   error = rox_array2d_float_new ( &source, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( source, source_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &halved, rows/2, cols/2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &halved_grt, rows/2, cols/2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( halved_grt, halved_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_remap_box_nomask_float_to_float_halved ( halved, source );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_array2d_float_print ( halved );

   error = rox_array2d_float_difference_l2_norm ( &l2_error, halved_grt, halved );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   rox_log("l2_error halved = %0.12f \n", l2_error);

   error = rox_array2d_float_del ( &source );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &halved );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &halved_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_array2d_float_remap_halved_box_perf )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float dest_float = NULL;
   Rox_Array2D_Float source_float = NULL;
   Rox_Array2D_Uchar source_uchar = NULL;
   Rox_Char filename[FILENAME_MAX];
   Rox_Sint rows = 0, cols = 0;
   Rox_Double time_ms = 0.0, total_time = 0.0;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 1000;
#endif

   // Define timer to measure performances
   Rox_Timer timer = NULL;

   // Init new timer
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Read source image
   error = rox_array2d_uchar_new_pgm ( &source_uchar, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new_from_uchar ( &source_float, source_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uchar_get_size ( &rows, &cols, source_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // New float image
   error = rox_array2d_float_new ( &dest_float, rows/2, cols/2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Sint i = 0; i < nb_tests; ++i)
   {  
      rox_timer_start(timer);

      error = rox_remap_box_nomask_float_to_float_halved ( dest_float, source_float );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time_ms, timer);
      total_time += time_ms;
   }

   rox_log("mean time for halving a (%d x %d) float image = %f (ms)\n", cols, rows, total_time/nb_tests);
   
   rox_array2d_float_save ( "test_halvedbox.txt", dest_float);

   error = rox_array2d_uchar_del ( &source_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_del ( &source_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &dest_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_timer_del ( &timer );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
} 

ROX_TEST_SUITE_END()
