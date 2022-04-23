//==============================================================================
//
//    OPENROX   : File test_remap_bilinear_nomask.cpp
//
//    Contents  : Tests for remap_bilinear_nomask.c
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
   #include <baseproc/maths/linalg/matsl3.h>
   #include <baseproc/array/error/l2_error.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
   #include <baseproc/image/remap/remap_bilinear_nomask_float_to_float/remap_bilinear_nomask_float_to_float.h>
	#include <baseproc/image/remap/remap_bilinear_nomask_uchar_to_uchar/remap_bilinear_nomask_uchar_to_uchar.h>
   #include <inout/numeric/array2d_save.h>
   #include <inout/numeric/array2d_print.h>   
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( remap_bilinear_nomask )

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
   #define PRECISION 1e-4

#else
   #define RESULT_PATH "./"
   #define PRECISION 1e-12   
#endif

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_remap_bilinear_nomask_float_to_float )
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

   error = rox_remap_bilinear_nomask_float_to_float ( image_out_float, image_inp_float, grid );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
   error = rox_array2d_float_difference_l2_norm ( &l2_error, image_grt_float, image_out_float );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error image_out = %f \n", l2_error);

   rox_array2d_float_print(image_out_float);
   rox_array2d_float_save ( RESULT_PATH"/image_remap_bilinear_nomask_float_to_float.txt", image_out_float );

   // ROX_TEST_CHECK_CLOSE (l2_error, 0.0, PRECISION);
}


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_remap_bilinear_nomask_uchar_to_uchar )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float l2_error = 0.0;
   Rox_Sint rows = 3, cols = 3;
   //Rox_Uchar image_inp_uchar_data[9] = {1,2,3,4,5,6,7,8,9};
   Rox_Uchar image_inp_uchar_data[9] = {  12, 23, 43,
                                          24, 35, 26,
                                          17, 38, 59 };
   // Rox_Uchar image_grt_uchar_data[9] = { 7,0,0,0,0,0,0,0,0};
   Rox_Uchar image_grt_uchar_data[9] = {  32, 35, 0, 
                                          40, 43, 0, 
                                          49, 59, 0 };


   Rox_Array2D_Uchar image_inp_uchar = NULL;
   Rox_Array2D_Uchar image_out_uchar = NULL;
   Rox_MeshGrid2D_Float grid = NULL;
   Rox_Array2D_Uchar image_grt_uchar = NULL;

   // Forward openrox callback to allow print of test_runner (needed for Android)
   rox_log_set_callback(RoxTest::_log_callback);

   Rox_MatSL3 homography = NULL;
   error = rox_matsl3_new ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value ( homography, 0, 2, 1.5 ); // 1.5 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value ( homography, 1, 2, 0.5 ); // 1.5 );
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

   error = rox_remap_bilinear_nomask_uchar_to_uchar ( image_out_uchar, image_inp_uchar, grid );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
      
   error = rox_array2d_uchar_difference_l2_norm ( &l2_error, image_grt_uchar, image_out_uchar );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error image_out = %f \n", l2_error);

   //ROX_TEST_CHECK_CLOSE (l2_error, 0.0, PRECISION);
   
   rox_array2d_uchar_print(image_out_uchar);

   rox_array2d_uchar_save ( RESULT_PATH"/image_remap_bilinear_nomask_uchar_to_uchar.txt", image_out_uchar );
}


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_remap_bilinear_nomask_uchar_fixed)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_remap_bilinear_nomask_uint)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_remap_bilinear_nomask_rgba)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_remap_bilinear_nomask_rgba_fixed)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
