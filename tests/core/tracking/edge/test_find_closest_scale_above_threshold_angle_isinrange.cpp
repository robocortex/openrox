//==============================================================================
//
//    OPENROX   : File test_find_closest_scale_above_threshold_angle_isinrange.cpp
//
//    Contents  : Tests for scan_scale_angle_matrix.c
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
   #include <baseproc/maths/maths_macros.h>
   #include <baseproc/image/gradient/gradient_anglenorm.h>
	#include <core/tracking/edge/search_edge.h>
   #include <inout/image/pgm/pgmfile.h>
   #include <inout//numeric/array2d_save.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( find_closest_scale_above_threshold_angle_isinrange )

#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/rox_opencl/mbo/capture1.pgm"
#define LINES2D_PATH ROX_DATA_HOME"/regression_tests/rox_opencl/mbo/result_lines2d_c_mod_met.txt"
#define POINTS2D_PATH ROX_DATA_HOME"/regression_tests/rox_opencl/mbo/result_points2d_c_mod_pix.txt"

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_scan_scale_angle_matrix )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   const Rox_Uint search_range = 20;
   const Rox_Uint nbs = 2*search_range+1;

   const Rox_Uint threshold_response = 51;
   const Rox_Uint scale_threshold = 1;

   Rox_Uint nbp = 7538;

   Rox_Array2D_Uchar image = NULL;

   Rox_Array2D_Uint gradient_scale = NULL;
   Rox_Array2D_Float gradient_angle = NULL;

   Rox_Array2D_Double scan_gs = NULL;
   Rox_Array2D_Double scan_ga = NULL;
   
   Rox_Array2D_Double scanline_u = NULL;
   Rox_Array2D_Double scanline_v = NULL;
   
   Rox_Array2D_Double points2d = NULL;
   Rox_Array2D_Double lines2d = NULL;

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm ( &image, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint rows, cols;
   error = rox_array2d_uchar_get_size ( &rows, &cols, image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create gradient scale and angle 
   error = rox_array2d_uint_new ( &gradient_scale, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &gradient_angle, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Compute gradient scale and angle 
   error = rox_image_gradient_sobel_angle_scale_nomask ( gradient_angle, gradient_scale, image, scale_threshold );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Build the scanline matrix

   // Create and read points2d and lines2d 
   error = rox_array2d_double_new ( &points2d, nbp, 2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", POINTS2D_PATH);
   rox_log("read file %s\n", filename);

   error = rox_array2d_double_read ( points2d, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new ( &lines2d, nbp, 2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", LINES2D_PATH);
   rox_log("read file %s\n", filename);

   error = rox_array2d_double_read ( lines2d, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create scanline
   error = rox_array2d_double_new ( &scanline_u, nbp, nbs );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new ( &scanline_v, nbp, nbs );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_scanline_matrix ( scanline_u, scanline_v, points2d, lines2d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_double_save ( "./result_scanline_u.txt", scanline_u );
   rox_array2d_double_save ( "./result_scanline_v.txt", scanline_v );

   // Create scan scale and angle 
   error = rox_array2d_double_new ( &scan_gs, nbp, nbs );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new ( &scan_ga, nbp, nbs );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_scan_scale_angle_matrix ( scan_gs, scan_ga, scanline_u, scanline_v, gradient_scale, gradient_angle );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_double_save ( "./result_scan_gs.txt", scan_gs );
   rox_array2d_double_save ( "./result_scan_ga.txt", scan_ga );

   Rox_Double ** lines2d_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &lines2d_data, lines2d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new ( &points2d, nbp, 2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double ** points2d_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &points2d_data, points2d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for ( Rox_Uint k=0; k<nbp; k++ )
   {
      Rox_Point2D_Double_Struct point_meas; 

      Rox_Double angle_model = lines2d_data[k][1];
      Rox_Double angle_range = 20 * ROX_PI / 180;

      Rox_Array2D_Double scanline_u_row = NULL;
      error = rox_array2d_double_new_subarray2d( &scanline_u_row, scanline_u, k, 0, 1, nbs );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      Rox_Array2D_Double scanline_v_row = NULL;
      error = rox_array2d_double_new_subarray2d( &scanline_v_row, scanline_v, k, 0, 1, nbs );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      Rox_Array2D_Double scan_gs_row = NULL;
      error = rox_array2d_double_new_subarray2d( &scan_gs_row, scan_gs, k, 0, 1, nbs );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      Rox_Array2D_Double scan_ga_row = NULL;
      error = rox_array2d_double_new_subarray2d( &scan_ga_row, scan_ga, k, 0, 1, nbs );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_find_closest_scale_peak_above_threshold_angle_isinrange_row ( &point_meas,
      scanline_u_row,
      scanline_v_row,
      scan_gs_row,
      scan_ga_row,
      threshold_response*threshold_response,
      angle_model,
      angle_range );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      points2d_data[k][0] = point_meas.u;
      points2d_data[k][1] = point_meas.v;
   }

   rox_array2d_double_save ( "./result_points2d_c_mes_pix.txt", points2d );

   error = rox_array2d_uchar_del ( &image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_del ( &gradient_angle );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_uint_del ( &gradient_scale );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()
