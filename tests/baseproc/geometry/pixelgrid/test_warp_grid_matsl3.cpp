//==============================================================================
//
//    OPENROX   : File test_warp_grid_matsl3.cpp
//
//    Contents  : Tests for warp_grid_matsl3.c
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
   #include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matsl3_fixed12_4.h>
   #include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>
   #include <baseproc/array/conversion/array2d_double_from_uchar.h>

   #include <inout/geometry/point/array2d_point2d_print.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( warp_grid_matsl3 )

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
   #define PRECISION 1e-4
#else
   #define RESULT_PATH "./"
   #define PRECISION 1e-6
#endif

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_warp_grid_sl3_float)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint width = 4;
   Rox_Sint height = 4;

   // Create the output grid array
   Rox_MeshGrid2D_Float grid = NULL;
   error = rox_meshgrid2d_float_new ( &grid, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Float ** grid_data_u = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_data_u, grid->u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Float ** grid_data_v = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_data_v, grid->v );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the reference array
   Rox_MeshGrid2D_Float ref = NULL;
   error = rox_meshgrid2d_float_new ( &ref, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Float ** dRef_u = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dRef_u, ref->u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Float ** dRef_v = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dRef_v, ref->v );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   dRef_u[0][0] = 0.294999986886978; dRef_v[0][0] = 0;
   dRef_u[1][0] = 0.294999986886978; dRef_v[1][0] = 1;
   dRef_u[2][0] = 0.294999986886978; dRef_v[2][0] = 2;
   dRef_u[3][0] = 0.294999986886978; dRef_v[3][0] = 3;

   dRef_u[0][1] = 1.884999990463257; dRef_v[0][1] = 0;
   dRef_u[1][1] = 1.884999990463257; dRef_v[1][1] = 1;
   dRef_u[2][1] = 1.884999990463257; dRef_v[2][1] = 2;
   dRef_u[3][1] = 1.884999990463257; dRef_v[3][1] = 3;

   dRef_u[0][2] = 3.475000143051148; dRef_v[0][2] = 0;
   dRef_u[1][2] = 3.475000143051148; dRef_v[1][2] = 1;
   dRef_u[2][2] = 3.475000143051148; dRef_v[2][2] = 2;
   dRef_u[3][2] = 3.475000143051148; dRef_v[3][2] = 3;

   dRef_u[0][3] = 5.065000057220459; dRef_v[0][3] = 0;
   dRef_u[1][3] = 5.065000057220459; dRef_v[1][3] = 1;
   dRef_u[2][3] = 5.065000057220459; dRef_v[2][3] = 2;
   dRef_u[3][3] = 5.065000057220459; dRef_v[3][3] = 3;

   // Create the homography
   Rox_Array2D_Double homography = NULL;
   error = rox_array2d_double_new(&homography, 3, 3);

   Rox_Double ** dH = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dH, homography );
   
   dH[0][0] = 1.59;
   dH[1][0] = 0;
   dH[2][0] = 0;
   dH[0][1] = 0;
   dH[1][1] = 1;
   dH[2][1] = 0;
   dH[0][2] = 0.295;
   dH[1][2] = 0;
   dH[2][2] = 1;

   // Compute remap parameters
   error = rox_warp_grid_sl3_float(grid, homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("Checking warp_grid_sl3_float...\n");
   for (Rox_Sint i = 0; i < height; i++) 
   {
      for (Rox_Sint j = 0; j < width; j++) 
      {
         // 1e-6 needed since sse use float while ansi use double
         ROX_TEST_MESSAGE("grid_data_u[i][j] = %f\n", grid_data_u[i][j]);
         ROX_TEST_MESSAGE("dRef_u[i][j] = %f\n", dRef_u[i][j]);
         ROX_TEST_MESSAGE("grid_data_v[i][j] = %f\n", grid_data_v[i][j]);
         ROX_TEST_MESSAGE("dRef_v[i][j] = %f\n", dRef_v[i][j]);

         ROX_TEST_CHECK_SMALL((double) (grid_data_u[i][j] - dRef_u[i][j]), PRECISION);
         ROX_TEST_CHECK_SMALL((double) (grid_data_v[i][j] - dRef_v[i][j]), PRECISION);
      }
   }

   rox_array2d_double_del ( &homography );
   rox_meshgrid2d_float_del ( &grid );
   rox_meshgrid2d_float_del ( &ref );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_warp_grid_sl3_float_time)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
 
   Rox_Double time_ms = 0.0, total_time_ms = 0.0;
   Rox_Sint cols = 1920, rows = 1080;

#ifdef DEBUG
   // Small number of tests for slow debug (e.g. with valgrind)
   Rox_Sint nb_tests = 1;
#else 
   // High number of tests for measuring average performance in release
   Rox_Sint nb_tests = 100;
#endif

   // Define timer to measure performances 
   Rox_Timer timer = NULL;
   
   // Init new timer 
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the output grid array
   Rox_MeshGrid2D_Float grid = NULL;
   error = rox_meshgrid2d_float_new ( &grid, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the homography
   Rox_Array2D_Double homography;
   error = rox_array2d_double_new(&homography, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double ** dH = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dH, homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   dH[0][0] = 1.590;
   dH[1][0] = 0.020;
   dH[2][0] = 0.220;
   dH[0][1] = 0.010;
   dH[1][1] = 1.230;
   dH[2][1] = 0.030;
   dH[0][2] = 0.295;
   dH[1][2] = 0.321;
   dH[2][2] = 1.032;

   total_time_ms = 0.0;

   for (Rox_Sint i = 0; i < nb_tests; i++)
   {
      rox_timer_start(timer);

      // Compute remap parameters
      error = rox_warp_grid_sl3_float(grid, homography);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      // Display elapsed time 
      rox_timer_stop(timer);
      rox_timer_get_elapsed_ms(&time_ms, timer);
      total_time_ms += time_ms;
   }
      

   ROX_TEST_MESSAGE("mean time to warp (%d x %d) grid = %f (ms)\n", cols, rows, total_time_ms/nb_tests);
   
   /*
   ROX_TEST_MESSAGE("Checking warp_grid_sl3_float...\n");
   for (Rox_Uint i = 0; i < height; i++) 
   {
      for (Rox_Uint j = 0; j < width; j++) 
      {
         // 1e-6 needed since sse use float while ansi use double
         ROX_TEST_CHECK_SMALL((double) (grid_data[i][j].u - dRef[i][j].u), PRECISION);
         ROX_TEST_CHECK_SMALL((double) (grid_data[i][j].v - dRef[i][j].v), PRECISION);
      }
   }
   */
   rox_array2d_double_del(&homography);
   rox_meshgrid2d_float_del(&grid);
   rox_timer_del(&timer);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_warp_grid_sl3_fixed12_4)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint width = 4;
   Rox_Sint height = 4;

   #ifdef old
   // Create the output grid array
   Rox_Array2D_Point2D_Sshort grid = NULL;
   error = rox_array2d_point2d_sshort_new(&grid, height, width);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Point2D_Sshort_Struct ** grid_data = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer ( &grid_data, grid );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the reference array
   Rox_Array2D_Point2D_Sshort ref = NULL;
   error = rox_array2d_point2d_sshort_new ( &ref, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Point2D_Sshort_Struct ** dRef = NULL;
   rox_array2d_point2d_sshort_get_data_pointer_to_pointer ( &dRef, ref );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   #endif

   // ------------------------------------------------------------------------------

   // Create the output grid array
   Rox_MeshGrid2D_Sshort grid = NULL;
   error = rox_meshgrid2d_sshort_new ( &grid, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sshort ** grid_data_u = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer ( &grid_data_u, grid->u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Sshort ** grid_data_v = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer ( &grid_data_v, grid->v );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Create the reference array
   Rox_MeshGrid2D_Sshort ref = NULL;
   error = rox_meshgrid2d_sshort_new ( &ref, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Sshort ** dRef_u = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer ( &dRef_u, ref->u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sshort ** dRef_v = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer ( &dRef_v, ref->v );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

// TODO: Investigate why results are different when using NEON vectorisation
#ifdef ANDROID
   dRef_u[0][0] = 4; dRef_v[0][0] = 0;
   dRef_u[1][0] = 4; dRef_v[1][0] = 15;
   dRef_u[2][0] = 4; dRef_v[2][0] = 31;
   dRef_u[3][0] = 4; dRef_v[3][0] = 47;

   dRef_u[0][1] = 30; dRef_v[0][1] = 0;
   dRef_u[1][1] = 30; dRef_v[1][1] = 15;
   dRef_u[2][1] = 30; dRef_v[2][1] = 31;
   dRef_u[3][1] = 30; dRef_v[3][1] = 47;

   dRef_u[0][2] = 55; dRef_v[0][2] = 0;
   dRef_u[1][2] = 55; dRef_v[1][2] = 15;
   dRef_u[2][2] = 55; dRef_v[2][2] = 31;
   dRef_u[3][2] = 55; dRef_v[3][2] = 47;

   dRef_u[0][3] = 81; dRef_v[0][3] = 0;
   dRef_u[1][3] = 81; dRef_v[1][3] = 15;
   dRef_u[2][3] = 81; dRef_v[2][3] = 31;
   dRef_u[3][3] = 81; dRef_v[3][3] = 47;
#else
   dRef_u[0][0] = 4; dRef_v[0][0] = 0;
   dRef_u[1][0] = 4; dRef_v[1][0] = 16;
   dRef_u[2][0] = 4; dRef_v[2][0] = 32;
   dRef_u[3][0] = 4; dRef_v[3][0] = 48;

   dRef_u[0][1] = 30; dRef_v[0][1] = 0;
   dRef_u[1][1] = 30; dRef_v[1][1] = 16;
   dRef_u[2][1] = 30; dRef_v[2][1] = 32;
   dRef_u[3][1] = 30; dRef_v[3][1] = 48;

   dRef_u[0][2] = 55; dRef_v[0][2] = 0;
   dRef_u[1][2] = 55; dRef_v[1][2] = 16;
   dRef_u[2][2] = 55; dRef_v[2][2] = 32;
   dRef_u[3][2] = 55; dRef_v[3][2] = 48;

   dRef_u[0][3] = 81; dRef_v[0][3] = 0;
   dRef_u[1][3] = 81; dRef_v[1][3] = 16;
   dRef_u[2][3] = 81; dRef_v[2][3] = 32;
   dRef_u[3][3] = 81; dRef_v[3][3] = 48;
#endif

   // Create the homography
   Rox_Array2D_Double homography;
   error = rox_array2d_double_new(&homography, 3, 3);

   Rox_Double ** dH = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dH, homography );
   
   dH[0][0] = 1.59; dH[0][1] = 0; dH[0][2] = 0.295;
   dH[1][0] = 0.00; dH[1][1] = 1; dH[1][2] = 0.000;
   dH[2][0] = 0.00; dH[2][1] = 0; dH[2][2] = 1.000;

   // Compute remap parameters
   error = rox_warp_grid_sl3_fixed12_4 ( grid, homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("Checking warp_grid_sl3_fixed12_4...\n");
   for (Rox_Sint i = 0; i < height; i++) {
      for (Rox_Sint j = 0; j < width; j++) {
         ROX_TEST_MESSAGE("grid_data_u[i][j] = %d\n", grid_data_u[i][j]);
         ROX_TEST_MESSAGE("dRef_u[i][j] = %d\n", dRef_u[i][j]);
         ROX_TEST_MESSAGE("grid_data_v[i][j] = %d\n", grid_data_v[i][j]);
         ROX_TEST_MESSAGE("dRef_v[i][j] = %d\n", dRef_v[i][j]);

         ROX_TEST_CHECK_SMALL((double) grid_data_u[i][j] - (double) dRef_u[i][j], PRECISION);
         ROX_TEST_CHECK_SMALL((double) grid_data_v[i][j] - (double) dRef_v[i][j], PRECISION);
      }
   }
   
   // error = rox_array2d_point2d_sshort_save ( RESULT_PATH"/grid.txt", grid );
   // ROX_TEST_MESSAGE("error = %d \n", error);

   rox_array2d_double_del(&homography);
   rox_meshgrid2d_sshort_del(&grid);
   rox_meshgrid2d_sshort_del(&ref);
}


ROX_TEST_SUITE_END()
