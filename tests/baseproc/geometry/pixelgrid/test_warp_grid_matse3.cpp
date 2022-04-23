//==============================================================================
//
//    OPENROX   : File test_warp_grid_matse3.cpp
//
//    Contents  : Tests for warpgridse3.c
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
   #include <baseproc/maths/linalg/matse3.h>
   #include <baseproc/maths/linalg/matut3.h>
	#include <baseproc/geometry/pixelgrid/warp_grid_matse3_z.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matse3_zi.h>
   #include <baseproc/geometry/transforms/transform_tools.h>   
   #include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>
   #include <baseproc/image/imask/imask.h>
   #include <baseproc/array/error/l2_error.h>

   #include <inout/numeric/array2d_save.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/geometry/point/array2d_point2d_print.h>
   #include <inout/lut/txt/array2d_point2d_float_txtfile.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(warpgridse3)

#define FU 2560.0
#define FV 2560.0
#define CU 255.5
#define CV 255.5

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_warp_grid_float_matse3_z_float )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;   
   Rox_Sint height = 3, width = 3;
   Rox_Float l2_error = 0.0;

   Rox_Float grid_warp_grt_u_data[9] = {  -3.0000, -0.5000, 1.25000, 
                                          -0.7500,  0.6250, 1.81250, 
                                          -0.3750,  0.8125, 1.90625};

   Rox_Float grid_warp_grt_v_data[9] = {   0.0000,  0.0000, 0.00000, 
                                           1.0000,  1.0000, 1.00000,
                                           2.0000,  2.0000, 2.00000};

   Rox_Float Zr_data[9] = {1.0, 2.0, 4.0, 4.0, 8.0, 16.0, 8.0, 16.0, 32.0};

   Rox_MeshGrid2D_Float grid_warp_grt = NULL; 
   error = rox_meshgrid2d_float_new ( &grid_warp_grt, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( grid_warp_grt->u, grid_warp_grt_u_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_set_buffer_no_stride ( grid_warp_grt->v, grid_warp_grt_v_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MeshGrid2D_Float grid_warp = NULL; 
   error = rox_meshgrid2d_float_new ( &grid_warp, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Imask grid_mask = NULL;
   error = rox_imask_new ( &grid_mask, width, height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Zr = NULL; 
   error = rox_array2d_float_new ( &Zr, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( Zr, Zr_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSE3 cTr = NULL;
   error = rox_matse3_new ( &cTr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double tra[3] = {-0.15/128.0, 0.0, 0.0};

   error = rox_matse3_set_translation ( cTr, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatUT3 Kr = NULL;
   error = rox_matut3_new ( &Kr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( Kr, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_MatUT3 Kc = NULL;
   error = rox_matut3_new ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( Kc, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_warp_grid_float_matse3_z_float ( grid_warp, grid_mask, Zr, cTr, Kr, Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //rox_imask_save_pgm("debug_grid_mask_z.pgm", grid_mask);
   //rox_array2d_point2d_float_save_txt ( "debug_grid_warp_z.txt", grid_warp );

   rox_array2d_uint_print ( grid_mask );
   rox_array2d_float_print ( grid_warp->u );
   rox_array2d_float_print ( grid_warp->v );
   
   error = rox_array2d_float_difference_l2_norm ( &l2_error, grid_warp_grt->u, grid_warp->u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error grid_warp->u = %0.12f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_array2d_float_difference_l2_norm ( &l2_error, grid_warp_grt->v, grid_warp->v );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error grid_warp->u = %0.12f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_warp_grid_float_matse3_zi_float )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint height = 3, width = 3;
   Rox_Float l2_error = 0.0;

   Rox_Float grid_warp_grt_u_data[9] = {  -3.0000, -0.5000, 1.25000, 
                                          -0.7500,  0.6250, 1.81250, 
                                          -0.3750,  0.8125, 1.90625};

   Rox_Float grid_warp_grt_v_data[9] = {   0.0000,  0.0000, 0.00000, 
                                           1.0000,  1.0000, 1.00000,
                                           2.0000,  2.0000, 2.00000};

   Rox_Float Zir_data[9] = { 1.0000, 0.5000, 0.25000, 
                             0.2500, 0.1250, 0.06250, 
                             0.1250, 0.0625, 0.03125 };

   Rox_MeshGrid2D_Float grid_warp_grt = NULL; 
   error = rox_meshgrid2d_float_new ( &grid_warp_grt, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( grid_warp_grt->u, grid_warp_grt_u_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_set_buffer_no_stride ( grid_warp_grt->v, grid_warp_grt_v_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MeshGrid2D_Float grid_warp = NULL; 
   error = rox_meshgrid2d_float_new ( &grid_warp, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Imask grid_mask = NULL;
   error = rox_imask_new ( &grid_mask, width, height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Array2D_Float Zir = NULL; 
   error = rox_array2d_float_new ( &Zir, height, width );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_set_buffer_no_stride ( Zir, Zir_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSE3 cTr = NULL;
   error = rox_matse3_new ( &cTr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double tra[3] = {-0.15/128.0, 0.0, 0.0};

   error = rox_matse3_set_translation ( cTr, tra );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatUT3 Kr = NULL;
   error = rox_matut3_new ( &Kr );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( Kr, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_MatUT3 Kc = NULL;
   error = rox_matut3_new ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( Kc, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_warp_grid_float_matse3_zi_float ( grid_warp, grid_mask, Zir, cTr, Kr, Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_uint_print ( grid_mask );
   rox_array2d_float_print ( grid_warp->u );
   rox_array2d_float_print ( grid_warp->v );

   error = rox_array2d_float_difference_l2_norm ( &l2_error, grid_warp_grt->u, grid_warp->u );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error grid_warp->u = %0.12f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);

   error = rox_array2d_float_difference_l2_norm ( &l2_error, grid_warp_grt->v, grid_warp->v );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error grid_warp->u = %0.12f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);
}

ROX_TEST_SUITE_END()
