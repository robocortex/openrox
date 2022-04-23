//==============================================================================
//
//    OPENROX   : File test_linsys_solve_cholesky.cpp
//
//    Contents  : Tests for linsys_solve_cholesky.c
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
	 #include <baseproc/array/solve/linsys_solve_cholesky.h>
   #include <baseproc/array/error/l2_error.h>
   #include <inout/numeric/ansi_array_print.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(linsys_solve_cholesky)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_linsys_solve_cholesky)
{
	 Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;   
   Rox_Double S_mes_data[6*6] = { 31, 19, 32, 26, 20, 26,
                                  19, 15, 22, 18, 14, 20,
                                  32, 22, 41, 29, 21, 31,
                                  26, 18, 29, 28, 20, 29,
                                  20, 14, 21, 20, 15, 21,
                                  26, 20, 31, 29, 21, 33 };

   Rox_Double v_mes_data[6*1] = {   3, 1, 5, 5, 3, 3 };
   
   Rox_Double x_grt_data[6*1] = {  -69.5, -80.5, 52.5, -93.0, 257.0, -27.5 };

   Rox_Array2D_Double S_mes = NULL;
   Rox_Array2D_Double v_mes = NULL;
   Rox_Array2D_Double x_mes = NULL;
   Rox_Array2D_Double x_grt = NULL;

   error = rox_array2d_double_new ( &S_mes, 6, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new ( &v_mes, 6, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new ( &x_mes, 6, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new ( &x_grt, 6, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( S_mes, S_mes_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_set_buffer_no_stride ( v_mes, v_mes_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_set_buffer_no_stride ( x_grt, x_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Matrix S_mes: \n");
   rox_array2d_double_print ( S_mes );

   error = rox_linsys_solve_cholesky ( x_mes, S_mes, v_mes );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
  
   rox_log("Measure: \n");
   rox_array2d_double_print ( x_mes );

   rox_log("Ground truth: \n");
   rox_array2d_double_print ( x_grt );

   error = rox_array2d_double_difference_l2_norm ( &l2_error, x_grt, x_mes );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error cTo = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-8);

   error = rox_array2d_double_del(&S_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&v_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&x_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&x_grt);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ansi_linsys_solve_cholesky_float )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   float S_data[6*6] = { 31, 19, 32, 26, 20, 26,
                         19, 15, 22, 18, 14, 20,
                         32, 22, 41, 29, 21, 31,
                         26, 18, 29, 28, 20, 29,
                         20, 14, 21, 20, 15, 21,
                         26, 20, 31, 29, 21, 33 };

   float v_data[6*1] = { 3, 1, 5, 5, 3, 3 };
   
   float x_grt_data[6*1] = { -69.5, -80.5, 52.5, -93.0, 257.0, -27.5 };

   float x_data[6*1] = {0.0};

   int S_size = 6;

   error = rox_ansi_array_float_linsys_solve_cholesky ( x_data, S_data, S_size, v_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
  
   rox_log("Measure: \n");
   rox_ansi_array_float_print_as_array2d ( x_data, 6 , 1 );

   rox_log("Ground truth: \n");
   rox_ansi_array_float_print_as_array2d ( x_grt_data , 6, 1 );

   for ( int i = 0; i < S_size; i++ )
   {
      ROX_TEST_CHECK_SMALL( x_grt_data[i] - x_data[i], 1e-1 );
   }

}

ROX_TEST_SUITE_END()
