//==============================================================================
//
//    OPENROX   : File test_svd_solve.cpp
//
//    Contents  : Tests for svd_solve.c
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
	#include <baseproc/array/solve/svd_solve.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(svd_solve)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_svd_solve)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double data_A_mes[6] = {5,     6,     3,     7,     8,     3};
   Rox_Double data_b_mes[6] = {2,     3,     4,     5,     6,     1};
   Rox_Double data_x_grt = 21.0/32.0;

   Rox_Array2D_Double A_mes = NULL;
   Rox_Array2D_Double b_mes = NULL;
   Rox_Array2D_Double x_mes = NULL;

   error = rox_array2d_double_new(&A_mes, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&b_mes, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&x_mes, 1, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double ** dA_mes = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dA_mes, A_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Double ** db_mes = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&db_mes, b_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   dA_mes[0][0] = data_A_mes[0];  dA_mes[1][0] = data_A_mes[1];  dA_mes[2][0] = data_A_mes[2];                           
   dA_mes[3][0] = data_A_mes[3];  dA_mes[4][0] = data_A_mes[4];  dA_mes[5][0] = data_A_mes[5];                           
   
   db_mes[0][0] = data_b_mes[0];  db_mes[1][0] = data_b_mes[1];  db_mes[2][0] = data_b_mes[2];                           
   db_mes[3][0] = data_b_mes[3];  db_mes[4][0] = data_b_mes[4];  db_mes[5][0] = data_b_mes[5];                           
   
   error = rox_svd_solve(x_mes, A_mes, b_mes);
   
   rox_log("Measure: \n");
   rox_array2d_double_print(x_mes);
   rox_log("Ground truth: %f \n", data_x_grt);

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&A_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&b_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&x_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()
