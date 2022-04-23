//==============================================================================
//
//    OPENROX   : File test_decomposition_cholesky.cpp
//
//    Contents  : Tests for cholesky.c
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
   #include <baseproc/array/decomposition/cholesky.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(cholesky)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_cholesky)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double data_mes[9] = { 1048576, 0, 327680, 0, 1048576, 245760, 327680, 245760, 160001};
   Rox_Double data_grt[9] = { 1024, 0, 0, 0, 1024, 0, 320, 240, 1};
   Rox_Array2D_Double S_mes = NULL;
   Rox_Array2D_Double K_mes = NULL;
   Rox_Array2D_Double K_grt = NULL;

   error = rox_array2d_double_new(&S_mes, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&K_mes, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&K_grt, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double ** dS_mes = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dS_mes, S_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Double ** dK_grt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dK_grt, K_grt);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   dS_mes[0][0] = data_mes[0];  dS_mes[0][1] = data_mes[1];  dS_mes[0][2] = data_mes[2];                           
   dS_mes[1][0] = data_mes[3];  dS_mes[1][1] = data_mes[4];  dS_mes[1][2] = data_mes[5];                           
   dS_mes[2][0] = data_mes[6];  dS_mes[2][1] = data_mes[7];  dS_mes[2][2] = data_mes[8];
   
   dK_grt[0][0] = data_grt[0];  dK_grt[0][1] = data_grt[1];  dK_grt[0][2] = data_grt[2];                           
   dK_grt[1][0] = data_grt[3];  dK_grt[1][1] = data_grt[4];  dK_grt[1][2] = data_grt[5];                           
   dK_grt[2][0] = data_grt[6];  dK_grt[2][1] = data_grt[7];  dK_grt[2][2] = data_grt[8];  
   
   error = rox_array2d_double_cholesky_decomposition(K_mes, S_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Measure: \n");
   rox_array2d_double_print(K_mes);

   rox_log("Ground truth: \n");
   rox_array2d_double_print(K_grt);

   error = rox_array2d_double_del(&S_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&K_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&K_grt);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
