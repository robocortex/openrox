//==============================================================================
//
//    OPENROX   : File test_matsl3.cpp
//
//    Contents  : Tests for matsl3.c
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
   #include <float.h>
   #include <baseproc/maths/linalg/matrix.h>
   #include <baseproc/maths/linalg/matsl3.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(matsl3)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_matsl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSL3 matsl3 = NULL;
   
   error = rox_matsl3_new(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matsl3_new(&matsl3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matsl3_del(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matsl3_del(&matsl3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matsl3_inv)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSL3 result = NULL;
   Rox_MatSL3 input = NULL;

   error = rox_matsl3_new ( &result );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matsl3_new ( &input );
   ROX_TEST_CHECK_EQUAL (error, ROX_ERROR_NONE );
   
   Rox_Double ** H  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &H, input  );
   Rox_Double ** Hi = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Hi, result );

   // Change here input data

   H[0][0] = 1.0; H[0][2] = 63.5; 
   H[1][1] = 1.0; H[1][2] = 63.5; 

   error = rox_matsl3_inv( result, input );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_matsl3_print(result);

   // ROX_TEST_CHECK_CLOSE ( Hi[0][0], 0.0078125, FLT_EPSILON );
   // ROX_TEST_CHECK_CLOSE ( Hi[1][1], 0.0078125, FLT_EPSILON );
   // ROX_TEST_CHECK_CLOSE ( Hi[0][2], -0.49609375, FLT_EPSILON );
   // ROX_TEST_CHECK_CLOSE ( Hi[1][2], -0.49609375, FLT_EPSILON );

   ROX_TEST_CHECK_CLOSE ( Hi[0][0], 1.0, FLT_EPSILON );
   ROX_TEST_CHECK_CLOSE ( Hi[1][1], 1.0, FLT_EPSILON );
   ROX_TEST_CHECK_CLOSE ( Hi[0][2], -63.5, FLT_EPSILON );
   ROX_TEST_CHECK_CLOSE ( Hi[1][2], -63.5, FLT_EPSILON );

   error = rox_matsl3_del ( &result );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del ( &input );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matsl3_mulmatmat)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSL3 result  = NULL;
   Rox_MatSL3 input_1 = NULL;
   Rox_MatSL3 input_2 = NULL;

   error = rox_matsl3_new(&result);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matsl3_new(&input_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matsl3_new(&input_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //Rox_Double ** T = rox_array2d_double_get_data_pointer_to_pointer( result );
   //Rox_Double ** T1 = rox_array2d_double_get_data_pointer_to_pointer( input_1 );
   //Rox_Double ** T2 = rox_array2d_double_get_data_pointer_to_pointer( input_2 );

   // Change here input data

   error = rox_matsl3_mulmatmat( result, input_1, input_2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_matsl3_print(result);

   error = rox_matsl3_del(&result);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&input_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&input_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matsl3_update_left)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSL3 matsl3 = NULL;
   Rox_Matrix vector = NULL;

   error = rox_matrix_new ( &vector, 8, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   data[0][0] = -3.14159265359/2.0;
   data[1][0] = 0.0;
   data[2][0] = 0.0;
   data[3][0] = 0.0;
   data[4][0] = 0.0;
   data[5][0] = -3.14159265359;
   data[6][0] = 0.0;
   data[7][0] = 0.0;

   error = rox_matsl3_new ( &matsl3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // error = rox_matsl3_print ( matsl3 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_update_left ( matsl3, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_matsl3_print ( matsl3 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del ( &matsl3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del ( &vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
