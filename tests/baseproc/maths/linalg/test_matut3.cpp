//==============================================================================
//
//    OPENROX   : File test_matut3.cpp
//
//    Contents  : Tests for matut3.c
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
   #include <baseproc/maths/linalg/matut3.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(matut3)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_matut3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatUT3 matut3 = NULL;
   
   error = rox_matut3_new(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matut3_new(&matut3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matut3_del(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matut3_del(&matut3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matut3_inv)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatUT3 result = NULL;
   Rox_MatUT3 input = NULL;

   error = rox_matut3_new ( &result );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matut3_new ( &input );
   ROX_TEST_CHECK_EQUAL (error, ROX_ERROR_NONE );
   
   Rox_Double ** H  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &H, input  );
   ROX_TEST_CHECK_EQUAL (error, ROX_ERROR_NONE );

   Rox_Double ** Hi = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Hi, result );
   ROX_TEST_CHECK_EQUAL (error, ROX_ERROR_NONE );

   // Change here input data

   H[0][0] = 128.0; H[0][2] = 63.5; 
   H[1][1] = 128.0; H[1][2] = 63.5; 

   error = rox_matut3_inv( result, input );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_matut3_print(result);

   ROX_TEST_CHECK_CLOSE ( Hi[0][0], 0.0078125, FLT_EPSILON );
   ROX_TEST_CHECK_CLOSE ( Hi[1][1], 0.0078125, FLT_EPSILON );
   ROX_TEST_CHECK_CLOSE ( Hi[0][2], -0.49609375, FLT_EPSILON );
   ROX_TEST_CHECK_CLOSE ( Hi[1][2], -0.49609375, FLT_EPSILON );

   error = rox_matut3_del ( &result );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &input );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matut3_mulmatmat)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatUT3 result  = NULL;
   Rox_MatUT3 input_1 = NULL;
   Rox_MatUT3 input_2 = NULL;

   error = rox_matut3_new(&result);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matut3_new(&input_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matut3_new(&input_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //Rox_Double ** T = rox_array2d_double_get_data_pointer_to_pointer( result );
   //Rox_Double ** T1 = rox_array2d_double_get_data_pointer_to_pointer( input_1 );
   //Rox_Double ** T2 = rox_array2d_double_get_data_pointer_to_pointer( input_2 );

   // Change here input data

   error = rox_matut3_mulmatmat( result, input_1, input_2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matut3_print(result);

   error = rox_matut3_del(&result);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del(&input_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del(&input_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_matut3_update_left)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatUT3 matut3 = NULL;
   Rox_Array2D_Double vector = NULL;

   error = rox_matrix_new(&vector, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Double ** data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &data, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   data[0][0] = -0.1;
   data[1][0] =  0.1;
   data[2][0] =  0.0;
   data[3][0] =  0.0;
   data[4][0] =  0.0;
   data[5][0] =  0.0;

   error = rox_matut3_new ( &matut3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // error = rox_matut3_print ( matut3 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_update_left ( matut3, vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_matut3_print ( matut3 );
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &matut3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &vector );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
