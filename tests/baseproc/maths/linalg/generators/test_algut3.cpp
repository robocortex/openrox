//==============================================================================
//
//    OPENROX   : File test_algut3.cpp
//
//    Contents  : Tests for algut3.c
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
   #include <baseproc/maths/linalg/matrix.h>
   #include <baseproc/maths/linalg/generators/algut3.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(algut3)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_algut3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Matrix matrix = NULL;
   
   error = rox_matrix_new(NULL, 3, 3);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matrix_new(&matrix, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_matrix_del(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_matrix_del(&matrix);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_algut3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Matrix result  = NULL;
   Rox_Matrix input_1 = NULL;
   Rox_Matrix input_2 = NULL;

   error = rox_matrix_new(&result, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_set_unit(result);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_new(&input_1, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   

   error = rox_matrix_set_unit(input_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_new(&input_2, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   

   error = rox_matrix_set_unit(input_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //Rox_Double ** T = rox_array2d_double_get_data_pointer_to_pointer( result );
   //Rox_Double ** T1 = rox_array2d_double_get_data_pointer_to_pointer( input_1 );
   //Rox_Double ** T2 = rox_array2d_double_get_data_pointer_to_pointer( input_2 );

   // Change here input data

   rox_matrix_print(result);

   error = rox_matrix_del(&result);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&input_1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_del(&input_2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
