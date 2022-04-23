//==============================================================================
//
//    OPENROX   : File test_svdinverse.cpp
//
//    Contents  : Tests for svdinverse.c
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
	#include <baseproc/array/inverse/svdinverse.h>
   #include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(svdinverse)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_svdinverse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double M = NULL;
   Rox_Array2D_Double M_inv = NULL;
   Rox_Double data[9] = { 2.0, 2.0, 2.0, 
                        2.0, 2.0, 2.0,
                        2.0, 2.0, 2.0 };

   error = rox_array2d_double_new(&M, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(M, data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&M_inv, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_svdinverse(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_svdinverse(NULL, M);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_svdinverse(M_inv, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_svdinverse(M_inv, M);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE);
   
   // error = rox_array2d_double_print(M_inv);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

   error = rox_array2d_double_del(&M);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&M_inv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
