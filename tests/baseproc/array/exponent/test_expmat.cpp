//==============================================================================
//
//    OPENROX   : File test_expmat.cpp
//
//    Contents  : Tests for expmat
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
	#include <baseproc/array/exponent/expmat.h>
   #include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(expmat)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_expmat)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double A = NULL;
   Rox_Array2D_Double M = NULL;
   Rox_Double data[9] = { 1.0, 0.0, 0.0, 
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0 };

   error = rox_array2d_double_new(&A, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(A, data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&M, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_expmat(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_expmat(NULL, A);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_expmat(M, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_expmat(M, A);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_print(M);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );  

   error = rox_array2d_double_del(&A);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );  

   error = rox_array2d_double_del(&M);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );  
}

ROX_TEST_SUITE_END()
