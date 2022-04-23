//==============================================================================
//
//    OPENROX   : File test_centered_error.cpp
//
//    Contents  : Tests for centered_error.c
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
	#include <baseproc/array/error/centered_error.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(centered_error)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_centered_error)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double median = 0.0, sum_square = 0.0;
   Rox_Uint count_valid = 0;
   Rox_Array2D_Float res = NULL;
   Rox_Array2D_Float v1 = NULL;
   Rox_Array2D_Float v2 = NULL;
   Rox_Array2D_Uint mask = NULL;

   error = rox_array2d_float_new(&res, 1, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_float_new(&v1, 1, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_float_new(&v2, 1, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_new(&mask, 1, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /* Test NULL pointers */
   error = rox_array2d_float_centered_error(NULL, &sum_square, &count_valid, res, mask, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_centered_error(&median, NULL, &count_valid, res, mask, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_centered_error(&median, &sum_square, NULL, res, mask, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_centered_error(&median, &sum_square, &count_valid, NULL, mask, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_centered_error(&median, &sum_square, &count_valid, res, NULL, v1, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_centered_error(&median, &sum_square, &count_valid, res, mask, NULL, v2);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_centered_error(&median, &sum_square, &count_valid, res, mask, v1, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_float_del(&res);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_float_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_float_del(&v2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_uint_del(&mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
