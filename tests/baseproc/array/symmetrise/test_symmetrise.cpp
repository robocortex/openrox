//==============================================================================
//
//    OPENROX   : File test_symmetrise.cpp
//
//    Contents  : Tests for symmetrise.c
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
	#include <baseproc/array/symmetrise/symmetrise.h>
   #include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(symmetrise)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_array2d_double_symmetrise_lower )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double lower_data[9]  = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0};
   Rox_Array2D_Double lower = NULL;

   error = rox_array2d_double_new ( &lower, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( lower, lower_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_symmetrise_lower ( lower );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_double_print (lower);   

   error = rox_array2d_double_del ( &lower );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_array2d_double_symmetrise_upper )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double upper_data[9]  = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0};
   Rox_Array2D_Double upper = NULL;

   error = rox_array2d_double_new ( &upper, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( upper, upper_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_symmetrise_upper ( upper );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_double_print (upper);   

   error = rox_array2d_double_del ( &upper );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
