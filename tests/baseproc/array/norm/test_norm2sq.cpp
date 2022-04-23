//==============================================================================
//
//    OPENROX   : File test_norm2sq.cpp
//
//    Contents  : Tests for norm2sq.c
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
	#include <baseproc/array/norm/norm2sq.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(norm2sq)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_norm2sq)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double data[7] = {1.0, 0.0, 2.0, 3.0, -1.0, 1.0, 0.0};
   Rox_Array2D_Double vector = NULL;
   Rox_Double norm2 = 0.0;
   Rox_Double norm2_squared = 0.0;

   error = rox_array2d_double_new_frombuffer(&vector, 1, 7, data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_norm2sq(&norm2_squared, vector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_SMALL(norm2_squared - 16.0, 1e-16);

   error = rox_array2d_double_norm2(&norm2, vector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_SMALL(norm2 - 4.0, 1e-16);

   error = rox_array2d_double_del(&vector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_norm_frobenius)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double data[7] = {1.0, 0.0, 2.0, 3.0, -1.0, 1.0, 0.0};
   Rox_Array2D_Double vector = NULL;
   Rox_Double norm_frobenius = 0.0;

   error = rox_array2d_double_new_frombuffer(&vector, 1, 7, data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_norm_frobenius(&norm_frobenius, vector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_SMALL(norm_frobenius - 4.0, 1e-16);
   
   error = rox_array2d_double_del(&vector);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
