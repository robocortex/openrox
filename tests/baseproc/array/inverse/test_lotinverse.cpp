//==============================================================================
//
//    OPENROX   : File test_lotinverse.cpp
//
//    Contents  : Tests for lotinverse.c
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
	#include <baseproc/array/inverse/lotinverse.h>
   #include <baseproc/array/error/l2_error.h>
	#include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(lotinverse)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_lotinverse)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;   

   Rox_Double L_data[36] = {  2, 0, 0, 0, 0, 0,
										1, 2, 0, 0, 0, 0,
										3, 3, 2, 0, 0, 0,
										1, 3, 1, 1, 0, 0,
										1, 3, 3, 2, 1, 0,
										3, 1, 2, 2, 1, 1 };

   Rox_Double Li_grt_data[36] = { 0.5000,       0,       0,       0,       0,      0,
   										-0.2500,  0.5000,       0,       0,       0,      0,
   										-0.3750, -0.7500,  0.5000,       0,       0,      0,
   										 0.6250, -0.7500, -0.5000,  1.0000,       0,      0,
   										 0.1250,  2.2500, -0.5000, -2.0000,  1.0000,      0,
   										-1.8750,  0.2500,  0.5000,       0, -1.0000, 1.0000 };


	Rox_Array2D_Double L = NULL;
	Rox_Array2D_Double Li = NULL;
	Rox_Array2D_Double Li_grt = NULL;

	error = rox_array2d_double_new ( &L, 6, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_new ( &Li, 6, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_new ( &Li_grt, 6, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( L, L_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( Li_grt, Li_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_lotinverse ( Li, L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_double_print ( Li );

   error = rox_array2d_double_difference_l2_norm ( &l2_error, Li_grt, Li );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error Li = %f \n", l2_error);
   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-8);

	error = rox_array2d_double_del ( &L );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_del ( &Li );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
