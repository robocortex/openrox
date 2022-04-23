//==============================================================================
//
//    OPENROX   : File test_gaussian2d.cpp
//
//    Contents  : Tests for gaussian2d.c
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
	#include <baseproc/maths/kernels/gaussian2d.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(gaussian2d)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_kernelgen_gaussian2d_separable_float)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float hfilter = NULL;
   Rox_Array2D_Float vfilter = NULL;
   Rox_Float sigma = 1.0;
   Rox_Float cutoff = 1.0;

   error = rox_kernelgen_gaussian2d_separable_float_new ( &hfilter, &vfilter, sigma, cutoff ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_del ( &hfilter );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_float_del ( &vfilter );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
