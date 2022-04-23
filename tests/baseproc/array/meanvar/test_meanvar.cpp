//==============================================================================
//
//    OPENROX   : File test_meanvar.cpp
//
//    Contents  : Tests for meanvar.c
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
   #include <baseproc/array/meanvar/meanvar.h>
   #include <baseproc/array/meanvar/ansi_meanvar.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(meanvar)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_array2d_float_meanvar )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
 	
 	int rows = 10;
 	int cols = 10;

	float vect_data[100] = { 2, 7, 6, 4, 8, 1, 1, 9, 4, 5, 
								 	 9, 4, 6, 7, 3, 5, 5, 6, 5, 5, 
								 	 3, 8, 9, 8, 9, 5, 2, 6, 2, 0, 
								 	 9, 1, 3, 8, 4, 5, 1, 6, 6, 7, 
								 	 1, 8, 2, 5, 9, 9, 9, 3, 7, 2, 
								 	 8, 6, 1, 6, 9, 8, 7, 7, 3, 1, 
								 	 5, 2, 2, 4, 6, 9, 3, 5, 9, 2, 
								 	 8, 1, 8, 5, 9, 7, 7, 1, 2, 3, 
								 	 1, 7, 8, 4, 2, 6, 8, 7, 2, 4, 
								 	 7, 5, 9, 3, 8, 4, 2, 1, 7, 4 };


	unsigned int mask_data[100] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 							    	 		  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 							    	 		  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 							    	 		  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 							    	 		  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 							    	 		  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 							    	 		  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 							    	 		  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 							    	 		  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 							    	 		  1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

	float mean = 0.0;
	float variance = 0.0;

	error = rox_ansi_array_float_meanvar_mask ( &mean, &variance, &vect_data[0], &mask_data[0], rows * cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("mean = %f\n",mean);
   rox_log("variance = %f\n",variance);

   ROX_TEST_CHECK_CLOSE ( mean,  5.1200, 1e-6 );
   ROX_TEST_CHECK_CLOSE ( variance, 7.29858585858586, 1e-6 );
}

ROX_TEST_SUITE_END()
