//==============================================================================
//
//    OPENROX   : File test_tukey.cpp
//
//    Contents  : Tests for tukey.c
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
	#include <baseproc/array/robust/tukey.h>
	#include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(tukey)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_tukey)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Sint nbp = 8;
   
   Rox_Array2D_Double weight = NULL;
   Rox_Array2D_Double work1 = NULL;
   Rox_Array2D_Double work2 = NULL;
   Rox_Array2D_Double subdist = NULL;

   // Rox_Double subdist_data[8] = { 	2.9925979321313072, 
   // 											2.5630403745222159, 
   // 											3.2759562138934730,
			// 									1.6615637426839398,
			// 									2.5721279416012708,
			// 									3.7656639000168064,
			// 									2.6860519219163681,
			// 									2.7050532808703709
			// 								};


   Rox_Double subdist_data[8] = { 	2.2039969705196074,
												0.39034363475126005,
												1.7514814078510064,
												0.45029697096593502,
												0.55148154436445540,
												3.0139314452792187,
												1.4218081244310092,
												1.8642221369895708};

   error = rox_array2d_double_new (&weight, nbp, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new (&work1, nbp, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new (&work2, nbp, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new (&subdist, nbp, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride ( subdist, subdist_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_tukey (weight, work1, work2, subdist);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_double_print ( weight );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del ( &weight );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del ( &work1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del ( &work2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del ( &subdist );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()
