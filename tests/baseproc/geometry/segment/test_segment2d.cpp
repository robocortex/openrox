//==============================================================================
//
//    OPENROX   : File test_segment2d.cpp
//
//  	Contents  : Tests for segment2d.c
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
   #include <generated/dynvec_point2d_double.h>
	#include <baseproc/geometry/segment/segment2d.h>
   #include <baseproc/geometry/segment/segment2d_struct.h>
   #include <inout/geometry/point/dynvec_point2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(segment2d)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_segment2d )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Segment2D_Struct segment2d;
   Rox_Double u1 = 10.0;
   Rox_Double v1 =  0.0;
   Rox_Double u2 = 20.0;
   Rox_Double v2 =  0.0;
   Rox_Double sampling_step = 10.0;
   Rox_DynVec_Point2D_Double dynvec_point2d = NULL;

   error = rox_dynvec_point2d_double_new ( &dynvec_point2d, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_segment2d_set ( &segment2d, u1, v1, u2, v2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_segment2d_sample ( dynvec_point2d, &segment2d, sampling_step );
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_double_print ( dynvec_point2d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_double_del ( &dynvec_point2d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
