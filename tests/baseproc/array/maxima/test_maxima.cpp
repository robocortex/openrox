//==============================================================================
//
//    OPENROX   : File test_maxima.cpp
//
//    Contents  : Tests for maxima.c
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
	#include <baseproc/array/maxima/maxima.h>
   #include <inout/numeric/dynvec_print.h>
   #include <inout/geometry/point/dynvec_point2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(maxima)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_array2d_double_maxima )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double y = NULL;

   Rox_Double yd[27] = {4,2,3,6,6,5,7,7,7,7,7,4.5,4.5,5,5,5,3,3,2,2,6,2,3.5,3.5,2,2,4};

   error = rox_array2d_double_new(&y,1,27);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(y, yd);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double y_thresh = 0.0;

   Rox_DynVec_Point2D_Uint local_maxima_indexes = NULL;
   error = rox_dynvec_point2d_uint_new(&local_maxima_indexes, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_DynVec_Double local_maxima_values = NULL;
   error = rox_dynvec_double_new(&local_maxima_values, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_DynVec_Double local_maxima_coords = NULL;
   error = rox_dynvec_double_new(&local_maxima_coords, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_local_maxima (local_maxima_values, local_maxima_coords, local_maxima_indexes, y, y_thresh);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_double_print(local_maxima_values);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_double_print(local_maxima_coords);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_uint_print(local_maxima_indexes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_double_del(&local_maxima_values);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_double_del(&local_maxima_coords);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_uint_del(&local_maxima_indexes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del ( &y );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_float_maxima)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}
ROX_TEST_SUITE_END()
