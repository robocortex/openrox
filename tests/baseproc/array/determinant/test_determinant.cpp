//==============================================================================
//
//    OPENROX   : File test_determinant.cpp
//
//    Contents  : Tests for determinant.c
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
	#include "baseproc/array/determinant/detgl3.h"
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(add)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_detgl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double det = 0.0;

   Rox_Array2D_Double v1 = NULL;
   Rox_Array2D_Double vb = NULL;

   error = rox_array2d_double_new(&v1, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_new(&vb, 3, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value(v1, 0, 0, 6.0); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(v1, 0, 1, 1.0); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(v1, 0, 2, 1.0); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(v1, 1, 0, 4.0); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(v1, 1, 1,-2.0); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(v1, 1, 2, 5.0); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(v1, 2, 0, 2.0); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(v1, 2, 1, 8.0); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(v1, 2, 2, 7.0); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /* Test NULL pointers */
   error = rox_array2d_double_detgl3(&det, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_detgl3(NULL, v1);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_array2d_double_detgl3(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   /* Test bad size */
   error = rox_array2d_double_detgl3(&det, vb);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   /* Test correct values */
   error = rox_array2d_double_detgl3(&det, v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_SMALL(det + 306.0, 1e-12);

   error = rox_array2d_double_del(&v1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_del(&vb);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
