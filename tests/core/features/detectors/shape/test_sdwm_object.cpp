//==============================================================================
//
//    OPENROX   : File test_sdwm_object.cpp
//
//    Contents  : Tests for sdwm_object.c
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
   #include <core/features/detectors/shape/sdwm_object.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(sdwm)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_sdwm_object_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sdwm_Object obj = NULL;
   
   // Test NULL pointer
   error = rox_sdwm_object_new(NULL, 100, 100);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   // Test ZERO object size
   error = rox_sdwm_object_new(&obj, 0, 0);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);
  
   // Test new object
   error = rox_sdwm_object_new(&obj, 100, 100);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Test del object
   error = rox_sdwm_object_del(&obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test del null pointer
   error = rox_sdwm_object_del(&obj);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   
   rox_error_print(error);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_sdwm_object_add_template)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
