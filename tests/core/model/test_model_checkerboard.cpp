//==============================================================================
//
//    OPENROX   : File test_model_checkerboard.cpp
//
//    Contents  : Tests for model_checkerboard.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//====== INCLUDED HEADERS   ====================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <system/time/timer.h>
   #include <core/model/model_checkerboard.h>
}

//====== INTERNAL MACROS    ====================================================

ROX_TEST_SUITE_BEGIN(model_checkerboard)

//====== INTERNAL TYPESDEFS ====================================================

//====== INTERNAL DATATYPES ====================================================

//====== INTERNAL VARIABLES ====================================================

//====== INTERNAL FUNCTDEFS ====================================================

//====== INTERNAL FUNCTIONS ====================================================

//====== EXPORTED FUNCTIONS ====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_model_checkerboard)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   Rox_Model_CheckerBoard model = NULL;

   // Tests for new    
   error = rox_model_checkerboard_new(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_model_checkerboard_new(&model);   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Tests for set
   error = rox_model_checkerboard_set_template(NULL, 0, 0, 0.0, 0.0);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   
   // use a double for to generate tests
   error = rox_model_checkerboard_set_template(model, 0, 0, 0.0, 0.0);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
   
   error = rox_model_checkerboard_set_template(model, 1, 0, 0.0, 0.0);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
   
   error = rox_model_checkerboard_set_template(model, 2, 0, 0.0, 0.0);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
   
   error = rox_model_checkerboard_set_template(model, 0, 1, 0.0, 0.0);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
   
   error = rox_model_checkerboard_set_template(model, 2, 2, 0.0, 0.0);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
   
   error = rox_model_checkerboard_set_template(model, 2, 2, 0.01, 0.01);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Tests for del
   error = rox_model_checkerboard_del(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_model_checkerboard_del(&model);   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_model_checkerboard)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_perf_model_checkerboard)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
