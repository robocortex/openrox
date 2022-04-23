//==============================================================================
//
//    OPENROX   : File test_model_projector_checkerboard.cpp
//
//    Contents  : Tests for model_projector_checkerboard.c
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
   #include <core/model/model_projector_checkerboard.h>
   #include <inout/geometry/point/point2d_print.h>
}

//====== INTERNAL MACROS    ====================================================

ROX_TEST_SUITE_BEGIN(model_projector_checkerboard)

//====== INTERNAL TYPESDEFS ====================================================

//====== INTERNAL DATATYPES ====================================================

//====== INTERNAL VARIABLES ====================================================

//====== INTERNAL FUNCTDEFS ====================================================

//====== INTERNAL FUNCTIONS ====================================================

//====== EXPORTED FUNCTIONS ====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_unit_model_projector_checkerboard)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Model_Projector_CheckerBoard model = NULL;

   // Tests for new    
   error = rox_model_projector_checkerboard_new ( NULL );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_model_projector_checkerboard_new ( &model );   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint cols = 1920;
   Rox_Sint rows = 1080;

   // Tests for set
   error = rox_model_projector_checkerboard_set_template(NULL, 0, 0, 0.0, 0.0, cols, rows);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   
   // use a double for to generate tests
   error = rox_model_projector_checkerboard_set_template(model, 0, 0, 0.0, 0.0, cols, rows);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
   
   error = rox_model_projector_checkerboard_set_template(model, 1, 0, 0.0, 0.0, cols, rows);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
   
   error = rox_model_projector_checkerboard_set_template(model, 2, 0, 0.0, 0.0, cols, rows);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
   
   error = rox_model_projector_checkerboard_set_template(model, 0, 1, 0.0, 0.0, cols, rows);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
   
   error = rox_model_projector_checkerboard_set_template(model, 2, 2, 0.0, 0.0, cols, rows);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_INVALID_VALUE);
   
   error = rox_model_projector_checkerboard_set_template(model, 2, 2, 80.0, 80.0, cols, rows);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // Tests for del
   error = rox_model_projector_checkerboard_del(NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_model_projector_checkerboard_del(&model);   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_model_projector_checkerboard)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint projector_cols = 1920;
   Rox_Sint projector_rows = 1080;
   
   Rox_Sint grid_cols = 11;
   Rox_Sint grid_rows =  6;

   Rox_Double space_u = 80.0;
   Rox_Double space_v = 80.0;

   Rox_Model_Projector_CheckerBoard model = NULL;

   error = rox_model_projector_checkerboard_new ( &model );   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_projector_checkerboard_set_template ( model, grid_cols, grid_rows, space_u, space_v, projector_cols, projector_rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_vector_point2d_double_print ( model->elements, grid_cols * grid_rows);

   Rox_Image image = NULL;
   error = rox_image_new ( &image, projector_cols, projector_rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_projector_checkerboard_generate_image ( model, image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_image_save_pgm ("test_calibration_projector_grid.pgm", image );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_perf_model_projector_checkerboard)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
