//==============================================================================
//
//    OPENROX   : File test_model_single_plane.cpp
//
//    Contents  : Tests for model_single_plane.c
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
   #include <baseproc/geometry/point/point3d_struct.h>
   #include <baseproc/geometry/rectangle/rectangle.h>
   #include <core/model/model_single_plane.h>
   #include <inout/system/print.h>
   #include <inout/geometry/point/point3d_print.h>
}

//====== INTERNAL MACROS    ====================================================

ROX_TEST_SUITE_BEGIN ( model_single_plane )

//====== INTERNAL TYPESDEFS ====================================================

//====== INTERNAL DATATYPES ====================================================

//====== INTERNAL VARIABLES ====================================================

//====== INTERNAL FUNCTDEFS ====================================================

//====== INTERNAL FUNCTIONS ====================================================

//====== EXPORTED FUNCTIONS ====================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_unit_model_single_plane )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_Single_Plane model = NULL;
   
   // Tests for new    
   error = rox_model_single_plane_new ( NULL );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_model_single_plane_new ( &model );   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Tests for set
   error = rox_model_single_plane_set_3d_template ( NULL, NULL, NULL, 0.0 );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   
   // Tests for del
   error = rox_model_single_plane_del ( NULL );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_model_single_plane_del(&model);   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_model_single_plane )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_Single_Plane model_single_plane = NULL;
   Rox_Image source = NULL;
   Rox_MatSE3 cTo = NULL;

   // Set model size in meters
   Rox_Double sizex = 1.0;
   Rox_Double sizey = 1.0;

   Rox_Point3D_Double_Struct vertices[4];
   Rox_Point3D_Double_Struct vertices_ref[4];
   Rox_Point3D_Double_Struct vertices_cur[4];

   rox_log_set_callback(RoxTest::_log_callback);

   error = rox_rectangle3d_create_centered_plane_xright_ydown ( vertices, sizex, sizey );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("input vertices:\n");
   rox_vector_point3d_double_print(vertices, 4);

   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_new ( &source, 128, 128 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_single_plane_new ( &model_single_plane );   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_single_plane_set_3d_template ( model_single_plane, source, vertices, 128 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_single_plane_get_vertices_ref ( vertices_ref, model_single_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("reference vertices:\n");
   rox_vector_point3d_double_print(vertices_ref, 4);


   error = rox_model_single_plane_transform ( model_single_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_single_plane_get_vertices_cur ( vertices_cur, model_single_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("current vertices:\n");
   rox_vector_point3d_double_print ( vertices_cur, 4);

   // Test get pose
   error = rox_model_single_plane_get_pose ( cTo, model_single_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("current pose:\n");
   rox_matse3_print ( cTo );

   // Test visibility

   error = rox_model_single_plane_del(&model_single_plane);   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &source );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_perf_model_single_plane)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
