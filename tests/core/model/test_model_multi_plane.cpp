//==============================================================================
//
//    OPENROX   : File test_model_multi_plane.cpp
//
//    Contents  : Tests for model_multi_plane.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =====================================================

#include <openrox_tests.hpp>

extern "C"
{
	#include <core/model/model_multi_plane.h>
   #include <baseproc/image/image.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( model_multi_plane )

#define mod_file_01 ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/models/photoframe_1.pgm"

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_model_multi_plane_new)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_model_multi_plane_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_model_multi_plane_append_3d_template)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Char filename[FILENAME_MAX];
   
   Rox_Model_Multi_Plane model_multi_plane = NULL;

   Rox_Array2D_Uchar image_template = NULL; 
   Rox_Point3D_Double_Struct vertices[4];
   Rox_Sint basesize = 128;
   Rox_Double vertices_size_x = 1.0;
   Rox_Double vertices_size_y = 1.0;

   sprintf(filename, "%s", mod_file_01);

   // Load model for identification
   error = rox_image_new_read_pgm ( &image_template, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_multi_plane_new ( &model_multi_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
   
   vertices[0].X = -vertices_size_x/2.0; vertices[0].Y = -vertices_size_y/2.0; vertices[0].Z = 0.0;
   vertices[1].X =  vertices_size_x/2.0; vertices[1].Y = -vertices_size_y/2.0; vertices[1].Z = 0.0;
   vertices[2].X =  vertices_size_x/2.0; vertices[2].Y =  vertices_size_y/2.0; vertices[2].Z = 0.0;
   vertices[3].X = -vertices_size_x/2.0; vertices[3].Y =  vertices_size_y/2.0; vertices[3].Z = 0.0;

   error = rox_model_multi_plane_append_3d_template ( model_multi_plane, image_template, vertices, basesize );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_model_multi_plane_del ( &model_multi_plane );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del ( &image_template ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_model_multi_plane_set_currentpose )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_model_multi_plane_transform)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_model_multi_plane_get_transformed_vertices)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_model_multi_plane_copy)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
