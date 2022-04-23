//==============================================================================
//
//    OPENROX   : File test_objset_edge_ellipse.cpp
//
//    Contents  : Tests for objset_edge_ellipse.c
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
   #include <core/odometry/edge/objset_edge_ellipse_tools.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(objset_edge_ellipse)

#define IMAGE_PATH ROX_DATA_HOME"/devapps/model_based/test_cadmodel/test_segments_ellipses.ppm"

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================


//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_objset_edge_ellipse_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse = NULL;
   //const Rox_Sint search_range = 20;
   //const Rox_Double contrast_threshold = 51;

   error = rox_objset_edge_ellipse_new(&objset_edge_ellipse, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_edge_ellipse_del(&objset_edge_ellipse);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_objset_edge_ellipse_add_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   //const Rox_Sint search_range = 20;
   //const Rox_Double contrast_threshold = 51;
   const Rox_Double sampling_step = 5;
   Rox_Ellipse3D ellipse3d = NULL;
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse = NULL;
   
   error = rox_ellipse3d_new(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_edge_ellipse_new(&objset_edge_ellipse, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_edge_ellipse_add_ellipse3d(objset_edge_ellipse, ellipse3d, sampling_step);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_edge_ellipse_del(&objset_edge_ellipse);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse3d_del(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_objset_edge_ellipse_make)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   //const Rox_Sint search_range = 20;
   //const Rox_Double contrast_threshold = 51;
   const Rox_Double sampling_step = 5;
   //const Rox_Uint max_iters = 10;
   //Rox_Array2D_Uchar image = NULL;
   Rox_Ellipse3D ellipse3d = NULL;
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse = NULL;
   
   error = rox_ellipse3d_new(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_edge_ellipse_new(&objset_edge_ellipse, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_edge_ellipse_add_ellipse3d(objset_edge_ellipse, ellipse3d, sampling_step);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_objset_edge_ellipse_make(objset_edge_ellipse, image, max_iters);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_edge_ellipse_del(&objset_edge_ellipse);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse3d_del(&ellipse3d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_objset_edge_ellipse_set_pose)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_objset_edge_ellipse_get_pose)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
