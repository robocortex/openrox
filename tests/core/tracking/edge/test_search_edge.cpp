//==============================================================================
//
//    OPENROX   : File test_search_edge.cpp
//
//    Contents  : Tests for tracking_patch_sl3.c
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
   #include <baseproc/maths/maths_macros.h>
   #include <baseproc/image/gradient/gradient_anglenorm.h>
	#include <core/tracking/edge/search_edge.h>
   #include <inout/image/pgm/pgmfile.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(search_edge)

// #define IMAGE_PATH  ROX_DATA_HOME"/regression_tests/openrox/odometry/edges/caterpillar_ids_29-08-2017-05_00000001.pgm"
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/photoframe/images/image_093.pgm"

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_search_edge_new_del )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Search_Edge search_edge = NULL;

   const Rox_Uint searchrange = 30;
   const Rox_Uint threshold_response = 0.1;

   error = rox_search_edge_new ( &search_edge, searchrange, threshold_response );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_search_edge_del ( &search_edge );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_rox_search_edge_track_image )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   const Rox_Uint searchrange = 30;

   const Rox_Uint threshold_response = 51;
   Rox_Search_Edge search_edge = NULL;
   Rox_Array2D_Uchar image = NULL;

   Rox_Point2D_Double_Struct point;
   Rox_Point2D_Double_Struct point_grt;

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_search_edge_new ( &search_edge, searchrange, threshold_response );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Set site coordinates and search direction

   // The search direction is the angle of the norm to the edge
   // If the edge is horizontal then angle = 90 or -90; if the edge is vertical then angle = 0 or 180;

   search_edge->_angle = ROX_PI/2.0;
   
   //point.u = 1568-1;
   //point.v = 1648-1;

   point_grt.u = 130;
   point_grt.v =  95;

   point.u = point_grt.u;
   point.v = point_grt.v+20;

   error = rox_search_edge_track ( search_edge, image, &point );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("found edge at coordinates: (%f, %f) \n", search_edge->_coords.u, search_edge->_coords.v);

   // Same results as Matlab is required
   ROX_TEST_CHECK_EQUAL(search_edge->_coords.u, point_grt.u);
   ROX_TEST_CHECK_EQUAL(search_edge->_coords.v, point_grt.v);

   error = rox_search_edge_del(&search_edge);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_rox_search_edge_track_gradient )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   const Rox_Uint searchrange = 30;

   const Rox_Uint threshold_response = 51;
   const Rox_Uint scale_threshold = 1;

   Rox_Search_Edge search_edge = NULL;
   Rox_Array2D_Uchar image = NULL;
   Rox_Array2D_Uint gradient_scale = NULL;
   Rox_Array2D_Float gradient_angle = NULL;

   Rox_Point2D_Double_Struct point;
   Rox_Point2D_Double_Struct point_grt;

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   // Create and read the uchar image
   error = rox_array2d_uchar_new_pgm ( &image, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint rows, cols;
   error = rox_array2d_uchar_get_size ( &rows, &cols, image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_new ( &gradient_scale, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_float_new ( &gradient_angle, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_gradient_sobel_angle_scale_nomask ( gradient_angle, gradient_scale, image, scale_threshold );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_search_edge_new ( &search_edge, searchrange, threshold_response );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Set site coordinates and search direction

   // The search direction is the angle of the norm to the edge
   // If the edge is horizontal then angle = 90 or -90; if the edge is vertical then angle = 0 or 180;

   search_edge->_angle = ROX_PI/2.0;
   
   //point.u = 1568-1;
   //point.v = 1648-1;

   point_grt.u = 130;
   point_grt.v =  95;

   point.u = point_grt.u;
   point.v = point_grt.v+20;

   error = rox_search_edge_track_gradient ( search_edge, gradient_scale, gradient_angle, &point );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("found edge at coordinates: (%f, %f) \n", search_edge->_coords.u, search_edge->_coords.v);

   // Same results as Matlab is required
   ROX_TEST_CHECK_EQUAL(search_edge->_coords.u, point_grt.u);
   ROX_TEST_CHECK_EQUAL(search_edge->_coords.v, point_grt.v);

   error = rox_search_edge_del(&search_edge);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uchar_del ( &image );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
