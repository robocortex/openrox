//==============================================================================
//
//    OPENROX   : File test_fastst.cpp
//
//    Contents  : Tests for fastst.c
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
   #include <generated/dynvec_segment_point_struct.h>
	#include <core/features/detectors/segment/fastst.h>
   #include <core/features/detectors/segment/fastst_score.h>
   #include <core/features/detectors/segment/segmentpoint_tools.h>
   #include <baseproc/image/image.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(fastst)

// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/debug/areva_quality_score_1_900x506.pgm"
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/toy_512x512.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_fastst_detector)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   //! Keypoints 
   Rox_DynVec_Segment_Point fast_points = NULL;

   // Keypoints with non maximum suppression 
   Rox_DynVec_Segment_Point fast_points_nonmax = NULL;

   Rox_Array2D_Uchar dest = NULL;

   // Read the image
   sprintf(filename, "%s", IMAGE_PATH);

   error = rox_image_new_read_pgm(&dest, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_segment_point_new ( &fast_points, 2000 );   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_segment_point_new ( &fast_points_nonmax, 2000 );   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("number of used keypoints before FAST detector = %d \n", fast_points->used);

   // Detect keypoints
   error = rox_fastst_detector ( fast_points, dest, 20, 0 ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("number of detected keypoints with FAST detector = %d \n", fast_points->used);

   error = rox_fastst_detector_score ( fast_points, dest, 20 ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_fastst_nonmax_suppression(fast_points_nonmax, fast_points); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_fastst_detector_sort(fast_points_nonmax); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("number of detected keypoints with FAST after nonmax suppression = %d \n", fast_points_nonmax->used);

   rox_segment_points_print ( fast_points_nonmax );

   error = rox_image_del(&dest);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_segment_point_del(&fast_points);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_segment_point_del(&fast_points_nonmax);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
