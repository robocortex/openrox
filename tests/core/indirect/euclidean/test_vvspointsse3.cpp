//==============================================================================
//
//    OPENROX   : File test_vvspointsse3.cpp
//
//    Contents  : Tests for vvspointsse3.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

// ====== INCLUDED HEADERS   ===================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <baseproc/maths/linalg/matse3.h>
   #include <baseproc/maths/linalg/matut3.h>
   #include <core/indirect/euclidean/vvspointsse3.h>
}

// ====== INTERNAL MACROS    ===================================================

ROX_TEST_SUITE_BEGIN(vvspointsse3)

// ====== INTERNAL TYPESDEFS ===================================================

// ====== INTERNAL DATATYPES ===================================================

// ====== INTERNAL VARIABLES ===================================================

// ====== INTERNAL FUNCTDEFS ===================================================

// ====== INTERNAL FUNCTIONS ===================================================

// ====== EXPORTED FUNCTIONS ===================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_points_float_refine_pose_vvs)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_points_double_refine_pose_vvs)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 cTo = NULL;
   Rox_MatUT3 Kc = NULL;
   Rox_DynVec_Point2D_Double pc;
   Rox_DynVec_Point3D_Double mo;
   Rox_Double maxdist_prefilter = 1.0;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   error = rox_matse3_new ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE); 

   error = rox_matut3_new ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE); 

   error = rox_dynvec_point2d_double_new ( &pc, 10 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE); 

   error = rox_dynvec_point3d_double_new ( &mo, 10 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE); 

   error = rox_points_double_refine_pose_vvs ( cTo, Kc, pc, mo, maxdist_prefilter );
   rox_error_print(error);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_BAD_SIZE ); 

   error = rox_matse3_del ( &cTo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE); 

   error = rox_matut3_del ( &Kc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE); 

   error = rox_dynvec_point2d_double_del ( &pc );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE); 

   error = rox_dynvec_point3d_double_del ( &mo );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE); 
}

ROX_TEST_SUITE_END()
