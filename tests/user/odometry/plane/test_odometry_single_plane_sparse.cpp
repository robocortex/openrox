//==============================================================================
//
//    OPENROX   : File test_odometry_single_plane_sparse.cpp
//
//    Contents  : Tests for odometry_single_plane_sparse.c
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
   #include <baseproc/geometry/point/point2d_struct.h>
   #include <baseproc/geometry/rectangle/rectangle.h>
   #include <user/odometry/plane/odometry_singleplane_sparse.h>
}

//=== INTERNAL MACROS     ======================================================

ROX_TEST_SUITE_BEGIN(odometry_single_plane_sparse)

//===  INTERNAL TYPESDEFS ======================================================

//===  INTERNAL DATATYPES ======================================================

//===  INTERNAL VARIABLES ======================================================

//===  INTERNAL FUNCTDEFS ======================================================

//===  INTERNAL FUNCTIONS ======================================================

//===  EXPORTED FUNCTIONS ======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_odometry_single_plane_sparse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_MatSE3 T = NULL; 
   Rox_MatUT3 K = NULL;
   Rox_Point2D_Double_Struct p[4];

   const Rox_Double size_x = 100;
   const Rox_Double size_y = 100;

   error = rox_matse3_new(&T);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new(&K);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_rectangle2d_create_centered_plane_xright_ydown ( p, 2*size_x, 2*size_y );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_odometry_singleplane_sparse ( T, K, p, size_x, size_y );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_matse3_print(T);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &T );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()
