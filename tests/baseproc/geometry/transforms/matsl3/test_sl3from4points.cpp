//==============================================================================
//
//    OPENROX   : File test_sl3from4points.cpp
//
//    Contents  : Tests for sl3from4points.c
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
	#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
   #include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( sl3from4points )

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_sl3_from_4_points_double)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
      
   Rox_Array2D_Double homography = NULL;
   
   Rox_Point2D_Double_Struct points_2d[4];
   Rox_Point2D_Double_Struct points_3d[4];
   
   Rox_Double size_x = 0.297; 
   Rox_Double size_y = 0.210;

   error = rox_array2d_double_new ( &homography, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   points_2d[0].u = 171.5; points_2d[1].u = 468.5; points_2d[2].u = 468.5; points_2d[3].u = 171.5;
   points_2d[0].v = 135.0; points_2d[1].v = 135.0; points_2d[2].v = 345.0; points_2d[3].v = 345.0;
   
   points_3d[0].u = -size_x/2.0; points_3d[1].u = +size_x/2.0; points_3d[2].u = +size_x/2.0; points_3d[3].u = -size_x/2.0;
   points_3d[0].v = -size_y/2.0; points_3d[1].v = -size_y/2.0; points_3d[2].v = +size_y/2.0; points_3d[3].v = +size_y/2.0;
   
   error = rox_matsl3_from_4_points_double ( homography, points_3d, points_2d );

   rox_array2d_double_print(homography);
   
   error = rox_array2d_double_del ( &homography ); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_sl3_from_4_points_float)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   Rox_Array2D_Double homography = NULL;
   
   Rox_Point2D_Float_Struct points_2d[4];
   Rox_Point2D_Float_Struct points_3d[4];
   
   Rox_Float size_x = 0.297f; 
   Rox_Float size_y = 0.210f;

   error = rox_array2d_double_new ( &homography, 3, 3 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   points_2d[0].u = 171.5f; points_2d[1].u = 468.5f; points_2d[2].u = 468.5f; points_2d[3].u = 171.5f;
   points_2d[0].v = 135.0f; points_2d[1].v = 135.0f; points_2d[2].v = 345.0f; points_2d[3].v = 345.0f;
   
   points_3d[0].u = -size_x/2.0; points_3d[1].u = +size_x/2.0; points_3d[2].u = +size_x/2.0; points_3d[3].u = -size_x/2.0;
   points_3d[0].v = -size_y/2.0; points_3d[1].v = -size_y/2.0; points_3d[2].v = +size_y/2.0; points_3d[3].v = +size_y/2.0;
   
   error = rox_matsl3_from_4_points_float ( homography, points_3d, points_2d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   if (error) goto terminate;
   
   rox_array2d_double_print(homography);

terminate:
   error = rox_array2d_double_del(&homography); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
