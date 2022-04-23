//==============================================================================
//
//    OPENROX   : File test_point2d_projection_from_points3d.cpp
//
//    Contents  : Tests for pointsproject.c
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
   #include <baseproc/geometry/point/point3d_struct.h>
	#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
   #include <baseproc/array/fill/fillunit.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(pointsproject)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_point2d_projection_from_points3d)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct output2d[4] ;
   Rox_Point3D_Double_Struct input3d[4];
   Rox_Array2D_Double K = NULL;
   Rox_Uint nbpts = 4;

   input3d[0].X = 1.0; input3d[0].Y = 2.0; input3d[0].Z = 2.0;
   input3d[1].X = 2.0; input3d[1].Y = 4.0; input3d[1].Z = 2.0;
   input3d[2].X = 4.0; input3d[2].Y = 4.0; input3d[2].Z = 4.0;
   input3d[3].X = 8.0; input3d[3].Y = 4.0; input3d[3].Z = 4.0;

   error = rox_array2d_double_new(&K,3,3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_fillunit(K);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_point2d_double_project(output2d, input3d, K, nbpts);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      ROX_TEST_MESSAGE("point[%d] = (%f, %f) \n", i, output2d[i].u, output2d[i].v);
   }

   error = rox_array2d_double_del(&K);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_point3d_float_project)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_point3d_double_perspective_projection)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}
ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_point3d_float_project_meters)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
