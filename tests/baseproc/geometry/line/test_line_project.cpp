//==============================================================================
//
//    OPENROX   : File test_line_project.cpp
//
//    Contents  : Tests for line_project.c
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
   #include <baseproc/geometry/line/line2d_struct.h>
   #include <baseproc/geometry/line/line3d_struct.h>
   #include <baseproc/geometry/line/line_project.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <inout/geometry/line/line2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(line_project)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_line3d_planes_project )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Line2D_Normal_Struct line2d_struct;
   Rox_Line3D_Planes_Struct line3d_struct; 
   Rox_Double fu = 1000; 
   Rox_Double fv = 1000; 
   Rox_Double cu = 320;
   Rox_Double cv = 240;

   Rox_MatUT3 K = NULL;

   error = rox_matut3_new( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix ( K, fu, fv, cu, cv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   line3d_struct.planes[0].a = 0.0; 
   line3d_struct.planes[0].b = 0.0; 
   line3d_struct.planes[0].c = 1.0; 
   line3d_struct.planes[0].d = 1.0;
   
   line3d_struct.planes[1].a = 0.0; 
   line3d_struct.planes[1].b = sqrt(2.0)/2.0; 
   line3d_struct.planes[1].c = sqrt(2.0)/2.0; 
   line3d_struct.planes[1].d = 2.0;

   error = rox_line3d_planes_project_ ( &line2d_struct, &line3d_struct, fu, fv, cu, cv );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_line2d_normal_print ( &line2d_struct );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_line3d_planes_project ( &line2d_struct, &line3d_struct, K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_line2d_normal_print ( &line2d_struct );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_del ( &K );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_line3d_planes_project_meters )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Line2D_Normal_Struct line2d_struct;
   Rox_Line3D_Planes_Struct line3d_struct; 

   line3d_struct.planes[0].a = 0.0; 
   line3d_struct.planes[0].b = 0.0; 
   line3d_struct.planes[0].c = 1.0; 
   line3d_struct.planes[0].d = 1.0;
   
   line3d_struct.planes[1].a = 0.0; 
   line3d_struct.planes[1].b = sqrt(2.0)/2.0; 
   line3d_struct.planes[1].c = sqrt(2.0)/2.0; 
   line3d_struct.planes[1].d = 2.0;

   error = rox_line3d_planes_project_meters ( &line2d_struct, &line3d_struct );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_line2d_normal_print ( &line2d_struct );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
