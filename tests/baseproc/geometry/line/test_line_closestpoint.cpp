//==============================================================================
//
//    OPENROX   : File test_line_closestpoint.cpp
//
//    Contents  : Tests for line_closestpoint.c
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
   #include <baseproc/geometry/line/line_from_points.h>
	#include <baseproc/geometry/line/line_closestpoint.h>
   #include <baseproc/geometry/line/line3d_struct.h>

   #include <inout/geometry/point/point3d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(line_closestpoint)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_segment3d_backproject)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct res;
   Rox_Segment3D_Struct segment;  
   Rox_Point2D_Double_Struct ray;

   ray.u = 0.015334;
   ray.v = 0.077041;

   segment.points[0].X = 0.324976;
   segment.points[0].Y = 0.373339;
   segment.points[0].Z = 4.835531; 

   segment.points[1].X = 0.324976;
   segment.points[1].Y =-0.057863;
   segment.points[1].Z = 5.088641;

   error = rox_segment3d_backproject ( &res, &segment, &ray );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_point3d_double_print ( &res );

   // old result
   // point3D = [0.0742275254901351; 0.3729335327563257; 4.8407151095692651]

   ROX_TEST_CHECK_CLOSE ( res.X, 0.3249760000000000, 1e-12 );
   ROX_TEST_CHECK_CLOSE ( res.Y, 0.3707742290809211, 1e-12 );
   ROX_TEST_CHECK_CLOSE ( res.Z, 4.8370364873755873, 1e-12 );

}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_point3d_closests_from_2_lines3d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct res;
   Rox_Segment3D_Struct segment;  
   Rox_Point2D_Double_Struct ray;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ray.u = 0.015334;
   ray.v = 0.077041;

   segment.points[0].X = 0.324976;
   segment.points[0].Y = 0.373339;
   segment.points[0].Z = 4.835531; 

   segment.points[1].X = 0.324976;
   segment.points[1].Y =-0.057863;
   segment.points[1].Z = 5.088641;

   error = rox_segment3d_backproject ( &res, &segment, &ray );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Point3D_Double_Struct pt1;
   Rox_Point3D_Double_Struct pt2;

   Rox_Point3D_Double_Struct pts_line1;
   Rox_Point3D_Double_Struct pts_line2;
   Rox_Line3D_Parametric_Struct line3d1; 
   Rox_Line3D_Parametric_Struct line3d2;

   pt1.X = 1.0; pt1.Y = 0.0; pt1.Z = 0.0;
   pt2.X = 2.0; pt2.Y = 0.0; pt2.Z = 0.0;

   error = rox_line3d_parametric_from_2_point3d ( &line3d1, &pt1, &pt2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   pt1.X = 0.0; pt1.Y = 1.0; pt1.Z = 0.1;
   pt2.X = 0.0; pt2.Y = 2.0; pt2.Z = 0.1;

   error = rox_line3d_parametric_from_2_point3d ( &line3d2, &pt1, &pt2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_point3d_closests_from_2_lines3d ( &pts_line1, &pts_line2, &line3d1, &line3d2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_point3d_double_print ( &pts_line1 );
   rox_point3d_double_print ( &pts_line2 );

}

ROX_TEST_SUITE_END()
