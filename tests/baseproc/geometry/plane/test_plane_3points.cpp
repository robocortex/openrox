//==============================================================================
//
//    OPENROX   : File test_plane_from_3_points.cpp
//
//  	Contents  : Tests for plane_3points.c
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
   #include <baseproc/geometry/point/point3d_struct.h>
	#include <baseproc/geometry/plane/plane_3points.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(plane_from_3_point3d)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_from_3_point3d)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	Rox_Point3D_Double_Struct m1 = { 0.325, -0.750,  1.370};
	Rox_Point3D_Double_Struct m2 = {-1.715, -0.102, -0.241};
	Rox_Point3D_Double_Struct m3 = { 0.319,  0.312, -0.864};

	Rox_Plane3D_Double_Struct plane_grt = {0.052205405943797, -0.901858352813558, -0.428866071228139, -0.105814003959353};
	Rox_Plane3D_Double_Struct plane_mes;

	error = rox_plane3d_from_3_point3d(&plane_mes, &m1, &m2, &m3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_SMALL(plane_mes.a - plane_grt.a, 1e-12);
   ROX_TEST_CHECK_SMALL(plane_mes.b - plane_grt.b, 1e-12);
   ROX_TEST_CHECK_SMALL(plane_mes.c - plane_grt.c, 1e-12);
   ROX_TEST_CHECK_SMALL(plane_mes.d - plane_grt.d, 1e-12);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_plane_from_3_point3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Point3D_Double_Struct m1 = { 0.325, -0.750,  1.370};
   Rox_Point3D_Double_Struct m2 = {-1.715, -0.102, -0.241};
   Rox_Point3D_Double_Struct m3 = { 0.319,  0.312, -0.864};

   Rox_Plane3D_Double_Struct plane_grt = {0.052205405943797, -0.901858352813558, -0.428866071228139, -0.105814003959353};
   Rox_Plane3D_Double_Struct plane_mes;

   error = rox_plane3d_from_3_point3d(&plane_mes, &m1, &m2, &m3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_SMALL(plane_mes.a - plane_grt.a, 1e-12);
   ROX_TEST_CHECK_SMALL(plane_mes.b - plane_grt.b, 1e-12);
   ROX_TEST_CHECK_SMALL(plane_mes.c - plane_grt.c, 1e-12);
   ROX_TEST_CHECK_SMALL(plane_mes.d - plane_grt.d, 1e-12);
}

ROX_TEST_SUITE_END()
