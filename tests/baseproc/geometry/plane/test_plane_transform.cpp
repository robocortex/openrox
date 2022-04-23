//==============================================================================
//
//    OPENROX   : File test_plane_transform.cpp
//
//  	Contents  : Tests for plane_transform.c
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
	#include <baseproc/geometry/plane/plane_transform.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(plane_transform)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_plane_transform)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
	Rox_Plane3D_Double_Struct plane_mes = {0.052205405943797, -0.901858352813558, -0.428866071228139, -0.105814003959353};
	Rox_Plane3D_Double_Struct plane_grt = {0.052205405943797, -0.901858352813558, -0.428866071228139, -0.105814003959353};
	Rox_MatSE3 pose = NULL;

	error = rox_matse3_new(&pose);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_plane3d_transform(&plane_mes, pose, &plane_grt);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_CHECK_SMALL(plane_mes.a - plane_grt.a, 1e-12);
   ROX_TEST_CHECK_SMALL(plane_mes.b - plane_grt.b, 1e-12);
   ROX_TEST_CHECK_SMALL(plane_mes.c - plane_grt.c, 1e-12);
   ROX_TEST_CHECK_SMALL(plane_mes.d - plane_grt.d, 1e-12);

	error = rox_matse3_del(&pose);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
