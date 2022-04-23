//==============================================================================
//
//    OPENROX   : File test_sl3interfrom3dpoints.cpp
//
//    Contents  : Tests for sl3interfrom3dpoints.c
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
   #include <baseproc/array/error/l2_error.h>
   #include <baseproc/geometry/point/point3d_struct.h>
	#include <baseproc/geometry/transforms/matsl3/sl3interfrom3dpoints.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(sl3interfrom3dpoints)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_sl3_from_3d_points_double)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double l2_error = 0.0;
   Rox_Double homography_grt_data[9] = { 12.4289300238154485,  0.0000000000000000, -3.0690394397348464, 
                                          0.0000000000000000, 12.4289300238154503,  0.4110609669334797,
                                          0.0000000000000000,  0.0000000000000000,  0.0064734010540705 }; 

   Rox_Plane3D_Double_Struct plane;
   Rox_MatSL3 homography_grt = NULL;
   Rox_MatSL3 homography = NULL;
   Rox_MatSE3 pose = NULL;
   Rox_Point3D_Double_Struct vertices[4]; 
   Rox_Sint width = 128; 
   Rox_Sint height =128;

   vertices[0].X = 0.28 -1.0/30.0;
   vertices[0].Y =      -1.0/30.0;
   vertices[0].Z = 0.0;

   vertices[1].X = 0.28 +1.0/30.0;
   vertices[1].Y =      -1.0/30.0;
   vertices[1].Z = 0.0;

   vertices[2].X = 0.28 +1.0/30.0;
   vertices[2].Y =       1.0/30.0;
   vertices[2].Z = 0.0;
   
   vertices[3].X = 0.28 -1.0/30.0;
   vertices[3].Y =       1.0/30.0;
   vertices[3].Z = 0.0;

   error = rox_matse3_new ( &pose );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_new ( &homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_new ( &homography_grt );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_set_data ( homography_grt, homography_grt_data );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_sl3_from_3d_points_double ( &plane, homography, pose, vertices, width, height );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_matse3_print(pose);
   rox_matsl3_print(homography);
   rox_matsl3_print(homography_grt);

   error = rox_array2d_double_difference_l2_norm ( &l2_error, homography_grt, homography );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("l2_error homography = %f \n", l2_error);

   ROX_TEST_CHECK_CLOSE (l2_error, 0.0, 1e-12);


}

ROX_TEST_SUITE_END()
