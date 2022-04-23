//==============================================================================
//
//    OPENROX   : File test_point2d_undistort.cpp
//
//    Contents  : Tests for pointundistort.c
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
   #include <baseproc/geometry/transforms/distortion/point2d_undistort.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <inout/system/errors_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(point2d_undistort)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

Rox_ErrorCode rox_transformtools_build_distortion_vector(Rox_Array2D_Double distortion, Rox_Double k1, Rox_Double k2, Rox_Double k3, Rox_Double k4, Rox_Double k5);

Rox_ErrorCode rox_transformtools_build_distortion_vector(Rox_Array2D_Double distortion, Rox_Double k1, Rox_Double k2, Rox_Double k3, Rox_Double k4, Rox_Double k5)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double * k = NULL;
   error = rox_array2d_double_get_data_pointer( &k, distortion);
   
   k[0] = k1;
   k[1] = k2;
   k[2] = k3;
   k[3] = k4;
   k[4] = k5;

   return error;
}

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_point_float_undistort)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Point2D_Double_Struct undistorted; // pu = [0.1;0.2;1]
   Rox_Point2D_Double_Struct distorted;
   Rox_Array2D_Double distortion = NULL;
   Rox_Array2D_Double calib = NULL;

   error = rox_array2d_double_new(&calib, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   // allocate calibration matrix
   error = rox_transformtools_build_calibration_matrix(calib, 1136.321181701803, 1134.623063493537, 949.527967806592, 541.331950117441); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_new(&distortion, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_transformtools_build_distortion_vector(distortion, -0.3424382003730514, 0.2061303912968982, 0.0007118786435634782, -0.0002685739056181367, -0.06979589096142937); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   distorted.u = 1061.283047150836; //0.098348144119671
   distorted.v =  764.578953968498; //0.196758739562082
   
   error = rox_point2d_double_undistort(&undistorted, &distorted, calib, distortion);

   ROX_TEST_CHECK_SMALL(undistorted.u - 0.1, 1e-12);
   ROX_TEST_CHECK_SMALL(undistorted.v - 0.2, 1e-12);


function_terminate:

   error = rox_array2d_double_del(&calib); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&distortion); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
