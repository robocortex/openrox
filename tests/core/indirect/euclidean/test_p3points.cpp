//==============================================================================
//
//    OPENROX   : File test_p3points.cpp
//
//    Contents  : Tests for p3points.c
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
   #include <core/indirect/euclidean/p3points.h>
   #include <inout/numeric/array2d_print.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <baseproc/geometry/point/point2d_matsl3_transform.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(p3points)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_pose_from_two_3D_triangles)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double cTo = NULL;
   Rox_Point3D_Double_Struct ctriangle[3];
   Rox_Point3D_Double_Struct otriangle[3];

   error = rox_array2d_double_new(&cTo,4,4);
   
   ctriangle[0].X=0.636407070917320; ctriangle[1].X=0.770473942372034; ctriangle[2].X= 0.625536310131962;
   ctriangle[0].Y=0.159414875689968; ctriangle[1].Y=0.306274942614747; ctriangle[2].Y= 0.442040973826998;
   ctriangle[0].Z=2.008906761879063; ctriangle[1].Z=1.985280617013802; ctriangle[2].Z= 2.010984070663372;
   
   /* True pose
   0.675548777432067  -0.729902581358856   0.104288403169853   0.629928806492942
   0.734135403681394   0.678995087003757  -0.003298618850060   0.300760910946984
  -0.068403642970219   0.078790186891761   0.994541627121743   2.000000000000000
   */
   
   otriangle[0].X=-0.10; otriangle[1].X=+0.10; otriangle[2].X=0.10;
   otriangle[0].Y=-0.10; otriangle[1].Y=-0.10; otriangle[2].Y=0.10;
   otriangle[0].Z=+0.01; otriangle[1].Z=+0.00; otriangle[2].Z=0.01;
   
   error = rox_pose_from_two_3D_triangles(cTo, ctriangle, otriangle);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_array2d_double_print(cTo);
   
   error = rox_array2d_double_del(&cTo);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_pose_from_3_points)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double_Collection possible_poses;
   Rox_Uint validposes = 0;
   Rox_Point3D_Double_Struct points3D_met[3];
   Rox_Point2D_Double_Struct points2D_pix[3]; 
   Rox_Point2D_Double_Struct points2D_nor[3]; 
   
   Rox_Array2D_Double K = NULL;
   Rox_Array2D_Double Ki = NULL;

   double fu = 1000.0, fv = 1000.0, cu = 320.0, cv = 240.0;
   
   error = rox_array2d_double_new(&K, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&Ki, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_collection_new(&possible_poses, 4, 4, 4); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   points3D_met[0].X=-0.10; points3D_met[1].X=+0.10; points3D_met[2].X=0.10;
   points3D_met[0].Y=-0.10; points3D_met[1].Y=-0.10; points3D_met[2].Y=0.10;
   points3D_met[0].Z=+0.01; points3D_met[1].Z=+0.00; points3D_met[2].Z=0.01;
   
   // 2D points in pixel coordinates
   points2D_pix[0].u = 636.7927367231548; points2D_pix[1].u = 708.0932175376586; points2D_pix[2].u = 631.0598036341548;
   points2D_pix[0].v = 319.3540440577026; points2D_pix[1].v = 394.2728720514265; points2D_pix[2].v = 459.8132646974077;
   
   // error = rox_pose_from_3_points(possible_poses, &validposes, points3D_met, points2D_pix, fu, fv, cu, cv);
   error = rox_transformtools_build_calibration_matrix(K, fu, fv, cu, cv);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_pose_from_p3p_pix(possible_poses, &validposes, points3D_met, points2D_pix, K);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_transformtools_build_calibration_matrix(Ki, 1.0/fu, 1.0/fv, -cu/fu, -cv/fv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Compute points2D_nor = Ki * points2D_pix
   error = rox_point2d_double_homography(points2D_nor, points2D_pix, Ki, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // error = rox_pose_from_p3p_nor(possible_poses, &validposes, points3D_met, points2D_nor);

   rox_log("valid poses = %d\n", validposes);
   for (Rox_Uint id_pose = 0; id_pose < validposes; id_pose++)
   {
      Rox_Array2D_Double current_pose = NULL;
      current_pose = rox_array2d_double_collection_get(possible_poses, id_pose);
      rox_array2d_double_print(current_pose);
   }
   
   /* True pose
   0.675548777432067  -0.729902581358856   0.104288403169853   0.629928806492942
   0.734135403681394   0.678995087003757  -0.003298618850060   0.300760910946984
  -0.068403642970219   0.078790186891761   0.994541627121743   2.000000000000000
   */
   

   error = rox_array2d_double_del(&K);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&Ki);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_collection_del(&possible_poses); 
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_SUITE_END()
