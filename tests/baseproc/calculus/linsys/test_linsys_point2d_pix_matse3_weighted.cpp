//==============================================================================
//
//    OPENROX   : File test_linsys_point2d_pix_matse3_weighted.cpp
//
//    Contents  : Tests for linsys_point2d_pix_matse3_weighted.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =====================================================

#include <openrox_tests.hpp>

extern "C"
{
	#include <baseproc/calculus/linsys/linsys_point2d_pix_matse3_weighted.h>

}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( linsys_point2d_pix_matse3_weighted )

#define FU 1415
#define FV 1431
#define CU 954
#define CV 559

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_jacobian_se3_from_points_pixels_weighted_premul_float)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Matrix                     LtL = NULL;
   Rox_Matrix                     Lte = NULL;
   Rox_Array2D_Double             diff = NULL;
   Rox_Array2D_Double             weight = NULL;
   Rox_DynVec_Point3D_Float       meters = NULL;
   Rox_Array2D_Double             calib = NULL;
   Rox_Sint nbp = 10;

   error = rox_matrix_new ( &Lte, 6, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matrix_new ( &LtL, 6, 6 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_new ( &calib );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matut3_build_calibration_matrix ( calib, FU, FV, CU, CV );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point3d_float_new ( &meters, nbp );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for ( Rox_Sint k=0; k<nbp; k++ )
   {
      Rox_Point3D_Float_Struct point;
      error = rox_dynvec_point3d_float_append ( meters, &point );
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_array2d_double_new ( &diff, 2*nbp, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new ( &weight, nbp, 1 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_jacobian_se3_from_points_pixels_weighted_premul_float ( LtL, Lte, diff, weight, meters, calib );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );


}

ROX_TEST_SUITE_END()
