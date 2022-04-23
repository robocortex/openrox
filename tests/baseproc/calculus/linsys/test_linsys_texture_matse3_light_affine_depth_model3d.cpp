//==============================================================================
//
//    OPENROX   : File test_linsys_matse3_light_affine_depth_texture_model3d.cpp
//
//    Contents  : Tests for linsys_matse3_light_affine_depth_texture_model3d.c
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
	#include <baseproc/calculus/linsys/linsys_texture_matse3_light_affine_depth_model3d.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( linsys_texture_matse3_light_affine_depth_model3d )

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_jacobian_se3_z_group_light_affine_weighted_premul)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE("This test has not been implemented yet !!! \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
