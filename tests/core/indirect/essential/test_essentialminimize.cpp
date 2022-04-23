//==============================================================================
//
//    OPENROX   : File test_essentialminimize.cpp
//
//    Contents  : Tests for essentialminimize.c
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
   #include <baseproc/maths/linalg/matse3.h>
	#include <core/indirect/essential/essentialminimize.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN(essentialminimize)

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_essential_minimize)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_MatSE3 pose = NULL;
   Rox_DynVec_Uint mask = NULL;
   Rox_DynVec_Point2D_Float ref = NULL;
   Rox_DynVec_Point2D_Float cur = NULL;

   error = rox_matse3_new ( &pose );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_float_new ( &ref, 10 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_float_new ( &cur, 10 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_uint_new ( &mask, 10 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (Rox_Uint k=0; k<10; k++)
   {
      Rox_Uint mask_value = ~0;
      error = rox_dynvec_uint_append (mask, &mask_value);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }

   error = rox_essential_minimize ( pose, mask, ref, cur );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_BAD_SIZE);

   error = rox_dynvec_uint_del ( &mask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_float_del ( &ref );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_point2d_float_del ( &cur );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del ( &pose );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
