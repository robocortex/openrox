//==============================================================================
//
//    OPENROX   : File test_quadtree_ref.cpp
//
//       Contents  : Tests for quadtree_ref.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

// ======= INCLUDED HEADERS   ==================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <core/occupancy/quadtree_ref.h>
   #include <inout/system/print.h>
}

// ======= INTERNAL MACROS    ==================================================

ROX_TEST_SUITE_BEGIN(quadtree_ref)

// ======= INTERNAL TYPESDEFS ==================================================

// ======= INTERNAL DATATYPES ==================================================

// ======= INTERNAL VARIABLES ==================================================

// ======= INTERNAL FUNCTDEFS ==================================================

// ======= INTERNAL FUNCTIONS ==================================================

// ======= EXPORTED FUNCTIONS ==================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quadtree_ref_new)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   rox_log("This test has not been implemented yet \n");
   
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quadtree_ref_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   rox_log("This test has not been implemented yet \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quadtree_ref_add)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   rox_log("This test has not been implemented yet \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quadtree_ref_reset)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   rox_log("This test has not been implemented yet \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quadtree_ref_search)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   rox_log("This test has not been implemented yet \n");

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
