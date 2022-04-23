//==============================================================================
//
//    OPENROX   : File test_random.cpp
//
//    Contents  : Tests for random.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

// ====== INCLUDED HEADERS   ===================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <system/errors/errors.h>
   #include <baseproc/maths/random/random.h>
   #include <inout/system/print.h>
}

// ====== INTERNAL MACROS    ===================================================

ROX_TEST_SUITE_BEGIN(random)

// ====== INTERNAL TYPESDEFS ===================================================

// ====== INTERNAL DATATYPES ===================================================

// ====== INTERNAL VARIABLES ===================================================

// ====== INTERNAL FUNCTDEFS ===================================================

// ====== INTERNAL FUNCTIONS ===================================================

// ====== EXPORTED FUNCTIONS ===================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_random_new_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Random random = NULL;
   
   error = rox_random_new(&random);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_random_del(&random);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_random_draw)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint draw = 0;
   
   Rox_Random random = NULL;
   
   error = rox_random_new(&random);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_random_get_lcg_draw(&draw, random);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("draw = %d\n", draw);
   rox_log("draw = %d\n", rox_rand());

   error = rox_random_get_lcg_draw(&draw, random);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_log("draw = %d\n", draw);
   rox_log("draw = %d\n", rox_rand());

   error = rox_random_del(&random);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_random_draw_again)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint draw = 0;
   
   Rox_Random random = NULL;
   
   rox_srand(1);

   error = rox_random_new(&random);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_random_get_lcg_draw(&draw, random);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("draw = %d\n", draw);
   rox_log("draw = %d\n", rox_rand());

   error = rox_random_get_lcg_draw(&draw, random);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_log("draw = %d\n", draw);
   rox_log("draw = %d\n", rox_rand());

   error = rox_random_del(&random);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
