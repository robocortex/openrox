//==============================================================================
//
//    OPENROX   : File test_combination.cpp
//
//    Contents  : Tests for combination.c
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
	#include <baseproc/maths/random/combination.h>
}

// ====== INTERNAL MACROS    ===================================================

ROX_TEST_SUITE_BEGIN(combination)

//#define DISPLAY

// ====== INTERNAL TYPESDEFS ===================================================

// ====== INTERNAL DATATYPES ===================================================

// ====== INTERNAL VARIABLES ===================================================

// ====== INTERNAL FUNCTDEFS ===================================================

// ====== INTERNAL FUNCTIONS ===================================================

// ====== EXPORTED FUNCTIONS ===================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_combination_new_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   Rox_Combination combination = NULL;
   Rox_Uint nb_items = 8;
   Rox_Uint nb_draws = 3;
   
   error = rox_combination_new(&combination, nb_items, nb_draws);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_combination_del(&combination);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_combination_draw)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   Rox_Combination combination = NULL;
   Rox_Uint nb_items = 8;
   Rox_Uint nb_draws = 3;
   
   error = rox_combination_new(&combination, nb_items, nb_draws);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_combination_draw(combination);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   rox_combination_print_draw(combination);
   rox_combination_print_bag(combination);

   error = rox_combination_draw(combination);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_combination_print_draw(combination);
   rox_combination_print_bag(combination);
   
//#ifdef DISPLAY
    //error = rox_combination_print(combination);
    //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
//#endif

   error = rox_combination_del(&combination);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
