//==============================================================================
//
//    OPENROX   : File openrox_tests.hpp
//
//    Contents  : Implementation of routines used for tests
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

// common_tests.cpp
#include "openrox_tests.hpp"

//std::vector<int> RoxTest::_rox_test_suit_global_results;
rox_tests_log_callback RoxTest::_log_callback = NULL;

ROX_EXPORT_C void rox_tests_set_log_callback(rox_tests_log_callback log_callback)
{
   RoxTest::_log_callback = log_callback;
}