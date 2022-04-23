//==============================================================================
//
//    OPENROX   : File test_version.cpp
//
//    Contents  : Tests for version.c
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
   #include <system/version/version.h>
   #include <system/errors/errors.h>
}

#include <stdio.h>

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(version)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_version)
{
   Rox_ErrorCode  error = ROX_ERROR_NONE;
   Rox_Uint major = 0, minor = 0, patch = 0;

   error = rox_get_version(&major, &minor, &patch); 
   rox_error_print(error);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("major = %d\n", major);
   ROX_TEST_MESSAGE("minor = %d\n", minor);
   ROX_TEST_MESSAGE("patch = %d\n", patch);

   ROX_TEST_CHECK_EQUAL(major, (Rox_Uint) atoi(OPENROX_MAJOR_VERSION));
   ROX_TEST_CHECK_EQUAL(minor, (Rox_Uint) atoi(OPENROX_MINOR_VERSION));
   ROX_TEST_CHECK_EQUAL(patch, (Rox_Uint) atoi(OPENROX_PATCH_VERSION));
}

ROX_TEST_SUITE_END()
