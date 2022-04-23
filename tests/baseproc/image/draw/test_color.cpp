//==============================================================================
//
//    OPENROX   : File test_color.cpp
//
//    Contents  : Tests for color.c
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
   #include <baseproc/image/draw/color.h>
   #include <inout/system/print.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(color)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Uchar r = 0;
   Rox_Uchar g = 0;
   Rox_Uchar b = 0;
   Rox_Uchar a = 0;

   Rox_Uint rgba = ROX_MAKERGBA(1, 2, 3, 4);

   rox_log("color = %d\n", rgba);

   error = rox_color_get_uchar_r_g_b_a_from_uint_rgba (&r, &g, &b, &a, rgba);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

   rox_log("(r, g, b, a) = (%d, %d, %d, %d) \n", r, g, b, a);

   ROX_TEST_CHECK_EQUAL(r, 1);
   ROX_TEST_CHECK_EQUAL(g, 2);
   ROX_TEST_CHECK_EQUAL(b, 3);
   ROX_TEST_CHECK_EQUAL(a, 4);
}

ROX_TEST_SUITE_END()