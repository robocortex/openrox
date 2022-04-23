//==============================================================================
//
//    OPENROX   : File test_sraid_match.cpp
//
//    Contents  : Tests for sraid_match.c
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
   #include <system/memory/datatypes.h>
   #include <core/features/descriptors/sraid/sraid_match.h>
   #include <inout/system/print.h>
}
//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(sraid_match)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_sraid_match)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Ushort feat1[128] ;
   Rox_Ushort feat2[128] ;

   // ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   for (int k = 0; k < 128; k++)
   {
      feat1[k] = (Rox_Ushort) k; 
      feat2[k] = (Rox_Ushort) k; 
   }


   Rox_Uint distance = rox_sraid_match ( feat1, feat2 );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("distance = %d \n", distance);
}

ROX_TEST_SUITE_END()
