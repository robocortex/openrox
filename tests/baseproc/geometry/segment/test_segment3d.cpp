//==============================================================================
//
//    OPENROX   : File test_segment3d.cpp
//
//  	Contents  : Tests for segment3d.c
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
	#include <baseproc/geometry/segment/segment3d.h>
   #include <baseproc/geometry/segment/segment3d_struct.h>
   #include <system/errors/errors.h>
   #include <inout/geometry/segment/segment3d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(segment3d)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_segment3d)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
	Rox_Segment3D_Struct segment3d;
   Rox_Double X1 = 0.0; 
   Rox_Double Y1 = 1.0;  
   Rox_Double Z1 = 2.0;  
   Rox_Double X2 = 3.0; 
   Rox_Double Y2 = 4.0; 
   Rox_Double Z2 = 5.0;

   error = rox_segment3d_set ( &segment3d, X1, Y1, Z1, X2, Y2, Z2 );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_segment3d_print ( &segment3d );

}

ROX_TEST_SUITE_END()
