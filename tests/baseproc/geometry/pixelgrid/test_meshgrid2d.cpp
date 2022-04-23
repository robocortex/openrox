//==============================================================================
//
//    OPENROX   : File test_meshgrid2d.cpp
//
//    Contents  : Tests for meshgrid2d.c
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
	#include <baseproc/geometry/pixelgrid/meshgrid2d.h>
   #include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>
   #include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN ( meshgrid2d )

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_meshgrid2d_float )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint rows = 3, cols = 3;

   Rox_MeshGrid2D_Float meshgrid2d = NULL; 
   error = rox_meshgrid2d_float_new ( &meshgrid2d, rows, cols );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // rox_array2d_float_save_txt ( "meshgrid2d_u.txt", meshgrid2d->u );
   // rox_array2d_float_save_txt ( "meshgrid2d_v.txt", meshgrid2d->v );

   rox_array2d_float_print ( meshgrid2d->u );
   rox_array2d_float_print ( meshgrid2d->v );
}

ROX_TEST_SUITE_END()
