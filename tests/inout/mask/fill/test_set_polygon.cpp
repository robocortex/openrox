//==============================================================================
//
//    OPENROX   : File test_set_polygon.cpp
//
//    Contents  : Tests for set_polygon.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//====== INCLUDED HEADERS   ====================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <system/time/timer.h>
   #include <baseproc/geometry/point/point2d_struct.h>
   #include <baseproc/image/imask/fill/set_polygon.h>
   #include <inout/mask/pgm/mask_pgmfile.h> 
}

// ====== INTERNAL MACROS    ===================================================

ROX_TEST_SUITE_BEGIN(set_polygon)

// ====== INTERNAL TYPESDEFS ===================================================

// ====== INTERNAL DATATYPES ===================================================

// ====== INTERNAL VARIABLES ===================================================

// ====== INTERNAL FUNCTDEFS ===================================================

// ====== INTERNAL FUNCTIONS ===================================================

// ====== EXPORTED FUNCTIONS ===================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_nreg_set_polygon)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct pts[3];
   Rox_Uint nbpts = 3;

   Rox_Array2D_Uint mask = NULL;

   error = rox_array2d_uint_new(&mask, 128, 128);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   pts[0].u =  10; pts[0].v =  20; 
   pts[1].u = 100; pts[1].v =  40; 
   pts[2].u =  50; pts[2].v =  80; 

   error = rox_array2d_uint_set_polygon(mask, pts, nbpts);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_array2d_uint_mask_save_pgm("test_set_polygon.pgm", mask);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_uint_del(&mask);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
