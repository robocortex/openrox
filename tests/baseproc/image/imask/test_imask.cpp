//==============================================================================
//
//    OPENROX   : File test_imask.cpp
//
//    Contents  : Tests for image.c
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
#ifndef WIN32
   // Include needed to create directory
   #include <sys/types.h>
   #include <sys/stat.h>
   #include <unistd.h>
#endif

   #include <system/time/timer.h>

   #include <baseproc/maths/maths_macros.h>

   #include <baseproc/image/imask/imask.h>
   #include <baseproc/image/imask/fill/set_polygon.h>

   #include <baseproc/geometry/ellipse/ellipse2d.h>
   #include <baseproc/geometry/ellipse/ellipse2d_struct.h>

   #include <inout/numeric/array2d_save.h>
   #include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(imask)

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./results/"
#endif

// Image  512 x  512 =  262144 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/plane/image_plane3D000.pgm"
// Image 1280 x  720 =  921600 pixels
//#define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
//#define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/pgm/switch_shield_seq1_img0001.pgm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/projects/apra/deformation/img005.ppm"
// Image  517 x  719 =  371723 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_517x719.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

// const type * pointer : is a pointer to a constant (i.e. we cannot do *pointer = NULL)

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_imask_centered_ellipse )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Imask imask = NULL;
   Rox_Sint cols = 640, rows = 480;

#ifndef WIN32
   struct stat st = {0};

   if (stat(RESULT_PATH, &st) == -1) {
       mkdir(RESULT_PATH, 0700);
   }
#endif

   error = rox_imask_new ( &imask, cols, rows );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_set_centered_ellipse ( imask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_save_pgm ( RESULT_PATH"test_imask_set_centered_ellipse.pgm", imask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &imask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_imask_new_ellipse )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Imask imask = NULL;

   Rox_Ellipse2D_Double_Struct ellipse2d;

   Rox_Double xc =  50;
   Rox_Double yc = 100;
   Rox_Double a = 100;
   Rox_Double b = 200;
   Rox_Double phi = ROX_PI/6;

#ifndef WIN32
   struct stat st = {0};

   if (stat(RESULT_PATH, &st) == -1) {
       mkdir(RESULT_PATH, 0700);
   }
#endif

   error = rox_ellipse2d_convert_parametric_to_canonical ( &ellipse2d, xc, yc, a, b, phi );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_new_ellipse ( &imask, &ellipse2d );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_save_pgm ( RESULT_PATH"test_imask_new_ellipse.pgm", imask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &imask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_imask_new_polygon )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Imask imask = NULL;
   Rox_Point2D_Double_Struct pts[8];
   Rox_Sint nbpts = 8;

#ifndef WIN32
   struct stat st = {0};

   if (stat(RESULT_PATH, &st) == -1) {
       mkdir(RESULT_PATH, 0700);
   }
#endif

   pts[0].u = 100; pts[0].v = 100;
   pts[1].u = 100; pts[1].v = 200;
   pts[2].u = 200; pts[2].v = 150;
   pts[3].u = 220; pts[3].v = 130;
   pts[4].u = 230; pts[4].v = 90;
   pts[5].u = 180; pts[5].v = 120;
   pts[6].u = 160; pts[6].v = 90;
   pts[7].u = 120; pts[7].v = 140;

   error = rox_imask_new_polygon ( &imask, pts, nbpts );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_save_pgm ( RESULT_PATH"test_imask_new_polygon.pgm", imask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_imask_del ( &imask );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

//Rox_ErrorCode rox_array2d_uint_set_polygon ( Rox_Imask mask, const Rox_Point2D_Double pts, Rox_Sint const nbpts )

ROX_TEST_SUITE_END()
