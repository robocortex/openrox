//==============================================================================
//
//    OPENROX   : File test_draw_ellipse.cpp
//
//    Contents  : Tests for draw_ellipse.c
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
   #include <baseproc/image/draw/color.h>
   #include <baseproc/image/draw/draw_circle.h>
   #include <baseproc/image/draw/draw_ellipse.h>
   #include <inout/image/ppm/ppmfile.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(draw_ellipse)

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./results/"
#endif

// Image 640  x  480 =  307200 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/motion_detection/img_motion_detection_00.ppm"
// Image 1280 x  720 =  921600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/projects/apra/deformation/img005.ppm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest,test_draw_ellipse2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Image_RGBA image_rgba = NULL;

   Rox_Ellipse2D ellipse2d = NULL;
   
   Rox_Uint color = ROX_MAKERGBA(255, 0, 0, 255);
  
#ifndef WIN32
   struct stat st = {0};

   if (stat(RESULT_PATH, &st) == -1) {
       mkdir(RESULT_PATH, 0700);
   }
#endif

   error = rox_ellipse2d_new(&ellipse2d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 

   error = rox_ellipse2d_set(ellipse2d, 320.0, 240.0, 1.0/(120.0*120.0), 1.0/(80.0*80.0), 1.0/(150.0*150.0));
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
   
   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);
   
   // Create and read the image
   error = rox_image_rgba_new_read_ppm(&image_rgba, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
   
   error = rox_image_rgba_draw_ellipse2d(image_rgba, ellipse2d, color);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // error = rox_image_rgba_draw_circle(image_rgba, 320.0, 240.0, 100, color);
   // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //sprintf(filename, RESULT_PATH"./test_draw_ellipse.ppm");
   //error = rox_image_rgba_save_ppm(filename, image_rgba);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_image_rgba_del(&image_rgba);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ellipse2d_del(&ellipse2d);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); 
}

ROX_TEST_SUITE_END()