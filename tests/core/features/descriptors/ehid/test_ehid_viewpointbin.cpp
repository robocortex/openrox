//==============================================================================
//
//    OPENROX   : File test_ehid_viewpointbin.cpp
//
//       Contents  : Tests for ehid_viewpointbin.c
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
   #include <core/features/descriptors/ehid/ehid_viewpointbin.h>
   #include <baseproc/image/image.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(ehid_viewpointbin)

// #define IMAGE_PATH ROX_DATA_HOME"/database/test_database_1280x0720.pgm"
// #define IMAGE_PATH ROX_DATA_HOME"/database/test_database_1920x1080.pgm"
 #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/plane/image_plane3D000.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ehid_viewpointbin_new)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ehid_viewpointbin_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ehid_viewpointbin_process)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ehid_viewpointbin_test)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Image image = NULL;
   Rox_Ehid_ViewpointBin obj = NULL;
   
   Rox_Sint image_width = 0; 
   Rox_Sint image_height = 0; 
   
   Rox_Double minscale = 1.0; 
   Rox_Double maxscale = 2.0; 
   
   Rox_Double minaffine = 0.0; 
   Rox_Double maxaffine = 40.0;

   Rox_Double sigma = 4.0;

   Rox_Sint pcount = 0;
   Rox_Sint pcount_total = 0;

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);
   
   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_get_size(&image_height, &image_width, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ehid_viewpointbin_new(&obj, image_width, image_height, minscale, maxscale, minaffine, maxaffine, sigma);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ehid_viewpointbin_test(&pcount, &pcount_total, obj, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("pcount = %d \n", pcount);
   rox_log("pcount_total = %d \n", pcount_total);

   error = rox_ehid_viewpointbin_del(&obj);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
