//==============================================================================
//
//    OPENROX   : File test_ident_database_se3.cpp
//
//    Contents  : Tests for ident_database_se3.c
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
   #include <generated/dynvec_ehid_point.h>
   #include <system/time/timer.h>

   #include <baseproc/image/draw/color.h>
   #include <baseproc/image/draw/draw_points.h>

   #include <baseproc/image/image_rgba.h>

   #include <inout/image/ppm/ppmfile.h>
   #include <inout/system/print.h>

   #include <user/identification/database/database_features.h>
   #include <user/identification/database/ident_database_se3.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(ident_database_sl3)

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./"
#endif

// Image  512 x  512 =  262144 pixels 
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/images/toytest000.pgm"

// Image 1280 x  720 =  921600 pixels
// Image 1920 x 1080 = 2073600 pixels
// Image 4096 x 2160 = 8847360 pixels

#define DATABASE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/database.rdb"

// Define the camera intrinsic parameters
#define PX 559.279
#define PY 559.279
#define U0 329.890
#define V0 242.164

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

// const type * pointer : is a pointer to a constant (i.e. we cannot do *pointer = NULL)

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ident_database_sl3_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_Database_SE3 ident = NULL;
   
   error = rox_ident_database_se3_new(&ident, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_se3_del(&ident);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ident_database_se3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Ident_Database_SE3 ident = NULL;
   Rox_Database database = NULL;
   Rox_Camera camera = NULL;
   Rox_DynVec_Ehid_Point ehid_point_features = NULL;
   Rox_Image_RGBA image_display = NULL;
   
   error = rox_dynvec_ehid_point_new(&ehid_point_features, 100);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", DATABASE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_database_new(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
  
   error = rox_database_load(database, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_camera_new_read_pgm(&camera, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_set_pinhole_params(camera, PX, PY, U0, V0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_new_read_pgm(&image_display, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_se3_new(&ident, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_se3_set_database(ident, database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_se3_make(ident, camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_se3_extract_features(ehid_point_features, ident, camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   // Test get of points
   Rox_Uint nbpts = 0;

   error = rox_database_features_get_points_size(&nbpts, ehid_point_features);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Point2D_Double_Struct * points = NULL;
   points = (Rox_Point2D_Double_Struct *) malloc(sizeof(*points) * nbpts);

   error = rox_database_features_get_points_copy(points, nbpts, ehid_point_features);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Number of extracted points = %d\n", nbpts);

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      rox_log("point[%d] = (%f, %f) \n", i, points[i].u, points[i].v);
   }

   error = rox_image_rgba_draw_2d_points ( image_display, points, nbpts, ROX_MAKERGBA(255,0,0,255) );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_image_rgba_save_ppm ( RESULT_PATH"./test_database_item_extracted_points.ppm", image_display );
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   free(points);

   error = rox_ident_database_se3_del(&ident);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_del(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_camera_del(&camera);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_ehid_point_del(&ehid_point_features);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del(&image_display);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
