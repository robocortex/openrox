//==============================================================================
//
//    OPENROX   : File test_ident_database_sl3.cpp
//
//    Contents  : Tests for ident_database_sl3.c
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
   #include <baseproc/maths/linalg/matsl3.h>
   #include <baseproc/image/draw/color.h>
   #include <baseproc/image/draw/draw_points.h>
   #include <baseproc/image/image_rgba.h>
   #include <user/identification/database/database_features.h>
   #include <user/identification/database/ident_database_sl3.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(ident_database_sl3)

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./results/"
#endif

// Image  512 x  512 =  262144 pixels
// #define IMAGE_PATH "regression_tests/openrox/plane/image_plane3D000.pgm"
// Image  517 x  719 =  371723 pixels
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_517x719.pgm"
//#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/images/toytest000.pgm"

// Image 1280 x  720 =  921600 pixels
// #define IMAGE_PATH "data/model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH "data/model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH "projects/apra/deformation/img005.ppm"

#define DATABASE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/database.rdb"

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
   Rox_Ident_Database_SL3 ident = NULL;

   error = rox_ident_database_sl3_new(&ident, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_sl3_del(&ident);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_ident_database_sl3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX];

   Rox_Ident_Database_SL3 ident = NULL;
   Rox_Database database = NULL;
   Rox_Image image = NULL;
   Rox_DynVec_Ehid_Point ehid_point_features = NULL;
   Rox_Image_RGBA image_display = NULL;

#ifndef WIN32
   struct stat st = {0};

   if (stat(RESULT_PATH, &st) == -1) {
       mkdir(RESULT_PATH, 0700);
   }
#endif

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

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_new_read_pgm(&image_display, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_sl3_new(&ident, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_sl3_set_database(ident, database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_sl3_make(ident, image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_MatSL3 homography = NULL;
   error = rox_matsl3_new(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Sint is_identified = -1;
   error = rox_ident_database_sl3_getresult(&is_identified, homography, ident, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double score = 0.0;
   error = rox_ident_database_sl3_getscore(&score, ident, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("is_identified %d with score %f \n", is_identified, score);

   #ifdef DISPLAY_RESULTS

   error = rox_ident_database_sl3_extract_features(ehid_point_features, ident, image);
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

   error = rox_image_rgba_draw_2d_points(image_display, points, nbpts, ROX_MAKERGBA(255,0,0));
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //error = rox_image_rgba_save_ppm ( RESULT_PATH"./test_database_item_extracted_points.ppm", image_display);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   free(points);

   #endif

   error = rox_ident_database_sl3_del(&ident);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_del(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_dynvec_ehid_point_del(&ehid_point_features);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del(&image_display);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
