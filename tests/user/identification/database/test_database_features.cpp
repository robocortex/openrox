//==============================================================================
//
//    OPENROX   : File test_database_features.cpp
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
   #include <system/time/timer.h>
   #include <baseproc/geometry/point/point2d_struct.h>

   #include <user/identification/database/database_features.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(database_features)

// Image  512 x  512 =  262144 pixels 
#define IMAGE_PATH "plane/image_plane3D000.pgm"
// Image 1280 x  720 =  921600 pixels
// #define IMAGE_PATH "data/model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH "data/model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH "projects/apra/deformation/img005.ppm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

// const type * pointer : is a pointer to a constant (i.e. we cannot do *pointer = NULL)

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_features)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Features features = NULL;

   error = rox_database_features_new(&features);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_features_del(&features);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_features_get_points)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Features database_features = NULL;
   Rox_Point2D_Double_Struct * points = NULL;

   error = rox_database_features_new(&database_features);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Uint nbpts = 0;
   error = rox_database_features_get_points_size(NULL, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   
   error = rox_database_features_get_points_size(NULL, database_features);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_database_features_get_points_size(&nbpts, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);
   
   error = rox_database_features_get_points_size(&nbpts, database_features);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_features_get_points_copy(NULL, nbpts, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_database_features_get_points_copy(NULL, nbpts, database_features);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   error = rox_database_features_get_points_copy(points, nbpts, NULL);
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NULL_POINTER);

   points = (Rox_Point2D_Double_Struct *) malloc(sizeof(*points) * 10);

   error = rox_database_features_get_points_copy(points, nbpts, database_features);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_features_del(&database_features);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   free(points);
}

ROX_TEST_SUITE_END()
