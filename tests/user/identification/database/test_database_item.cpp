//==============================================================================
//
//    OPENROX   : File test_database_item.cpp
//
//    Contents  : Tests for database_item.c
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
   
   #include <baseproc/geometry/point/point2d_struct.h>

   #include <baseproc/image/draw/color.h>
   #include <baseproc/image/draw/draw_points.h>

   #include <user/identification/database/database_item.h>
   #include <baseproc/image/image.h>
   #include <baseproc/image/image_rgba.h>
   #include <inout/system/print.h>

}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(database_item)

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./results/"
#endif

// Image  517 x  719
#define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_517x719.pgm"
// #define IMAGE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/toy_128x128.pgm"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

// const type * pointer : is a pointer to a constant (i.e. we cannot do *pointer = NULL)

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_database_item_new_del )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Item item = NULL;

   error = rox_database_item_new(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_del(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_database_item_learn_template )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Item database_item = NULL;
   Rox_Image image_template = NULL;
   Rox_Char filename[FILENAME_MAX] ;
   Rox_Image_RGBA image_display = NULL;
   
#ifndef WIN32
   struct stat st = {0};

   if (stat(RESULT_PATH, &st) == -1) {
       mkdir(RESULT_PATH, 0700);
   }
#endif

   error = rox_database_item_new(&database_item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("Read image %s\n", filename);

   error = rox_image_new_read_pgm ( &image_template, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_new_read_pgm ( &image_display, filename );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_learn_template ( database_item, image_template );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   /*
   // Test get of points
   Rox_Uint nbpts = 0;

   error = rox_database_item_get_points_size(&nbpts, database_item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Point2D_Double points = NULL;
   points = (Rox_Point2D_Double) malloc(sizeof(*points) * nbpts);

   error = rox_database_item_get_points_copy(points, nbpts, database_item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Number of extracted points = %d\n", nbpts);

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      rox_log("point[%d] = (%f, %f) \n", i, points[i].u, points[i].v);
   }

   error = rox_image_rgba_draw_2d_points(image_display, points, nbpts, ROX_MAKERGBA(255,0,0,255));
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_save_ppm(RESULT_PATH"./test_database_item_extracted_points.ppm", image_display);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   free(points);
   */

   error = rox_database_item_del ( &database_item );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_rgba_del(&image_display);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_item_save)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Item item = NULL;
   Rox_Image image_template = NULL;
   Rox_Char filename[FILENAME_MAX] ;

   error = rox_database_item_new(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   sprintf(filename, "%s", IMAGE_PATH);

   error = rox_image_new_read_pgm(&image_template, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_learn_template(item, image_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //error = rox_database_item_save(RESULT_PATH"./test_save_database_item.rdi", item);
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_database_item_del(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_image_del(&image_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_item_load)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Item item = NULL;

   error = rox_database_item_new(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //error = rox_database_item_load(item, "./test_save_database_item.rdi");
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_database_item_del(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_item_serialize)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Item item = NULL;

   error = rox_database_item_new(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //error = rox_database_item_load(item, "./test_save_database_item.rdi");
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // 
   Rox_Uint size = 0;
   error =  rox_database_item_get_structure_size(&size, item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Char * buffer = NULL;
   buffer = (Rox_Char *) rox_memory_allocate(sizeof(Rox_Char), size);
   
   error = rox_database_item_serialize(buffer, item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_del(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   free(buffer);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_item_deserialize)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Item item = NULL;

   error = rox_database_item_new(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //error = rox_database_item_load(item, "./test_save_database_item.rdi");
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   // 
   Rox_Uint size = 0;
   error =  rox_database_item_get_structure_size(&size, item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Char * buffer = NULL;
   buffer = (Rox_Char *) rox_memory_allocate(sizeof(Rox_Char), size);
   
   error = rox_database_item_serialize(buffer, item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Database_Item item_deserialize = NULL;

   error = rox_database_item_new(&item_deserialize);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_database_item_deserialize(item_deserialize, buffer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_del(&item_deserialize);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_database_item_del(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   free(buffer);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_item_get_structure_size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Item item = NULL;

   error = rox_database_item_new(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   //error = rox_database_item_load(item, "./test_save_database_item.rdi");
   //ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   Rox_Uint size = 0;
   error =  rox_database_item_get_structure_size(&size, item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_del(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_item_set_params)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char filename[FILENAME_MAX] ;
   Rox_Double scales[9] = {1.00, 1.33, 1.66, 2.0, 2.33, 2.66, 3.00, 3.5, 4.0};
   Rox_Double angle_max = 30;
   Rox_Double sigma = 2.0;

   Rox_Database_Item database_item = NULL;
   Rox_Image image_template = NULL;

   error = rox_database_item_new(&database_item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_set_params(database_item, scales, angle_max, sigma);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   sprintf(filename, "%s", IMAGE_PATH);
   rox_log("Read image %s\n", filename);

   error = rox_image_new_read_pgm(&image_template, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_learn_template(database_item, image_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_del(&database_item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image_template);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
