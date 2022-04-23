//==============================================================================
//
//    OPENROX   : File test_database.cpp
//
//    Contents  : Tests for database.c
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
   #include <user/identification/database/database.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(database)

// Image  512 x  512 =  262144 pixels 
#define IMAGE_PATH ROX_DATA_HOME"/plane/image_plane3D000.pgm"
// Image 1280 x  720 =  921600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/data/model_based/switch/shield/sequence_1/ppm/switch_shield_seq1_img0001.ppm"
// Image 1920 x 1080 = 2073600 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/data/model_based/switch/lumia/sequence_1/ppm/switch_lumia_seq1_img0001.ppm"
// Image 4096 x 2160 = 8847360 pixels
// #define IMAGE_PATH ROX_DATA_HOME"/projects/apra/deformation/img005.ppm"

#define DATABASE_PATH ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/database.rdb"

#define nb_models 2

#define rdi_file_01 ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/toy_512x512.rdi"
#define rdi_file_02 ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_517x719.rdi"

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

// const type * pointer : is a pointer to a constant (i.e. we cannot do *pointer = NULL)

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_new_del)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database database = NULL;
  
   error = rox_database_new(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_del(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_load)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database database = NULL;
   Rox_Char filename[FILENAME_MAX];

   sprintf(filename, "%s", DATABASE_PATH);
   rox_log("read file %s\n", filename);

   error = rox_database_new(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
  
   error = rox_database_load(database, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_del(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_compile)
{
   Rox_ErrorCode error;
   Rox_Char filename[FILENAME_MAX] ;   

   Rox_Double  model_sizex[nb_models] = {0.2, 0.14};
   Rox_Double  model_sizey[nb_models] = {0.2, 0.2};
   
   Rox_Database database = NULL;
   Rox_Database_Item item = NULL;

   Rox_Char rdi_file[nb_models][1024] = {rdi_file_01, rdi_file_02};

   error = rox_database_new(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_database_item_new(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   for(int k = 0; k < nb_models; k++)
   {
      // Load db item
      sprintf(filename, "%s", rdi_file[k]);

      error = rox_database_item_load(item, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      // Add item to the database
      error = rox_database_add_item(database, item, model_sizex[k], model_sizey[k]);
      // error = rox_database_add_item_default_size(database, item);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;
   }

   // Compile database
   error = rox_database_compile(database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

function_return:

   error = rox_database_item_del(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_del ( &database );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE );

}

#ifdef schneider

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_database_compile_schneider)
{
   Rox_ErrorCode error;
   Rox_Char filename[FILENAME_MAX] ;   

   int count = 0;

   Rox_Double  model_sizex = 0.64;
   Rox_Double  model_sizey = 0.48;
   
   Rox_Database database = NULL;
   Rox_Database_Item item = NULL;

   Rox_Char rdi_file[nb_models][1024] = {rdi_file_01, rdi_file_02};

   error = rox_database_new(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_database_item_new(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;


    DIR *d;

    struct dirent *dir;

    d = opendir("/mnt/shared/Customers/Scnheider/items");

    if (d)

    {

        while ((dir = readdir(d)) != NULL)

        {
            sprintf(filename, "%s/%s", "/mnt/shared/Customers/Scnheider/items/", dir->d_name);

            rox_log("%s\n", filename);

            error = rox_database_item_load(item, filename);

            if(error == 0)
            {
               count++;

               // Add item to the database
               error = rox_database_add_item(database, item, model_sizex, model_sizey);
               // error = rox_database_add_item_default_size(database, item);
               ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

               if (count > 100)
               {
                  break;
               }
            }
        }

        closedir(d);

    }

   // Compile database
   error = rox_database_compile(database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

function_return:

   error = rox_database_item_del(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_del ( &database );
   ROX_TEST_CHECK_EQUAL(error, ROX_ERROR_NONE );

}
#endif
ROX_TEST_SUITE_END()
