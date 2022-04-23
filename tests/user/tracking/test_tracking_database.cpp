//==============================================================================
//
//    OPENROX   : File test_tracking_database.cpp
//
//    Contents  : Tests for tracking
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
	#include <api/openrox.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(tracking_database)

// Model and database paths
#define mod_file_01 ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/toy_128x128.pgm"
#define rdi_file_01 ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/toy_512x512.rdi"

#define mod_file_02 ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_128x178.pgm"
#define rdi_file_02 ROX_DATA_HOME"/regression_tests/openrox/identification/database/models/gormiti_517x719.rdi"

// File paths
#define seq_file    ROX_DATA_HOME"/regression_tests/openrox/identification/database/images/toytest%03d.pgm"

// Define the number of model to be detected
#define nb_models 2
#define nb_images 45

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_tracking_database)
{
   // ---------- ALLOCATE GLOBAL DATA -------------------------------------
   Rox_ErrorCode error;
   Rox_Char filename[FILENAME_MAX] ;

   Rox_Uint i = 0, k= 0;
   Rox_Double score;

   Rox_Uint n_model_tracked[nb_models] = {0, 0};

   Rox_Char mod_file[nb_models][1024] = {mod_file_01, mod_file_02};
   Rox_Char rdi_file[nb_models][1024] = {rdi_file_01, rdi_file_02};

   // Define timer to measure performances
   Rox_Timer timer = 0;

   // ----------- ALLOCATE VISUAL DATA ------------------------------------
   // Define the image for processing data
   Rox_Image image = NULL;

   // Define homography matrix
   Rox_MatSL3 homography = NULL;

   // ----------- ALLOCATE MODEL DATA -------------------------------------
   Rox_Double  model_sizeu[nb_models] = {128.0, 128.0};
   Rox_Double  model_sizev[nb_models] = {128.0, 178.0};

   Rox_Image model_image[nb_models];

   // Define tracking object
   Rox_Tracking         tracking[nb_models];
   Rox_Tracking_Params  params[nb_models];

   // Define rox database identification object
   Rox_Ident_Database_SL3 ident = NULL;
   Rox_Database_Item item = NULL;
   Rox_Database database = NULL;

   //init in case of fail, in order for cleanup to be done without seg faults...
   for (k = 0; k < nb_models; k++)
   {
      params[k] = NULL;
      model_image[k] = NULL;
      tracking[k] = NULL;
   }

   // Define database objects
   error = rox_ident_database_sl3_new(&ident, nb_models);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_database_new(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_database_item_new(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_matsl3_new(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   // Define timer object
   error = rox_timer_new(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   rox_log("Initialize models \n");
   // Initialize the tracking and ident
   for(k = 0; k < nb_models; k++)
   {
      // Load model
      sprintf(filename, "%s", mod_file[k]);

      error = rox_image_new_read_pgm(&model_image[k], filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      // Load db item
      sprintf(filename, "%s", rdi_file[k]);
      error = rox_database_item_load(item, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      // Add item to the database
      error = rox_database_add_item_default_size(database, item);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      error = rox_tracking_params_new(&params[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      // Create tracking
      error = rox_tracking_new(&tracking[k], params[k], model_image[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;
   }

   // Compile database
   error = rox_database_compile(database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   error = rox_ident_database_sl3_set_database(ident, database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;
      
   sprintf(filename, "%s", seq_file);
   sprintf(filename, filename, 0);

   error = rox_image_new_read_pgm(&image, filename);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

   rox_log("Start tracking \n");
   for (i = 0; i < nb_images; i++)
   {
      sprintf(filename, "%s", seq_file);
      sprintf(filename, filename, i);

      error = rox_image_read_pgm(image, filename);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      // Make the detection using the current image
      error = rox_ident_database_sl3_make(ident, image);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

      rox_log("Identification SL3 on image %d \n", i);

      for(k = 0; k < nb_models; k++)
      {
         Rox_Sint is_identified = 0;

         error = rox_ident_database_sl3_getresult(&is_identified, homography, ident, k);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;
         
         error = rox_ident_database_sl3_get_result_force_sizeu(&is_identified, homography, ident, model_sizeu[k], k);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

         error = rox_ident_database_sl3_get_result_force_sizev(&is_identified, homography, ident, model_sizev[k], k);
         ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

         if(is_identified)
         {
            rox_log("Identified model %d \n", k);
            // error = rox_matsl3_print(homography);
            // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

            error = rox_tracking_set_homography(tracking[k], homography);
            ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

            error = rox_tracking_make(tracking[k], image);
            if (!error)
            {
               rox_log("Model %d tracked\n", k);

               error = rox_tracking_get_score(&score, tracking[k]);
               ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

               ROX_TEST_CHECK_SUPERIOR_OR_EQUAL(score, 0.89);

               error = rox_tracking_get_homography(homography, tracking[k]);
               ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE ); if(error) goto function_return;

               rox_log("Tracking score = %f \n", score);

               // error = rox_matsl3_print(homography);
               // ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

               n_model_tracked[k]++;
            }
            else
            {
               rox_log("Model %d not tracked\n", k);
            }
         }
      }
   }

function_return:

   // Delete objects and free memory
   error = rox_timer_del(&timer);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_image_del(&image);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matsl3_del(&homography);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_ident_database_sl3_del(&ident);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_item_del(&item);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_database_del(&database);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for (k = 0; k < nb_models; k++)
   {
      error = rox_image_del(&model_image[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

      error = rox_tracking_params_del(&params[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
       
      error = rox_tracking_del(&tracking[k]);
      ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   }
}

ROX_TEST_SUITE_END()
