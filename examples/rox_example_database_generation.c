//==============================================================================
//
//    OPENROX   : File rox_example_database_generation.c
//
//    Contents  : A simple example program for generating a database.
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== HEADERS   ================================================================

#include <api/openrox.h>
#include <stdio.h>
#include <stdlib.h>

//=== MACROS    ================================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/identification.zip

// Model and database paths
#define mod_file_1 "../seq/identification/database/models/toy_512x512.pgm"
#define rdi_file_1 "../res/toy_512x512.rdi"

#define mod_file_2 "../seq/identification/database/models/gormiti_517x719.pgm"
#define rdi_file_2 "../res/gormiti_517x719.rdi"

#define compiled_db "../res/database.rdb"
#define compiled_db_copy "../res/database_copy.rdb"

//=== MAIN PROGRAM =============================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   Rox_Error error;
   
   Rox_Uint major = 0, minor = 0, patch = 0;

   // Global data
   Rox_Image image1 = NULL, image2 = NULL;
   Rox_Database_Item database_item = NULL;
   Rox_Database database = NULL;
   Rox_Uint database_size = 0;
   Rox_Char *buffer = 0;
   
   Rox_Database database_copy = NULL;

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Allocate memory
   error = rox_database_item_new(&database_item);
   if(error) goto function_terminate;

   error = rox_database_new(&database);
   if(error) goto function_terminate;
   
   error = rox_database_new(&database_copy);
   if(error) goto function_terminate;
   
   // Generate and save the database item 1
   // Load the model to learn
   error = rox_image_new_read_pgm(&image1, mod_file_1);
   if(error) goto function_terminate;

   // Learn the template
   printf("Generating the dabatase item 1, this can take few minutes... \n");
   error = rox_database_item_learn_template(database_item, image1);
   if(error) goto function_terminate;

   printf("Database item 1 is successfully generated \n");

   // Save the generated database item in file
   error = rox_database_item_save(rdi_file_1, database_item);
   if(error) goto function_terminate;

   printf("Database item 1 is successfully saved in %s file \n", rdi_file_1);

   // Add item to the database
   error = rox_database_add_item(database, database_item, 0.2, 0.2);
   if(error) goto function_terminate;

   // Load the model to learn
   error = rox_image_new_read_pgm(&image2, mod_file_2);
   if(error) goto function_terminate;

   // Set the learn parameters for the second template
   Rox_Double scales[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
   Rox_Double angle_max = 40;
   Rox_Double sigma = 4.0;
   
   error = rox_database_item_set_params(database_item, scales, angle_max, sigma);
   if(error) goto function_terminate;

   // Learn the template
   printf("Generating the dabatase item 2, this can take few minutes... \n");
   error = rox_database_item_learn_template(database_item, image2);
   if(error) goto function_terminate;

   printf("Database item 2 is successfully generated \n");

   // Save the generated database item in file
   error = rox_database_item_save(rdi_file_2, database_item);
   if(error) goto function_terminate;

   printf("Database item 2 is successfully saved in %s file \n", rdi_file_2);

   // Add item to the database
   error = rox_database_add_item(database, database_item, 0.14, 0.2);
   if(error) goto function_terminate;

   // Compile the database
   error = rox_database_compile(database);
   if(error) goto function_terminate;

   // Save the compiled database
   error = rox_database_save(compiled_db, database);
   if(error) goto function_terminate;
   
   // Serialize the compiled databse
   // Step 1: Get in octects the database size
   error = rox_database_get_structure_size(&database_size, database);
   if(error) goto function_terminate;
   
   // Step 2: Allocate the buffer
   buffer = (Rox_Char*)malloc(database_size);
   if(!buffer) goto function_terminate;
   
   // Step 3: Fill the buffer
   error = rox_database_serialize(buffer, database);
   if(error) goto function_terminate;
   
   // Step 4: Do what you want with the buffer
   error = rox_database_deserialize(database_copy, buffer);
   if(error) goto function_terminate;

   // Save the compiled database
   error = rox_database_save(compiled_db_copy, database_copy);
   if(error) goto function_terminate;
   
function_terminate:

   // Close Files
   if(buffer) free(buffer);

   // Delete objects and free memory
   rox_database_item_del(&database_item);
   rox_database_del(&database);
   rox_image_del(&image1);
   rox_image_del(&image2);
   
   // Display Error
   rox_error_print(error);

   printf("\nPress Enter to end the program\n");
   getchar();

   return error;
}

