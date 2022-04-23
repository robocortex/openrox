//==============================================================================
//
//    OPENROX   : File rox_example_identification_database_cloud.c
//
//    Contents  : A simple example program for image identification 
//                from a database on a server (i.e. the cloud) 
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
#include <string.h>

//=== MACROS    ================================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/identification.zip

// Model and database paths
#define rdi_file_01 "../seq/identification/database/models/toy_512x512.rdi"
#define rdi_file_02 "../seq/identification/database/models/gormiti_517x719.rdi"

// File paths
#define seq_file    "../seq/identification/database/images/toytest%03d.pgm"

// Define the number of model to be detected
#define nb_models 2
#define nb_images 44

// Define the camera intrinsic parameters
#define PX 559.279
#define PY 559.279
#define U0 329.890
#define V0 242.164

//=== VARIABLES ================================================================

// Server variables
static Rox_Ident_Database_SE3 server_ident = NULL;
static Rox_Database database = NULL;
static Rox_Database_Features server_features = NULL;
static Rox_Database_Item item = NULL;
static Rox_Char *server_buffer = NULL;
static Rox_Matrix camera_calibration = NULL;
static Rox_MatSE3 pose = NULL;

// Client variables
static Rox_Ident_Database_SE3 client_ident = NULL;
static Rox_Database_Features client_features = NULL;
static Rox_Char *client_buffer = NULL;
static Rox_Camera camera = NULL;

// Global variables
static Rox_Uint cols = 640;
static Rox_Uint rows = 480;

//=== FUNCTIONS ================================================================

Rox_Error rox_init_server_data();
Rox_Error rox_init_client_data();
Rox_Error rox_free_server_data();
Rox_Error rox_free_client_data();
Rox_Error rox_process_image(Rox_Uint image_index);

Rox_Error rox_init_server_data()
{
   Rox_Error error;
  
   // Allocate database objects
   error = rox_database_new(&database); 
   if(error) goto function_terminate;

   error = rox_database_item_new(&item); 
   if(error) goto function_terminate;

   error = rox_database_features_new(&server_features); 
   if(error) goto function_terminate;

   error = rox_ident_database_se3_new(&server_ident, nb_models); 
   if(error) goto function_terminate;

   error = rox_matse3_new(&pose); 
   if(error) goto function_terminate;

   error = rox_matrix_new(&camera_calibration, 3, 3); 
   if(error) goto function_terminate;

   error = rox_matrix_build_calibration_matrix(camera_calibration, PX, PY, U0, V0); 
   if(error) goto function_terminate;
   
   // Load item 1
   error = rox_database_item_load(item, rdi_file_01); 
   if(error) goto function_terminate;

   // Add item 1 to the database
   error = rox_database_add_item(database, item, 0.2, 0.2); 
   if(error) goto function_terminate;

   // Load item 2
   error = rox_database_item_load(item, rdi_file_02); 
   if(error) goto function_terminate;

   // Add item 2 to the database
   error = rox_database_add_item(database, item, 0.14, 0.2); 
   if(error) goto function_terminate;

   // Compile the global database
   error = rox_database_compile(database); if(error) goto function_terminate;
   if(error) goto function_terminate;

   // Set the detection database to the ident object
   error = rox_ident_database_se3_set_database(server_ident, database); 
   if(error) goto function_terminate;

function_terminate:
   return error;
}

Rox_Error rox_init_client_data()
{
   Rox_Error error;
   
   // Allocate objects
   error = rox_database_features_new(&client_features); 
   if(error) goto function_terminate;

   error = rox_ident_database_se3_new(&client_ident, 1); 
   if(error) goto function_terminate;

   error = rox_camera_new(&camera, cols, rows); 
   if(error) goto function_terminate;
   
   // Set the calibration matrix
   error = rox_camera_set_pinhole_params(camera, PX, PY, U0, V0); if(error) goto function_terminate;

function_terminate:
   return error;
}

Rox_Error rox_free_server_data()
{
   Rox_Error error;
  
   error = rox_database_del(&database); 
   if(error) goto function_terminate;

   error = rox_database_item_del(&item); 
   if(error) goto function_terminate;

   error = rox_database_features_del(&server_features); 
   if(error) goto function_terminate;

   error = rox_ident_database_se3_del(&server_ident); 
   if(error) goto function_terminate;

   error = rox_matrix_del(&camera_calibration); 
   if(error) goto function_terminate;

   error = rox_matse3_del(&pose); 
   if(error) goto function_terminate;

   if(server_buffer)
   {   
      free(server_buffer);
      server_buffer = 0;
   }
   
function_terminate:
   return error; 
}

Rox_Error rox_free_client_data()
{
   Rox_Error error;
  
   error = rox_database_features_del(&client_features); 
   if(error) goto function_terminate;

   error = rox_ident_database_se3_del(&client_ident); 
   if(error) goto function_terminate;

   error = rox_camera_del(&camera); 
   if(error) goto function_terminate;
   
   if(client_buffer)
   {   
      free(client_buffer);
      client_buffer = 0;
   }
    
function_terminate:
   return error;   
}

Rox_Error rox_process_image(Rox_Uint image_index)
{
   Rox_Error error;
   Rox_Char filename[1024];
   Rox_Uint size, i, nb_items;
   
   // Step 1: The client loads the current image
   sprintf(filename, seq_file, image_index);
   printf("Reading %s file\n", filename);
 
   error = rox_camera_read_pgm(camera, filename); 
   if(error) goto function_terminate;
   
   // Step 2: The client extracts the features
   error = rox_ident_database_se3_extract_features(client_features, client_ident, camera); 
   if(error) goto function_terminate;
   
   // Step 3: The client serializes the extracted features
   error = rox_database_features_get_structure_size(&size, client_features); 
   if(error) goto function_terminate;
   
   client_buffer = (Rox_Char*)malloc(size);
   if(!client_buffer) goto function_terminate;
   
   error = rox_database_features_serialize(client_buffer, client_features); 
   if(error) goto function_terminate;
   
   // Step 4: The client sends the buffer to the server (in this example, a simple memcpy)
   server_buffer = (Rox_Char*)malloc(size);
   if(!server_buffer) goto function_terminate;
   
   memcpy(server_buffer, client_buffer, size);
   
   // Step 5: The server deserializes the received buffer
   error = rox_database_features_deserialize(server_features, server_buffer); 
   if(error) goto function_terminate;
   
   // Step 6: The server makes the detection
   error = rox_ident_database_se3_make_features(server_ident, server_features, camera_calibration); 
   if(error) goto function_terminate;
   
   // Step 7: Get the detection results
   error = rox_ident_database_se3_getcountframes(&nb_items, server_ident); 
   if(error) goto function_terminate;
   
   for(i = 0; i < nb_items; i++)
   {
      Rox_Sint is_identified = 0;
      
      error = rox_ident_database_se3_getresult(&is_identified, pose, server_ident, i); 
      if(error) goto function_terminate;

      if(is_identified)
      {
         printf("The template %d is detected\n", i);
      }
      else
      {
         printf("The template %d is not detected\n", i);
      }
   }
   
   // Step 9: Free serialization buffers
   free(server_buffer); server_buffer = 0;
   free(client_buffer); client_buffer = 0;

function_terminate:
   return error;     
}

//=== MAIN PROGRAM ======================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   Rox_Error error;
   Rox_Uint major = 0, minor = 0, patch = 0;
   
   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Initialize the server
   error = rox_init_server_data(); 
   if(error) goto function_terminate;
   
   // Initialize the client
   error = rox_init_client_data(); 
   if(error) goto function_terminate;

   // Process all available images
   for(Rox_Uint i = 0; i < nb_images; i++)
   {
      error = rox_process_image(i); 
      if(error) goto function_terminate;
   }
   
   // Close the client
   error = rox_free_client_data();  
   if(error) goto function_terminate;
   
   // Close the server
   error = rox_free_server_data();  
   if(error) goto function_terminate;

function_terminate:
   
   // Display Error
   rox_error_print(error);

   printf("\nPress Enter to end the program\n");
   getchar();

   return error;
}
