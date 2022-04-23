//==============================================================================
//
//    OPENROX   : File rox_example_odometry_database_compiled.c
//
//    Contents  : A simple example program for odometry with database compiled.
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   ======================================================

#include <api/openrox.h>
#include <user/identification/database/database_features.h>
#include <stdio.h>
#include <sys/stat.h>

//=== INTERNAL MACROS    ======================================================

#define compiled_rdb "/Users/fservant/Downloads/database2.rdb"

// Define the camera intrinsic parameters 
#define FU 1000.0
#define FV 1000.0
#define CU  320.0
#define CV  240.0

//=== MAIN PROGRAM ============================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   // ---------- ALLOCATE GLOBAL DATA -----------------------------------------
   Rox_ErrorCode error;
   // Rox_Char filename[1024] ;

   Rox_Uint k=0;
   // Rox_Double score;
   Rox_Matrix K;

   // Define timer to measure performances 
   Rox_Timer timer = NULL;
   Rox_Database_Features feats;

   // Define pose matrix 
   Rox_MatSE3 pose = NULL;

   // Define rox database identification object 
   Rox_Ident_Database_SE3 ident = NULL;
   Rox_Database database = NULL;

   error = rox_database_features_new(&feats);
   if (error) return error;
   
   struct stat info;
   char * lfname = "/Users/fservant/Downloads/features.bin";
   stat(lfname, &info);
   
   char *content = (char *)malloc(info.st_size * sizeof(char));
   FILE *fp = fopen(lfname, "rb");
   if (!fp) return -1;
   int read = fread(content, info.st_size, 1, fp);
   ROX_UNUSED(read);
   fclose(fp);
   
   error = rox_database_features_deserialize(feats, content);
   if (error) return error;

   // Define database objects 
   error = rox_ident_database_se3_new(&ident, 200);
   if(error) goto function_terminate;

   error = rox_database_new(&database);
   if(error) goto function_terminate;

   error = rox_matse3_new(&pose);
   if(error) goto function_terminate;

   // Load the compiled database
   error = rox_database_load(database, compiled_rdb);
   if(error) goto function_terminate;

   error = rox_ident_database_se3_set_database(ident, database);
   if(error) goto function_terminate;

   error = rox_matrix_new(&K, 3, 3);
   if(error) goto function_terminate;
   
   rox_matrix_set_unit(K);
   rox_matrix_set_value(K, 0, 0, FU);
   rox_matrix_set_value(K, 1, 1, FV);
   rox_matrix_set_value(K, 0, 2, CU);
   rox_matrix_set_value(K, 1, 2, CV);

   // Make the detection using the current image 
   error = rox_ident_database_se3_make_features(ident, feats, K);
   if(error) goto function_terminate;
      
   Rox_Uint count;
   rox_ident_database_se3_getcountframes(&count, ident);
   printf("%d\n", count);

   for(k = 0; k < 200; k++)
   {
      Rox_Sint is_identified = 0;
      error = rox_ident_database_se3_getresult(&is_identified, pose, ident, k); 
      printf("%d\n", is_identified);
      if(error) goto function_terminate;
   }

function_terminate:

   // Delete objects and free Memory 
   rox_timer_del(&timer);
   rox_matse3_del(&pose);
   rox_matrix_del(&K);
   rox_ident_database_se3_del(&ident);
   rox_database_del(&database);

   return error;
}
