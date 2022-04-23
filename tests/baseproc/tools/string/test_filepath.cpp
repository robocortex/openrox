//==============================================================================
//
//    OPENROX   : File test_filepath.cpp
//
//    Contents  : Tests for filepath.c
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
   #include <string.h>
   #include <baseproc/tools/string/filepath.h>
   #include <system/errors/errors.h>
   #include <inout/system/print.h>   
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(filepath)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_filepath)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Char dir[FILENAME_MAX];  
   Rox_Char name[FILENAME_MAX];  
   Rox_Char ext[FILENAME_MAX]; 
   Rox_Char path[FILENAME_MAX] = "C:\\Users\\emalis\\test\\file.png"; 

   error = rox_filepath_split(dir, name, ext, path);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = strcmp(dir,"C:\\Users\\emalis\\test");
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = strcmp(name,"file");
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = strcmp(ext,".png");
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("path : %s \n", path);
   rox_log("dir  : %s \n", dir);
   rox_log("name : %s \n", name);
   rox_log("ext  : %s \n", ext);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_filepath_double_dot)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Char dir[FILENAME_MAX];  
   Rox_Char name[FILENAME_MAX];  
   Rox_Char ext[FILENAME_MAX]; 
   Rox_Char path[FILENAME_MAX] = "C:\\Users\\emalis\\test\\file.suffix.png"; 

   error = rox_filepath_split(dir, name, ext, path);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = strcmp(dir,"C:\\Users\\emalis\\test");
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = strcmp(name,"file.suffix");
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = strcmp(ext,".png");
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("path : %s \n", path);
   rox_log("dir  : %s \n", dir);
   rox_log("name : %s \n", name);
   rox_log("ext  : %s \n", ext);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_filepath_triple_dot)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Char dir[FILENAME_MAX];  
   Rox_Char name[FILENAME_MAX];  
   Rox_Char ext[FILENAME_MAX]; 
   Rox_Char path[FILENAME_MAX] = "C:\\Users\\emalis\\test\\file.suffix1.suffix2.png"; 

   error = rox_filepath_split(dir, name, ext, path);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = strcmp(dir,"C:\\Users\\emalis\\test");
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = strcmp(name,"file.suffix1.suffix2");
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = strcmp(ext,".png");
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("path : %s \n", path);
   rox_log("dir  : %s \n", dir);
   rox_log("name : %s \n", name);
   rox_log("ext  : %s \n", ext);
}

ROX_TEST_SUITE_END()
