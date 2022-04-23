//==============================================================================
//
//    OPENROX   : File test_file.cpp
//
//    Contents  : Tests for file.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//====== INCLUDED HEADERS   ====================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <inout/system/file.h> 
   #include <inout/system/print.h>
}

// ====== INTERNAL MACROS    ===================================================

ROX_TEST_SUITE_BEGIN(file)

#define CAD_MODEL_PATH           ROX_DATA_HOME"/regression_tests/rox_opencl/mbo/cad_model.txt"

// ====== INTERNAL TYPESDEFS ===================================================

// ====== INTERNAL DATATYPES ===================================================

// ====== INTERNAL VARIABLES ===================================================

// ====== INTERNAL FUNCTDEFS ===================================================

// ====== INTERNAL FUNCTIONS ===================================================

// ====== EXPORTED FUNCTIONS ===================================================


ROX_TEST_CASE_DECLARE ( rox::OpenROXTest, test_nreg_file )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Size lines_number = 0;

   error = rox_file_count_lines ( &lines_number, CAD_MODEL_PATH );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("lines_number = %zu\n", lines_number);
}

ROX_TEST_SUITE_END()
