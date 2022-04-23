//==============================================================================
//
//    OPENROX   : File test_real_eigenvalues_eigenvectors.cpp
//
//    Contents  : Tests for real_eigenvalues_eigenvectors.c
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
	#include <baseproc/array/eigenv/real_eigenvalues_eigenvectors.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/numeric/objset_array2d_print.h>
   #include <inout/numeric/dynvec_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(real_eigenvalues_eigenvectors)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_real_eigenvalues_eigenvectors)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double M = NULL;
   Rox_ObjSet_Array2D_Double V = NULL; // size V will be m x n
   Rox_DynVec_Double e = NULL; // size e will be 1 x n
   
   Rox_Double data[16] = { -2.0, 2.0, 2.0, 2.0, 
                           -3.0, 3.0, 2.0, 2.0,
                           -2.0, 0.0, 4.0, 2.0,
                           -1.0, 0.0, 0.0, 5.0 };
  
   // Eigenvectors of the matrix
   // 1 2 3 4 

   // Eigenvalues of the matrix
   // -0.730296743340221   0.625543242171225  -0.554700196225229   0.500000000000000
   // -0.547722557505166   0.625543242171224  -0.554700196225229   0.500000000000000
   // -0.365148371670111   0.417028828114149  -0.554700196225229   0.500000000000000
   // -0.182574185835055   0.208514414057075  -0.277350098112615   0.500000000000000

   error = rox_dynvec_double_new(&e, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_objset_array2d_double_new(&V, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&M, 4, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_buffer_no_stride(M, data);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_real_eigenvalues_eigenvectors(e, V, M);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("The Eigenvalues \n");

   error = rox_dynvec_double_print(e);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   ROX_TEST_MESSAGE("The Eigenvectors \n");
   error = rox_objset_array2d_double_print(V);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&M);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
