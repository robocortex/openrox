//==============================================================================
//
//    OPENROX   : File test_pseudoinverse.cpp
//
//    Contents  : Tests for pseudoinverse.c
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
   #include <cstdio>

	#include <baseproc/array/inverse/pseudoinverse.h>
   #include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(pseudoinverse)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_pseudoinverse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double Ainv = NULL;
   Rox_Array2D_Double A = NULL;
   Rox_Double res = 0;

   error = rox_array2d_double_new(&A, 12, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&Ainv, 3, 12);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //Matlab/Octave
   //
   //A = [
   //-479.998566, 1880.00281, 1.00000000
   //- 467.497955, 1876.65283, 1.00000000
   //- 492.499176, 1876.65295, 1.00000000
   //- 458.348602, 1867.50366, 1.00000000
   //- 501.648590, 1867.50378, 1.00000000
   //- 454.998627, 1855.00269, 1.00000000
   //- 504.998627, 1855.00293, 1.00000000
   //- 458.348694, 1842.50195, 1.00000000
   //- 501.648682, 1842.50208, 1.00000000
   //- 467.498108, 1833.35278, 1.00000000
   //- 492.499329, 1833.35291, 1.00000000
   //- 479.998718, 1830.00281, 1.00000000]
   //
   //Ainv = pinv(A)
   //
   //Ainv =
   //2.2823e-008  3.3336e-003 - 3.3335e-003  5.7734e-003 - 5.7734e-003  6.6668e-003 - 6.6668e-003  5.7734e-003 - 5.7734e-003  3.3335e-003 - 3.3336e-003 - 2.4156e-008
   //6.6667e-003  5.7734e-003  5.7734e-003  3.3336e-003  3.3336e-003 - 3.9000e-008  1.8555e-008 - 3.3336e-003 - 3.3336e-003 - 5.7734e-003 - 5.7734e-003 - 6.6667e-003
   //- 1.2283e+001 - 9.0262e+000 - 1.2226e+001 - 3.3292e+000 - 8.8717e+000  3.2834e+000 - 3.1167e+000  9.0384e+000  3.4959e+000  1.2393e+001  9.1928e+000  1.2450e+001
  

   error = rox_array2d_double_set_value(A, 0, 0, -479.998566); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 1, 0, -467.497955); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 2, 0, -492.499176); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 3, 0, -458.348602); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 4, 0, -501.648590); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 5, 0, -454.998627); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 6, 0, -504.998627); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 7, 0, -458.348694); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 8, 0, -501.648682); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 9, 0, -467.498108); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 10, 0, -492.499329); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 11, 0, -479.998718); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value(A, 0, 1, 1880.00281); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 1, 1, 1876.65283); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 2, 1, 1876.65295); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 3, 1, 1867.50366); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 4, 1, 1867.50378); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 5, 1, 1855.00269); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 6, 1, 1855.00293); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 7, 1, 1842.50195); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 8, 1, 1842.50208); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 9, 1, 1833.35278); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 10, 1, 1833.35291); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 11, 1, 1830.00281); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_set_value(A, 0, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 1, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 2, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 3, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 4, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 5, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 6, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 7, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 8, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 9, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 10, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   error = rox_array2d_double_set_value(A, 11, 2, 1.00000000); ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_pseudoinverse(Ainv, A);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_get_value(&res, Ainv, 0, 0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 2.2823e-008, 1e-7);
   
   error = rox_array2d_double_get_value(&res, Ainv, 0, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 3.3336e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 0, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -3.3335e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 0, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 5.7734e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 0, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -5.7734e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 0, 5);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 6.6668e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 0, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -6.6668e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 0, 7);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 5.7734e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 0, 8);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -5.7734e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 0, 9);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 3.3335e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 0, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -3.3335e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 0, 11);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -2.4156e-008, 1e-12);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 6.6667e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 5.7734e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 5.7734e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 3.3336e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 3.3336e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 5);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -3.9000e-008, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 1.8555e-008, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 7);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -3.3336e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 8);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -3.3336e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 9);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -5.7734e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -5.7734e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 1, 11);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -6.6667e-003, 1e-7);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 0);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -1.2283e+001, 1e-3);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -9.0262e+000, 1e-4);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 2);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -1.2226e+001, 1e-3);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -3.3292e+000, 1e-4);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 4);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -8.8717e+000, 1e-4);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 5);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 3.2834e+000, 1e-4);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, -3.1167e+000, 1e-4);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 7);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 9.0384e+000, 1e-4);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 8);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 3.4959e+000, 1e-4);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 9);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 1.2393e+001, 1e-4);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 10);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 9.1928e+000, 1e-4);

   error = rox_array2d_double_get_value(&res, Ainv, 2, 11);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   ROX_TEST_CHECK_CLOSE(res, 1.2450e+001, 1e-4);


   error = rox_array2d_double_del ( &A );
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del ( &Ainv);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
