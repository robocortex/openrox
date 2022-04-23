//==============================================================================
//
//    OPENROX   : File test_polynomials.cpp
//
//    Contents  : Tests for polynomials.c
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
   #include <system/memory/datatypes.h>
	#include <baseproc/maths/nonlin/polynomials.h>
   #include <baseproc/geometry/specifics/init_vvs_se3_so3z_so3z.h>
   #include <inout/system/errors_print.h>
   #include <inout/numeric/complex_print.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(polynomials)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_polynom_new_del)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Polynom poly = NULL;
   Rox_Uint degree = 0;

   error = rox_polynom_new(NULL, degree);

   error = rox_polynom_new(&poly, degree);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_polynom_del(&poly);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   degree = 1;
   error = rox_polynom_new(&poly, degree);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_polynom_del(&poly);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   degree = 2;
   error = rox_polynom_new(&poly, degree);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_polynom_del(&poly);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   degree = 3;
   error = rox_polynom_new(&poly, degree);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_polynom_del(&poly);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quadratic_roots)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Complex_Struct roots[2];
   Rox_Double coeffs[3] = {1.2, 3.1, -1.0};

   error = rox_quadratic_roots(roots, coeffs);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //rox_complex_print(roots[0]);
   //rox_complex_print(roots[1]);

   ROX_TEST_CHECK_SMALL(roots[0].real - 0.290021074675192, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[0].imag - 0.0, 1e-12);

   ROX_TEST_CHECK_SMALL(roots[1].real + 2.873354408008526, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[1].imag - 0.0, 1e-12);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_cubic_roots)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Complex_Struct roots[3];
   Rox_Double coeffs[4] = {1.2, 3.1, -1.0, -2.3};

   error = rox_cubic_roots(roots, coeffs);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   //rox_complex_print(roots[0]);
   //rox_complex_print(roots[1]);
   //rox_complex_print(roots[2]);

   ROX_TEST_CHECK_SMALL(roots[0].real - 0.874673536491086, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[0].imag - 0.0, 1e-12);

   ROX_TEST_CHECK_SMALL(roots[1].real + 0.835605923592775, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[1].imag - 0.0, 1e-12);

   ROX_TEST_CHECK_SMALL(roots[2].real + 2.622400946231644, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[2].imag - 0.0, 1e-12);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_quartic_roots)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Complex_Struct roots[4];
   Rox_Double coeffs[5] = {1.2, 3.1, -1.0, -2.3, 0.1};

   error = rox_quartic_roots(roots, coeffs);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_complex_print(&roots[0]);
   rox_complex_print(&roots[1]);
   rox_complex_print(&roots[2]);
   rox_complex_print(&roots[3]);

   ROX_TEST_CHECK_SMALL(roots[1].real + 2.617283302056281, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[1].imag - 0.0, 1e-12);

   ROX_TEST_CHECK_SMALL(roots[2].real - 0.858202923266443, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[2].imag - 0.0, 1e-12);

   ROX_TEST_CHECK_SMALL(roots[0].real + 0.867042497277222, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[0].imag - 0.0, 1e-12);

   ROX_TEST_CHECK_SMALL(roots[3].real - 0.042789542733727, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[3].imag - 0.0, 1e-12);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_polynom_roots_sturm)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double roots[5];
   // Matlab coeffs : Rox_Double coeffs[6] = {1.2, 3.1, -1.0, -2.3, 0.1, 0.2}; // roots([1.2, 3.1, -1, -2.3, 0.1, 0.2])
   Rox_Double coeffs[6] = {0.2, 0.1, -2.3, -1.0, 3.1, 1.2};

   Rox_Uint nbroots = 0;

   error = rox_polynomial_roots_sturm(roots, &nbroots, coeffs, 5);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("nbroots = %d\n", nbroots);
   rox_log("root[%d] = %f\n", 1, roots[0]);
   rox_log("root[%d] = %f\n", 2, roots[1]);
   rox_log("root[%d] = %f\n", 3, roots[2]);
   rox_log("root[%d] = %f\n", 4, roots[3]);
   rox_log("root[%d] = %f\n", 5, roots[4]);

   ROX_TEST_CHECK_SMALL(roots[0] + 2.621193934668954, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[1] + 0.780975596126682, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[2] + 0.313199500512193, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[3] - 0.320201947323350, 1e-12);
   ROX_TEST_CHECK_SMALL(roots[4] - 0.811833750651143, 1e-12);
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_polynom_eval)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_polynom_fit)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
