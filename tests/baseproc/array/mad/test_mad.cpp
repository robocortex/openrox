//==============================================================================
//
//    OPENROX   : File test_mad.cpp
//
//  	Contents  : Tests for mad.c
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
	#include <baseproc/array/mad/mad.h>
   #include <system/time/timer.h>
   #include <inout/system/print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(mad)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_array2d_double_mad)
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
	Rox_Array2D_Double input = NULL;
	Rox_Array2D_Double adfrommedian = NULL;
	Rox_Array2D_Double workbuffer = NULL;
	Rox_Double mad = 0.0;
	
	double data_grt[7] = {-1804289383.0, 846930886.0, -1681692777.0, 1714636915.0, -1957747793.0, 424238335.0, -719885386.0};
	double mad_grt[4] = {1325610134.5, 122596606.0, 1252291367.0, 1144123721.0};
	
	Rox_Sint size = 0;
	Rox_Double * data = NULL;


	// Size is even (on a column), case "two elements only" inside the algorithm 
	size = 2;
	
	error = rox_array2d_double_new(&input, size, 1);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_new(&adfrommedian, size, 1);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	
	error = rox_array2d_double_new(&workbuffer, size, 1);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_get_data_pointer (&data, input);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	for (Rox_Sint i = 0; i<size; i++)
	{
		data[i] = data_grt[i];
	}
	
	error = rox_array2d_double_mad(&mad, adfrommedian, workbuffer, input);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	
	rox_log("mad = %f\n", mad);
	
	ROX_TEST_CHECK_CLOSE(mad, mad_grt[0], 1e-16);
	rox_array2d_double_del(&input);
	rox_array2d_double_del(&adfrommedian);
	rox_array2d_double_del(&workbuffer);
	
	// Size is even, case "two elements only" inside the algorithm 
	size = 2;
	
	error = rox_array2d_double_new(&input, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	
	error = rox_array2d_double_new(&adfrommedian, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	
	error = rox_array2d_double_new(&workbuffer, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_get_data_pointer ( &data, input );
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	for ( Rox_Sint i=0;i<size;i++)
	{
		data[i] = data_grt[i];	
	}

	error = rox_array2d_double_mad(&mad, adfrommedian, workbuffer, input);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	
	rox_log("mad = %f\n",mad);
	
	ROX_TEST_CHECK_CLOSE(mad, mad_grt[0], 1e-16);
	
	rox_array2d_double_del(&input);
	rox_array2d_double_del(&adfrommedian);
	rox_array2d_double_del(&workbuffer);

	// Size is odd, case "one element only" inside the algorithm
	size = 3; 
	error = rox_array2d_double_new(&input, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	error = rox_array2d_double_new(&adfrommedian, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	error = rox_array2d_double_new(&workbuffer, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_get_data_pointer (&data, input);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	for ( Rox_Sint i=0;i<size;i++)
	{
		data[i] = data_grt[i];	
	}
	error = rox_array2d_double_mad(&mad, adfrommedian, workbuffer, input);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	rox_log("mad = %f\n",mad);

	ROX_TEST_CHECK_CLOSE(mad, mad_grt[1], 1e-16);
	rox_array2d_double_del(&input);
	rox_array2d_double_del(&adfrommedian);
	rox_array2d_double_del(&workbuffer);
	
	// Size is even, case "one element only" inside the algorithm
	size = 6; 
	error = rox_array2d_double_new(&input, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	error = rox_array2d_double_new(&adfrommedian, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	error = rox_array2d_double_new(&workbuffer, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_get_data_pointer ( &data, input );
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	for ( Rox_Sint i=0;i<size;i++)
	{
		data[i] = data_grt[i];	
	}
	error = rox_array2d_double_mad(&mad, adfrommedian, workbuffer, input);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	rox_log("mad = %f\n",mad);
	ROX_TEST_CHECK_CLOSE(mad, mad_grt[2], 1e-16);
	rox_array2d_double_del(&input);
	rox_array2d_double_del(&adfrommedian);
	rox_array2d_double_del(&workbuffer);
	
	// Size is odd, case "two elements only" inside the algorithm
	size = 7;
	error = rox_array2d_double_new(&input, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	error = rox_array2d_double_new(&adfrommedian, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	error = rox_array2d_double_new(&workbuffer, 1, size);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

	error = rox_array2d_double_get_data_pointer ( &data, input );
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	
	for ( Rox_Sint i=0;i<size;i++)
	{
		data[i] = data_grt[i];	
	}
	error = rox_array2d_double_mad(&mad, adfrommedian, workbuffer, input);
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
	
	rox_log("mad = %f\n",mad);
	ROX_TEST_CHECK_CLOSE(mad, mad_grt[3], 1e-16);
	rox_array2d_double_del(&input);
	rox_array2d_double_del(&adfrommedian);
	rox_array2d_double_del(&workbuffer);
}

ROX_TEST_SUITE_END()
