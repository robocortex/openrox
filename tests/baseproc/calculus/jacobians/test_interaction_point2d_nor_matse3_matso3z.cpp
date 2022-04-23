//==============================================================================
//
//    OPENROX   : File test_se3points_pixels_weighted_premul.cpp
//
//    Contents  : Tests for se3points_pixels_weighted_premul.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =====================================================

#include <iostream>


extern "C"
{
	#include <openrox_tests.hpp>

	#include <baseproc/calculus/jacobians/interaction_point2d_nor_matse3_matso3z.h>
	#include <dynvec_point3d_float.h>
	#include <dynvec_point3d_float_struct.h>
	#include <baseproc/array/fill/fillval.h>
	#include <inout/system/errors_print.h>
	#include <baseproc/maths/linalg/matrix.h>
}

//=== INTERNAL MACROS    =====================================================

ROX_TEST_SUITE_BEGIN ( se3_7dof_intmat )

//=== INTERNAL TYPESDEFS =====================================================

//=== INTERNAL DATATYPES =====================================================

//=== INTERNAL VARIABLES =====================================================

//=== INTERNAL FUNCTDEFS =====================================================

//=== INTERNAL FUNCTIONS =====================================================

//=== EXPORTED FUNCTIONS =====================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_se3_7dof_intmat )
{
	Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );

	Rox_Array2D_Double        K=NULL;
	Rox_Uint                  cpt=0      , i  =0   , j=0;
	Rox_Array2D_Double        JtJ=NULL   , LtL=NULL;
	Rox_Array2D_Double        Jtf=NULL   , Ltd=NULL;
	Rox_Uint                  n_pts =0   , nr_pts =4   , nl_pts =4;
	Rox_Array2D_Double        diff  =NULL, diffr  =NULL, diffl  =NULL;
	Rox_Array2D_Double        weight=NULL, weightr=NULL, weightl=NULL;
	Rox_Double                X     =0.0 , Y      =0.0 , Z      =0.0 , d     =0.0;
	Rox_DynVec_Point3D_Float  mc    =NULL, mrc    =NULL, mlc    =NULL, mltmp =NULL;
	Rox_Double                **dJtJ=NULL, **dLtL =NULL, **dJtf =NULL, **dLtd=NULL;
	Rox_Double                maxdev=0.0 , dev    =0.0 ;

	n_pts = nr_pts + nl_pts;

	// base_generator_type generator(42);
	//boost::uniform_real<> uni_dist(0,10);
	//boost::variate_generator<base_generator_type&, boost::uniform_real<> > uni(generator, uni_dist);

	error = rox_array2d_double_new(   &K, 3, 3 );               if (error) goto on_terminate;
	error = rox_array2d_double_new( &JtJ, 6, 6 );               if (error) goto on_terminate;
	error = rox_array2d_double_new( &LtL, 7, 7 );               if (error) goto on_terminate;
	error = rox_array2d_double_new( &Jtf, 6, 1 );               if (error) goto on_terminate;
	error = rox_array2d_double_new( &Ltd, 7, 1 );               if (error) goto on_terminate;

	error = rox_array2d_double_new(  &diff,  n_pts * 2, 1 );    if (error) goto on_terminate;
	error = rox_array2d_double_new( &diffr, nr_pts * 2, 1 );    if (error) goto on_terminate;
	error = rox_array2d_double_new( &diffl, nl_pts * 2, 1 );    if (error) goto on_terminate;

	error = rox_array2d_double_new(  &weight,  n_pts, 1 );      if (error) goto on_terminate;
	error = rox_array2d_double_new( &weightr, nr_pts, 1 );      if (error) goto on_terminate;
	error = rox_array2d_double_new( &weightl, nl_pts, 1 );      if (error) goto on_terminate;

	error = rox_dynvec_point3d_float_new(    &mc,  n_pts );     if (error) goto on_terminate;
	error = rox_dynvec_point3d_float_new(   &mrc, nr_pts );     if (error) goto on_terminate;
	error = rox_dynvec_point3d_float_new(   &mlc, nl_pts );     if (error) goto on_terminate;
	error = rox_dynvec_point3d_float_new( &mltmp, nl_pts );     if (error) goto on_terminate;

	error = rox_dynvec_point3d_float_usecells(    mc,  n_pts ); if (error) goto on_terminate;
	error = rox_dynvec_point3d_float_usecells(   mrc, nr_pts ); if (error) goto on_terminate;
	error = rox_dynvec_point3d_float_usecells(   mlc, nl_pts ); if (error) goto on_terminate;
	error = rox_dynvec_point3d_float_usecells( mltmp, nl_pts ); if (error) goto on_terminate;

	error = rox_array2d_double_fillval( K, 0 );                 if (error) goto on_terminate;
	error = rox_array2d_double_set_value( K, 0, 0, 1.0 );       if (error) goto on_terminate;
	error = rox_array2d_double_set_value( K, 1, 1, 1.0 );       if (error) goto on_terminate;
	error = rox_array2d_double_set_value( K, 2, 2, 1.0 );       if (error) goto on_terminate;

	dJtJ = rox_array2d_double_get_data_pointer_to_pointer( JtJ );
	dLtL = rox_array2d_double_get_data_pointer_to_pointer( LtL );
	dJtf = rox_array2d_double_get_data_pointer_to_pointer( Jtf );
	dLtd = rox_array2d_double_get_data_pointer_to_pointer( Ltd );

	if ( ( NULL == dJtJ ) //// ( NULL == dLtL ) //// ( NULL == dJtf ) //// ( NULL == dLtd ) )
	{
		error = ROX_ERROR_NULL_POINTER;
		goto on_terminate;
	}

	for ( cpt = 0; cpt < n_pts; cpt++ )
	{
		X = uni();
		Y = uni();
		Z = uni();
		d = uni();

		mc->data[cpt].X = X;
		mc->data[cpt].Y = Y;
		mc->data[cpt].Z = Z;
		error = rox_array2d_double_set_value( weight, cpt, 0, 1.0 );
		if (error) goto on_terminate;
		error = rox_array2d_double_set_value(   diff, cpt, 0,   d );
		if (error) goto on_terminate;

		if ( cpt < nr_pts )
		{
			 mrc->data[cpt].X = X;
			 mrc->data[cpt].Y = Y;
			 mrc->data[cpt].Z = Z;
			 error = rox_array2d_double_set_value( weightr, cpt, 0, 1.0 );
			 if (error) goto on_terminate;
			 error = rox_array2d_double_set_value(   diffr, cpt, 0,   d );
			 if (error) goto on_terminate;
		}
		else
		{
			 mlc->data[(cpt-nr_pts)].X = X;
			 mlc->data[(cpt-nr_pts)].Y = Y;
			 mlc->data[(cpt-nr_pts)].Z = Z;
			 error = rox_array2d_double_set_value( weightl, (cpt-nr_pts), 0, 1.0 );
			 if (error) goto on_terminate;
			 error = rox_array2d_double_set_value(   diffl, (cpt-nr_pts), 0,   d );
			 if (error) goto on_terminate;

			 mltmp->data[(cpt-nr_pts)].X = 0.0;
			 mltmp->data[(cpt-nr_pts)].Y = 0.0;
			 mltmp->data[(cpt-nr_pts)].Z = 0.0;
		}
	}

	error = rox_intmat_se3_so3z_weighted_premul_float( LtL, Ltd, diffr, weightr, mrc, diffl, weightl, mlc, mltmp );
	if (error) goto on_terminate;

	error = rox_jacobian_se3_from_points_pixels_weighted_premul_float( JtJ, Jtf, diff, weight, mc, K );
	if (error) goto on_terminate;

	for ( i = 0; i < 6; i++ )
	{
		for (  j = 0; j < 6; j++ )
		{
			dev = fabs( dJtJ[i][j] - dLtL[i][j] );
			maxdev = (dev > maxdev) ? dev : maxdev;
		}

		/* Jtf and Ltd can not be equal */
		//dev = fabs( dJtf[i][0] - dLtd[i][0] );
		//maxdev = (dev > maxdev) ? dev : maxdev;
	}

	std::cout << "maxdev " << maxdev << std::endl;

	// BOOST_CHECK( maxdev < 1e-16 );

on_terminate:
	ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()