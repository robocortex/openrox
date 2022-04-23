//==============================================================================
//
//    OPENROX   : File test_init_vvs_se3_so3z_so3z.cpp
//
//    Contents  : Tests for init_vvs_se3_so3z_so3z.c
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
   #include <baseproc/geometry/specifics/init_vvs_se3_so3z_so3z.h>
   #include <baseproc/maths/linalg/matse3.h>
   #include <inout/numeric/array2d_print.h>
   #include <inout/system/errors_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(plane_transform)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================


ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_init_se3_so3z_so3z_fixed_G)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

	ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   Rox_Array2D_Double bTg=NULL;
   Rox_Array2D_Double pTs=NULL;
   Rox_Array2D_Double cTb=NULL;
   Rox_Array2D_Double cTs=NULL;
   Rox_Array2D_Double pTg=NULL;
   Rox_Double         dzs_p=0.068;
   Rox_Double         dzb_g=0.056;
   Rox_Sint           force_model=0;
   //
   Rox_Array2D_Double bTg_gdt=NULL;
   Rox_Array2D_Double pTs_gdt=NULL;

   // Build ground truth with Matlab
   // bTg = [expmSO3([0 0 1]*pi*1.01), [0.0;0.0;+0.056]; 0 0 0 1]
   // pTs = [expmSO3(-[0 0 1]*pi*1.001), [0.0;0.0;-0.068]; 0 0 0 1]
   // pTg = [[0 0 -1; 0 1 0; 1 0 0], [-2.0; 0; -2.2]; 0 0 0 1]
   // cTb = [expmSO3([0 1 0]*pi/4), [1.66;-0.48;3.44]; 0 0 0 1]
   // cTs = cTb * bTg * inv(pTg) * pTs
   
   /*
   Rox_Double cTb_data[16]={
   0.703268000000000  , 0.007602000000000  , 0.710884000000000 , 1.659829000000000,
   -0.022674000000000 , 0.999674000000000  , 0.011741000000000 , -0.483647000000000,
   -0.710563000000000 , -0.024375000000000 , 0.703211000000000 , 3.443615000000000,
   0.000000000000000  , 0.000000000000000  , 0.000000000000000 , 1.000000000000000 };
   */
   
   Rox_Double cTb_data[16]={
   0.707106781186548,                   0,   0.707106781186548,   1.660000000000000,
                   0,   1.000000000000000,                   0,  -0.480000000000000,
  -0.707106781186547,                   0,   0.707106781186548,   3.440000000000000,
                   0,                   0,                   0,   1.000000000000000
   };
   
   Rox_Double cTs_data[16]={
   0.707173068805330,  -0.019989213325784,  -0.706757866675050,  -1.221423354377856,
  -0.003140037302099,   0.999501628002615,  -0.031410759078128,  -0.546967738354569,
   0.707033514709305,   0.024432088955696,   0.706757866675050,   3.572192189124558,
                   0,                   0,                   0,   1.000000000000000
   };
   
   /*
   Rox_Double cTs_data[16]={
   -0.749083000000000 , 0.007185000000000  , -0.662437000000000 , -1.259869000000000,
   -0.026958000000000 , -0.999444000000000 , 0.019644000000000  , -0.497960000000000,
   -0.661928000000000 , 0.032573000000000  , 0.748860000000000  , 3.581618000000000,
   0.000000000000000  , 0.000000000000000  , 0.000000000000000  , 1.000000000000000};
   */
   
   /*
   Rox_Double pTg_data[16]={
   -0.036095000000000 , 0.042907000000000  , -0.998427000000000 , -2.034269000000000,
   0.041555000000000  , 0.998278000000000  , 0.041398000000000  , 0.000000000000000,
   0.998484000000000  , -0.039995000000000 , -0.037815000000000 , -2.182359000000000,
   0.000000000000000  , 0.000000000000000  , 0.000000000000000  , 1.000000000000000 
   };
   */
   
   Rox_Double pTg_data[16]={
                   0,                   0,  -1.000000000000000,  -2.000000000000000,
                   0,   1.000000000000000,                   0,                   0,
   1.000000000000000,                   0,                   0,  -2.200000000000000,
                   0,                   0,                   0,   1.000000000000000,
   };

/*
   Rox_Double bTg_gdt_data[16]={
   -0.9947138683913782 , 0.0419729060585354  , 0.0937149916495106  , 0.0000000000000000,
   -0.0446809896437480 , -0.9986371700008397 , -0.0269794879173262 , 0.0000000000000000,
   0.0924563932840593  , -0.0310238451429052 , 0.9952341225128000  , 0.0560000000000000,
   0.0000000000000001  , 0.0000000000000001  , -0.0000000000000001 , 0.9999999999999988
   };
*/ 
   Rox_Double bTg_gdt_data[16]={
  -0.999506560365732,   0.031410759078128,                   0,                   0,
  -0.031410759078128,  -0.999506560365732,                   0,                   0,
                   0,                   0,   1.000000000000000,   -0.056000000000000,
                   0,                   0,                   0,   1.000000000000000
   };

/*   
   Rox_Double pTs_gdt_data[16]={
   0.9999975566436177 , -0.0022105896938431 , 0.0000000000000000  , 0.0000000000000000,
   0.0022105896938431 , 0.9999975566436178  , 0.0000000000000000  , 0.0000000000000000,
   0.0000000000000000 , 0.0000000000000000  , 1.0000000000000000  , -0.0680000000000001,
   0.0000000000000000 , 0.0000000000000000  , -0.0000000000000001 , 1.0000000000000000
   };
*/

   Rox_Double pTs_gdt_data[16]={
  -0.999995065201858,  -0.003141587485879,                   0,                   0,
   0.003141587485879,  -0.999995065201858,                   0,                   0,
                   0,                   0,   1.000000000000000,  -0.068000000000000,
                   0,                   0,                   0,   1.000000000000000
   };

   error = rox_matse3_new( &bTg );     if (error) goto function_terminate;
   error = rox_matse3_new( &pTs );     if (error) goto function_terminate;
   error = rox_matse3_new( &cTb );     if (error) goto function_terminate;
   error = rox_matse3_new( &cTs );     if (error) goto function_terminate;
   error = rox_matse3_new( &pTg );     if (error) goto function_terminate;
   error = rox_matse3_new( &bTg_gdt ); if (error) goto function_terminate;
   error = rox_matse3_new( &pTs_gdt ); if (error) goto function_terminate;

   rox_matse3_set_data( pTg, pTg_data );             if (error) goto function_terminate;
   rox_matse3_set_data( cTs, cTs_data );             if (error) goto function_terminate;
   rox_matse3_set_data( cTb, cTb_data );             if (error) goto function_terminate;

   rox_matse3_set_data( bTg_gdt, bTg_gdt_data );     if (error) goto function_terminate;
   rox_matse3_set_data( pTs_gdt, pTs_gdt_data );     if (error) goto function_terminate;

   error = rox_init_se3_so3z_so3z_fixed_G( bTg, pTs, cTb, cTs, pTg, dzs_p, dzb_g, force_model );
   if (error) goto function_terminate;

   ROX_TEST_MESSAGE("bTg \n");
   rox_matse3_print( bTg );

   ROX_TEST_MESSAGE("pTs \n");
   rox_matse3_print( pTs );

function_terminate:
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_matse3_del( &bTg     );
   error = rox_matse3_del( &pTs     );
   error = rox_matse3_del( &cTb     );
   error = rox_matse3_del( &cTs     );
   error = rox_matse3_del( &pTg     );
   error = rox_matse3_del( &bTg_gdt );
   error = rox_matse3_del( &pTs_gdt );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_stelia)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   ROX_TEST_MESSAGE ( "This test has not been implemented yet !!! \n" );
   // Ground truth with matlab
   // Rox_Double Tv[16] = {0.995720304870356, 0, 0.092417933697340, -0.029765520459428, 0, 1.000000000000000, 0, 0, -0.092417933697340, 0, 0.995720304870356, 0.037908036222387, 0, 0, 0, 1.000000000000000};
   Rox_Double Td[16] = {0.302836482985766, 0, 0.953042530306393, 2.545808258358236, 0, 1.000000000000000, 0, 0, -0.953042530306393, 0, 0.302836482985766, -2.688180952856296, 0, 0, 0, 1.000000000000000};
   Rox_Double Tr[16] = {0.122260838196515, 0, 0.992498003747859, 2.226945805167551, 0, 1.000000000000000, 0, 0, -0.992498003747859, 0, 0.122260838196515, -2.836138624307071, 0, 0, 0, 1.000000000000000};
   Rox_Array2D_Double T = NULL, pTb_mes = NULL, pTb_mod = NULL;
   
   error = rox_array2d_double_new(&T, 4, 4);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_new(&pTb_mod, 4, 4);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_new(&pTb_mes, 4, 4);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_matse3_set_data(pTb_mes, Td);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_matse3_set_data(pTb_mod, Tr);
   ROX_ERROR_CHECK_TERMINATE(error)
      
   error = rox_init_se3_model_adjustment(T, pTb_mes, pTb_mod);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   // rox_array2d_double_print(T);
   // rox_array2d_double_print(pTb_mes);
   // rox_array2d_double_print(pTb_mod);

function_terminate:
   rox_array2d_double_del(&T);
   rox_array2d_double_del(&pTb_mod);
   rox_array2d_double_del(&pTb_mes);
}

ROX_TEST_SUITE_END()
