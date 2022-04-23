//==============================================================================
//
//    OPENROX   : File rox_example_vvs_se3_so3z_so3z.c
//
//    Contents  : A simple example program for virtual visual servoing with a
//                full 6dof pose matrix and two 1dof Z-rotation matrices
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== HEADERS   ================================================================

#include <stdio.h>
#include <api/openrox.h>
#include <core/indirect/euclidean/vvs_tools.h>
#include <core/indirect/euclidean/vvs_se3_so3z_so3z.h>
#include <baseproc/geometry/point/points_struct.h>
#include <inout/system/errors_print.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>

Rox_Uint i = 0;

/*====== MAIN PROGRAM ======================================================*/

Rox_Sint main (Rox_Void)
{
   Rox_ErrorCode error;

   /* Define the result object */
   Rox_Sint i=0;
   Rox_Array2D_Double cTr=NULL;
   Rox_Array2D_Double wTl=NULL;
   Rox_Array2D_Double aTw=NULL;
   Rox_Array2D_Double rTa=NULL;
   Rox_Array2D_Double K  =NULL;
   Rox_Double **dcTr=NULL;
   Rox_Double **daTw=NULL;
   Rox_Double **dK  =NULL;
   Rox_Point3D_Float_Struct pml[4];
   Rox_Point3D_Float_Struct pmr[4];
   Rox_DynVec_Point3D_Float ml=NULL;
   Rox_DynVec_Point3D_Float mr=NULL;

   Rox_Point2D_Float_Struct pql[4];
   Rox_Point2D_Float_Struct pqr[4];
   Rox_DynVec_Point2D_Float ql=NULL;
   Rox_DynVec_Point2D_Float qr=NULL;

   Rox_Double pix_tresh=600.0;
   Rox_Double norm_tresh=-1.0;

   /* create intrinsics matrix */
   error = rox_array2d_double_new( &K, 3, 3 );
   if (error) goto function_terminate;

   error = rox_array2d_double_fillunit( K );
   if (error) goto function_terminate;

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dK, K );
   if (error) goto function_terminate;

   dK[0][0] = 1900.0;
   dK[0][2] = 1900.0;
   dK[1][1] = 1900.0;
   dK[1][2] = 1400.0;

   /* Convert pixel units distance treshold to normalized units */
   error = conv_dist_pixel_to_normalized( &norm_tresh, pix_tresh, K );
   if (error) goto function_terminate;

   printf( "norm_tresh %f \n", norm_tresh );

   /* Create cTr matrix */
   error = rox_array2d_double_new( &cTr, 4, 4 );
   if (error) goto function_terminate;

   error = rox_array2d_double_fillunit( cTr );
   if (error) goto function_terminate;

   error = rox_array2d_double_get_data_pointer_to_pointer( &dcTr, cTr );
   if (error) goto function_terminate;

   dcTr[0][0] = 9.99999740000000e-01;
   dcTr[0][1] = 9.61600000000000e-05;
   dcTr[0][2] = -7.09150000000000e-04;
   dcTr[0][3] = 6.01258360000000e-01;
                
   dcTr[1][0] = -9.60400000000000e-05;
   dcTr[1][1] = 9.99999980000000e-01;
   dcTr[1][2] = 1.63690000000000e-04;
   dcTr[1][3] = 3.00740900000000e-01;
                
   dcTr[2][0] = 7.09160000000000e-04;
   dcTr[2][1] = -1.63620000000000e-04;
   dcTr[2][2] = 9.99999740000000e-01;
   dcTr[2][3] = 1.60252767000000e+00;

   /* Create wTl matrix */
   error = rox_array2d_double_new( &wTl, 4, 4 );
   if (error) goto function_terminate;

   error = rox_array2d_double_fillunit( wTl );
   if (error) goto function_terminate;

   /* Create aTw matrix */
   error = rox_array2d_double_new( &aTw, 4, 4 );
   if (error) goto function_terminate;

   error = rox_array2d_double_fillunit( aTw );
   if (error) goto function_terminate;

   error = rox_array2d_double_get_data_pointer_to_pointer( &daTw, aTw );
   if (error) goto function_terminate;

   daTw[0][0] = 9.84807753012208e-01;
   daTw[0][1] = 1.73648177666930e-01;
   daTw[0][2] = 0.00000000000000e+00;
   daTw[0][3] = -9.99459868531933e-01;
                
   daTw[1][0] = -1.73648177666930e-01;
   daTw[1][1] = 9.84807753012208e-01;
   daTw[1][2] = 0.00000000000000e+00;
   daTw[1][3] = -4.70191419736826e-01;
                
   daTw[2][0] = 0.00000000000000e+00;
   daTw[2][1] = 0.00000000000000e+00;
   daTw[2][2] = 1.00000000000000e+00;
   daTw[2][3] = -1.00000000000000e-01;

   /* Create rTa matrix */
   error = rox_array2d_double_new( &rTa, 4, 4 );
   if (error) goto function_terminate;

   error = rox_array2d_double_fillunit( rTa );
   if (error) goto function_terminate;

   /* Coordinates of the corners of the left photoframe */
   {
      pml[0].X =-2.46153846153846e-01;
      pml[0].Y =-2.46153846153846e-01;
      pml[0].Z =0.00000000000000e+00;
                
      pml[1].X =2.46153846153846e-01;
      pml[1].Y =-2.46153846153846e-01;
      pml[1].Z =0.00000000000000e+00;
                
      pml[2].X =2.46153846153846e-01;
      pml[2].Y =2.46153846153846e-01;
      pml[2].Z =0.00000000000000e+00;
                
      pml[3].X =-2.46153846153846e-01;
      pml[3].Y =2.46153846153846e-01;
      pml[3].Z =0.00000000000000e+00;
   }

   error = rox_dynvec_point3d_float_new( &ml, 4);
   for ( i = 0; i<4; i++ )
   {
      error = rox_dynvec_point3d_float_append( ml, &(pml[i]) );
      if (error) goto function_terminate;
   }

   /* Coordinates of the corners of the right photoframe */
   {
      pmr[0].X =-2.46153846153846e-01;
      pmr[0].Y =-2.46153846153846e-01;
      pmr[0].Z =0.00000000000000e+00;
                
      pmr[1].X =2.46153846153846e-01;
      pmr[1].Y =-2.46153846153846e-01;
      pmr[1].Z =0.00000000000000e+00;
                
      pmr[2].X =2.46153846153846e-01;
      pmr[2].Y =2.46153846153846e-01;
      pmr[2].Z =0.00000000000000e+00;
                
      pmr[3].X =-2.46153846153846e-01;
      pmr[3].Y =2.46153846153846e-01;
      pmr[3].Z =0.00000000000000e+00;
   }

   error = rox_dynvec_point3d_float_new( &mr, 4);
   for ( i = 0; i<4; i++ )
   {
      error = rox_dynvec_point3d_float_append( mr, &(pmr[i]) );
      if (error) goto function_terminate;
   }

   /* Normalized coordinates of corners observations */
   {
      pql[0].u = -4.97435897435897e-01;
      pql[0].v = -3.07692307692308e-02;
                 
      pql[1].u = -1.69230769230769e-01;
      pql[1].v = -3.07692307692308e-02;
                 
      pql[2].u = -1.69230769230769e-01;
      pql[2].v = 2.97435897435897e-01;
                 
      pql[3].u = -4.97435897435897e-01;
      pql[3].v = 2.97435897435897e-01;
   }

   {
      pqr[0].u = 2.21153846153846e-01;
      pqr[0].v = 3.36538461538461e-02;
                 
      pqr[1].u = 5.28846153846154e-01;
      pqr[1].v = 3.36538461538461e-02;
                 
      pqr[2].u = 5.28846153846154e-01;
      pqr[2].v = 3.41346153846154e-01;
                 
      pqr[3].u = 2.21153846153846e-01;
      pqr[3].v = 3.41346153846154e-01;
   }

   error = rox_dynvec_point2d_float_new( &ql, 4);
   for ( i = 0; i<4; i++ )
   {
      error = rox_dynvec_point2d_float_append( ql, &(pql[i]) );
      if (error) goto function_terminate;
   }

   error = rox_dynvec_point2d_float_new( &qr, 4);
   for ( i = 0; i<4; i++ )
   {
      error = rox_dynvec_point2d_float_append( qr, &(pqr[i]) );
      if (error) goto function_terminate;
   }

   printf("Input cTr: \n");
   error = rox_matrix_print( cTr );
   if(error) goto function_terminate;
   printf("\n");
   printf("Input rTa: \n");
   error = rox_matrix_print( rTa );
   if(error) goto function_terminate;
   printf("\n");
   printf("Input wTl: \n");
   error = rox_matrix_print( wTl );
   if(error) goto function_terminate;
   printf("\n");

   error = rox_points_float_refine_pose_vvs_se3_so3z_so3z( cTr, rTa, wTl, aTw, qr, mr, ql, ml, norm_tresh );
   if(error) goto function_terminate;

   printf("=== \n");
   printf("Result cTr: \n");
   error = rox_matrix_print( cTr );
   if(error) goto function_terminate;
   printf("\n");
   printf("Expected cTr: \n");
   printf("Matrix (4x4) \n");
   printf(" 1.00000   0.00000   0.00000   0.60000 \n");
   printf(" 0.00000   1.00000   0.00000   0.30000 \n");
   printf(" 0.00000   0.00000   1.00000   1.60000 \n");
   printf(" 0.00000   0.00000   0.00000   1.00000 \n");
   printf("\n");

   printf("=== \n");
   printf("Result rTa: \n");
   error = rox_matrix_print( rTa );
   if(error) goto function_terminate;
   printf("\n");
   printf("Expected rTa: \n");
   printf("Matrix (4x4) \n");
   printf("  0.93969   0.34202   0.00000   0.00000 \n");
   printf(" -0.34202   0.93969   0.00000   0.00000 \n");
   printf("  0.00000   0.00000   1.00000   0.00000 \n");
   printf("  0.00000   0.00000   0.00000   1.00000 \n");
   printf("\n");

   printf("=== \n");
   printf("Result wTl: \n");
   error = rox_matrix_print( wTl );
   if(error) goto function_terminate;
   printf("\n");
   printf("Expected wTl: \n");
   printf("Matrix (4x4) \n");
   printf(" 0.86603  -0.50000   0.00000   0.00000 \n");
   printf(" 0.50000   0.86603   0.00000   0.00000 \n");
   printf(" 0.00000   0.00000   1.00000   0.00000 \n");
   printf(" 0.00000   0.00000   0.00000   1.00000 \n");

function_terminate:
   rox_array2d_double_del( &cTr );
   rox_array2d_double_del( &wTl );
   rox_array2d_double_del( &aTw );
   rox_array2d_double_del( &rTa );
   rox_dynvec_point3d_float_del( &ml );
   rox_dynvec_point3d_float_del( &mr );

   rox_dynvec_point2d_float_del( &ql );
   rox_dynvec_point2d_float_del( &qr );

   rox_error_print(error);

   return error;
}
