//==============================================================================
//
//    OPENROX   : File rox_example_camera_calibration.c
//
//    Contents  : A simple example program for camera calibration.
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
#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/array/fill/fillval.h>
#include <core/indirect/euclidean/vvs_se3_so3z.h>
#include <inout/system/errors_print.h>

Rox_Uint i = 0;

// ====== MAIN PROGRAM =========================================================

Rox_Sint main (Rox_Void)
{
   Rox_ErrorCode error;

   // Define the result object 
   Rox_Array2D_Double K  =NULL;
   Rox_Array2D_Double cTr=NULL;
   Rox_Array2D_Double rTl=NULL;
   Rox_Array2D_Double Tr =NULL;
   Rox_Double **dK  =NULL;
   Rox_Double **dcTr=NULL;
   Rox_Double **drTl=NULL;
   Rox_Double **dTr =NULL;
   Rox_Sint i=0;
   Rox_Point3D_Float_Struct pml[4];
   Rox_Point3D_Float_Struct pmr[4];
   Rox_DynVec_Point3D_Float ml=NULL;
   Rox_DynVec_Point3D_Float mr=NULL;

   Rox_Point2D_Float_Struct pql[4];
   Rox_Point2D_Float_Struct pqr[4];
   Rox_DynVec_Point2D_Float ql=NULL;
   Rox_DynVec_Point2D_Float qr=NULL;

   // Create the intrinsic matrix 
   error = rox_array2d_double_new( &K, 3, 3 );
   if (error) goto function_terminate;

   error = rox_array2d_double_fillval( K, 0 );
   if (error) goto function_terminate;

   error = rox_array2d_double_get_data_pointer_to_pointer( &dK, K );
   if (error) goto function_terminate;

   dK[0][0] = 1900.0; dK[0][2] = 1900.0;
   dK[1][1] = 1900.0; dK[1][2] = 1400.0;
   dK[2][2] =    1.0;

   // Create cTr matrix 
   error = rox_array2d_double_new( &cTr, 4, 4 );
   if (error) goto function_terminate;

   error = rox_array2d_double_fillval( cTr, 0 );
   if (error) goto function_terminate;

   error = rox_array2d_double_get_data_pointer_to_pointer( &dcTr, cTr );
   if (error) goto function_terminate;


   dcTr[0][0] = -0.08190181;
   dcTr[0][1] =  0.63711170;
   dcTr[0][2] = -0.76640771;
   dcTr[0][3] =  1.31108288;

   dcTr[1][0] = -0.99615008;
   dcTr[1][1] = -0.07644955;
   dcTr[1][2] =  0.04290091;
   dcTr[1][3] = -0.03737920;

   dcTr[2][0] = -0.03125885;
   dcTr[2][1] =  0.76697076;
   dcTr[2][2] =  0.64092023;
   dcTr[2][3] =  4.33483215;

   dcTr[3][3] = 1.0;

   // Create rTl matrix 
   error = rox_array2d_double_new( &rTl, 4, 4 );
   if (error) goto function_terminate;

   error = rox_array2d_double_fillval( rTl, 0 );
   if (error) goto function_terminate;

   error = rox_array2d_double_get_data_pointer_to_pointer( &drTl, rTl );
   if (error) goto function_terminate;

   drTl[0][0] = -0.98623;
   drTl[0][1] =  0.04358;
   drTl[0][2] = -0.15950;
   drTl[0][3] =  0.20554;

   drTl[1][0] = -0.15957;
   drTl[1][1] =  0.00165;
   drTl[1][2] =  0.98719;
   drTl[1][3] = -2.17386;

   drTl[2][0] = 0.04328;
   drTl[2][1] = 0.99905;
   drTl[2][2] = 0.00533;
   drTl[2][3] = 2.04627;

   drTl[3][3] = 1.0;

   // Create Tr matrix 
   error = rox_array2d_double_new( &Tr, 4, 4 );
   if (error) goto function_terminate;

   error = rox_array2d_double_fillval( Tr, 0 );
   if (error) goto function_terminate;

   error = rox_array2d_double_get_data_pointer_to_pointer( &dTr, Tr );
   if (error) goto function_terminate;

   dTr[0][0] =  0.89458; dTr[0][1] = -0.44691;
   dTr[1][0] = 0.44691;  dTr[1][1] = 0.89458;
   dTr[2][2] = 1.0;
   dTr[3][3] = 1.0;

   // Coordinates of the corners of the left photoframe in the _right_ frame 
   // up to the XY-plane rotation uncertainty, of course 
   {
      pml[0].X = 4.37580535189651e-01;
      pml[0].Y =-2.13498590250456e+00;
      pml[0].Z = 1.78970033618109e+00;

      pml[1].X =-4.79495194221776e-02;
      pml[1].Y =-2.21354378129828e+00;
      pml[1].Z = 1.81100960026196e+00;

      pml[2].X =-2.64968713344944e-02;
      pml[2].Y =-2.21273340973868e+00;
      pml[2].Z = 2.30284829581369e+00;

      pml[3].X = 4.59033183277334e-01;
      pml[3].Y =-2.13417553094495e+00;
      pml[3].Z = 2.28153903173282e+00;
   }

   error = rox_dynvec_point3d_float_new( &ml, 4);
   for ( i = 0; i<4; i++ )
   {
      error = rox_dynvec_point3d_float_append( ml, &(pml[i]) );
      if (error) goto function_terminate;
   }

   // Coordinates of the corners of the right photoframe in the right frame 
   {
      pmr[0].X = -0.4*128.0/208.0;
      pmr[0].Y = -0.4*128.0/208.0;
      pmr[0].Z = 0.0;

      pmr[1].X = +0.4*128.0/208.0;
      pmr[1].Y = -0.4*128.0/208.0;
      pmr[1].Z = 0.0;

      pmr[2].X = 0.4*128.0/208.0;
      pmr[2].Y = 0.4*128.0/208.0;
      pmr[2].Z = 0.0;

      pmr[3].X = -0.4*128.0/208.0;
      pmr[3].Y = +0.4*128.0/208.0;
      pmr[3].Z = 0.0;
   }

   error = rox_dynvec_point3d_float_new( &mr, 4);
   for ( i = 0; i<4; i++ )
   {
      error = rox_dynvec_point3d_float_append( mr, &(pmr[i]) );
      if (error) goto function_terminate;
   }

   // Normalized coordinates of corners observations 
   {
      pql[0].u = -0.383356;
      pql[0].v = -0.060720;

      pql[1].u = -0.393634;
      pql[1].v =  0.068219;

      pql[2].u = -0.455675;
      pql[2].v =  0.062953;

      pql[3].u = -0.445701;
      pql[3].v = -0.056132;
   }

   {
      pqr[0].u = 0.282737;
      pqr[0].v = 0.054564;

      pqr[1].u =  0.274046;
      pqr[1].v = -0.063737;

      pqr[2].u =  0.320587;
      pqr[2].v = -0.066742;

      pqr[3].u = 0.328397;
      pqr[3].v = 0.041712;
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

   printf("The input pose parameters are: \n");
   error = rox_matrix_print( cTr );
   if(error) goto function_terminate;

   error = rox_points_float_refine_pose_vvs_se3_so3z( cTr, Tr, qr, mr, ql, ml, 10.0 );
   if(error) goto function_terminate;

   printf("The result pose parameters are: \n");
   error = rox_matrix_print( cTr );
   if(error) goto function_terminate;

   printf("The ground-truth pose parameters are: \n");
   printf("Matrix (4x4) \n");
   printf(" -0.08179   0.63694  -0.76657   1.30709 \n");
   printf(" -0.99620  -0.07548   0.04358  -0.03520 \n");
   printf(" -0.03010   0.76721   0.64069   4.31865 \n");
   printf("  0.00000   0.00000   0.00000   1.00000 \n");

function_terminate:
   rox_array2d_double_del( &K );
   rox_array2d_double_del( &cTr );
   rox_array2d_double_del( &rTl );
   rox_dynvec_point3d_float_del( &ml );
   rox_dynvec_point3d_float_del( &mr );

   rox_dynvec_point2d_float_del( &ql );
   rox_dynvec_point2d_float_del( &qr );

   rox_error_print(error);

   return error;
}
