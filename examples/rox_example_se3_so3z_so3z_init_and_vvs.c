//==============================================================================
//
//    OPENROX   : File rox_example_vvs_se3_so3z_so3z.c
//
//    Contents  : A simple example program for virtual visual servoing with a
//                full 6dof pose matrix and two 1dof Z-rotation matrices
//                Inputs are read from a file data_se3_so3z_so3z.txt, it can be
//                generated with the octave file
//                vvs_se3_so3z_so3z_initilization_and_file_generation.m
//                in the directory
//                rox_mat/modules/geometry/specifics/
//                Since the generation can be configured the file should be read
//                before execution
//                Made for the APRA project
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
#include <string.h>

#include <api/openrox.h>

#include <baseproc/array/add/add.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/specifics/init_vvs_se3_so3z_so3z.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <core/indirect/euclidean/vvs_se3_so3z_so3z.h>
#include <core/indirect/euclidean/vvs_tools.h>
#include <inout/system/errors_print.h>
#include <inout/numeric/array2d_print.h>
#include <baseproc/maths/maths_macros.h>

#include <generated/array2d_double.h>

// Inputs are read from a file data_se3_so3z_so3z.txt, it can be
// generated with the octave file
// vvs_se3_so3z_so3z_initilization_and_file_generation.m
// in the directory
// rox_mat/modules/geometry/specifics/
// Since the generation can be configured the file should be read
// before execution
Rox_Sint main (Rox_Void)
{
   Rox_ErrorCode            error = ROX_ERROR_NONE;
   Rox_Sint                 i=0;
   Rox_Array2D_Double       cTb=NULL;
   Rox_Array2D_Double       cTs=NULL;
   Rox_Double               **dcTb=NULL;
   Rox_Double               **dcTs=NULL;
   Rox_Array2D_Double       cTb_pert=NULL;
   Rox_Array2D_Double       cTs_pert=NULL;
   Rox_Double               **dcTb_pert=NULL;
   Rox_Double               **dcTs_pert=NULL;
   Rox_Array2D_Double       cTb_ref=NULL;
   Rox_Array2D_Double       cTs_ref=NULL;
   Rox_Array2D_Double       pTs=NULL;
   Rox_Array2D_Double       gTp=NULL;
   Rox_Array2D_Double       bTg=NULL;
   Rox_Array2D_Double       pTs_in=NULL;
   Rox_Array2D_Double       pTg_in=NULL;
   Rox_Array2D_Double       gTb_in=NULL;
   Rox_Double               **dpTs_in=NULL;
   Rox_Double               **dpTg_in=NULL;
   Rox_Double               **dgTb_in=NULL;
   Rox_Array2D_Double       K=NULL;
   Rox_Double               **dK  =NULL;
   Rox_Point3D_Float_Struct        pms[4];
   Rox_Point3D_Float_Struct        pmb[4];
   Rox_DynVec_Point3D_Float ms=NULL;
   Rox_DynVec_Point3D_Float mb=NULL;
   FILE                     *fp=NULL;
   char                     sharp;
   char                     name[256];
   int                      scanf_ret=0;
   int                      free_G=~0;

   // From the model:
   Rox_Array2D_Double zg_p=NULL;
   Rox_Double         **dzg_p=NULL;
   Rox_Array2D_Double ptg=NULL;
   Rox_Double         dist_zs_p=0.0;
   Rox_Double         dist_zb_g=0.0;

   Rox_Point2D_Float_Struct pqs[4];
   Rox_Point2D_Float_Struct pqb[4];
   Rox_DynVec_Point2D_Float qs = NULL;
   Rox_DynVec_Point2D_Float qb = NULL;

   Rox_Double pix_tresh=5.0;
   Rox_Double norm_tresh=-1.0;

   // Open data file
   fp = fopen( "data_se3_so3z_so3z.txt", "r" );
   if ( NULL == fp )
   {
      printf("fail");
      return ROX_ERROR_NULL_POINTER;
   }

   // Allocate stuff
   error = rox_array2d_double_new      ( &K       , 3, 3 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &cTb     , 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &cTs     , 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &cTb_pert, 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &cTs_pert, 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &cTb_ref , 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &cTs_ref , 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &pTs     , 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &gTp     , 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &bTg     , 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &ptg     , 3, 1 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &zg_p    , 3, 1 ); if (error) goto on_error;
   error = rox_dynvec_point3d_float_new( &ms      , 4    ); if (error) goto on_error;
   error = rox_dynvec_point3d_float_new( &mb      , 4    ); if (error) goto on_error;
   error = rox_dynvec_point2d_float_new( &qs      , 4    ); if (error) goto on_error;
   error = rox_dynvec_point2d_float_new( &qb      , 4    ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &pTs_in  , 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &pTg_in  , 4, 4 ); if (error) goto on_error;
   error = rox_array2d_double_new      ( &gTb_in  , 4, 4 ); if (error) goto on_error;

   // Get access
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dK, K        );
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dcTb,  cTb      );
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dcTs,  cTs      );
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dcTb_pert,  cTb_pert );
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dcTs_pert,  cTs_pert );
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dzg_p,  zg_p     );
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dpTs_in,  pTs_in   );
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dpTg_in,  pTg_in   );
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dgTb_in,  gTb_in   );

   if ( ( NULL == dK      ) || ( NULL == dcTb      ) || ( NULL == dcTs      ) ||
        ( NULL == dzg_p   ) || ( NULL == dpTs_in   ) || ( NULL == dpTg_in   ) ||
        ( NULL == dgTb_in ) || ( NULL == dcTb_pert ) || ( NULL == dcTs_pert )  )
   {
      error = ROX_ERROR_NULL_POINTER;
      goto on_error;
   }

   // Read data file
   while ( EOF != fscanf( fp, "%c %s", &sharp, name ) )
   {
      if ( 0 == strcmp( name, "free" ) )
      {
         scanf_ret = fscanf(fp, "%d  \n", (int*) &free_G );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }

      if ( 0 == strcmp( name, "zg_p" ) )
      {
         scanf_ret = fscanf(fp, "%lf %lf %lf \n", (double*) &dzg_p[0][0], (double*) &dzg_p[0][1], (double*) &dzg_p[0][2] );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "dzs_p" ) )
      {
         scanf_ret = fscanf(fp, "%lf \n", (double*) &dist_zs_p );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "dzb_g" ) )
      {
         scanf_ret = fscanf(fp, "%lf \n", (double*) &dist_zb_g );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "pTs" ) )
      {
         scanf_ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n",
         (double*) &dpTs_in[0][0], (double*) &dpTs_in[0][1], (double*) &dpTs_in[0][2], (double*) &dpTs_in[0][3],
         (double*) &dpTs_in[1][0], (double*) &dpTs_in[1][1], (double*) &dpTs_in[1][2], (double*) &dpTs_in[1][3],
         (double*) &dpTs_in[2][0], (double*) &dpTs_in[2][1], (double*) &dpTs_in[2][2], (double*) &dpTs_in[2][3],
         (double*) &dpTs_in[3][0], (double*) &dpTs_in[3][1], (double*) &dpTs_in[3][2], (double*) &dpTs_in[3][3] );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "pTg" ) )
      {
         scanf_ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n",
         (double*) &dpTg_in[0][0], (double*) &dpTg_in[0][1], (double*) &dpTg_in[0][2], (double*) &dpTg_in[0][3],
         (double*) &dpTg_in[1][0], (double*) &dpTg_in[1][1], (double*) &dpTg_in[1][2], (double*) &dpTg_in[1][3],
         (double*) &dpTg_in[2][0], (double*) &dpTg_in[2][1], (double*) &dpTg_in[2][2], (double*) &dpTg_in[2][3],
         (double*) &dpTg_in[3][0], (double*) &dpTg_in[3][1], (double*) &dpTg_in[3][2], (double*) &dpTg_in[3][3] );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "gTb" ) )
      {
         scanf_ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n",
         (double*) &dgTb_in[0][0], (double*) &dgTb_in[0][1], (double*) &dgTb_in[0][2], (double*) &dgTb_in[0][3],
         (double*) &dgTb_in[1][0], (double*) &dgTb_in[1][1], (double*) &dgTb_in[1][2], (double*) &dgTb_in[1][3],
         (double*) &dgTb_in[2][0], (double*) &dgTb_in[2][1], (double*) &dgTb_in[2][2], (double*) &dgTb_in[2][3],
         (double*) &dgTb_in[3][0], (double*) &dgTb_in[3][1], (double*) &dgTb_in[3][2], (double*) &dgTb_in[3][3] );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "cTs" ) )
      {
         scanf_ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n",
         (double*) &dcTs[0][0], (double*) &dcTs[0][1], (double*) &dcTs[0][2], (double*) &dcTs[0][3],
         (double*) &dcTs[1][0], (double*) &dcTs[1][1], (double*) &dcTs[1][2], (double*) &dcTs[1][3],
         (double*) &dcTs[2][0], (double*) &dcTs[2][1], (double*) &dcTs[2][2], (double*) &dcTs[2][3],
         (double*) &dcTs[3][0], (double*) &dcTs[3][1], (double*) &dcTs[3][2], (double*) &dcTs[3][3] );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "cTb" ) )
      {
         scanf_ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n",
         (double*) &dcTb[0][0], (double*) &dcTb[0][1], (double*) &dcTb[0][2], (double*) &dcTb[0][3],
         (double*) &dcTb[1][0], (double*) &dcTb[1][1], (double*) &dcTb[1][2], (double*) &dcTb[1][3],
         (double*) &dcTb[2][0], (double*) &dcTb[2][1], (double*) &dcTb[2][2], (double*) &dcTb[2][3],
         (double*) &dcTb[3][0], (double*) &dcTb[3][1], (double*) &dcTb[3][2], (double*) &dcTb[3][3] );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "cTs_pert" ) )
      {
         scanf_ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n",
         (double*) &dcTs_pert[0][0], (double*) &dcTs_pert[0][1], (double*) &dcTs_pert[0][2], (double*) &dcTs_pert[0][3],
         (double*) &dcTs_pert[1][0], (double*) &dcTs_pert[1][1], (double*) &dcTs_pert[1][2], (double*) &dcTs_pert[1][3],
         (double*) &dcTs_pert[2][0], (double*) &dcTs_pert[2][1], (double*) &dcTs_pert[2][2], (double*) &dcTs_pert[2][3],
         (double*) &dcTs_pert[3][0], (double*) &dcTs_pert[3][1], (double*) &dcTs_pert[3][2], (double*) &dcTs_pert[3][3] );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "cTb_pert" ) )
      {
         scanf_ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n",
         (double*) &dcTb_pert[0][0], (double*) &dcTb_pert[0][1], (double*) &dcTb_pert[0][2], (double*) &dcTb_pert[0][3],
         (double*) &dcTb_pert[1][0], (double*) &dcTb_pert[1][1], (double*) &dcTb_pert[1][2], (double*) &dcTb_pert[1][3],
         (double*) &dcTb_pert[2][0], (double*) &dcTb_pert[2][1], (double*) &dcTb_pert[2][2], (double*) &dcTb_pert[2][3],
         (double*) &dcTb_pert[3][0], (double*) &dcTb_pert[3][1], (double*) &dcTb_pert[3][2], (double*) &dcTb_pert[3][3] );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "Kc" ) )
      {
         scanf_ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf \n",
         (double*) &dK[0][0], (double*) &dK[0][1], (double*) &dK[0][2],
         (double*) &dK[1][0], (double*) &dK[1][1], (double*) &dK[1][2],
         (double*) &dK[2][0], (double*) &dK[2][1], (double*) &dK[2][2] );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
      }
      if ( 0 == strcmp( name, "ms" ) )
      {
         scanf_ret = fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f \n",
         (float*) &(pms[0].X), (float*) &(pms[0].Y), (float*) &(pms[0].Z),
         (float*) &(pms[1].X), (float*) &(pms[1].Y), (float*) &(pms[1].Z),
         (float*) &(pms[2].X), (float*) &(pms[2].Y), (float*) &(pms[2].Z),
         (float*) &(pms[3].X), (float*) &(pms[3].Y), (float*) &(pms[3].Z) );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }
         /*
         printf( "Ms: %f %f %f %f %f %f %f %f %f %f %f %f \n",
         (pms[0].X), (pms[0].Y), (pms[0].Z),
         (pms[1].X), (pms[1].Y), (pms[1].Z),
         (pms[2].X), (pms[2].Y), (pms[2].Z),
         (pms[3].X), (pms[3].Y), (pms[3].Z) ); */


         for ( i = 0; i<4; i++ )
         {
            error = rox_dynvec_point3d_float_append( ms, &(pms[i]) );
            if (error) goto on_error;
         }
      }
      if ( 0 == strcmp( name, "mb" ) )
      {
         scanf_ret = fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f \n",
         (float*) &pmb[0].X, (float*) &pmb[0].Y, (float*) &pmb[0].Z,
         (float*) &pmb[1].X, (float*) &pmb[1].Y, (float*) &pmb[1].Z,
         (float*) &pmb[2].X, (float*) &pmb[2].Y, (float*) &pmb[2].Z,
         (float*) &pmb[3].X, (float*) &pmb[3].Y, (float*) &pmb[3].Z );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }

         for ( i = 0; i<4; i++ )
         {
            error = rox_dynvec_point3d_float_append( mb, &(pmb[i]) );
            if (error) goto on_error;
         }
      }
      if ( 0 == strcmp( name, "qs" ) )
      {
         scanf_ret = fscanf(fp, "%f %f %f %f %f %f %f %f \n",
         (float*) &pqs[0].u, (float*) &pqs[0].v,
         (float*) &pqs[1].u, (float*) &pqs[1].v,
         (float*) &pqs[2].u, (float*) &pqs[2].v,
         (float*) &pqs[3].u, (float*) &pqs[3].v );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }

         for ( i = 0; i<4; i++ )
         {
            error = rox_dynvec_point2d_float_append( qs, &(pqs[i]) );
            if (error) goto on_error;
         }
      }
      if ( 0 == strcmp( name, "qb" ) )
      {
         scanf_ret = fscanf(fp, "%f %f %f %f %f %f %f %f \n",
         (float*) &pqb[0].u, (float*) &pqb[0].v,
         (float*) &pqb[1].u, (float*) &pqb[1].v,
         (float*) &pqb[2].u, (float*) &pqb[2].v,
         (float*) &pqb[3].u, (float*) &pqb[3].v );
         if ( 0 == scanf_ret )
         {
            error = ROX_ERROR_EXTERNAL;
            goto on_error;
         }

         for ( i = 0; i<4; i++ )
         {
            error = rox_dynvec_point2d_float_append( qb, &(pqb[i]) );
            if (error) goto on_error;
         }
      }
   }

   /* Set ptg */
   error = rox_array2d_double_set_value( ptg, 0, 0, dpTg_in[0][3] ); if (error) goto on_error;
   error = rox_array2d_double_set_value( ptg, 1, 0, dpTg_in[1][3] ); if (error) goto on_error;
   error = rox_array2d_double_set_value( ptg, 2, 0, dpTg_in[2][3] ); if (error) goto on_error;


   printf("========================\n");
   printf("======== Inputs ========\n");
   printf("========================\n");

   printf("Input cTb:    \n"); error = rox_matrix_print( cTb    ); printf("\n"); if (error) goto on_error;
   printf("Input cTs:    \n"); error = rox_matrix_print( cTs    ); printf("\n"); if (error) goto on_error;
   printf("Input pTs_in: \n"); error = rox_matrix_print( pTs_in ); printf("\n"); if (error) goto on_error;
   printf("Input pTg_in: \n"); error = rox_matrix_print( pTg_in ); printf("\n"); if (error) goto on_error;
   printf("Input gTb_in: \n"); error = rox_matrix_print( gTb_in ); printf("\n"); if (error) goto on_error;
   printf("Input ptg:    \n"); error = rox_matrix_print( ptg    ); printf("\n"); if (error) goto on_error;
   printf("Input zg_p:   \n"); error = rox_matrix_print( zg_p   ); printf("\n"); if (error) goto on_error;

   // Initialize/compute transformations
   if ( free_G )
   {
      error = rox_init_se3_so3z_so3z_free_G( bTg, gTp, pTs, cTb, cTs, ptg, zg_p, dist_zs_p, dist_zb_g, 0 );
      if (error) goto on_error;
   }
   else
   {
      rox_init_se3_so3z_so3z_fixed_G( bTg, pTs, cTb, cTs, pTg_in, dist_zs_p, dist_zb_g, 0 );
      if (error) goto on_error;

      error = rox_array2d_double_svdinverse( gTp, pTg_in );  if (error) goto on_error;
   }

   printf("========================\n");
   printf("========= Inits ========\n");
   printf("========================\n");
   printf("init bTg: \n"); error = rox_array2d_double_print( bTg ); printf("\n"); if (error) goto on_error;
   printf("init gTp: \n"); error = rox_array2d_double_print( gTp ); printf("\n"); if (error) goto on_error;
   printf("init pTs: \n"); error = rox_array2d_double_print( pTs ); printf("\n"); if (error) goto on_error;

   printf("========================\n");
   printf("====== Check inits =====\n");
   printf("========================\n");
   Rox_Array2D_Double pTb_in=NULL, bTp=NULL, sTp=NULL, pTp=NULL, sTs=NULL, bTb=NULL;
   error = rox_array2d_double_new( &pTb_in, 4, 4 );                                if (error) goto on_error;
   error = rox_array2d_double_new( &bTp,    4, 4 );                                if (error) goto on_error;
   error = rox_array2d_double_new( &sTp,    4, 4 );                                if (error) goto on_error;
   error = rox_array2d_double_new( &pTp,    4, 4 );                                if (error) goto on_error;
   error = rox_array2d_double_new( &bTb,    4, 4 );                                if (error) goto on_error;
   error = rox_array2d_double_new( &sTs,    4, 4 );                                if (error) goto on_error;
   error = rox_array2d_double_mulmatmat( pTb_in, pTg_in, gTb_in );                 if (error) goto on_error;
   error = rox_array2d_double_mulmatmat( bTp, bTg, gTp );                          if (error) goto on_error;
   error = rox_array2d_double_svdinverse( sTp, pTs );                              if (error) goto on_error;
   error = rox_array2d_double_mulmatmat( pTp, pTb_in, bTp );                       if (error) goto on_error;
   error = rox_array2d_double_mulmatmat( sTs, sTp, pTs_in );                       if (error) goto on_error;

   printf("check sTp estimated * pTs_input: \n");
   error = rox_array2d_double_print( sTs );
   printf("\n"); if (error) goto on_error;

   printf("check pTb_input * bTp estimated: \n");
   error = rox_array2d_double_print( pTp );
   printf("\n"); if (error) goto on_error;

   if ( !free_G )
   {
      error = rox_array2d_double_mulmatmat( bTb, bTg, gTb_in );
      if (error) goto on_error;

      printf("check bTg estimated * gTb_input: \n");
      error = rox_array2d_double_print( bTb );
      if (error) goto on_error;
   }

   printf("------------------------\n");
   if ( free_G )
      printf("G's X and Y axes were free\n");
   else
      printf("G's X and Y axes were fixed\n");
   printf("------------------------\n\n");


   printf("========================\n");
   printf("======== Refined =======\n");
   printf("========================\n");

   /* Convert pixel units distance treshold to normalized units */
   error = conv_dist_pixel_to_normalized( &norm_tresh, pix_tresh, K );
   if (error) goto on_error;
   norm_tresh=10.0;

   error = rox_array2d_double_copy( cTb_ref, cTb_pert ); if (error) goto on_error;

   error = rox_points_float_refine_pose_vvs_se3_so3z_so3z( cTb_ref, bTg, pTs, gTp, qb, mb, qs, ms, norm_tresh );
   if(error) goto on_error;

   printf("cTb_perturbated: \n");
   error = rox_matrix_print( cTb_pert );
   printf("\n"); if(error) goto on_error;

   printf("cTb_input: \n");
   error = rox_matrix_print( cTb );
   printf("\n"); if(error) goto on_error;

   printf("cTb_refined: \n");
   error = rox_matrix_print( cTb_ref );
   printf("\n"); if(error) goto on_error;

on_error:
   rox_array2d_double_del( &sTs );
   rox_array2d_double_del( &pTp );
   rox_array2d_double_del( &sTp );
   rox_array2d_double_del( &bTp );
   rox_array2d_double_del( &pTb_in );
   rox_array2d_double_del( &gTb_in );
   rox_array2d_double_del( &pTg_in );
   rox_array2d_double_del( &pTs_in );
   rox_dynvec_point2d_float_del( &qb );
   rox_dynvec_point2d_float_del( &qs );
   rox_dynvec_point3d_float_del( &mb );
   rox_dynvec_point3d_float_del( &ms );
   rox_array2d_double_del( &zg_p );
   rox_array2d_double_del( &ptg );
   rox_array2d_double_del( &bTg );
   rox_array2d_double_del( &gTp );
   rox_array2d_double_del( &pTs );
   rox_array2d_double_del( &cTs_ref );
   rox_array2d_double_del( &cTb_ref );
   rox_array2d_double_del( &cTs_pert );
   rox_array2d_double_del( &cTb_pert );
   rox_array2d_double_del( &cTs );
   rox_array2d_double_del( &cTb );
   rox_array2d_double_del( &K );

   return error;
}
