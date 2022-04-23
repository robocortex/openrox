//============================================================================
//
//    OPENROX   : File init_vvs_se3_so3z_so3z.c
//
//    Contents  : Implementation to estimate some specific transformations between four
//                frames. Initially used for the APRA project.
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "init_vvs_se3_so3z_so3z.h"

#include <baseproc/maths/maths_macros.h>
#include <generated/array2d_double.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/solve/svd_solve.h>
#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/maths/linalg/generators/algso3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matso3.h>
#include <baseproc/array/fill/fillzero.h>
#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_matso3_optimal_rz_from_vectors ( Rox_Array2D_Double R, Rox_Array2D_Double vo, Rox_Array2D_Double vi )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double rz=0.0, rzpi=0.0;
   Rox_Array2D_Double Rz=NULL, Rzpi=NULL;
   Rox_Array2D_Double rz_vec=NULL, rzpi_vec=NULL;
   Rox_Array2D_Double rz_alg=NULL, rzpi_alg=NULL;
   Rox_Array2D_Double rz_vi=NULL, rzpi_vi=NULL;
   Rox_Array2D_Double vo_m_rz_vi=NULL, vo_m_rzpi_vi=NULL;
   Rox_Double **dzero=NULL, **dpi=NULL;

   if ( !R || !vo || !vi ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size( R, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_check_size( vo, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_check_size( vi, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)

   Rox_Double ** dvo = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dvo, vo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dvi = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dvi, vi );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!dvo || !dvi) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // rz vectors
   error = rox_array2d_double_new( &rz_vec, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &rzpi_vec, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval( rz_vec, 0.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval( rzpi_vec, 0.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // rz algebra
   error = rox_array2d_double_new( &rz_alg, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &rzpi_alg, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // rz_vi
   error = rox_array2d_double_new( &rz_vi, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &rzpi_vi, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // vo_m_rz_vi
   error = rox_array2d_double_new( &vo_m_rz_vi, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &vo_m_rzpi_vi, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &Rz, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &Rzpi , 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dzero, vo_m_rz_vi );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dpi, vo_m_rzpi_vi );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!dzero || !dpi) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rz = atan( (dvi[0][0]*dvo[1][0] - dvi[1][0]*dvo[0][0]) / (dvi[0][0]*dvo[0][0] + dvi[1][0]*dvo[1][0]) ) ;
   rzpi = rz + 3.14159265358979323846;

   error = rox_array2d_double_set_value( rz_vec, 2, 0, (Rox_Double) rz );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_set_value( rzpi_vec, 2, 0, (Rox_Double) rzpi );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_linalg_so3generator( rz_alg, rz_vec );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_expmat_so3( Rz, rz_alg );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_linalg_so3generator( rzpi_alg, rzpi_vec );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_expmat_so3( Rzpi, rzpi_alg );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat( rz_vi, Rz, vi );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_mulmatmat( rzpi_vi, Rzpi, vi );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_substract( vo_m_rz_vi, vo, rz_vi );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_substract( vo_m_rzpi_vi, vo, rzpi_vi );
   ROX_ERROR_CHECK_TERMINATE(error)

   if ( ( dzero[0][0]*dzero[0][0] + dzero[1][0]*dzero[1][0] + dzero[2][0]*dzero[2][0] )
      < ( dpi[0][0]*dpi[0][0] + dpi[1][0]*dpi[1][0] + dpi[2][0]*dpi[2][0] ) )
   {
      error = rox_array2d_double_copy( R, Rz );
      ROX_ERROR_CHECK_TERMINATE(error)
   }
   else
   {
      error = rox_array2d_double_copy( R, Rzpi );
      ROX_ERROR_CHECK_TERMINATE(error)
   }

function_terminate:
   ROX_ERROR_CHECK( rox_array2d_double_del( &rz_vec ))
   ROX_ERROR_CHECK( rox_array2d_double_del( &rzpi_vec ))
   ROX_ERROR_CHECK( rox_array2d_double_del( &rz_alg ))
   ROX_ERROR_CHECK( rox_array2d_double_del( &rzpi_alg ))
   ROX_ERROR_CHECK( rox_array2d_double_del( &rz_vi ))
   ROX_ERROR_CHECK( rox_array2d_double_del( &rzpi_vi ))
   ROX_ERROR_CHECK( rox_array2d_double_del( &vo_m_rz_vi ))
   ROX_ERROR_CHECK( rox_array2d_double_del( &vo_m_rzpi_vi ))
   ROX_ERROR_CHECK( rox_array2d_double_del( &Rz ))
   ROX_ERROR_CHECK( rox_array2d_double_del( &Rzpi ))

   return error;
}


Rox_ErrorCode rox_matso3_optimal_rz( Rox_Array2D_Double Rz, Rox_Array2D_Double R )
{
   Rox_ErrorCode  error = ROX_ERROR_NONE;

   // Check inputs
   if ( !Rz || !R )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_check_size( Rz, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size( R, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get access
   Rox_Double ** dR = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dR,  R );
   ROX_ERROR_CHECK_TERMINATE ( error );


   // Estimate the closest rotation Rz to R
   Rox_Double ca = ( dR[0][0] + dR[1][1] )/2.0;
   Rox_Double sa = ( dR[1][0] - dR[0][1] )/2.0;

   error = rox_array2d_double_fillunit( Rz );       
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( Rz, 0, 0,  ca ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( Rz, 0, 1, -sa ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( Rz, 1, 0,  sa ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( Rz, 1, 1,  ca ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_init_se3_so3z_so3z_free_G(
   Rox_Array2D_Double bTg,
   Rox_Array2D_Double gTp,
   Rox_Array2D_Double pTs,
   Rox_Array2D_Double cTb,
   Rox_Array2D_Double cTs,
   Rox_Array2D_Double ptg_mod,
   Rox_Array2D_Double zg_p,
   Rox_Double         dzs_p,
   Rox_Double         dzb_g,
   Rox_Sint           force_model )
{
   // Declarations
   Rox_ErrorCode      error=ROX_ERROR_NONE;
   Rox_Array2D_Double sTc=NULL;
   Rox_Array2D_Double sTb=NULL, sRb=NULL, stb=NULL;
   Rox_Array2D_Double sTp=NULL, sRp=NULL, stp=NULL;
   Rox_Array2D_Double sTg=NULL, sRg=NULL, stg=NULL, stg_4x1=NULL;
   Rox_Array2D_Double pTg=NULL, pRg=NULL, ptg=NULL;
   Rox_Array2D_Double gTs=NULL;
   Rox_Array2D_Double gTb=NULL;
   Rox_Array2D_Double bTg_temp=NULL, bRg_temp=NULL;
   Rox_Array2D_Double bRg=NULL, btg=NULL, btg_4x1=NULL;
   Rox_Array2D_Double zp_p=NULL;
   Rox_Array2D_Double zg_g=NULL;
   Rox_Array2D_Double sTb_cor=NULL, sRb_cor=NULL, stb_cor4x1=NULL;
   Rox_Array2D_Double zg_s=NULL;
   Rox_Array2D_Double ptb_mod=NULL, ptb_mod4x1=NULL, ptb=NULL;
   Rox_Array2D_Double gtb_p=NULL;
   Rox_Array2D_Double pRs=NULL;

   // Check inputs
   if ( NULL == bTg || NULL == gTp || NULL == pTs || NULL == cTb || NULL == cTs || NULL == ptg_mod || NULL == zg_p )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size( bTg, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( gTp, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( pTs, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( cTb, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( cTs, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( ptg_mod, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( zg_p, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)

   // Allocations
   error = rox_array2d_double_new( &sTc, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // sTb
   error = rox_array2d_double_new( &sTb, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &sRb, sTb, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &stb, sTb, 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // sTp
   error = rox_array2d_double_new( &sTp, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &sRp, sTp, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &stp, sTp, 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // sTg
   error = rox_array2d_double_new( &sTg, 4, 4);
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &sRg, sTg, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &stg, sTg, 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &stg_4x1, sTg, 0, 3, 4, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // gTs
   error = rox_array2d_double_new( &gTs, 4, 4);
   ROX_ERROR_CHECK_TERMINATE(error)
   // gTb
   error = rox_array2d_double_new( &gTb, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &gtb_p, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // bTg_temp
   error = rox_array2d_double_new( &bTg_temp, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &bRg_temp, bTg_temp, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // bTg (input)
   error = rox_array2d_double_new_subarray2d( &bRg, bTg, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &btg, bTg, 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &btg_4x1, bTg, 0, 3, 4, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // zp_p, zg_g, zg_s
   error = rox_array2d_double_new( &zp_p, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &zg_g, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &zg_s, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // sTb_cor
   error = rox_array2d_double_new( &sTb_cor, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &sRb_cor, sTb_cor, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &stb_cor4x1, sTb_cor, 0, 3, 4, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // ptb_mod and ptb
   error = rox_array2d_double_new( &ptb_mod4x1, 4, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &ptb_mod, ptb_mod4x1, 0, 0, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &ptb       , 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // pTg
   error = rox_array2d_double_new( &pTg, 4, 4);
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &pRg, pTg, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new_subarray2d( &ptg, pTg, 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   // pRs
   error = rox_array2d_double_new_subarray2d( &pRs, pTs, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   //------------------
   // Core estimations
   //------------------

   // Matrices initializations
   error = rox_array2d_double_fillunit( sTc );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit( sTb );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit( sTp );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit( sTg );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit( gTs );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit( gTb );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit( bTg_temp );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit( bTg );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit( sTb_cor );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillval( ptb_mod4x1, 0.0 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_set_value( ptb_mod4x1, 3, 0,  1.0 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit( pTg );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit( pTs );
   ROX_ERROR_CHECK_TERMINATE(error)

   //--- zp_p
   error = rox_array2d_double_fillval( zp_p, 0.0 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_set_value( zp_p, 2, 0, 1.0 );
   ROX_ERROR_CHECK_TERMINATE(error)

   //--- zg_g, btg. Assumes bRg is pure around Z axis
   error = rox_array2d_double_fillval( zg_g, 0.0 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_set_value( zg_g, 2, 0, 1.0 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_scale( btg, zg_g, dzb_g );
   ROX_ERROR_CHECK_TERMINATE(error)

   //--- sTb
   error = rox_array2d_double_svdinverse( sTc, cTs );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_mulmatmat( sTb, sTc, cTb );
   ROX_ERROR_CHECK_TERMINATE(error)

   //--- sTp, assumes sRp is pure around Z axis
   error = rox_array2d_double_scale( stp, zp_p, dzs_p );
   ROX_ERROR_CHECK_TERMINATE(error)
   //     sTb = sTp * pTb
   //  => stb = sRp * ptb + stp
   // <=> stb - stp = sRp * ptb
   //
   error = rox_array2d_double_scale(  gtb_p , zg_p   , -1.0*dzb_g );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_add(   ptb_mod, ptg_mod, gtb_p      );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_substract( ptb, stb    , stp        );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_matso3_optimal_rz_from_vectors( sRp, ptb, ptb_mod );
   ROX_ERROR_CHECK_TERMINATE(error)

   //--- pTs
   error = rox_array2d_double_svdinverse( pTs, sTp );
   ROX_ERROR_CHECK_TERMINATE(error)

   // Enforce the model: correct stb using sRp and ptb_mod
   if ( 1 == force_model )
   {
      // stb  = sRp*ptb + stp
      error = rox_array2d_double_fillunit( sTb_cor );
      ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_copy( sRb_cor, sRb );
      ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_mulmatmat( stb_cor4x1, sTp, ptb_mod4x1 );
      ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_copy( sTb, sTb_cor );
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   //--- sTg => bTg
   error = rox_array2d_double_mulmatmat( stg_4x1, sTb, btg_4x1 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_mulmatmat( zg_s, sRp, zg_p );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_matso3_from_2_vectors( sRg, stg, zg_s );
   ROX_ERROR_CHECK_TERMINATE(error)

   // gTs
   error = rox_array2d_double_svdinverse( gTs, sTg );
   ROX_ERROR_CHECK_TERMINATE(error)

   // gTb
   error = rox_array2d_double_mulmatmat( gTb, gTs, sTb );
   ROX_ERROR_CHECK_TERMINATE(error)

   // bTg
   error = rox_array2d_double_svdinverse( bTg_temp, gTb );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_matso3_optimal_rz( bRg, bRg_temp );
   ROX_ERROR_CHECK_TERMINATE(error)

   //--- gTp
   if ( 1 == force_model )
   {
      error = rox_array2d_double_mulmatmat( pRg, pRs, sRg );
      ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_copy( ptg, ptg_mod );
      ROX_ERROR_CHECK_TERMINATE(error)
   }
   else
   {
      error = rox_array2d_double_mulmatmat( pTg, pTs, sTg );
      ROX_ERROR_CHECK_TERMINATE(error)
   }
   error = rox_array2d_double_svdinverse( gTp, pTg );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   rox_array2d_double_del( &pRs        );
   rox_array2d_double_del( &ptg        );
   rox_array2d_double_del( &pRg        );
   rox_array2d_double_del( &pTg        );
   rox_array2d_double_del( &ptb        );
   rox_array2d_double_del( &ptb_mod    );
   rox_array2d_double_del( &ptb_mod4x1 );
   rox_array2d_double_del( &stb_cor4x1 );
   rox_array2d_double_del( &sRb_cor    );
   rox_array2d_double_del( &sTb_cor    );
   rox_array2d_double_del( &zg_s       );
   rox_array2d_double_del( &zg_g       );
   rox_array2d_double_del( &zp_p       );
   rox_array2d_double_del( &btg_4x1    );
   rox_array2d_double_del( &btg        );
   rox_array2d_double_del( &bRg        );
   rox_array2d_double_del( &bRg_temp   );
   rox_array2d_double_del( &bTg_temp   );
   rox_array2d_double_del( &gtb_p      );
   rox_array2d_double_del( &gTb        );
   rox_array2d_double_del( &gTs        );
   rox_array2d_double_del( &stg_4x1    );
   rox_array2d_double_del( &stg        );
   rox_array2d_double_del( &sRg        );
   rox_array2d_double_del( &sTg        );
   rox_array2d_double_del( &stp        );
   rox_array2d_double_del( &sRp        );
   rox_array2d_double_del( &sTp        );
   rox_array2d_double_del( &stb        );
   rox_array2d_double_del( &sRb        );
   rox_array2d_double_del( &sTb        );
   rox_array2d_double_del( &sTc        );
   return error;
}

Rox_ErrorCode rox_init_se3_so3z_so3z_fixed_G(
   Rox_Array2D_Double bTg,
   Rox_Array2D_Double pTs,
   Rox_Array2D_Double cTb,
   Rox_Array2D_Double cTs,
   Rox_Array2D_Double pTg,
   Rox_Double         dzs_p,
   Rox_Double         dzb_g,
   Rox_Sint           force_model )
{
   Rox_ErrorCode      error=ROX_ERROR_NONE;
   Rox_Array2D_Double bTg_free=NULL, gTp_free=NULL;
   Rox_Array2D_Double ptg=NULL, zg_p=NULL;
   Rox_Array2D_Double bTc=NULL, bTs=NULL, sTp=NULL, sTg=NULL;
   Rox_Double         **dpTg=NULL;
   Rox_Array2D_Double btg_free=NULL, btg=NULL;

   // Check input
   if ( NULL == bTg || NULL == pTs || NULL == cTb || NULL == cTs || NULL == pTg )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size( bTg, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( pTs, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( cTb, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( cTs, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( pTg, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE(error)

   // Allocate memory
   error = rox_array2d_double_new( &bTg_free, 4, 4 );   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &gTp_free, 4, 4 );   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &ptg     , 3, 1 );   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &zg_p    , 3, 1 );   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &bTc     , 4, 4 );   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &bTs     , 4, 4 );   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &sTp     , 4, 4 );   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &sTg     , 4, 4 );   
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &btg_free, bTg_free, 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &btg     , bTg     , 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Extract ptg and zg_p from pTg
   error  = rox_array2d_double_get_data_pointer_to_pointer( &dpTg, pTg );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( ptg , 0, 0, dpTg[0][3] );  
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( ptg , 1, 0, dpTg[1][3] );  
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( ptg , 2, 0, dpTg[2][3] );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( zg_p, 0, 0, dpTg[0][2] );  
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( zg_p, 1, 0, dpTg[1][2] );  
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( zg_p, 2, 0, dpTg[2][2] );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get a first estimation with free rotation
   error = rox_init_se3_so3z_so3z_free_G( bTg_free, gTp_free, pTs, cTb, cTs, ptg, zg_p, dzs_p, dzb_g, force_model );
   ROX_ERROR_CHECK_TERMINATE(error)

   // Enforce the known pTg
   // bTg = bTc * cTs * sTp * pTg
   error = rox_array2d_double_svdinverse( bTc, cTb );      
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_svdinverse( sTp, pTs );      
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat( bTs, bTc, cTs );  
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat( sTg, sTp, pTg );  
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat( bTg, bTs, sTg );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   // btg is now messed up, however btg_free is the "good" translation
   error = rox_array2d_double_copy( btg, btg_free );       
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del( &btg      );
   rox_array2d_double_del( &btg_free );
   rox_array2d_double_del( &bTg_free );
   rox_array2d_double_del( &gTp_free );
   rox_array2d_double_del( &ptg      );
   rox_array2d_double_del( &zg_p     );
   rox_array2d_double_del( &bTc      );
   rox_array2d_double_del( &bTs      );
   rox_array2d_double_del( &sTp      );
   rox_array2d_double_del( &sTg      );

   return error;
}


Rox_ErrorCode rox_compute_jacobian_hessian(Rox_Array2D_Double J, Rox_Array2D_Double H, Rox_Array2D_Double xn, Rox_Array2D_Double xd, Rox_Array2D_Double xr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double * xn_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &xn_data, xn);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * xd_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &xd_data, xd);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * xr_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &xr_data, xr);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double *  J_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &J_data, J );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** H_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &H_data, H );
   ROX_ERROR_CHECK_TERMINATE ( error );

   //rox_array2d_double_print(xn);
   //rox_array2d_double_print(xd);
   //rox_array2d_double_print(xr);

   Rox_Double t1 = xn_data[0];
   Rox_Double t2 = xn_data[1];
   Rox_Double r1 = xn_data[2];

   Rox_Double td1 = xd_data[0];
   Rox_Double td2 = xd_data[1];

   Rox_Double tr1 = xr_data[0];
   Rox_Double tr2 = xr_data[1];

   // Jacobian
   J_data[0] = 2*t1 - tr1 + td1*cos(r1) + td2*sin(r1);
   J_data[1] = 2*t2 - tr2 + td2*cos(r1) - td1*sin(r1);
   J_data[2] = (td2*cos(r1) - td1*sin(r1))*(t1 - tr1 + td1*cos(r1) + td2*sin(r1)) - (td1*cos(r1) + td2*sin(r1))*(t2 - tr2 + td2*cos(r1) - td1*sin(r1));

   // Hessian
   H_data[0][0] = 2.0;
   H_data[0][1] = 0.0;
   H_data[0][2] = +td2*cos(r1) - td1*sin(r1) ;
   H_data[1][0] = 0.0;
   H_data[1][1] = 2.0;
   H_data[1][2] = -td1*cos(r1) - td2*sin(r1) ;
   H_data[2][0] = +td2*cos(r1) - td1*sin(r1);
   H_data[2][1] = -td1*cos(r1) - td2*sin(r1);
   H_data[2][2] = (td1*tr1 -t2*td2 -t1*td1 + td2*tr2)*cos(r1) + (t2*td1 -t1*td2* -td1*tr2 + td2*tr1)*sin(r1);

   //rox_array2d_double_print(J);
   //rox_array2d_double_print(H);

function_terminate:
   return error;
}

Rox_ErrorCode rox_compute_pose(Rox_Array2D_Double T, Rox_Array2D_Double x)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double * x_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &x_data, x );
   Rox_Double rot[3];
   Rox_Double tra[3];

   tra[0] = x_data[0]; tra[1] =  0.0; tra[2] = x_data[1];
   rot[0] =  0.0; rot[1] = x_data[2]; rot[2] =  0.0;

   error = rox_matse3_set_rot_tra(T, rot, tra);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_compute_vect(Rox_Array2D_Double x, Rox_Array2D_Double T)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double axis_x = 0.0, axis_y = 0.0, axis_z = 0.0;
   Rox_Double angle = 0.0;

   if (!x || !T)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** T_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T_data, T );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * x_data = NULL;
   error = rox_array2d_double_get_data_pointer( &x_data, x);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Array2D_Double R = NULL;

   error = rox_array2d_double_new_subarray2d(&R, T, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_logmat_so3(&axis_x, &axis_y, &axis_z, &angle, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   x_data[0] = T_data[0][3];
   x_data[1] = T_data[2][3];
   x_data[2] = angle*axis_y;

function_terminate:
   rox_array2d_double_del(&R);
   return error;
}

Rox_ErrorCode rox_update_vect(Rox_Array2D_Double xn, Rox_Array2D_Double dx, Rox_Double lambda)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_array2d_double_scale(dx, dx, -lambda);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_add(xn, xn, dx);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}


Rox_ErrorCode rox_init_se3_model_adjustment(Rox_Array2D_Double pTpn, Rox_Array2D_Double pTb_mes, Rox_Array2D_Double pTb_mod)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double xn = NULL;
   Rox_Array2D_Double xd = NULL;
   Rox_Array2D_Double xr = NULL;
   Rox_Array2D_Double Jn = NULL;
   Rox_Array2D_Double Hn = NULL;
   Rox_Array2D_Double dx = NULL;

   Rox_Double lambda = 0.9;

   error = rox_array2d_double_new(&xn, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&xd, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&xr, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&dx, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&Jn, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&Hn, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_compute_vect(xd, pTb_mes);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_compute_vect(xr, pTb_mod);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_fillzero(xn);
   ROX_ERROR_CHECK_TERMINATE(error)

   //rox_array2d_double_print(xn);
   //rox_array2d_double_print(xd);
   //rox_array2d_double_print(xr);

   for ( Rox_Sint k = 0; k < 100; k++)
   {
      error = rox_compute_jacobian_hessian(Jn, Hn, xn, xd, xr);
      ROX_ERROR_CHECK_TERMINATE(error)

      // solve system dx = pinv(Hn)*Jn
      error = rox_svd_solve(dx, Hn, Jn);
      ROX_ERROR_CHECK_TERMINATE(error)

      // update xn = xn - lambda * dx ;
      error = rox_update_vect(xn, dx, lambda);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_compute_pose(pTpn, xn);
      ROX_ERROR_CHECK_TERMINATE(error)

      // compute current for error check
      // Tc = T * Td;
      // xc = compute_vect(Tc)
   }

function_terminate:

   error = rox_array2d_double_del(&xn);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_del(&xd);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_del(&xr);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_del(&dx);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_del(&Jn);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_del(&Hn);
   ROX_ERROR_CHECK_TERMINATE(error)

   return error;
}

Rox_ErrorCode rox_matso3_from_2_vectors(Rox_Array2D_Double Rz, Rox_Array2D_Double v, Rox_Array2D_Double z)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double skew_z = NULL, skew_y = NULL;
   Rox_Array2D_Double y = NULL, x = NULL;
   Rox_Double norm_y = 0.0, norm_x = 0.0;

   Rox_Double **dR = NULL, **dz = NULL, **dy = NULL, **dx = NULL;

   // Check inputs
   if ( !Rz || !v || !z )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size( Rz, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size( v, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size( z, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate
   error = rox_array2d_double_new( &skew_z, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new( &skew_y, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new( &y, 3, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new( &x, 3, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Access y and x
   error = rox_array2d_double_get_data_pointer_to_pointer( &dR, Rz );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dz, z );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dy, y );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dx, x );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( NULL == dx || NULL == dy || NULL == dz || NULL == dR )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // The stuff
   error = rox_transformtools_skew_from_vector( skew_z, z );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat( y, skew_z, v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   norm_y = sqrt( dy[0][0]*dy[0][0] + dy[1][0]*dy[1][0] + dy[2][0]*dy[2][0] );
   error = rox_array2d_double_scale( y, y, 1.0/norm_y );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_skew_from_vector( skew_y, y );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat( x, skew_y, z );
   ROX_ERROR_CHECK_TERMINATE ( error );

   norm_x = sqrt( dx[0][0]*dx[0][0] + dx[1][0]*dx[1][0] + dx[2][0]*dx[2][0] );

   error = rox_array2d_double_scale( x, x, 1.0/norm_x );
   ROX_ERROR_CHECK_TERMINATE ( error );

   dR[0][0] = dx[0][0];
   dR[1][0] = dx[1][0];
   dR[2][0] = dx[2][0];

   dR[0][1] = dy[0][0];
   dR[1][1] = dy[1][0];
   dR[2][1] = dy[2][0];

   dR[0][2] = dz[0][0];
   dR[1][2] = dz[1][0];
   dR[2][2] = dz[2][0];

function_terminate:

   rox_array2d_double_del( &skew_z );
   rox_array2d_double_del( &skew_y );
   rox_array2d_double_del( &y );
   rox_array2d_double_del( &x );

   return error;
}
