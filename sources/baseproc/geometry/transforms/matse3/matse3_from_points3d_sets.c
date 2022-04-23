//==============================================================================
//
//    OPENROX   : File matse3_from_points3d_sets.c
//
//    Contents  : Implementation of matse3 from point3D sets module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "matse3_from_points3d_sets.h"

#include <math.h>

#include <generated/array2d_double.h>
#include <generated/dynvec_double.h>
#include <generated/objset_array2d_double_struct.h>

#include <baseproc/array/add/add.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/determinant/detgl3.h>
#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/transpose/transpose.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/point/point3d_tools.h>
#include <baseproc/geometry/point/dynvec_point3d_tools.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>

#include <inout/system/errors_print.h>

// References:
//  . github/rox_mat/modules/maths/pose_from_3D_to_3D_points.m
//  . "Least-squares estimation of transformation parameters between two point patterns" S. Umeyama, PAMI, 1991
// Although this is not exactly the same problem
Rox_ErrorCode rox_matse3_from_dynvec_points3d_double (
  Rox_MatSE3                bTa,
  const Rox_DynVec_Point3D_Double a_points,
  const Rox_DynVec_Point3D_Double b_points 
)
{
   Rox_ErrorCode               error = ROX_ERROR_NONE;
   Rox_Uint                    n_a=0, n_b=0;
   Rox_DynVec_Point3D_Double   a_pts=NULL, b_pts=NULL;
   Rox_Matrix                  bRa=NULL;
   Rox_Matrix                  As=NULL, Bs=NULL;
   Rox_Matrix                  U=NULL, S=NULL, V=NULL;
   Rox_Double                  det=1.0;
   Rox_Point3D_Double_Struct   b_mean, b_r_mean;

   // Check inputs
   if ( NULL == bTa || NULL == a_points || NULL == b_points )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point3d_double_get_used( &n_a, a_points );          
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_get_used( &n_b, b_points );          
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( n_a != n_b ) 
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( n_a < 3 )    
      { error = ROX_ERROR_INSUFFICIENT_DATA; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Allocations
   error = rox_array2d_double_new_subarray2d( &bRa, bTa, 0, 0, 3, 3 );    
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_new( &a_pts, n_a );                  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_new( &b_pts, n_b );                  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new( &As, 3 , n_a );                                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new( &Bs, 3 , n_b );                                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new( &U , 3 , 3 );                                  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new( &S , 3 , 1 );                                  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new( &V , 3 , 3 );                                  
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** bTa_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &bTa_data, bTa );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** V_data=NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &V_data, V );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Prepare bTa
   error = rox_matse3_set_unit( bTa );                                    
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute bRa
   error = rox_dynvec_point3d_double_center_normalize( a_pts, a_points ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_center_normalize( b_pts, b_points ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_from_dynvec_point3d_double( As, a_pts );            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_from_dynvec_point3d_double( Bs, b_pts );            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmattrans( bRa, Bs, As );              
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svd( U, S, V, bRa );                        
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmattrans( bRa, U, V );                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_detgl3 ( &det, bRa );                        
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( det < 0.0 )
   {
      V_data[0][0] = -V_data[0][0];
      V_data[0][1] = -V_data[0][1];
      V_data[0][2] = -V_data[0][2];

      error = rox_array2d_double_mulmatmattrans( bRa, U, V );             
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Compute bta
   error = rox_dynvec_point3d_double_mean( &b_mean, b_points );           
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_point3d_double_transform( &b_r_mean, bTa, &b_mean, 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );

   bTa_data[0][3] = - b_r_mean.X;
   bTa_data[1][3] = - b_r_mean.Y;
   bTa_data[2][3] = - b_r_mean.Z;

function_terminate:
   if ( NULL != V     ) rox_matrix_del               ( &V     );
   if ( NULL != S     ) rox_matrix_del               ( &S     );
   if ( NULL != U     ) rox_matrix_del               ( &U     );
   if ( NULL != Bs    ) rox_matrix_del               ( &Bs    );
   if ( NULL != As    ) rox_matrix_del               ( &As    );
   if ( NULL != b_pts ) rox_dynvec_point3d_double_del( &b_pts );
   if ( NULL != a_pts ) rox_dynvec_point3d_double_del( &a_pts );
   if ( NULL != bRa   ) rox_array2d_double_del       ( &bRa   );

   return error;
}

Rox_ErrorCode rox_matse3_from_vector_points3d_double (
  Rox_MatSE3  bTa,
  const Rox_Point3D_Double a_points_vector,
  const Rox_Point3D_Double b_points_vector,
  const Rox_Sint n_points
)
{
   Rox_ErrorCode               error = ROX_ERROR_NONE;

   Rox_DynVec_Point3D_Double   a_points = NULL, b_points = NULL;

   // Check inputs
   if ( !a_points_vector || !b_points_vector )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }


   //rox_vector_point3d_double_print ( a_points_vector, n_points );
   //rox_vector_point3d_double_print ( b_points_vector, n_points );

   error = rox_dynvec_point3d_double_new ( &a_points, n_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_set_data_vector ( a_points, a_points_vector, n_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_new ( &b_points, n_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_set_data_vector ( b_points, b_points_vector, n_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   //rox_dynvec_point3d_double_print ( a_points );
   //rox_dynvec_point3d_double_print ( b_points );

   error = rox_matse3_from_dynvec_points3d_double ( bTa, a_points, b_points );
   ROX_ERROR_CHECK_TERMINATE ( error );


function_terminate:
   rox_dynvec_point3d_double_del( &a_points );
   rox_dynvec_point3d_double_del( &b_points );

   return error;
}
