//==============================================================================
//
//    OPENROX   : File interaction_point2d_nor_matse3_matso3z_r2.c
//
//    Contents  : Implementation of interaction_point2d_nor_matse3_matso3z_r2
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <baseproc/calculus/jacobians/interaction_point2d_nor_matse3_matso3z_r2.h>

#include <generated/dynvec_double_struct.h>

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point2d_double_struct.h>

#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>

#include <baseproc/geometry/point/dynvec_point2d_projection_from_dynvec_point3d.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_interaction_matse3_matso3z_r2_point2d_nor (
   Rox_Matrix Lo,
   Rox_Matrix Lb,
   const Rox_MatSE3 cTo,
   const Rox_DynVec_Point3D_Double mo
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double ** Lo_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Lo_data, Lo);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Lb_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Lb_data, Lb);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** cTo_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &cTo_data, cTo);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint nb_points = mo->used;

   Rox_DynVec_Point2D_Double qc = NULL;
   Rox_DynVec_Double z = NULL;

   Rox_Matrix calib = NULL;

   error = rox_dynvec_point2d_double_new ( &qc, nb_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_usecells( qc, nb_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_new ( &z, nb_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_usecells( z, nb_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &calib , 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_set_unit ( calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_matse3_print( cTo );
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // Projects points
   error = rox_dynvec_point2d_double_transform_project (qc, z, calib, cTo, mo);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_dynvec_point2d_double_print ( qc );
   // ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint k = 0; k < nb_points; k++)
   {
      Rox_Double zc = z->data[k];

      Rox_Double xc = qc->data[k].u;
      Rox_Double yc = qc->data[k].v;

      Rox_Double Xo = mo->data[k].X;
      Rox_Double Yo = mo->data[k].Y;

      // Rox_Double Zo = mo->data[k].Z;

      Lo_data[2*k ][0] = -1/zc ;
      Lo_data[2*k ][1] =  0 ;
      Lo_data[2*k ][2] = xc/zc ;
      Lo_data[2*k ][3] = xc*yc ;
      Lo_data[2*k ][4] = - xc*xc - 1 ;
      Lo_data[2*k ][5] = yc ;

      Lo_data[2*k+1][0] =  0 ;
      Lo_data[2*k+1][1] = -1/zc ;
      Lo_data[2*k+1][2] = yc/zc ;
      Lo_data[2*k+1][3] = yc*yc + 1 ;
      Lo_data[2*k+1][4] = -xc*yc ;
      Lo_data[2*k+1][5] = -xc ;

      Lb_data[2*k  ][0] = ( cTo_data[2][0]*xc - cTo_data[0][0] )/zc ;
      Lb_data[2*k  ][1] = ( cTo_data[2][1]*xc - cTo_data[0][1] )/zc ;
      Lb_data[2*k  ][2] = Yo*(cTo_data[0][0] - cTo_data[2][0]*xc)/zc - Xo*(cTo_data[0][1] - cTo_data[2][1]*xc)/zc ;

      Lb_data[2*k+1][0] = ( cTo_data[2][0]*yc - cTo_data[1][0] )/zc ;
      Lb_data[2*k+1][1] = ( cTo_data[2][1]*yc - cTo_data[1][1] )/zc ;
      Lb_data[2*k+1][2] = Yo*(cTo_data[1][0] - cTo_data[2][0]*yc)/zc - Xo*(cTo_data[1][1] - cTo_data[2][1]*yc)/zc ;
   }

function_terminate:

   rox_dynvec_point2d_double_del ( &qc );
   rox_dynvec_double_del ( &z );
   rox_matrix_del ( &calib );

   return error;
}