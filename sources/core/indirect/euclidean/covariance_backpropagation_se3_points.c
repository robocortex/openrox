//==============================================================================
//
//    OPENROX   : File covariance_backpropagation_se3.c
//
//    Contents  : Implementation of covariance_backpropagation_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "covariance_backpropagation_se3_points.h"

#include <float.h>
#include <string.h>

#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <baseproc/array/add/add.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/mad/mad.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/point/point2d_tools.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/dynvec_point2d_tools.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/maths_macros.h>

#include <core/indirect/euclidean/vvspointsse3.h>
#include <baseproc/calculus/linsys/linsys_point2d_pix_matse3_weighted.h>
#include <baseproc/calculus/linsys/linsys_point2d_nor_matse3_weighted.h>

#include <inout/system/errors_print.h>

#define MIN_POINTS_NUMBER       3
#define MAD_SCALE_FACTOR_NORMAL 1.4826

ROX_API Rox_ErrorCode
rox_covariance_se3_points(
  Rox_Array2D_Double               E,
  Rox_Array2D_Double               cTo,
  Rox_Array2D_Double               K,
  Rox_DynVec_Point2D_Double measures,
  Rox_DynVec_Point3D_Double references )
{
   Rox_ErrorCode                    error=ROX_ERROR_NONE;
   Rox_Array2D_Double               sum_Jt_iE_J=NULL;
   Rox_Array2D_Double               Jt_iE_J=NULL;
   Rox_Array2D_Double               Jt_iE=NULL;
   Rox_Array2D_Double               dproj=NULL;
   Rox_Double                     **dproj_data=NULL;
   Rox_Array2D_Double               Gyz=NULL, Gzx=NULL, Gxy=NULL, Gx=NULL, Gy=NULL, Gz=NULL;
   Rox_Array2D_Double               Gyz_cTo_Ps=NULL,   Gzx_cTo_Ps=NULL,   Gxy_cTo_Ps=NULL;
   Rox_Double                     **Gyz_cTo_Ps_data, **Gzx_cTo_Ps_data, **Gxy_cTo_Ps_data;
   Rox_Array2D_Double               Gx_cTo_Ps=NULL,    Gy_cTo_Ps=NULL,    Gz_cTo_Ps=NULL;
   Rox_Double                     **Gx_cTo_Ps_data,  **Gy_cTo_Ps_data,  **Gz_cTo_Ps_data;
   Rox_Array2D_Double               iEmeas=NULL;
   Rox_Array2D_Double               dproj_G_T_P=NULL;
   Rox_Double                     **dproj_G_T_P_data=NULL;
   Rox_Array2D_Double               J=NULL;
   Rox_Double                     **J_data=NULL;
   Rox_Double                     **G_T_Ps_data=NULL;
   Rox_Array2D_Double               G_T_P=NULL;
   Rox_Double                     **G_T_P_data=NULL;
   Rox_Array2D_Double               c_ref=NULL;
   Rox_Double                     **c_ref_data=NULL;
   Rox_Array2D_Double               work1=NULL;
   Rox_Array2D_Double               work2=NULL;
   Rox_Array2D_Double               dist=NULL;
   Rox_Array2D_Double               weights=NULL;
   Rox_Double                     **weights_data=NULL;
   Rox_Array2D_Double               iK=NULL;
   Rox_Array2D_Double               I_3x3=NULL;
   Rox_DynVec_Point3D_Double ref_cam=NULL;
   Rox_DynVec_Point2D_Double ref_proj=NULL;
   Rox_DynVec_Point2D_Double meas_norm=NULL;
   Rox_Uint                         n_pts=0;
   Rox_Uint                         idpt=0, basis=0;
   Rox_Double                         mad=0.0, std_dev=0.0, i_var=0.0;
   Rox_Double                         x=0.0, y=0.0, Z=0.0, iZ=0.0;

   // Check arguments
   if ( NULL == E || NULL == cTo || NULL == K || NULL == measures || NULL == references )
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   n_pts = measures->used;
   if ( n_pts < MIN_POINTS_NUMBER )
   {
      error = ROX_ERROR_BAD_SIZE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if ( references->used != n_pts )
   {
      error = ROX_ERROR_BAD_SIZE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_array2d_double_check_size(   E, 6, 6 );             ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size( cTo, 4, 4 );             ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(   K, 3, 3 );             ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate internal variables
   error = rox_dynvec_point3d_double_new(   &ref_cam, n_pts );     ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point2d_double_new(  &ref_proj, n_pts );     ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point2d_double_new( &meas_norm, n_pts );     ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &sum_Jt_iE_J,     6,     6 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(     &Jt_iE_J,     6,     6 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(       &Jt_iE,     6,     2 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(       &dproj,     2,     4 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &dproj_G_T_P,     2,     1 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(       &G_T_P,     4,     1 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(           &J,     2,     6 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(        &dist, n_pts,     1 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(     &weights, n_pts,     1 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(         &Gyz,     4,     4 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(         &Gzx,     4,     4 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(         &Gxy,     4,     4 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(          &Gx,     4,     4 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(          &Gy,     4,     4 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(          &Gz,     4,     4 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(  &Gyz_cTo_Ps,     4, n_pts );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(  &Gzx_cTo_Ps,     4, n_pts );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(  &Gxy_cTo_Ps,     4, n_pts );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(   &Gx_cTo_Ps,     4, n_pts );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(   &Gy_cTo_Ps,     4, n_pts );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(   &Gz_cTo_Ps,     4, n_pts );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(      &iEmeas,     2,     2 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(       &c_ref,     4, n_pts );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(          &iK,     3,     3 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(       &I_3x3,     3,     3 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(       &work1, n_pts,     1 );   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(       &work2, n_pts,     1 );   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get access to arrays data
   error = rox_array2d_double_get_data_pointer_to_pointer( &c_ref_data, c_ref );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dproj_data, dproj );
   error = rox_array2d_double_get_data_pointer_to_pointer( &G_T_P_data, G_T_P );
   error = rox_array2d_double_get_data_pointer_to_pointer( &Gyz_cTo_Ps_data, Gyz_cTo_Ps );
   error = rox_array2d_double_get_data_pointer_to_pointer( &Gzx_cTo_Ps_data, Gzx_cTo_Ps );
   error = rox_array2d_double_get_data_pointer_to_pointer( &Gxy_cTo_Ps_data, Gxy_cTo_Ps );
   error = rox_array2d_double_get_data_pointer_to_pointer( &Gx_cTo_Ps_data, Gx_cTo_Ps );
   error = rox_array2d_double_get_data_pointer_to_pointer( &Gy_cTo_Ps_data, Gy_cTo_Ps );
   error = rox_array2d_double_get_data_pointer_to_pointer( &Gz_cTo_Ps_data, Gz_cTo_Ps );
   error = rox_array2d_double_get_data_pointer_to_pointer( &J_data, J );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dproj_G_T_P_data, dproj_G_T_P );
   error = rox_array2d_double_get_data_pointer_to_pointer( &weights_data, weights );

   if ( ( NULL == c_ref_data       )
     || ( NULL == dproj_data       )
     || ( NULL == G_T_P_data       )
     || ( NULL == Gyz_cTo_Ps_data  )
     || ( NULL == Gzx_cTo_Ps_data  )
     || ( NULL == Gxy_cTo_Ps_data  )
     || ( NULL == Gx_cTo_Ps_data   )
     || ( NULL == Gy_cTo_Ps_data   )
     || ( NULL == Gz_cTo_Ps_data   )
     || ( NULL == J_data           )
     || ( NULL == dproj_G_T_P_data )
     || ( NULL == weights_data     ) )
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Fill generators
   error = rox_array2d_double_fillval( Gyz, 0.0 );                ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillval( Gzx, 0.0 );                ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillval( Gxy, 0.0 );                ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillval(  Gx, 0.0 );                ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillval(  Gy, 0.0 );                ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillval(  Gz, 0.0 );                ROX_ERROR_CHECK_TERMINATE ( error );
   // Gyz
   error = rox_array2d_double_set_value( Gyz, 1, 2, -1.0 );       ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( Gyz, 2, 1,  1.0 );       ROX_ERROR_CHECK_TERMINATE ( error );
   // Gzx
   error = rox_array2d_double_set_value( Gzx, 0, 2,  1.0 );       ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( Gzx, 2, 0, -1.0 );       ROX_ERROR_CHECK_TERMINATE ( error );
   // Gxy
   error = rox_array2d_double_set_value( Gxy, 0, 1, -1.0 );       ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( Gxy, 1, 0,  1.0 );       ROX_ERROR_CHECK_TERMINATE ( error );
   // Gx
   error = rox_array2d_double_set_value(  Gx, 0, 3,  1.0 );       ROX_ERROR_CHECK_TERMINATE ( error );
   // Gy
   error = rox_array2d_double_set_value(  Gy, 1, 3,  1.0 );       ROX_ERROR_CHECK_TERMINATE ( error );
   // Gz
   error = rox_array2d_double_set_value(  Gz, 2, 3,  1.0 );       ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate dynamic vectors
   ref_cam->used   = 0;
   ref_proj->used  = 0;
   meas_norm->used = 0;
   rox_dynvec_point3d_double_usecells(   ref_cam, n_pts );
   rox_dynvec_point2d_double_usecells(  ref_proj, n_pts );
   rox_dynvec_point2d_double_usecells( meas_norm, n_pts );

   // Normalize measures
   error = rox_array2d_double_svdinverse( iK, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_transform_homogeneous( meas_norm, measures, iK );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimate measures' standard deviation and tukey weights
   error = rox_array2d_double_fillunit( I_3x3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point3d_double_transform( ref_cam->data, cTo, references->data, references->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point2d_double_project( ref_proj->data, ref_cam->data, I_3x3, ref_cam->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_double_distance_point2d_double( dist, ref_proj, meas_norm );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mad( &mad, work1, work2, dist );
   ROX_ERROR_CHECK_TERMINATE ( error );

   std_dev = MAD_SCALE_FACTOR_NORMAL * mad;

   error = rox_array2d_double_tukey( weights, work1, work2, dist );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Prepare measures' inverse covariance matrix
   error = rox_array2d_double_set_value( iEmeas, 1, 0, 0.0 );      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( iEmeas, 0, 1, 0.0 );      ROX_ERROR_CHECK_TERMINATE ( error );

   // Prepare jacobian computation
   for ( idpt = 0; idpt < n_pts; idpt++)
   {
      c_ref_data[0][idpt] = ref_cam->data[idpt].X;
      c_ref_data[1][idpt] = ref_cam->data[idpt].Y;
      c_ref_data[2][idpt] = ref_cam->data[idpt].Z;
      c_ref_data[3][idpt] = 1.0;
   }

   error = rox_array2d_double_mulmatmat( Gyz_cTo_Ps, Gyz, c_ref ); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat( Gzx_cTo_Ps, Gzx, c_ref ); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat( Gxy_cTo_Ps, Gxy, c_ref ); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(  Gx_cTo_Ps,  Gx, c_ref ); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(  Gy_cTo_Ps,  Gy, c_ref ); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(  Gz_cTo_Ps,  Gz, c_ref ); ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval( dproj, 0.0 );               ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillval( sum_Jt_iE_J, 0.0 );         ROX_ERROR_CHECK_TERMINATE ( error );

   // Back propagate covariance
   for ( idpt = 0; idpt < n_pts; idpt++)
   {
      // Setup measure's inverse covariance matrix
      i_var = weights_data[idpt][0] / ( std_dev*std_dev );
      error = rox_array2d_double_set_value( iEmeas, 0, 0, i_var ); ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_set_value( iEmeas, 1, 1, i_var ); ROX_ERROR_CHECK_TERMINATE ( error );

      // Access point's coordinates
      x  = ref_proj->data[idpt].u; // I_3x3 was used to project
      y  = ref_proj->data[idpt].v;
      Z  = ref_cam->data[idpt].Z;
      iZ = 1.0 / Z;

      // Compute projection derivative
      dproj_data[0][0] = iZ;
      dproj_data[1][1] = iZ;
      dproj_data[0][2] = -x*iZ;
      dproj_data[1][2] = -y*iZ;

      // Compute jacobian
      for ( basis = 0; basis < 6; basis++ )
      {
         switch ( basis )
         {
            case 0:
            {
               // Select R_yz derivative
               G_T_Ps_data = Gyz_cTo_Ps_data;
            }break;
            case 1:
            {
               // Select R_zx derivative
               G_T_Ps_data = Gzx_cTo_Ps_data;
            }break;
            case 2:
            {
               // Select R_xy derivative
               G_T_Ps_data = Gxy_cTo_Ps_data;
            }break;
            case 3:
            {
               // Select t_x derivative
               G_T_Ps_data = Gx_cTo_Ps_data;
            }break;
            case 4:
            {
               // Select t_y derivative
               G_T_Ps_data = Gy_cTo_Ps_data;
            }break;
            case 5:
            {
               // Select t_z derivative
               G_T_Ps_data = Gz_cTo_Ps_data;
            }break;
         }

         G_T_P_data[0][0] = G_T_Ps_data[0][idpt];
         G_T_P_data[1][0] = G_T_Ps_data[1][idpt];
         G_T_P_data[2][0] = G_T_Ps_data[2][idpt];
         G_T_P_data[3][0] = G_T_Ps_data[3][idpt];

         error = rox_array2d_double_mulmatmat( dproj_G_T_P, dproj, G_T_P );
         ROX_ERROR_CHECK_TERMINATE ( error );

         J_data[0][basis] = dproj_G_T_P_data[0][0];
         J_data[1][basis] = dproj_G_T_P_data[1][0];
      }

      error = rox_array2d_double_mulmattransmat( Jt_iE, J, iEmeas );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat( Jt_iE_J, Jt_iE, J );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add( sum_Jt_iE_J, sum_Jt_iE_J, Jt_iE_J );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_array2d_double_svdinverse( E, sum_Jt_iE_J );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_array2d_double_del(       &work2);
   rox_array2d_double_del(       &work1);
   rox_array2d_double_del(       &I_3x3 );
   rox_array2d_double_del(          &iK );
   rox_array2d_double_del(       &c_ref );
   rox_array2d_double_del(      &iEmeas );
   rox_array2d_double_del(   &Gz_cTo_Ps );
   rox_array2d_double_del(   &Gy_cTo_Ps );
   rox_array2d_double_del(   &Gx_cTo_Ps );
   rox_array2d_double_del(  &Gxy_cTo_Ps );
   rox_array2d_double_del(  &Gzx_cTo_Ps );
   rox_array2d_double_del(  &Gyz_cTo_Ps );
   rox_array2d_double_del(          &Gz );
   rox_array2d_double_del(          &Gy );
   rox_array2d_double_del(          &Gx );
   rox_array2d_double_del(         &Gxy );
   rox_array2d_double_del(         &Gzx );
   rox_array2d_double_del(         &Gyz );
   rox_array2d_double_del(     &weights );
   rox_array2d_double_del(        &dist );
   rox_array2d_double_del(           &J );
   rox_array2d_double_del(       &G_T_P );
   rox_array2d_double_del( &dproj_G_T_P );
   rox_array2d_double_del(       &dproj );
   rox_array2d_double_del(       &Jt_iE );
   rox_array2d_double_del(     &Jt_iE_J );
   rox_array2d_double_del( &sum_Jt_iE_J );
   rox_dynvec_point2d_double_del( &meas_norm );
   rox_dynvec_point2d_double_del(  &ref_proj );
   rox_dynvec_point3d_double_del(   &ref_cam );

   return error;
}
