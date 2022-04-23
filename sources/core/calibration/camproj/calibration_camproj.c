//==============================================================================
//
//    OPENROX   : File camproj_calibration.c
//
//    Contents  : Implementation of camproj_calibration module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "calibration_camproj.h"
#include "calibration_camproj_struct.h"

#include <stdio.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/solve/svd_solve.h>
#include <baseproc/array/minmax/minmax.h>
#include <baseproc/array/median/median.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/transforms/matsl3/sl3fromNpoints.h>
#include <baseproc/geometry/plane/plane_transform.h>
#include <baseproc/maths/linalg/generators/algut3.h>
#include <baseproc/maths/linalg/matse3.h>

#include <baseproc/calculus/jacobians/interaction_point2d_pix_matut3.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#define threshold_reprojection 1.5
#define KDOF 4

// Hypothesis: plane parameters in world coordinates are ( 0 0 1 0 )
Rox_ErrorCode rox_pixel_to_device_space (
   Rox_Point3D_Double  Mc,
   const Rox_MatSE3 cTo,
   const Rox_MatUT3 Kc,
   const Rox_Point2D_Double pc,
   const Rox_Uint n_pts
)
{
   Rox_ErrorCode      error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct nc; Rox_Double dc = 0.0;
   Rox_Point3D_Double_Struct qc, ppc;
   Rox_Double         Ki00 = 0.0, Ki01 = 0.0, Ki02 = 0.0, Ki11 = 0.0, Ki12 = 0.0, d=0.0; //Ki22=0.0,
   Rox_Double         **dcTo=NULL,**dKinv=NULL;
   Rox_MatUT3         Kinv = NULL;
   Rox_Uint           i=0;

   // Check Mc, cTo, Kc, pc, with n_pts
   if ( !Mc || !cTo || !Kc || !pc )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size( cTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size( Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Obtain plane parameters in camera frame
   error = rox_plane_transform ( &(nc.X), &(nc.Y), &(nc.Z), &(dc), cTo, 0.0, 0.0, 1.0, 0.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Kinv
   error = rox_matut3_new ( &Kinv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_inv ( Kinv, Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dKinv, Kinv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Ki00 = dKinv[0][0]; Ki01 = dKinv[0][1]; Ki02 = dKinv[0][2];
   Ki11 = dKinv[1][1]; Ki12 = dKinv[1][2];
   //Ki22 = dKinv[2][2];

   // dcTo
   error = rox_array2d_double_get_data_pointer_to_pointer( &dcTo, cTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( !dcTo )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Starting point of the ray: l0 = [0, 0, 0]
   for ( i=0; i < n_pts; i++ )
   {
      qc.X = Ki00 * pc[i].u + Ki01 * pc[i].v + Ki02;
      qc.Y = Ki11 * pc[i].v + Ki12;
      qc.Z = 1.0;

      // Direction of the ray
      // lc = qc - l0 = qc

      // Point on the plane in world reference: ppo = [0, 0, 0, 1]
      // Point on the plane in camera reference
      ppc.X = dcTo[0][3];
      ppc.Y = dcTo[1][3];
      ppc.Z = dcTo[2][3];

      // Compute d (delta) so that l0+d.lc intersects the plane
      // d = ( (ppc - l0 )'*n_c ) / ( lc'*n_c )
      d = ( ppc.X*nc.X + ppc.Y*nc.Y + ppc.Z*nc.Z ) / ( qc.X*nc.X + qc.Y*nc.Y + qc.Z*nc.Z );

      // Compute 3D position of the point
      // Mc = d*lc + l0
      Mc[i].X = d*qc.X;
      Mc[i].Y = d*qc.Y;
      Mc[i].Z = d*qc.Z;
   }

function_terminate:
   rox_array2d_double_del( &Kinv );
   return error;
}

// Hypothesis: plane parameters in world coordinates are ( 0 0 1 0 )
Rox_ErrorCode rox_pixel_to_world_space(
   Rox_Point3D_Double Mo,
   Rox_MatSE3 cTo,
   Rox_MatUT3 Kc,
   Rox_Point2D_Double pc,
   Rox_Uint n_pts )
{
   Rox_ErrorCode      error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct qc, qo, l0, l;
   Rox_Double         Ki00=0.0, Ki01=0.0, Ki02=0.0, Ki11=0.0, Ki12=0.0,  d=0.0; //Ki22=0.0,
   Rox_Double         **doTc=NULL,**dKinv=NULL;
   Rox_MatUT3         Kinv = NULL;
   Rox_MatSE3         oTc = NULL;
   Rox_Uint           i=0;

   // Check Mo, cTo, Kc, pc, with n_pts
   if (!Mo || !cTo || !Kc || !pc)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size( cTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size( Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Kinv
   error = rox_matut3_new ( &Kinv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_inv ( Kinv, Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dKinv, Kinv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Ki00 = dKinv[0][0]; Ki01 = dKinv[0][1]; Ki02 = dKinv[0][2];
   Ki11 = dKinv[1][1]; Ki12 = dKinv[1][2];
   //Ki22 = dKinv[2][2];

   // oTc and doTc
   error = rox_array2d_double_new( &oTc, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_inv( oTc, cTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &doTc, oTc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the rays starting point i.e. camera origin
   l0.X = doTc[0][3];
   l0.Y = doTc[1][3];
   l0.Z = doTc[2][3];
   for ( i=0; i < n_pts; i++ )
   {
      qc.X = Ki00 * pc[i].u + Ki01 * pc[i].v + Ki02;
      qc.Y = Ki11 * pc[i].v + Ki12;
      qc.Z = 1.0;

      // Get normalized coordinates in the world/calibration plane frame
      error = rox_point3d_double_transform( &qo, oTc, &qc, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute direction of the ray
      l.X = qo.X - l0.X;
      l.Y = qo.Y - l0.Y;
      l.Z = qo.Z - l0.Z;

      // Compute d (delta) so that l0+dl intersects the plane [0, 0, 1, 0]
      // d = ( (p0 - l0 )'*n_o ) / ( l'*n_o )
      // with n_o = (0, 0, 1) and p0 = (0, 0, 0)
      d = -l0.Z / l.Z;

      // Compute Mo, the 3D position of the point on the calibration plane
      Mo[i].X = l0.X + d*l.X;
      Mo[i].Y = l0.Y + d*l.Y;
      Mo[i].Z = l0.Z + d*l.Z;
   }

function_terminate:
   rox_array2d_double_del( &Kinv );
   rox_array2d_double_del( &oTc );
   return error;
}

Rox_ErrorCode rox_skew_from_doubles(Rox_Array2D_Double skew, Rox_Double X, Rox_Double Y, Rox_Double Z )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!skew) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size( skew, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dskew = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dskew, skew );
   ROX_ERROR_CHECK_TERMINATE ( error );

   dskew[0][0] = 0.0;
   dskew[0][1] = -Z;
   dskew[0][2] = Y;

   dskew[1][0] = Z;
   dskew[1][1] = 0.0;
   dskew[1][2] = -X;

   dskew[2][0] = -Y;
   dskew[2][1] = X;
   dskew[2][2] = 0.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_new(Rox_Calibration_CamProj_Perspective * obj)
{
   Rox_ErrorCode error =  ROX_ERROR_NONE;
   Rox_Calibration_CamProj_Perspective ret=NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *obj = NULL;

   ret = (Rox_Calibration_CamProj_Perspective) rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->cam           = NULL;
   ret->proj          = NULL;
   ret->poses         = NULL;
   ret->pTc           = NULL;
   ret->Kp            = NULL;
   ret->camproj_pts2D = NULL;
   ret->Gp            = NULL;
   ret->proj_refs2D   = NULL;
   ret->nproj_pts     = 0;
   ret->known_kc      = 0;
   ret->Kc            = NULL;
   ret->cTos          = NULL;

   error = rox_calibration_mono_perspective_new(&ret->cam);       
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_calibration_projector_perspective_new(&ret->proj); 
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->poses, 10);        
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->Gp, 3, 3);                
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->pTc, 4, 4);               
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->Kp, 3, 3);                
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->Kc, 3, 3);                
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->cTos, 10);         
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_objset_dynvec_point2d_double_new(&ret->camproj_pts2D, 20);
   ROX_ERROR_CHECK_TERMINATE(error)

   *obj = ret;

function_terminate:
   if(error) rox_calibration_camproj_perspective_del(&ret);
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_del(Rox_Calibration_CamProj_Perspective * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_CamProj_Perspective todel=NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_calibration_mono_perspective_del( &todel->cam );
   rox_calibration_projector_perspective_del( &todel->proj );
   rox_objset_array2d_double_del( &todel->poses );
   rox_array2d_double_del( &todel->pTc );
   rox_array2d_double_del( &todel->Kp );
   rox_objset_dynvec_point2d_double_del( &todel->camproj_pts2D );
   rox_array2d_double_del( &todel->Gp );
   rox_memory_delete( todel->proj_refs2D );
   rox_array2d_double_del( &todel->Kc );
   rox_objset_array2d_double_del( &todel->cTos );

   rox_memory_delete( todel );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_set_cam_model_points(Rox_Calibration_CamProj_Perspective obj, Rox_Point3D_Double refs3D, Rox_Uint count)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !obj || !refs3D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_mono_perspective_set_model_points(obj->cam, refs3D, count);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_set_proj_model_points(Rox_Calibration_CamProj_Perspective obj, Rox_Point2D_Double refs2D, Rox_Uint count)
{
   Rox_ErrorCode error=ROX_ERROR_NONE;

   if(!obj || !refs2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if( 0 >= count ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   error = rox_calibration_projector_perspective_set_model_points(obj->proj, refs2D, count);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->proj_refs2D = (Rox_Point2D_Double )rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), count );
   if ( !obj->proj_refs2D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->nproj_pts = count;
   for (Rox_Uint i = 0; i < count; i++)
   {
      obj->proj_refs2D[i].u = refs2D[i].u;
      obj->proj_refs2D[i].v = refs2D[i].v;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_add_current_points(Rox_Calibration_CamProj_Perspective obj, Rox_Point2D_Double cam_pts, Rox_Uint cam_count, Rox_Point2D_Double proj_pts, Rox_Uint proj_count  )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point2D_Double pts;

   if ( !obj || !cam_pts || !proj_pts ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !cam_count || !proj_count )     
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( proj_count != obj->nproj_pts )  
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_mono_perspective_add_current_points( obj->cam, cam_pts, cam_count );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_new( &pts, 50 );
   ROX_ERROR_CHECK_TERMINATE(error)

   for (Rox_Uint i = 0; i < proj_count; i++)
   {
      Rox_Point2D_Double_Struct pt = proj_pts[i];
      error = rox_dynvec_point2d_double_append(pts, &pt);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   error = rox_objset_dynvec_point2d_double_append ( obj->camproj_pts2D, pts );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   if (error) rox_dynvec_point2d_double_del(&pts);

   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_add_current_cam_homography(Rox_Calibration_CamProj_Perspective obj, Rox_Array2D_Double G)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double pose = NULL;
   Rox_Array2D_Double cTo  = NULL;

   if ( !obj || !G ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new( &pose, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &cTo, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_mono_perspective_add_homography( obj->cam, G );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_objset_array2d_double_append( obj->poses, pose );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_objset_array2d_double_append( obj->cTos, cTo );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   if (error) rox_array2d_double_del( &pose );
   if (error) rox_array2d_double_del( &cTo );

   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_set_camera_intrinsics ( 
   Rox_Calibration_CamProj_Perspective obj, 
   Rox_Array2D_Double Kin
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( NULL == obj || NULL == Kin || NULL == obj->Kc )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size( Kin, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size( obj->Kc, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy( obj->Kc, Kin );
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->known_kc = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_set_intrinsics(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_Array2D_Double Kc,
   Rox_Array2D_Double Kp )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !obj || !Kc || !Kp ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_mono_perspective_set_intrinsics( obj->cam, Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_projector_perspective_set_intrinsics( obj->proj, Kp );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_set_cam_resolution (
   Rox_Calibration_CamProj_Perspective obj, Rox_Sint cols, Rox_Sint rows)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( obj->known_kc == 0)
   {
      error = rox_array2d_double_set_value( obj->Kc, 0, 2, cols / 2.0 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_set_value( obj->Kc, 1, 2, rows / 2.0 );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_set_proj_resolution ( 
   Rox_Calibration_CamProj_Perspective obj, 
   Rox_Sint cols, Rox_Sint rows)
{
   Rox_ErrorCode error=ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_set_value( obj->proj->K, 0, 2, cols / 2.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( obj->proj->K, 1, 2, rows / 2.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_make_cam ( Rox_Calibration_CamProj_Perspective obj )
{
   Rox_ErrorCode error=ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( 0 == obj->cam->homographies->used )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( obj->cam->homographies->used != obj->camproj_pts2D->used )
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // if Kc is not known a priori we need to calibrate the camera
   if ( obj->known_kc == 0 )
   {
      // Perform camera calibration (intrinsic and extrinsic parameters)
      error = rox_calibration_mono_perspective_compute_parameters( obj->cam, KDOF );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Copy the estimated camera intrinsics : matrix Kc
      error = rox_array2d_double_copy( obj->Kc, obj->cam->K );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Copy the estimated camera extrinsics : poses cTo
      for ( Rox_Uint i = 0; i < obj->cam->poses->used; i++ )
      {
         error = rox_array2d_double_copy( obj->cTos->data[i], obj->cam->poses->data[i] );
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   } 
   else // Kc is known we does not need to calibrate the camera
   {
      // Estimate camera poses from homographies
      for (Rox_Uint i=0; i < obj->cam->poses->used; i++)
      {
         error = rox_transformtools_build_pose_intermodel ( obj->cTos->data[i], obj->cam->homographies->data[i], obj->Kc );
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_make_proj ( Rox_Calibration_CamProj_Perspective obj )
{
   Rox_ErrorCode      error = ROX_ERROR_NONE;
   Rox_Point2D_Double Mo_s = NULL; //Mo_sliced
   Rox_Point3D_Double Mo = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Using the camera poses, we can compute the 3D positions of detected
   // projected points, and feed the projector_calibrator with it
   Mo_s = (Rox_Point2D_Double ) rox_memory_allocate( sizeof(Rox_Point2D_Double_Struct), obj->nproj_pts );
   if ( !Mo_s )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   Mo = (Rox_Point3D_Double )rox_memory_allocate( sizeof(Rox_Point3D_Double_Struct), obj->nproj_pts );
   if ( !Mo )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   for (Rox_Uint i = obj->proj->poses->used; i < obj->cTos->used; i++)
   {
      error = rox_pixel_to_world_space (
         Mo,
         obj->cTos->data[i],
         obj->Kc,
         obj->camproj_pts2D->data[i]->data,
         obj->camproj_pts2D->data[i]->used );
      ROX_ERROR_CHECK_TERMINATE(error)

      for (Rox_Uint j = 0; j < obj->camproj_pts2D->data[i]->used; j++)
      {
         Mo_s[j].u = Mo[j].X;
         Mo_s[j].v = Mo[j].Y;
      }

      // Compute homography between sliced 3D positions and 2D positions
      error = rox_matsl3_from_n_points_double ( obj->Gp, Mo_s, obj->proj_refs2D, obj->nproj_pts );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Add homography
      error = rox_calibration_projector_perspective_add_homography ( obj->proj, obj->Gp );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Add 3D sliced points
      error = rox_calibration_projector_perspective_add_current_points ( obj->proj, Mo_s, obj->nproj_pts );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Perform projector calibration
   error = rox_calibration_projector_perspective_compute_parameters ( obj->proj, KDOF );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_memory_delete( Mo_s );
   rox_memory_delete( Mo );
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_compute_relative_poses ( 
   Rox_Calibration_CamProj_Perspective obj 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 oTc  = NULL;

   if ( obj->cTos->used != obj->proj->poses->used )
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( obj->cTos->used != obj->poses->used )
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_new ( &oTc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < obj->poses->used; i++)
   {
      error = rox_matse3_inv ( oTc, obj->cTos->data[i] );
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_matse3_mulmatmat ( obj->poses->data[i], obj->proj->poses->data[i], oTc );
      ROX_ERROR_CHECK_TERMINATE(error)
   }

function_terminate:
   rox_matse3_del(&oTc);
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_make ( Rox_Calibration_CamProj_Perspective obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_calibration_camproj_perspective_make_cam( obj );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_camproj_perspective_make_proj( obj );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_camproj_perspective_compute_relative_poses( obj );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_camproj_perspective_process_nolinear( obj );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_get_linear_intrinsics(Rox_Calibration_CamProj_Perspective obj, Rox_Array2D_Double Kc, Rox_Array2D_Double Kp)
{
   Rox_ErrorCode error=ROX_ERROR_NONE;

   if ( !Kc || !Kp || !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy( Kc, obj->Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy( Kp, obj->proj->K );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_get_refined_intrinsics (
   Rox_Calibration_CamProj_Perspective obj, 
   Rox_Array2D_Double Kc, 
   Rox_Array2D_Double Kp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !Kc || !Kp || !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy( Kc, obj->Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy( Kp, obj->Kp );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_get_indexed_linear_pTc( 
   Rox_Calibration_CamProj_Perspective obj, 
   Rox_Array2D_Double pTc, 
   Rox_Uint index
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pTc || !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( index >= obj->poses->used ) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy( pTc, obj->poses->data[index] );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_get_refined_pTc (
   Rox_Calibration_CamProj_Perspective obj, Rox_Array2D_Double pTc
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pTc || !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy( pTc, obj->pTc );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_check_cam_results (
   Rox_Double * error_cam,
   Rox_Array2D_Double cTo,
   Rox_Array2D_Double Kc,
   Rox_Point3D_Double Mo,
   Rox_Point2D_Double ref2D,
   Rox_Uint n_pts 
)
{
   Rox_ErrorCode      error=ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct Mc;
   Rox_Point2D_Double_Struct pc, obs;
   Rox_Double         du=0.0, dv=0.0;

   if (!error_cam || !cTo || !Kc || !Mo || !ref2D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init cam errors
   *error_cam = 0.0;
   // For printed points
   for (Rox_Uint i = 0; i < n_pts; i++ )
   {
      // Transform to camera frame
      error = rox_point3d_double_transform( &Mc, cTo, &(Mo[i]), 1 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // project and pixelize
      error = rox_point2d_double_project( &pc, &Mc, Kc, 1 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Select index'th detected printed points
      obs = ref2D[i];

      // Compute euclidean norm
      du = obs.u - pc.u;
      dv = obs.v - pc.v;
      *error_cam += sqrt( du*du + dv*dv );
   }
   *error_cam /= (Rox_Double) n_pts;

   function_terminate:
      return error;
}

// d: destination
// s: source
Rox_ErrorCode rox_calibration_camproj_perspective_check_dTs_results (
   Rox_Double * errors,
   Rox_Array2D_Double dTs,
   Rox_Array2D_Double Kd,
   Rox_Array2D_Double sTo,
   Rox_Array2D_Double Ks,
   Rox_Point2D_Double d_p2D,
   Rox_Point2D_Double s_p2D,
   Rox_Uint           n_pts )
{
   Rox_ErrorCode      error=ROX_ERROR_NONE;
   Rox_Point3D_Double Ms=NULL;
   Rox_Point3D_Double Md=NULL;
   Rox_Point2D_Double pd=NULL;

   // Check inputs
   if (!errors || !dTs || !Kd || !sTo || !Ks || !d_p2D || !s_p2D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size( dTs, 4, 4 ); 
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_array2d_double_check_size( sTo, 4, 4 ); 
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_array2d_double_check_size(  Kd, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_array2d_double_check_size(  Ks, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE(error);

   // allocate variables
   Ms = (Rox_Point3D_Double ) rox_memory_allocate( sizeof(*Ms), n_pts );
   if (!Ms)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Md = (Rox_Point3D_Double ) rox_memory_allocate( sizeof(*Md), n_pts );
   if (!Md)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pd = (Rox_Point2D_Double ) rox_memory_allocate( sizeof(*pd), n_pts );
   if (!pd)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // The stuff
   error = rox_pixel_to_device_space( Ms, sTo, Ks, s_p2D, n_pts ); 
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_point3d_double_transform( Md, dTs, Ms, n_pts );     
   ROX_ERROR_CHECK_TERMINATE(error);
   
   error = rox_point2d_double_project( pd, Md, Kd, n_pts );        
   ROX_ERROR_CHECK_TERMINATE(error);

   *errors = 0.0;
   for (Rox_Uint i = 0; i < n_pts; i++ )
   {
      // Compute euclidean norm
      Rox_Double du = pd[i].u - d_p2D[i].u;
      Rox_Double dv = pd[i].v - d_p2D[i].v;
      *errors += sqrt( du*du + dv*dv );
   }
   *errors /= (Rox_Double) n_pts;

function_terminate:
   rox_memory_delete( Ms );
   rox_memory_delete( Md );
   rox_memory_delete( pd );

   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_check_linear_results (
   Rox_Calibration_CamProj_Perspective obj,
   Rox_Double *                        error_cam,
   Rox_Double *                        error_proj,
   Rox_Uint                            index 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double error_proj_loc=0.0;
   Rox_Uint i=0;

   if ( ( index >= obj->proj->poses->used ) || ( index >= obj->cTos->used ) )
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( obj->proj->poses->used != obj->cTos->used )
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *error_cam=0.0;
   *error_proj=0.0;

   error = rox_calibration_camproj_perspective_check_cam_results(
      error_cam,
      obj->cTos->data[index],
      obj->Kc,
      obj->cam->model->data,
      obj->cam->points->data[index]->data,
      obj->cam->model->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( i=0; i<obj->poses->used; i++ )
   {
      error = rox_calibration_camproj_perspective_check_dTs_results(
         &error_proj_loc,
         obj->poses->data[index],
         obj->proj->K,
         obj->cTos->data[i],
         obj->Kc,
         obj->proj_refs2D,
         obj->camproj_pts2D->data[i]->data,
         obj->nproj_pts );
      ROX_ERROR_CHECK_TERMINATE ( error );

      *error_proj += error_proj_loc;
   }
   *error_proj /= obj->poses->used;

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_camproj_perspective_check_refined_results(
   Rox_Calibration_CamProj_Perspective obj,
   Rox_Double *                        error_cam,
   Rox_Double *                        mean_error_proj_fwd,
   Rox_Double *                        median_error_proj_fwd,
   Rox_Double *                        mean_error_proj_bwd,
   Rox_Double *                        median_error_proj_bwd,
   Rox_Sint                            index                   )
{
   Rox_ErrorCode      error=ROX_ERROR_NONE;
   Rox_Double         error_proj_loc = 0.0, error_cam_loc = 0.0;
   Rox_Sint           start = 0, stop = 0;
   Rox_Array2D_Double individual_errors_fwd = NULL;
   Rox_Array2D_Double individual_errors_bwd = NULL;
   Rox_Array2D_Double cTp = NULL;

   if ( obj->proj->poses->used != obj->cTos->used )
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new( &individual_errors_fwd, 1, obj->poses->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &individual_errors_bwd, 1, obj->poses->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &cTp, 4, 4 );           
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_array2d_double_svdinverse( cTp, obj->pTc ); 
   ROX_ERROR_CHECK_TERMINATE(error);

   *error_cam=0.0;
   *mean_error_proj_fwd=0.0;
   *mean_error_proj_bwd=0.0;

   if ( index < 0 )
   {
      start = 0;
      stop  = obj->poses->used;
   }
   else
   {
      start = index;
      stop  = index+1;
   }

   for ( Rox_Sint i = start; i < stop; i++ )
   {
      // Camera error
      error = rox_calibration_camproj_perspective_check_cam_results (
         &error_cam_loc,
         obj->cTos->data[i],
         obj->Kc,
         obj->cam->model->data,
         obj->cam->points->data[i]->data,
         obj->cam->model->used );
      ROX_ERROR_CHECK_TERMINATE ( error );

      *error_cam += error_cam_loc;

      // Forward projector error
      error = rox_calibration_camproj_perspective_check_dTs_results (
         &error_proj_loc,
         obj->pTc,
         obj->Kp,
         obj->cTos->data[i],
         obj->Kc,
         obj->proj_refs2D,
         obj->camproj_pts2D->data[i]->data,
         obj->nproj_pts );
      ROX_ERROR_CHECK_TERMINATE ( error );

      *mean_error_proj_fwd += error_proj_loc;

      error = rox_array2d_double_set_value( individual_errors_fwd, 0, i, error_proj_loc );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Backward projector error
      error = rox_calibration_camproj_perspective_check_dTs_results(
         &error_proj_loc,
         cTp,
         obj->Kc,
         obj->proj->poses->data[i],
         obj->Kp,
         obj->camproj_pts2D->data[i]->data,
         obj->proj_refs2D,
         obj->nproj_pts );
      ROX_ERROR_CHECK_TERMINATE ( error );

      *mean_error_proj_bwd += error_proj_loc;

      error = rox_array2d_double_set_value( individual_errors_bwd, 0, i, error_proj_loc );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   *error_cam           /= stop - start;
   *mean_error_proj_fwd /= stop - start;
   *mean_error_proj_bwd /= stop - start;

   error = rox_array2d_double_median( median_error_proj_fwd, individual_errors_fwd );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_median(median_error_proj_bwd, individual_errors_bwd);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del( &individual_errors_fwd );
   rox_array2d_double_del( &individual_errors_bwd );
   return error;
}

// !Warning: . currently A is hard coded for Kdof == 4
//           . there is no cost function robustification
Rox_ErrorCode rox_calibration_camproj_perspective_process_nolinear ( Rox_Calibration_CamProj_Perspective obj )
{
   Rox_ErrorCode                    error=ROX_ERROR_NONE;
   Rox_Uint                         Tdof=6, Kdof=KDOF, nbpts=0, nbpos=0, maxiter=100, iter=0;
   Rox_Uint                         i=0, j=0, ind1=0, ind2=0;
   Rox_Array2D_Double               Kp=NULL, pTc=NULL, x=NULL, A=NULL, b=NULL;
   Rox_Array2D_Double               best_Kp=NULL, best_pTc=NULL;
   Rox_Array2D_Double               B=NULL, pRc=NULL, skewp=NULL, skewm=NULL, KpRc=NULL;
   Rox_Array2D_Double               xK=NULL, xK_full=NULL, xpTc=NULL;
   Rox_Array2D_Double               skewpKpRc=NULL, BskewpKpRc=NULL, iz2x2=NULL, opiz2x2=NULL;
   Rox_Array2D_Double               BskewpKpRcskewm=NULL, Lv=NULL, Lw=NULL, Lk=NULL;
   Rox_ObjSet_DynVec_Point3D_Double Mc;
   Rox_DynVec_Point3D_Double        *Mc_data = NULL;
   Rox_Point3D_Double_Struct               M;
   Rox_Point2D_Double_Struct               *pp=NULL;
   Rox_Point2D_Double_Struct               p;
   Rox_Double                       **dx=NULL, **dA=NULL, **db=NULL, z=0.0;
   Rox_Double                       lambda=0.9, cu=0.0, cv=0.0, best_error=DBL_MAX, cur_error=0.0;

   // Check input variables
   if (obj == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbpos = obj->poses->used;
   if ( nbpos < 1 ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbpts = obj->camproj_pts2D->data[0]->used;

   // 4 points for pTc
   if ( nbpts < 4 ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // 2 constraints per image
   if ( 2*nbpos < Kdof ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check object's variables
   if ( obj->cTos->used != obj->camproj_pts2D->used ) 
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Local variables allocation
   error = rox_array2d_double_new( &x, Kdof+Tdof, 1 );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &A, 2*nbpts*nbpos, Kdof+Tdof ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &b, 2*nbpts*nbpos, 1 );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &Kp, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy( Kp, obj->proj->K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &pTc, 4, 4 );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy( pTc, obj->poses->data[0] );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &best_Kp, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &best_pTc, 4, 4 );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dx, x );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dA, A );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &db, b );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( !dx || !dA || !db )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Allocate the vector for the algut3
   error = rox_array2d_double_new( &xK_full, 6, 1 );
   ROX_ERROR_CHECK_TERMINATE(error)

   // Allocate temporary utilities matrices
   error = rox_array2d_double_new( &B              , 2, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new( &skewp          , 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &skewm          , 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &KpRc           , 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &skewpKpRc      , 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &BskewpKpRc     , 2, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &iz2x2          , 2, 2 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &opiz2x2        , 2, 2 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &BskewpKpRcskewm, 2, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval( iz2x2  , 0 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval( opiz2x2, 0 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval( B      , 0 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( B, 0, 1, +1.0 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( B, 1, 0, -1.0 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate Mc
   error = rox_objset_dynvec_point3d_double_new( &Mc, nbpos );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get access to Mc_data
   error = rox_objset_dynvec_point3d_double_get_data_pointer( &Mc_data, Mc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( i = 0; i < nbpos; i++ )
   {
      error = rox_dynvec_point3d_double_new( &(Mc_data[i]), nbpts );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Allocate pp
   pp = (Rox_Point2D_Double ) rox_memory_allocate( sizeof(*pp), nbpts*nbpos );
   if (!pp)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Iterative minimization
   for ( iter = 0; iter < maxiter; iter++ )
   {
      // Get projection in projector of detected points by the camera
      for ( i = 0; i < nbpos; i++ )
      {
         // Compute 3D position in camera frame of camproj_pts2D
         error = rox_pixel_to_device_space(
            Mc_data[i]->data,
            obj->cTos->data[i],
            obj->Kc,
            obj->camproj_pts2D->data[i]->data,
            nbpts );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Project into projector image plane
         error = rox_point3d_double_transform_project(
            &(pp[ i*nbpts ]), pTc, Kp, Mc_data[i]->data, nbpts );
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      error = rox_array2d_double_new_subarray2d( &pRc, pTc, 0, 0, 3, 3 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // fill A and b
      // compute errors
      cur_error = 0.0;
      for ( i = 0; i < nbpos; i++ )
      {
         for ( j = 0; j < nbpts; j++ )
         {
            // Prepare indices
            ind1= i*nbpts + j;
            ind2= 2*(i*nbpts + j);

            p = pp[ind1];

            db[ind2][0]   = p.u - obj->proj_refs2D[j].u;
            db[ind2+1][0] = p.v - obj->proj_refs2D[j].v;

            cur_error = sqrt( db[ind2][0]*db[ind2][0] + db[ind2+1][0]*db[ind2+1][0] );

            M = Mc_data[i]->data[j];
            z = M.Z;

            error = rox_array2d_double_mulmatmat( KpRc, Kp, pRc );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_skew_from_doubles( skewp, p.u, p.v, 1.0 );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_skew_from_doubles( skewm, M.X, M.Y, M.Z );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_mulmatmat( skewpKpRc, skewp, KpRc );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_mulmatmat( BskewpKpRc, B, skewpKpRc );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_set_value( iz2x2, 0, 0, +1.0/z );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_set_value( iz2x2, 1, 1, +1.0/z );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_set_value( opiz2x2, 0, 0, -1.0/z );
            ROX_ERROR_CHECK_TERMINATE ( error );
            error = rox_array2d_double_set_value( opiz2x2, 1, 1, -1.0/z );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_mulmatmat( BskewpKpRcskewm, BskewpKpRc, skewm );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_new_subarray2d( &Lv, A, ind2, 0, 2, 3 );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_mulmatmat( Lv, iz2x2, BskewpKpRc );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_new_subarray2d( &Lw, A, ind2, 3, 2, 3 );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_mulmatmat( Lw, opiz2x2, BskewpKpRcskewm );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_new_subarray2d( &Lk, A, ind2, Tdof, 2, Kdof );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_fillval( Lk, 0 );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_set_value( Lk, 0, 0, p.u );
            ROX_ERROR_CHECK_TERMINATE ( error );
            
            error = rox_array2d_double_set_value( Lk, 0, 1, 1.0 );
            ROX_ERROR_CHECK_TERMINATE ( error );
            
            error = rox_array2d_double_set_value( Lk, 1, 2, p.v );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_set_value( Lk, 1, 3, 1.0 );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_del( &Lv ); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            
            error = rox_array2d_double_del( &Lw ); 
            ROX_ERROR_CHECK_TERMINATE ( error );
            
            error = rox_array2d_double_del( &Lk ); 
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
      }
      if ( cur_error < best_error )
      {
         best_error = cur_error;

         error = rox_array2d_double_copy( best_pTc, pTc );
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_array2d_double_copy( best_Kp, Kp );
         ROX_ERROR_CHECK_TERMINATE(error)
      }
      cur_error /= nbpos*nbpts;

      // Solve A.x = b
      error = rox_svd_solve( x, A, b );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_scale( x, x, -1.0*lambda );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d( &xpTc, x, 0, 0, Tdof, 1 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d( &xK, x, Tdof, 0, Kdof, 1 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // for Kdof >= 3: whatever the cu, cv values?
      error = rox_campar_ddl( xK_full, xK, cu, cv, Kdof );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_update_right( pTc, xpTc );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matut3_update_left ( Kp, xK_full );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_del( &xpTc ); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_del( &xK   ); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_del( &pRc  ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_array2d_double_copy( obj->pTc, best_pTc );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_copy( obj->Kp, best_Kp );
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   for ( i = 0; i < nbpos; i++ )
      error = rox_dynvec_point3d_double_del( &(Mc_data[i]) );

   rox_memory_delete( pp );
   error = rox_objset_dynvec_point3d_double_del( &Mc );
   error = rox_array2d_double_del( &BskewpKpRcskewm );
   error = rox_array2d_double_del( &opiz2x2         );
   error = rox_array2d_double_del( &iz2x2           );
   error = rox_array2d_double_del( &BskewpKpRc      );
   error = rox_array2d_double_del( &skewpKpRc       );
   error = rox_array2d_double_del( &KpRc            );
   error = rox_array2d_double_del( &skewm           );
   error = rox_array2d_double_del( &skewp           );
   error = rox_array2d_double_del( &B               );
   error = rox_array2d_double_del( &xK_full         );
   error = rox_array2d_double_del( &best_pTc        );
   error = rox_array2d_double_del( &best_Kp         );
   error = rox_array2d_double_del( &pTc             );
   error = rox_array2d_double_del( &Kp              );
   error = rox_array2d_double_del( &b               );
   error = rox_array2d_double_del( &A               );

   return error;
}

#if 0
   Rox_ErrorCode rox_calibration_camproj_perspective_print_statistics(Rox_Calibration_CamProj_Perspective obj)
   {
      Rox_ErrorCode error = ROX_ERROR_NONE;
      Rox_Array2D_Double norm = 0;
      Rox_Uint nbimages, nbpoints, i, j;

      Rox_Point2D_Double *reproj = 0;
      Rox_Double **dn;

      if(!obj) 
      { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

      nbimages = obj->left->poses->used;
      nbpoints = obj->left->model->used;

      reproj = (Rox_Point2D_Double *)rox_memory_allocate(sizeof(*reproj), nbpoints);
      if(!reproj)
      {
         error = ROX_ERROR_NULL_POINTER;
         ROX_ERROR_CHECK_TERMINATE(error)
      }

      error = rox_array2d_double_new(&norm, nbpoints, 1); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error  = rox_array2d_double_get_data_pointer_to_pointer( &dh, norm);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Right
      rox_log("Right \n");
      for(i = 0; i < nbimages; i++)
      {
         Rox_Double min, max, mean, sum, var, med, u1, v1, u2, v2;

         // valid images
         if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

         error = rox_point3d_double_transform_project(reproj, obj->right->poses->data[i], obj->right->K, obj->right->model->data, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         // Compute norm
         for(j = 0; j < nbpoints; j++)
         {
            // Compute norm
            u1 = reproj[j].u; v1 = reproj[j].v;
            u2 = obj->right->points->data[i]->data[j].u; v2 = obj->right->points->data[i]->data[j].v;

            dn[j][0] = sqrt((u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2));
         }

         // make stats
         error = rox_array2d_double_minmax(&min, &max, norm); ROX_ERROR_CHECK_TERMINATE(error)
    error = rox_array2d_double_median(&med, norm);ROX_ERROR_CHECK_TERMINATE(error)

         sum = 0;
         for(j = 0; j < nbpoints; j++)
         {
            sum += dn[j][0];
         }

         mean = sum / (Rox_Double)nbpoints;

         sum = 0;
         for(j = 0; j < nbpoints; j++)
         {
            sum += (dn[j][0] - mean) * (dn[j][0] - mean);
         }
         var = sqrt(sum / (Rox_Double)nbpoints);

         rox_log("min %f, max %f, mean %f, variance %f, median %f\n", min, max, mean, var, med);
      }

      // Left
      rox_log("Left \n");
      for(i = 0; i < nbimages; i++)
      {
         Rox_Double min, max, mean, sum, var, med, u1, v1, u2, v2;

         // valid images
         if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

         error = rox_point3d_double_transform_project(reproj, obj->left->poses->data[i], obj->left->K, obj->left->model->data, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         // Compute norm
         for(j = 0; j < nbpoints; j++)
         {
            // Compute norm
            u1 = reproj[j].u; v1 = reproj[j].v;
            u2 = obj->left->points->data[i]->data[j].u; v2 = obj->left->points->data[i]->data[j].v;

            dn[j][0] = sqrt((u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2));
         }

         // make stats
         error = rox_array2d_double_minmax(&min, &max, norm); ROX_ERROR_CHECK_TERMINATE(error)
    error = rox_array2d_double_median(&med, norm);ROX_ERROR_CHECK_TERMINATE(error)

         sum = 0;
         for(j = 0; j < nbpoints; j++)
         {
            sum += dn[j][0];
         }

         mean = sum / (Rox_Double)nbpoints;

         sum = 0;
         for(j = 0; j < nbpoints; j++)
         {
            sum += (dn[j][0] - mean) * (dn[j][0] - mean);
         }
         var = sqrt(sum / (Rox_Double)nbpoints);

         rox_log("min %f, max %f, mean %f, variance %f, median %f\n", min, max, mean, var, med);
      }

      function_terminate:
      rox_memory_delete(reproj);
      rox_array2d_double_del(&norm);

      return error;
   }
#endif


