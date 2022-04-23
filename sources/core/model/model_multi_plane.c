//==============================================================================
//
//    OPENROX   : File model_multi_plane.c
//
//    Contents  : Implementation of model_multi_plane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "model_multi_plane.h"

#include <string.h>
#include <generated/objset_matse3_struct.h>

#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <baseproc/geometry/transforms/matse3/matse3_from_points3d_sets.h>
#include <baseproc/geometry/point/dynvec_point3d_tools.h>
#include <baseproc/geometry/point/point3d_tools.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_matse3_from_photoframe_vertices_dynvec (
  Rox_MatSE3 pTo,
  const Rox_DynVec_Point3D_Double p_points,
  const Rox_DynVec_Point3D_Double o_points 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct  mean_pt;
   Rox_DynVec_Point3D_Double  o_n_pts = NULL, p_n_pts = NULL;

   // o_points contains the vertices of the reference photoframe. 
   // o_n_points will contain the normalized vertices of the reference photoframe. 
   // The frame of the first photoframe is also the global frame.
   error = rox_dynvec_point3d_double_new ( &o_n_pts,  4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_mean ( &mean_pt, o_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_center_normalize ( o_n_pts, o_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_shift ( o_n_pts, &mean_pt );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // p_points contains the vertices of the current photoframe. 
   // p_n_points will contain the normalized vertices of the first photoframe. 

   error = rox_dynvec_point3d_double_new ( &p_n_pts,  4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_mean ( &mean_pt, p_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_center_normalize ( p_n_pts, p_points );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_shift ( p_n_pts, &mean_pt );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the pose pTo between the photoframe F_p and the global object frame Fo
   error = rox_matse3_from_dynvec_points3d_double ( pTo, o_n_pts, p_n_pts );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_dynvec_point3d_double_del( &p_n_pts  );
   rox_dynvec_point3d_double_del( &o_n_pts  );
   return error;
}

Rox_ErrorCode rox_model_multi_plane_new (
   Rox_Model_Multi_Plane * model_multi_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_Multi_Plane ret = NULL;

   // Test Inputs
   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   *model_multi_plane = NULL;

   ret = ( Rox_Model_Multi_Plane ) rox_memory_allocate( sizeof( struct Rox_Model_Multi_Plane_Struct ), 1 );
   if ( !ret )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   ret->planes = 0;

   error = rox_objset_model_single_plane_new ( &ret->planes, 10 );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_objset_matse3_new ( &ret->pTo, 10 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *model_multi_plane = ret;

function_terminate:
   if (error) rox_model_multi_plane_del( &ret );
   return error;
}


Rox_ErrorCode rox_model_multi_plane_del (
   Rox_Model_Multi_Plane * model_multi_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_Multi_Plane todel = NULL;

   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   todel = *model_multi_plane;
   *model_multi_plane = NULL;

   if ( !todel )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   rox_objset_model_single_plane_del( &todel->planes );
   rox_objset_matse3_del ( &todel->pTo );
   rox_memory_delete( todel );

function_terminate:
   return error;
}


Rox_ErrorCode rox_model_multi_plane_append_3d_template (
   Rox_Model_Multi_Plane model_multi_plane,
   const Rox_Image image_template,
   const Rox_Point3D_Double vertices,
   const Rox_Sint basesize
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_Single_Plane added = NULL;

   // Test Inputs
   if ( !image_template || !vertices )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   // Test Outputs
   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_model_single_plane_new ( &added );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_model_single_plane_set_3d_template ( added, image_template, vertices, basesize );
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_objset_model_single_plane_append ( model_multi_plane->planes, added );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   if (error) rox_model_single_plane_del( &added );
   return error;
}


Rox_ErrorCode rox_model_multi_plane_append_plane (
   Rox_Model_Multi_Plane         model_multi_plane,
   const Rox_Image          image_template,
   const Rox_Point3D_Double vertices_cur 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;   
   Rox_MatSE3 pTo = NULL;

   // Test Inputs
   if ( !image_template || !vertices_cur )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   // Test Outputs
   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   error = rox_matse3_new ( &pTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Force basesize to 128 pixels 
   error = rox_model_multi_plane_append_3d_template ( model_multi_plane, image_template, vertices_cur, 128 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // set the current pTo
   if ( model_multi_plane->pTo->used > 0 )
   {  
      Rox_Point3D_Double vertices_ref = model_multi_plane->planes->data[0]->vertices_ref;

      error = rox_matse3_from_photoframe_vertices_vector ( pTo, vertices_cur, vertices_ref ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_objset_matse3_append ( model_multi_plane->pTo, pTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_multi_plane_set_currentpose (
   Rox_Model_Multi_Plane model_multi_plane,
   const Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   for ( Rox_Uint id = 0; id < model_multi_plane->planes->used; id++ )
   {
      error = rox_model_single_plane_set_pose ( model_multi_plane->planes->data[id], pose );
      ROX_ERROR_CHECK_TERMINATE( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_model_multi_plane_transform (
   Rox_Model_Multi_Plane model_multi_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   // Transform each plane
   for (Rox_Uint id = 0; id < model_multi_plane->planes->used; id++ )
   {
      error = rox_model_single_plane_transform ( model_multi_plane->planes->data[id] );
      ROX_ERROR_CHECK_TERMINATE( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_multi_plane_get_number (
   Rox_Sint * nbp,
   const Rox_Model_Multi_Plane model_multi_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Test Inputs
   if ( !nbp )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Test Outputs
   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *nbp = model_multi_plane->planes->used;

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_multi_plane_get_vertices_cur (
   Rox_Point3D_Double vertices,
   const Rox_Model_Multi_Plane model_multi_plane,
   const Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Test Inputs
   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( id >= (Rox_Sint) model_multi_plane->planes->used )
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Test Outputs
   if ( !vertices )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_model_single_plane_get_vertices_cur ( vertices, model_multi_plane->planes->data[id] );
   ROX_ERROR_CHECK_TERMINATE( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_model_multi_plane_copy (
   Rox_Model_Multi_Plane dst,
   const Rox_Model_Multi_Plane src
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_Single_Plane toadd = NULL;

   if ( !src )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !dst )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Cleanup destination model
   error = rox_objset_model_single_plane_reset ( dst->planes );
   ROX_ERROR_CHECK_TERMINATE( error );

   // Create new templates
   for (Rox_Uint id = 0; id < src->planes->used; id++ )
   {
      error = rox_model_single_plane_new ( &toadd );
      ROX_ERROR_CHECK_TERMINATE( error );

      error = rox_model_single_plane_copy ( toadd, src->planes->data[id] );
      ROX_ERROR_CHECK_TERMINATE( error );

      error = rox_objset_model_single_plane_append ( dst->planes, toadd );
      ROX_ERROR_CHECK_TERMINATE( error );
   }

function_terminate:
   if (error) rox_model_single_plane_del ( &toadd );
   return error;
}


Rox_ErrorCode rox_model_multi_plane_check_visibility (
   Rox_Model_Multi_Plane model_multi_plane,
   const Rox_Sint image_rows,
   const Rox_Sint image_cols,   
   const Rox_MatUT3 Kc,
   const Rox_MatSE3 cTo
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE( error ); }

   // Transform each plane
   for (Rox_Uint id = 0; id < model_multi_plane->planes->used; id++ )
   {
      error = rox_model_single_plane_check_visibility ( model_multi_plane->planes->data[id], image_rows, image_cols, Kc, cTo );
      ROX_ERROR_CHECK_TERMINATE( error );
   }

function_terminate:
   return error;
}