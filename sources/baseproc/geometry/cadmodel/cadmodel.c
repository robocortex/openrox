//==============================================================================
//
//    OPENROX   : File cadmodel.c
//
//    Contents  : Implementation of cadmodel module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "cadmodel.h"

#include <string.h>

#include <baseproc/geometry/point/point3d_struct.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

//! CAD Model structure
struct Rox_CadModel_Struct
{
   // The 8 corners of the 3D Bounding Box
   Rox_Point3D_Float_Struct   box_points[8];

   // Number of faces
   Rox_Sint                   n_faces;

   // Number of vertices
   Rox_Sint                   n_vertices;
   
   // Pointer to vertices 3D coordinates
   Rox_Float                * vertices;
   
   // Pointer to list of indices
   Rox_Uint                 * indices;
   
   // Pointer to normal to each face
   Rox_Float                * normals;
   
};

Rox_ErrorCode rox_cadmodel_new ( Rox_CadModel * cadmodel )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CadModel ret = NULL;

   if (!cadmodel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *cadmodel = NULL;

   ret = (Rox_CadModel) rox_memory_allocate(sizeof(struct Rox_CadModel_Struct), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init internal variables
   ret->n_vertices = 0;
   ret->n_faces = 0;
   ret->vertices = NULL;
   ret->indices = NULL;
   ret->normals = NULL;

   *cadmodel = ret;
    
function_terminate:
   if (error) rox_cadmodel_del(&ret);
   return error;
}

Rox_ErrorCode rox_cadmodel_del ( Rox_CadModel * cadmodel )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CadModel todel = NULL;

   if (!cadmodel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *cadmodel;
   *cadmodel = NULL;

   rox_memory_delete(todel);
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_cadmodel_set_model (
   Rox_CadModel cadmodel,
   const int    n_vertices,
   const int     n_faces,
   const float * vertices,
   const int   * indices,
   const float * normals, 
   const float scale
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !vertices || !indices || !normals )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cadmodel->n_vertices = n_vertices;
   cadmodel->n_faces    = n_faces;

   // Clean previous data
   if ( !cadmodel->normals  ) 
   { rox_memory_delete( cadmodel->normals  ); cadmodel->normals  = NULL; }
   
   if ( !cadmodel->indices  ) 
   { rox_memory_delete( cadmodel->indices  ); cadmodel->indices  = NULL; }
   
   if ( !cadmodel->vertices ) 
   { rox_memory_delete( cadmodel->vertices ); cadmodel->vertices = NULL; }

   cadmodel->vertices = (float *)        rox_memory_allocate( sizeof(float)       , 3*n_vertices );
   if ( !cadmodel->vertices ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cadmodel->indices  = (unsigned int *) rox_memory_allocate( sizeof(unsigned int), 3*n_faces    );
   if ( !cadmodel->indices  ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cadmodel->normals  = (float *)        rox_memory_allocate( sizeof(float)       , 3*n_faces    );   
   if ( !cadmodel->normals  ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   memcpy( cadmodel->vertices, vertices, 3 * n_vertices * sizeof(float)        );
   memcpy( cadmodel->indices , indices , 3 * n_faces    * sizeof(unsigned int) );
   memcpy( cadmodel->normals , normals , 3 * n_faces    * sizeof(float)        );

   // Apply scale
   for ( size_t i = 0; i < 3 * n_vertices; i++ )
   {
      cadmodel->vertices[i] *= scale;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_compute_znear_zfar ( 
   Rox_Float * Z_near, 
   Rox_Float * Z_far, 
   const Rox_Float Z_min,
   const Rox_MatSE3 cTo, 
   const Rox_Point3D_Float box_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !Z_near || !Z_far || !box_points ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Compute z_near and z_far 
   for ( int ii = 0; ii < 8; ii++ ) // transform bounding box points to set Z_near and Z_far
   {
      Rox_Point3D_Float_Struct pt;

      error = rox_point3d_float_transform ( &pt, cTo, &(box_points[ii]), 1 );                                          
      ROX_ERROR_CHECK_TERMINATE ( error );

      float Z = ( pt.Z < Z_min ) ? Z_min : pt.Z;

      if ( 0 == ii )
      {
         *Z_near = Z;
         *Z_far  = Z;
      }
      else
      {
         if ( pt.Z < *Z_near ) *Z_near = Z;
         if ( pt.Z > *Z_far  ) *Z_far  = Z;
      }
   }

function_terminate:
   return error;  
}
