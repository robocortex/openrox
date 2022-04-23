//==============================================================================
//
//    OPENROX   : File dynvec_points3d_tools.c
//
//    Contents  : Implementation of dynvec points3d tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dynvec_point3d_tools.h"

#include <float.h>

#include <generated/dynvec_point3d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/point/point3d_tools.h>

#include <inout/system/errors_print.h>

#include <system/errors/errors.h>

#define THRESH_NUMERICAL_ZERO 1e-16


Rox_ErrorCode rox_dynvec_point3d_float_set_data (
   Rox_DynVec_Point3D_Float points3D,
   Rox_Float * data_points3D,
   Rox_Sint numb_points3D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Float_Struct point3D;

   // TODO check inputs

   for ( Rox_Sint i = 0; i<numb_points3D; i++)
   {
      point3D.X = data_points3D[3*i  ];
      point3D.Y = data_points3D[3*i+1];
      point3D.Z = data_points3D[3*i+2];
      error = rox_dynvec_point3d_float_append ( points3D, &point3D );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point3d_float_append_vector_point3d_double (
   Rox_DynVec_Point3D_Float points3D,
   Rox_Point3D_Double points3D_double,
   Rox_Sint numb_points3D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Float_Struct point3D_struct_float;

   // TODO check inputs

   for (Rox_Sint i=0; i<numb_points3D; i++)
   {
      point3D_struct_float.X = (Rox_Float) points3D_double[i].X;
      point3D_struct_float.Y = (Rox_Float) points3D_double[i].Y;
      point3D_struct_float.Z = (Rox_Float) points3D_double[i].Z;
      error = rox_dynvec_point3d_float_append(points3D, &point3D_struct_float);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point3d_double_append_vector_point3d_double (
   Rox_DynVec_Point3D_Double dynvec_points3D,
   Rox_Point3D_Double points3D,
   Rox_Sint numb_points3D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !dynvec_points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < numb_points3D; i++ )
   {
      error = rox_dynvec_point3d_double_append ( dynvec_points3D, &points3D[i] );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_set_data (
   Rox_DynVec_Point3D_Double points3D,
   const Rox_Double * data_points3D,
   const Rox_Sint numb_points3D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct point3D;

   if ( !points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !data_points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < numb_points3D; i++ )
   {
      point3D.X = data_points3D[3*i];
      point3D.Y = data_points3D[3*i+1];
      point3D.Z = data_points3D[3*i+2];
      error = rox_dynvec_point3d_double_append ( points3D, &point3D );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_set_data_vector (
   Rox_DynVec_Point3D_Double dynvec_points3D,
   const Rox_Point3D_Double  vector_points3D,
   const Rox_Sint numb_points3D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct point3D;

   if ( !dynvec_points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !vector_points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point3d_double_reset ( dynvec_points3D );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint k = 0; k < numb_points3D; k++ )
   {
      point3D.X = vector_points3D[k].X;
      point3D.Y = vector_points3D[k].Y;
      point3D.Z = vector_points3D[k].Z;
      error = rox_dynvec_point3d_double_append ( dynvec_points3D, &point3D );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_normalize_unit (
   Rox_DynVec_Point3D_Double point3D_dynvec
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !point3D_dynvec )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_vector_point3d_double_normalize_unit ( point3D_dynvec->data, point3D_dynvec->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_get_point3d (
  Rox_Point3D_Double point3D_vector,
  Rox_Sint                  *numb_points3D,
  Rox_DynVec_Point3D_Double  point3D_dynvec
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !point3D_vector )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !point3D_dynvec )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !numb_points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // TODO check inputs

   *numb_points3D = point3D_dynvec->used;

   for (Rox_Uint i=0; i< point3D_dynvec->used; i++ )
   {
      point3D_vector[i].X = point3D_dynvec->data[i].X;
      point3D_vector[i].Y = point3D_dynvec->data[i].Y;
      point3D_vector[i].Z = point3D_dynvec->data[i].Z;
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point3d_float_append_rectangle (
   Rox_DynVec_Point3D_Float model,
   Rox_Float model_size_x,
   Rox_Float model_size_y
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Point3D_Float_Struct model_rect;

   // create input model points
   model_rect.X = -model_size_x/2.0f;
   model_rect.Y = -model_size_y/2.0f;
   model_rect.Z = 0.0;

   error = rox_dynvec_point3d_float_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

   model_rect.X = +model_size_x/2.0f;
   model_rect.Y = -model_size_y/2.0f;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_float_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

   model_rect.X = +model_size_x/2.0f;
   model_rect.Y = +model_size_y/2.0f;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_float_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

   model_rect.X = -model_size_x/2.0f;
   model_rect.Y = +model_size_y/2.0f;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_float_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

   function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point3d_double_append_rectangle (
   Rox_DynVec_Point3D_Double model,
   const Rox_Double model_size_x,
   const Rox_Double model_size_y
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Point3D_Double_Struct model_rect;

   // create input model points
   model_rect.X = -model_size_x/2.0;
   model_rect.Y = -model_size_y/2.0;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_double_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

   model_rect.X = +model_size_x/2.0;
   model_rect.Y = -model_size_y/2.0;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_double_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

   model_rect.X = +model_size_x/2.0;
   model_rect.Y = +model_size_y/2.0;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_double_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

   model_rect.X = -model_size_x/2.0;
   model_rect.Y = +model_size_y/2.0;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_double_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_append_rectangle_newframe (
   Rox_DynVec_Point3D_Double model,
   const Rox_Double model_size_x,
   const Rox_Double model_size_y
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Point3D_Double_Struct model_rect;

   // create input model points
   model_rect.X = -model_size_x/2.0;
   model_rect.Y = +model_size_y/2.0;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_double_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

   model_rect.X = +model_size_x/2.0;
   model_rect.Y = +model_size_y/2.0;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_double_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

   model_rect.X = +model_size_x/2.0;
   model_rect.Y = -model_size_y/2.0;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_double_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

   model_rect.X = -model_size_x/2.0;
   model_rect.Y = -model_size_y/2.0;
   model_rect.Z = 0.0;
   error = rox_dynvec_point3d_double_append( model, &model_rect );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_new_rectangle (
   Rox_DynVec_Point3D_Double * model,
   const Rox_Double            model_size_x,
   const Rox_Double            model_size_y
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_dynvec_point3d_double_new( model, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_append_rectangle( *model, model_size_x, model_size_y );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate :
   if (error) rox_dynvec_point3d_double_del( model );
   return error;
}


Rox_ErrorCode rox_dynvec_point3d_double_mean (
   Rox_Point3D_Double pt_mean,
   Rox_DynVec_Point3D_Double pts_vec
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pt_mean || !pts_vec )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_vector_point3d_double_mean ( pt_mean, pts_vec->data, pts_vec->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_shift (
   Rox_DynVec_Point3D_Double pts_vec,
   Rox_Point3D_Double pt_shift
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pt_shift || !pts_vec )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_vector_point3d_double_shift ( pts_vec->data, pts_vec->used, pt_shift );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point3d_double_center_normalize (
   Rox_DynVec_Point3D_Double pts_dst,
   Rox_DynVec_Point3D_Double pts_src
)
{
   Rox_ErrorCode             error = ROX_ERROR_NONE;
   Rox_Uint                  n_src = 0;
   Rox_Point3D_Double_Struct center;

   if ( !pts_src || !pts_dst )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point3d_double_get_used( &n_src, pts_src );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( n_src > pts_dst->allocated )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point3d_double_mean( &center, pts_src );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint ii = 0; ii < n_src; ii++ )
   {
      pts_dst->data[ ii ].X = pts_src->data[ ii ].X;
      pts_dst->data[ ii ].Y = pts_src->data[ ii ].Y;
      pts_dst->data[ ii ].Z = pts_src->data[ ii ].Z;
   }
   pts_dst->used = n_src;

   center.X = - center.X;
   center.Y = - center.Y;
   center.Z = - center.Z;

   error = rox_dynvec_point3d_double_shift ( pts_dst, &center );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_normalize_unit ( pts_dst );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point3d_double_from_matrix (
   Rox_DynVec_Point3D_Double vec,
   Rox_Matrix mat
)
{
   Rox_ErrorCode   error=ROX_ERROR_NONE;
   Rox_Sint n_mat = 0;
   Rox_Uint n_vec = 0;
   Rox_Sint n_row = 0;
   Rox_Double ** mat_data=NULL;

   // Check inputs
   if ( NULL == mat || NULL == vec )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point3d_double_get_used( &n_vec, vec );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_get_cols( &n_mat, mat );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_get_rows( &n_row, mat );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( n_mat != n_vec )
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( n_row != 3 )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer( &mat_data, mat );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Fill
   for ( int ii = 0; ii < n_mat; ii++ )
   {
      vec->data[ii].X = mat_data[0][ii];
      vec->data[ii].Y = mat_data[1][ii];
      vec->data[ii].Z = mat_data[2][ii];
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_matrix_from_dynvec_point3d_double (
   Rox_Matrix mat,
   Rox_DynVec_Point3D_Double vec
)
{
   Rox_ErrorCode  error = ROX_ERROR_NONE;
   Rox_Sint       n_mat = 0;
   Rox_Uint       n_vec = 0;
   Rox_Sint       n_row = 0;
   Rox_Double     ** mat_data = NULL;

   // Check inputs
   if ( NULL == mat || NULL == vec )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point3d_double_get_used( &n_vec, vec );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_get_cols( &n_mat, mat );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_get_rows( &n_row, mat );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( n_mat != n_vec )
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( n_row != 3 )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer( &mat_data, mat );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Fill
   for ( int ii = 0; ii < n_mat; ii++ )
   {
      mat_data[0][ii] = vec->data[ii].X;
      mat_data[1][ii] = vec->data[ii].Y;
      mat_data[2][ii] = vec->data[ii].Z;
   }

function_terminate:
   return error;
}
