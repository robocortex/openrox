//==============================================================================
//
//    OPENROX   : File point2d_save.c
//
//    Contents  : Implementation of point2d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <stdio.h>
#include "point2d_save.h"
#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_point2d_double_fprint ( FILE *stream, Rox_Point2D_Double point2D )
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!stream || !point2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   fprintf ( stream, "%16.16f %16.16f \n", point2D->u, point2D->v );

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point2d_double_save ( 
   const Rox_Char * filename, 
   const Rox_Point2D_Double points2D, 
   const Rox_Sint nb_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !filename || !points2D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   FILE *stream = fopen(filename, "w");

   for ( Rox_Sint i = 0; i < nb_points; i ++ )
   {
      error = rox_point2d_double_fprint ( stream, &points2D[i] );
      ROX_ERROR_CHECK_TERMINATE ( error ); 
   }

   fclose ( stream );

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point2d_double_save_append (
   const Rox_Char * filename, 
   const Rox_Point2D_Double points2D, 
   const Rox_Sint nb_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!filename || !points2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   FILE *stream = fopen(filename, "a");
   
   for ( Rox_Sint i=0; i < nb_points; i++ )
   {
      error = rox_point2d_double_fprint(stream, &points2D[i]);
      ROX_ERROR_CHECK_TERMINATE ( error ); 
   }

   fclose(stream);

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point2d_double_load ( 
   Rox_Point2D_Double points2D, 
   const Rox_Sint nb_points,
   const Rox_Char * filename
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !filename || !points2D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   FILE *stream = fopen(filename, "r");

   if(!stream)
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < nb_points; i ++ )
   {
      Rox_Sint nbr = 0;
      nbr = fscanf ( stream, "%lf %lf", &points2D[i].u, &points2D[i].v );
      if (nbr != 2) 
      { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

   fclose ( stream );

function_terminate:
   return error;
}
