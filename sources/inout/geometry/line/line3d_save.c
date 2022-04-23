//==============================================================================
//
//    OPENROX   : File point3d_save.c
//
//    Contents  : Implementation of point3d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <stdio.h>
#include "line3d_save.h"
#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_line3d_planes_fprint ( FILE *stream, Rox_Line3D_Planes lines3d )
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!stream || !lines3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   fprintf(stream, "%f %f %f %f %f %f %f %f \n", lines3d->planes[0].a, lines3d->planes[0].b, lines3d->planes[0].c, lines3d->planes[0].d, lines3d->planes[1].a, lines3d->planes[1].b, lines3d->planes[1].c, lines3d->planes[1].d);

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_line3d_planes_save ( 
   const Rox_Char * filename, 
   const Rox_Line3D_Planes lines3d, 
   const Rox_Sint nb_lines
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *stream = fopen(filename, "w");

   if ( !filename || !lines3d ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < nb_lines; i ++ )
   {
      error = rox_line3d_planes_fprint ( stream, &lines3d[i] );
      ROX_ERROR_CHECK_TERMINATE ( error ); 
   }


function_terminate:
   fclose ( stream );
   return error;
}

Rox_ErrorCode rox_vector_line3d_planes_save_append (
   const Rox_Char * filename, 
   const Rox_Line3D_Planes lines3d, 
   const Rox_Sint nb_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !filename || !lines3d ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   FILE *stream = fopen(filename, "a");
   
   for (Rox_Sint i=0; i < nb_points; i ++)
   {
      error = rox_line3d_planes_fprint ( stream, &lines3d[i] );
      ROX_ERROR_CHECK_TERMINATE ( error ); 
   }

   fclose(stream);

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_line3d_planes_load ( 
   Rox_Line3D_Planes lines3d, 
   const Rox_Sint nb_points,
   const Rox_Char * filename 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !filename || !lines3d ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   FILE *stream = fopen(filename, "r");

   for ( Rox_Sint i = 0; i < nb_points; i ++ )
   {
      Rox_Sint nbr = 0; 
      nbr = fscanf ( stream, "%lf %lf %lf %lf %lf %lf %lf %lf", &lines3d[i].planes[0].a, &lines3d[i].planes[0].b, &lines3d[i].planes[0].c, &lines3d[i].planes[0].d , &lines3d[i].planes[1].a, &lines3d[i].planes[1].b, &lines3d[i].planes[1].c, &lines3d[i].planes[1].d );
      if (nbr != 8) 
      { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

   fclose ( stream );

function_terminate:
   return error;
}
