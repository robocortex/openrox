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
#include "point3d_save.h"
#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_point3d_double_fprint ( FILE *stream, Rox_Point3D_Double point3D )
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!stream || !point3D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   fprintf(stream, "%f %f %f \n", point3D->X, point3D->Y, point3D->Z);

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point3d_double_save ( 
   const Rox_Char * filename, 
   const Rox_Point3D_Double points3D, 
   const Rox_Sint nb_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *stream = fopen(filename, "w");

   if ( !filename || !points3D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < nb_points; i ++ )
   {
      error = rox_point3d_double_fprint ( stream, &points3D[i] );
      ROX_ERROR_CHECK_TERMINATE ( error ); 
   }


function_terminate:
   fclose ( stream );
   return error;
}

Rox_ErrorCode rox_vector_point3d_double_save_append (
   const Rox_Char * filename, 
   const Rox_Point3D_Double points3D, 
   const Rox_Sint nb_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !filename || !points3D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   FILE *stream = fopen(filename, "a");
   
   for (Rox_Sint i=0; i < nb_points; i ++)
   {
      error = rox_point3d_double_fprint ( stream, &points3D[i] );
      ROX_ERROR_CHECK_TERMINATE ( error ); 
   }

   fclose(stream);

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point3d_double_load ( 
   Rox_Point3D_Double points3D, 
   const Rox_Sint nb_points,
   const Rox_Char * filename 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !filename || !points3D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   FILE *stream = fopen(filename, "r");

   for ( Rox_Sint i = 0; i < nb_points; i ++ )
   {
      Rox_Sint nbr = 0;
      nbr = fscanf ( stream, "%lf %lf %lf", &points3D[i].X, &points3D[i].Y, &points3D[i].Z );
      if (nbr != 3) 
      { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

   fclose ( stream );

function_terminate:
   return error;
}
