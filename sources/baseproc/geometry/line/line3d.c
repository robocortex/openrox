//==============================================================================
//
//    OPENROX   : File line_3d.c
//
//    Contents  : Implementation of line 3D module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "line3d.h"
#include "line3d_struct.h"

#include <math.h>
#include <float.h>

#include <system/errors/errors.h>

#include <baseproc/geometry/point/point3d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_line3d_parametric_get_point3d (
   Rox_Point3D_Double point3d, 
   const Rox_Line3D_Parametric line3d_parametric, 
   const Rox_Double lambda
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !point3d || !line3d_parametric ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   point3d->X = line3d_parametric->origin[0] + lambda * line3d_parametric->direction[0];
   point3d->Y = line3d_parametric->origin[1] + lambda * line3d_parametric->direction[1];
   point3d->Z = line3d_parametric->origin[2] + lambda * line3d_parametric->direction[2];

function_terminate:
   return error;
}


Rox_ErrorCode rox_line3d_get_farthest_plane ( Rox_Double * a, Rox_Double * b, Rox_Double * c, Rox_Double * d, const Rox_Line3D_Planes line3D )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !line3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !a || !b || !c || !d )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double d0 = line3D->planes[0].d;
   Rox_Double d1 = line3D->planes[1].d;

   if ( fabs(d1) >= fabs(d0) )
   {
      *d = d1;
      *c = line3D->planes[1].c;
      *b = line3D->planes[1].b;
      *a = line3D->planes[1].a;
   }
   else
   {
      *d = d0;
      *c = line3D->planes[0].c;
      *b = line3D->planes[0].b;
      *a = line3D->planes[0].a;
   }

function_terminate:
   return error;
}
