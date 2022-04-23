//==============================================================================
//
//    OPENROX   : File cylinder_transform.c
//
//    Contents  : Implementation of cylinder_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "cylinder_transform.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/cylinder/cylinder2d_struct.h>
#include <baseproc/geometry/cylinder/cylinder3d_struct.h>
#include <baseproc/geometry/segment/segment_transform.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_cylinder3d_transform(Rox_Cylinder3D cylinder3d_out, Rox_Array2D_Double matse3, Rox_Cylinder3D cylinder3d_inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder3d_out || !cylinder3d_inp || !matse3) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cylinder3d_out->a = cylinder3d_inp->a;
   cylinder3d_out->b = cylinder3d_inp->b;
   cylinder3d_out->h = cylinder3d_inp->h;

   error = rox_matse3_mulmatmat(cylinder3d_out->T, matse3, cylinder3d_inp->T);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder2d_transform(Rox_Cylinder2D cylinder2d_out, Rox_Array2D_Double matct2, Rox_Cylinder2D cylinder2d_inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder2d_out || !cylinder2d_inp || !matct2) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Tranform segments
   error = rox_segment2d_transform(cylinder2d_out->s1, matct2, cylinder2d_inp->s1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_segment2d_transform(cylinder2d_out->s2, matct2, cylinder2d_inp->s2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // TODO : Transform ellipses

function_terminate:
   return error;
}
