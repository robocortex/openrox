//==============================================================================
//
//    OPENROX   : File dynvec_point2d_print.c
//
//    Contents  : Implementation of point2d_print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dynvec_point2d_print.h"
#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point2d_uint_struct.h>

#include <system/errors/errors.h>

#include <inout/geometry/point/point2d_print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_dynvec_point2d_float_print(Rox_DynVec_Point2D_Float points2D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!points2D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Uint k = 0; k < points2D->used; k++)
   {
      rox_point2d_float_print(&points2D->data[k]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_double_print(Rox_DynVec_Point2D_Double points2D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!points2D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Uint k = 0; k < points2D->used; k++)
   {
      rox_point2d_double_print(&points2D->data[k]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_uint_print(Rox_DynVec_Point2D_Uint points2D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!points2D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Uint k = 0; k < points2D->used; k++)
   {
      rox_point2d_uint_print(&points2D->data[k]);
   }

function_terminate:
   return error;
}
