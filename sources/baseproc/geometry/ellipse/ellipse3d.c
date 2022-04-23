//==============================================================================
//
//    OPENROX   : File ellipse3d.c
//
//    Contents  : Implementation of ellipse_3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#include "ellipse3d.h"

#include <baseproc/geometry/ellipse/ellipse3d_struct.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_ellipse3d_new(Rox_Ellipse3D * ellipse3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ellipse3D ret = NULL;

   if (!ellipse3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *ellipse3d = NULL;

   ret = (Rox_Ellipse3D) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->Te = NULL;

   error = rox_matse3_new(&ret->Te);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->a = 1.0;
   ret->b = 1.0;

   *ellipse3d = ret;
   
function_terminate:
   if(error) rox_ellipse3d_del(&ret);
   return error;
}

Rox_ErrorCode rox_ellipse3d_del(Rox_Ellipse3D * ellipse3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ellipse3D todel = NULL;

   if (!ellipse3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ellipse3d;
   *ellipse3d = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matse3_del(&todel->Te);
   
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse3d_set(Rox_Ellipse3D ellipse3d, Rox_Double a, Rox_Double b, Rox_MatSE3 T)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (a <= 0.0 || b <= 0.0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ellipse3d->a = a;
   ellipse3d->b = b;

   error = rox_matse3_copy(ellipse3d->Te, T);   
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse3d_copy(Rox_Ellipse3D ellipse3d_out, Rox_Ellipse3D ellipse3d_inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse3d_out || !ellipse3d_inp)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ellipse3d_out->a = ellipse3d_inp->a;
   ellipse3d_out->b = ellipse3d_inp->b;
   error = rox_matse3_copy(ellipse3d_out->Te, ellipse3d_inp->Te);
   ROX_ERROR_CHECK_TERMINATE ( error ); 
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse3d_print(Rox_Ellipse3D ellipse3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   rox_log("ellipse3d : \n");
   rox_log("a = %f \n", ellipse3d->a);
   rox_log("b = %f \n", ellipse3d->b);
   rox_log("pose Te : \n");
   rox_matse3_print(ellipse3d->Te);

function_terminate:
   return error;
}