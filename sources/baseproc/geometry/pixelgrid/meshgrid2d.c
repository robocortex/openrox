//==============================================================================
//
//    OPENROX   : File meshgrid2d.c
//
//    Contents  : Implementation of meshgrid2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "meshgrid2d.h"
#include "meshgrid2d_struct.h"


Rox_ErrorCode rox_meshgrid2d_float_new ( 
   Rox_MeshGrid2D_Float * meshgrid2d, 
   const Rox_Sint rows, 
   const Rox_Sint cols
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MeshGrid2D_Float ret = NULL;
 
   if (!meshgrid2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *meshgrid2d = NULL;

   if ( cols < 1 || rows < 1 )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_MeshGrid2D_Float) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->u = NULL;
   ret->v = NULL;

   error = rox_array2d_float_new ( &ret->u, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &ret->v, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *meshgrid2d = ret;

function_terminate:
   if(error) rox_meshgrid2d_float_del(&ret);
   return error;
}

Rox_ErrorCode rox_meshgrid2d_float_del ( Rox_MeshGrid2D_Float * meshgrid2d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MeshGrid2D_Float todel = NULL;

   if (!meshgrid2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *meshgrid2d;
   *meshgrid2d = NULL;

   if ( !todel )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_float_del(&todel->u);
   rox_array2d_float_del(&todel->v);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_meshgrid2d_float_get_size (
   Rox_Sint * rows, 
   Rox_Sint * cols, 
   const Rox_MeshGrid2D_Float meshgrid2d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!meshgrid2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_get_size ( rows, cols, meshgrid2d->u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   function_terminate:
   return error; 
}

Rox_ErrorCode rox_meshgrid2d_float_check_size (   
   const Rox_MeshGrid2D_Float meshgrid2d,
   const Rox_Sint rows, 
   const Rox_Sint cols 
)
{
   return rox_array2d_float_check_size ( meshgrid2d->u, rows, cols );
}

Rox_ErrorCode rox_meshgrid2d_sshort_new ( 
   Rox_MeshGrid2D_Sshort * meshgrid2d, 
   const Rox_Sint rows, 
   const Rox_Sint cols
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MeshGrid2D_Sshort ret = NULL;
 
   if (!meshgrid2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *meshgrid2d = NULL;

   if ( cols < 1 || rows < 1 )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_MeshGrid2D_Sshort) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->u = NULL;
   ret->v = NULL;

   error = rox_array2d_sshort_new ( &ret->u, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sshort_new ( &ret->v, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *meshgrid2d = ret;

function_terminate:
   if(error) rox_meshgrid2d_sshort_del(&ret);
   return error;
}

Rox_ErrorCode rox_meshgrid2d_sshort_del ( Rox_MeshGrid2D_Sshort * meshgrid2d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MeshGrid2D_Sshort todel = NULL;

   if (!meshgrid2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *meshgrid2d;
   *meshgrid2d = NULL;

   if ( !todel )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_sshort_del(&todel->u);
   rox_array2d_sshort_del(&todel->v);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_meshgrid2d_sshort_get_size (
   Rox_Sint * rows, 
   Rox_Sint * cols, 
   const Rox_MeshGrid2D_Sshort meshgrid2d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!meshgrid2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_sshort_get_size ( rows, cols, meshgrid2d->u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   function_terminate:
   return error; 
}

Rox_ErrorCode rox_meshgrid2d_sshort_check_size (   
   const Rox_MeshGrid2D_Sshort meshgrid2d,
   const Rox_Sint rows, 
   const Rox_Sint cols 
)
{
   return rox_array2d_sshort_check_size ( meshgrid2d->u, rows, cols );
}
