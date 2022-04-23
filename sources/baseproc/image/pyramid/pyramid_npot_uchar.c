//==============================================================================
//
//    OPENROX   : File pyramid_npot_uchar.c
//
//    Contents  : Implementation of pyramid non power of two uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "pyramid_npot_uchar.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/image/pyramid/pyramid_tools.h>
#include <baseproc/image/pyramid/pyramid_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_bilinear_nomask_uchar_to_uchar/remap_bilinear_nomask_uchar_to_uchar.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_pyramid_npot_uchar_new (
   Rox_Pyramid_Npot_Uchar * obj,
   const Rox_Sint width, 
   const Rox_Sint height, 
   const Rox_Uint nb_levels, 
   const Rox_Double zoom_perlevel
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Pyramid_Npot_Uchar ret = NULL;
   Rox_Double lwidth;
   Rox_Double lheight;
   Rox_Uint level;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (nb_levels == 0)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_Pyramid_Npot_Uchar) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->base_width = width;
   ret->base_height = height;
   ret->nb_levels = nb_levels;
   ret->grids = NULL;
   ret->levels = NULL;
   ret->homography = NULL;

   error = rox_array2d_double_new(&ret->homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillunit(ret->homography);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_set_value(ret->homography, 0, 0, zoom_perlevel);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(ret->homography, 1, 1, zoom_perlevel);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->levels = (Rox_Image *) rox_memory_allocate(sizeof(struct Rox_Image *), nb_levels);
   if (!ret->levels)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (level = 0; level < nb_levels; level++)
   {
      ret->levels[level] = NULL;
   }

   lwidth = ret->base_width;
   lheight = ret->base_height;
   for (level = 0; level < nb_levels; level++)
   {
      error = rox_array2d_uchar_new(&ret->levels[level], (Rox_Sint)lheight, (Rox_Sint)lwidth);
      ROX_ERROR_CHECK_TERMINATE ( error );

      lwidth /= zoom_perlevel;
      lheight /= zoom_perlevel;
      lwidth -= 1;
      lheight -= 1;
   }

   ret->grids = (Rox_MeshGrid2D_Float *) rox_memory_allocate(sizeof(struct Rox_MeshGrid2D_Float *), nb_levels - 1);
   if (!ret->grids)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (level = 0; level < nb_levels - 1; level++)
   {
      ret->grids[level] = NULL;
   }

   lwidth = ret->base_width;
   lheight = ret->base_height;
   for (level = 0; level < nb_levels - 1; level++)
   {
      lwidth /= zoom_perlevel;
      lheight /= zoom_perlevel;
      lwidth -= 1;
      lheight -= 1;

      error = rox_meshgrid2d_float_new(&ret->grids[level], (Rox_Sint)lheight, (Rox_Sint)lwidth);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_warp_grid_sl3_float(ret->grids[level], ret->homography);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   *obj = ret;

function_terminate:
   if (error) rox_pyramid_npot_uchar_del(&ret);
   return error;
}

Rox_ErrorCode rox_pyramid_npot_uchar_del(Rox_Pyramid_Npot_Uchar * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Pyramid_Npot_Uchar todel = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (todel->levels)
   {
      for (Rox_Uint i = 0; i < todel->nb_levels; i++)
      {
         rox_array2d_uchar_del(&todel->levels[i]);
      }

      rox_memory_delete(todel->levels);
   }

   if (todel->grids)
   {
      for (Rox_Uint i = 0; i < todel->nb_levels - 1; i++)
      {
         rox_meshgrid2d_float_del(&todel->grids[i]);
      }

      rox_memory_delete(todel->grids);
   }

   rox_array2d_double_del(&todel->homography);

   // Delete object
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_pyramid_npot_uchar_assign (
   Rox_Pyramid_Npot_Uchar obj,
   const Rox_Image source
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_copy(obj->levels[0], source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 1; i < obj->nb_levels; i++)
   {
      error = rox_remap_bilinear_nomask_uchar_to_uchar(obj->levels[i], obj->levels[i - 1], obj->grids[i - 1]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
