//==============================================================================
//
//    OPENROX   : File patchplane.c
//
//    Contents  : Implementation of patchplane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "patchplane.h"

#include <system/errors/errors.h>
#include <system/memory/memory.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_bilinear_omo_float_to_float/remap_bilinear_omo_float_to_float.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/scale/scaleshift.h>
#include <baseproc/array/mean/mean.h>
#include <baseproc/array/band/band.h>
#include <baseproc/image/gradient/basegradient.h>
#include <baseproc/array/crosscor/zncrosscor.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/numeric/array2d_save.h>

#include <stdio.h>

Rox_ErrorCode rox_patchplane_new ( Rox_PatchPlane *patch_plane, Rox_Sint height, Rox_Sint width )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PatchPlane ret = NULL;

   if (!patch_plane) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   *patch_plane = NULL;

   ret = (Rox_PatchPlane)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->alpha = 1.0;
   ret->beta  = 0.0;

   // Reference info
   ret->reference = NULL;
   ret->reference_mask = NULL;

   // Remap info
   ret->grid = NULL;

   // Warp
   ret->current = NULL;
   ret->current_mask = NULL;

   // Gradient
   ret->gx = NULL;
   ret->gy = NULL;
   ret->gradient_mask = NULL;

   // Misc
   ret->warped_lum = NULL;
   ret->difference = NULL;
   ret->mean = NULL;
   ret->mean_lum = NULL;

   error = rox_array2d_float_new(&ret->reference, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret->reference_mask, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&ret->grid, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->current, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->gx, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->gy, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret->gradient_mask, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->warped_lum, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret->current_mask, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->difference, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->mean, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->mean_lum, height, width); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->width = width;
   ret->height = height;

   *patch_plane = ret;

function_terminate:
   if (error) rox_patchplane_del(&ret);

   return error;
}

Rox_ErrorCode rox_patchplane_del ( Rox_PatchPlane * patch_plane )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PatchPlane todel = NULL;

   if (!patch_plane) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *patch_plane;
   *patch_plane = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_float_del(&todel->reference);
   rox_array2d_uint_del(&todel->reference_mask);
   rox_array2d_uint_del(&todel->current_mask);
   rox_meshgrid2d_float_del(&todel->grid);
   rox_array2d_float_del(&todel->current);
   rox_array2d_float_del(&todel->gx);
   rox_array2d_float_del(&todel->gy);
   rox_array2d_uint_del(&todel->gradient_mask);
   rox_array2d_float_del(&todel->warped_lum);
   rox_array2d_float_del(&todel->difference);
   rox_array2d_float_del(&todel->mean);
   rox_array2d_float_del(&todel->mean_lum);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_set_reference ( 
   Rox_PatchPlane patch_plane, 
   Rox_Array2D_Float source, 
   Rox_Imask sourcemask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!patch_plane || !source || !sourcemask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_copy(patch_plane->reference, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uint_copy(patch_plane->reference_mask, sourcemask); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_reset_luminance(Rox_PatchPlane patch_plane)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!patch_plane) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   patch_plane->alpha = 1.0;
   patch_plane->beta = 0.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_prepare_sl3 ( 
   Rox_PatchPlane patch_plane, 
   const Rox_Array2D_Double homography, 
   const Rox_Array2D_Float source
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !patch_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !homography || !source )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_check_size ( homography, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Warp coordinates
   error = rox_warp_grid_sl3_float ( patch_plane->grid, homography ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Remap with bilinear interpolation
   error = rox_remap_bilinear_omo_float_to_float ( patch_plane->current, patch_plane->current_mask, source, patch_plane->grid ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Merge masks
   error = rox_array2d_uint_band ( patch_plane->current_mask, patch_plane->current_mask, patch_plane->reference_mask ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_patchplane_prepare_finish ( Rox_PatchPlane patch_plane )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !patch_plane ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Luminance
   error = rox_array2d_float_scaleshift ( patch_plane->warped_lum, patch_plane->current, patch_plane->alpha, patch_plane->beta); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Mean
   error = rox_array2d_float_mean ( patch_plane->mean, patch_plane->reference, patch_plane->current); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_mean ( patch_plane->mean_lum, patch_plane->reference, patch_plane->warped_lum); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Difference
   error = rox_array2d_float_substract ( patch_plane->difference, patch_plane->reference, patch_plane->warped_lum); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Gradient
   error = rox_array2d_float_basegradient ( patch_plane->gx, patch_plane->gy, patch_plane->gradient_mask, patch_plane->mean_lum, patch_plane->current_mask); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_patchplane_compute_score ( 
   Rox_Double * score, 
   Rox_PatchPlane patch_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!score || !patch_plane) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_float_zncc_normalizedscore ( score, patch_plane->reference, patch_plane->warped_lum, patch_plane->current_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}
