//==============================================================================
//
//    OPENROX   : File image_rgba_warp_matsl3.c
//
//    Contents  : Implementation of image rgba warp matsl3 module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "image_rgba_warp_matsl3.h"
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>
#include <baseproc/image/remap/remap_bilinear_nomask_uchar_to_uchar/remap_bilinear_nomask_uchar_to_uchar.h>
#include <baseproc/image/remap/remap_bilinear_nomask_uint_to_uint/remap_bilinear_nomask_uint_to_uint.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_rgba_warp_matsl3 (
   Rox_Image_RGBA image_warped, 
   const Rox_Image_RGBA image_source, 
   const Rox_MatSL3 homography
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MeshGrid2D_Float grids = NULL;

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, image_warped);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&grids, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_warp_grid_sl3_float(grids, homography); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_remap_bilinear_nomask_rgba(image_warped, image_source, grids); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   rox_meshgrid2d_float_del(&grids);
   return(error);
}
