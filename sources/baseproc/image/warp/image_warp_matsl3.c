//==============================================================================
//
//    OPENROX   : File image_warp_matsl3.h
//
//    Contents  : API of image warping module with matrix in SL3
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "image_warp_matsl3.h"
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>

#include <baseproc/image/remap/remap_bilinear_nomask_uchar_to_uchar/remap_bilinear_nomask_uchar_to_uchar.h>
#include <baseproc/image/remap/remap_bilinear_nomask_float_to_float/remap_bilinear_nomask_float_to_float.h>
#include <baseproc/image/remap/remap_bilinear_omo_float_to_float/remap_bilinear_omo_float_to_float.h>
#include <baseproc/image/remap/remap_bilinear_omo_uchar_to_uchar/remap_bilinear_omo_uchar_to_uchar.h>

#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_warp_matsl3 (
   Rox_Image image_warped, 
   const Rox_Image image_source, 
   const Rox_MatSL3 homography)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MeshGrid2D_Float grid = NULL;

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, image_warped);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&grid, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_warp_grid_sl3_float(grid, homography); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_remap_bilinear_nomask_uchar_to_uchar(image_warped, image_source, grid); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   rox_meshgrid2d_float_del(&grid);
   return(error);
}

Rox_ErrorCode rox_image_imask_warp_matsl3(Rox_Image image_warped, Rox_Imask imask_warped, Rox_Image image_source, Rox_MatSL3 homography)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MeshGrid2D_Float grid = NULL;

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, image_warped);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&grid, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_warp_grid_sl3_float(grid, homography); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_remap_bilinear_omo_uchar_to_uchar(image_warped, imask_warped, image_source, grid); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   rox_meshgrid2d_float_del(&grid);
   return(error);
}

Rox_ErrorCode rox_array2d_float_warp_matsl3(Rox_Array2D_Float image_warped, Rox_Array2D_Float image_source, Rox_MatSL3 homography)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MeshGrid2D_Float grid = NULL;

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_float_get_size(&rows, &cols, image_warped);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&grid, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_warp_grid_sl3_float(grid, homography); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_remap_bilinear_nomask_float_to_float(image_warped, image_source, grid); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   rox_meshgrid2d_float_del(&grid);
   return(error);
}

Rox_ErrorCode rox_array2d_float_imask_warp_matsl3(Rox_Array2D_Float image_warped, Rox_Imask imask_warped, Rox_Array2D_Float image_source, Rox_MatSL3 homography)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MeshGrid2D_Float grid = NULL;

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_float_get_size(&rows, &cols, image_warped);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new(&grid, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_warp_grid_sl3_float(grid, homography); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_remap_bilinear_omo_float_to_float(image_warped, imask_warped, image_source, grid); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   rox_meshgrid2d_float_del(&grid);
   return(error);
}
