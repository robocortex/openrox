//==============================================================================
//
//    OPENROX   : File warp_grid_matse3_zi.c
//
//    Contents  : Implementation of warp_grid_matse3_zi module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "warp_grid_matse3_zi.h"
#include "ansi_warp_grid_matse3_zi.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_warp_grid_float_matse3_zi_float (
   Rox_MeshGrid2D_Float grid_warp, 
   Rox_Imask grid_mask, 
   const Rox_Array2D_Float Zir, 
   const Rox_MatSE3 cTr, 
   const Rox_MatUT3 Kr, 
   const Rox_MatUT3 Kc
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Array2D_Double cQr = NULL;
   
   if ( !grid_warp || !grid_mask )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !Zir || !cTr || !Kr || !Kc ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, Zir );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_check_size ( grid_warp, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uint_check_size ( grid_mask, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matut3_check_size ( Kr ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matut3_check_size ( Kc ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_check_size ( cTr ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new ( &cQr, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_transformtools_build_matsa3_matrix ( cQr, cTr, Kc, Kr ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** cQr_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &cQr_data, cQr );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** grid_u_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_u_data, grid_warp->u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** grid_v_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &grid_v_data, grid_warp->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Zir_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Zir_data, Zir );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** grid_mask_data = NULL;
   error = rox_imask_get_data_pointer_to_pointer ( &grid_mask_data, grid_mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_warp_grid_float_matse3_zi_float ( grid_u_data, grid_v_data, grid_mask_data, Zir_data, rows, cols, cQr_data );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:   

   rox_array2d_double_del ( &cQr );

   return error;  
}