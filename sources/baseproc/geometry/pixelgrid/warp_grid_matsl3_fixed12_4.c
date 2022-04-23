//==============================================================================
//
//    OPENROX   : File warp_grid_matsl3_fixed12_4.c
//
//    Contents  : Implementation of warp_grid_matsl3_fixed12_4 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "warp_grid_matsl3_fixed12_4.h"
#include "ansi_warp_grid_matsl3_fixed12_4.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_warp_grid_sl3_fixed12_4 (
   Rox_MeshGrid2D_Sshort grid,
   Rox_Array2D_Double homography
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!grid || !homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** H_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer (&H_data, homography );

   // Rox_Point2D_Sshort * mxy = NULL;
   // error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer ( &mxy, grid );

   Rox_Sshort ** grid_u_data = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer( &grid_u_data, grid->u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sshort ** grid_v_data = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer( &grid_v_data, grid->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_meshgrid2d_sshort_get_size ( &rows, &cols, grid );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_warp_grid_sl3_fixed12_4 ( grid_u_data, grid_v_data, rows, cols, H_data );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
