//==============================================================================
//
//    OPENROX   : File warp_grid_distortion.c
//
//    Contents  : Implementation of warp_grid_distortion module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "warp_grid_distortion.h"
#include <generated/array2d_float.h>

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_warp_grid_distortion_float (
   Rox_MeshGrid2D_Float grid,
   Rox_Array2D_Double calib_in,
   Rox_Array2D_Double calib_out,
   Rox_Array2D_Double distortion
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float pxi, pyi, u0i, v0i, skewi;
   Rox_Float pxo, pyo, u0o, v0o, skewo;
   Rox_Float ipxi, ipyi, iu0i, iv0i, iskewi;

   Rox_Float u,v;
   Rox_Float x,y,xx,yy,xy;
   Rox_Float r2,r4,radial,tangx,tangy;
   Rox_Float nx, ny;

   Rox_Double ** dki, **dko;
   Rox_Double ** ddist;
   Rox_Float * row_pu;
   Rox_Float * row_pv;

   if (!grid || !calib_in || !calib_out || !distortion) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(calib_in, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(distortion, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_meshgrid2d_float_get_size(&rows, &cols, grid); 
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_array2d_double_get_data_pointer_to_pointer( &ddist, distortion);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dki, calib_in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dko, calib_out);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dp_u;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dp_u, grid->u );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** dp_v;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dp_v, grid->v );
   ROX_ERROR_CHECK_TERMINATE ( error );

   pxi = (Rox_Float) dki[0][0]; pyi = (Rox_Float) dki[1][1];
   u0i = (Rox_Float) dki[0][2]; v0i = (Rox_Float) dki[1][2];
   skewi = (Rox_Float) dki[0][1];

   pxo = (Rox_Float) dko[0][0]; pyo = (Rox_Float) dko[1][1];
   u0o = (Rox_Float) dko[0][2]; v0o = (Rox_Float) dko[1][2];
   skewo = (Rox_Float) dko[0][1];

   // Inverse of calibration
   ipxi = 1.0f / pxi;
   ipyi = 1.0f / pyi;
   iskewi = -skewi/(pxi*pyi);
   iu0i = (skewi*v0i - u0i*pyi)/(pxi*pyi);
   iv0i = -v0i/pyi;

   // Get distortion constants
   Rox_Float k1 = (Rox_Float) ddist[0][0];
   Rox_Float k2 = (Rox_Float) ddist[1][0];
   Rox_Float k3 = (Rox_Float) ddist[2][0];
   Rox_Float k4 = (Rox_Float) ddist[3][0];
   Rox_Float k5 = (Rox_Float) ddist[4][0];

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      v = (Rox_Float) i;
      y = ipyi * v + iv0i;
      yy = y * y;

      row_pu = dp_u[i];
      row_pv = dp_v[i];

      for ( Rox_Sint j = 0; j < cols; j++)
      {
         u = (Rox_Float) j;
         x = ipxi * u + iskewi * v + iu0i;

         // precompute values
         xx = x * x;
         xy = x * y;
         r2 = yy + xx;
         r4 = r2 * r2;

         // Radial distortion
         radial = 1.0f + k1*r2 + k2*r4 + k5*r4*r2;

         // Tangential distortion
         tangx = 2.0f * k3 * xy + k4 * (r2 + 2.0f*xx);
         tangy = k3 * (r2 + 2.0f * yy) + 2.0f * k4 * xy;

         // Apply distortion
         nx = radial * x + tangx;
         ny = radial * y + tangy;

         // Back to pixels
         row_pu[j] = pxo * nx + skewo * ny + u0o;
         row_pv[j] = pyo * ny + v0o;
      }
   }

function_terminate:
   return error;
}
