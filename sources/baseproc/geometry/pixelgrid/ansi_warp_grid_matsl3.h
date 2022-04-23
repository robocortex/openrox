//==============================================================================
//
//    OPENROX   : File ansi_warp_grid_matsl3.h
//
//    Contents  : API of warp_grid_matsl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_WARP_GRID_SL3__
#define __OPENROX_ANSI_WARP_GRID_SL3__

int rox_ansi_warp_grid_sl3_float (
   float ** grid_u_data, 
   float ** grid_v_data,
   int rows,
   int cols,
   double ** H_data
);

#endif