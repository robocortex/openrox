//==============================================================================
//
//    OPENROX   : File ansi_warp_grid_matsl3_fixed12_4.h
//
//    Contents  : API of warp_grid_matsl3_fixed12_4 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_WARP_GRID_SL3_FIXED12_4__
#define __OPENROX_ANSI_WARP_GRID_SL3_FIXED12_4__

int rox_ansi_warp_grid_sl3_fixed12_4 (
   short ** grid_u_data, 
   short ** grid_v_data,
   int rows,
   int cols,
   double ** H_data
);

#endif