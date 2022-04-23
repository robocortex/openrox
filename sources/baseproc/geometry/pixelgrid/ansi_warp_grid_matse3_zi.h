//==============================================================================
//
//    OPENROX   : File ansi_warp_grid_matse3_zi.h
//
//    Contents  : API of warp_grid_matse3_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

int rox_ansi_warp_grid_float_matse3_zi_float (
   float ** grid_u_data, 
   float ** grid_v_data,
   unsigned int ** grid_mask_data,
   float ** Zir_data,
   int rows,
   int cols,
   double ** cQr_data
);