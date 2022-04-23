//==============================================================================
//
//    OPENROX   : File ansi_remap_box_halved.h
//
//    Contents  : API of ansi_remap_box_halved module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

int rox_ansi_remap_box_nomask_uchar_to_uchar_halved (
   unsigned char ** dd,
   unsigned char ** ds,
   int hrows,
   int hcols
);

int rox_ansi_remap_box_nomask_float_to_float_halved (
   float ** dd,
   float ** ds,
   int hrows,
   int hcols
);