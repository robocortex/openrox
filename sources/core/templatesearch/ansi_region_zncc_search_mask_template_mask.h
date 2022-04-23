//============================================================================
//
//    OPENROX   : File ansi_region_zncc_search_mask_template_mask.h
//
//    Contents  : API of region_zncc_search_mask_template_mask module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_ANSI_REGION_ZNCC_SEARCH_MASK_TEMPLATE_MASK__
#define __OPENROX_ANSI_REGION_ZNCC_SEARCH_MASK_TEMPLATE_MASK__

#include <generated/array2d_float.h>

// Defin the % of the template that should be visible
#define MIN_VISIBLE_RATIO 0.25 


int rox_ansi_array2d_float_region_zncc_search_mask_template_mask (
   float * res_score,
   int * res_topleft_x,
   int * res_topleft_y,
   float ** ds,
   unsigned int ** dsm,
   int sheight,
   int swidth,
   float ** dt,
   unsigned int ** dtm,
   int theight,
   int twidth
);

int rox_ansi_array2d_uchar_region_zncc_search_mask_template_mask (
   float * res_score,
   int * res_topleft_x,
   int * res_topleft_y,
   unsigned char ** ds,
   unsigned int ** dsm,
   int sheight,
   int swidth,
   unsigned char ** dt,
   unsigned int ** dtm,
   int theight,
   int twidth
);

#endif // __OPENROX_ANSI_REGION_ZNCC_SEARCH_MASK_TEMPLATE_MASK__
