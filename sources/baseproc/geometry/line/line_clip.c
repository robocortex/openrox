//==============================================================================
//
//    OPENROX   : File line_clip.c
//
//    Contents  : Implementation of line_clip module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "line_clip.h"
#include <baseproc/geometry/rectangle/rectangle_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_line2d_clip_inside_rectangle (
   Rox_Point2D_Double pt1, 
   Rox_Point2D_Double pt2, 
   const Rox_Line2D_Homogeneous line, 
   const Rox_Rect_Real rect
)
{
   // A new approach to line and line segment clipping in homogeneous coordinates
   // Vaclav Skala
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct cmp[4];

   const Rox_Sint tab1[] = {-1,0,0,1,1,-1,0,2,2,0,-1,1,1,0,0,-1};
   const Rox_Sint tab2[] = {-1,3,1,3,2,-1,2,3,3,2,-1,2,3,1,3,-1};
   
   const Rox_Line2D_Homogeneous_Struct e[4] =
   {
      {0,1,-rect->y},
      {1,0,-(rect->x+rect->width)},
      {0,1,-(rect->y+rect->height)},
      {1,0,-rect->x}
   };
   
   if (!pt1 || !pt2) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   cmp[0].u = rect->x;
   cmp[0].v = rect->y;
   cmp[1].u = rect->x + rect->width;
   cmp[1].v = rect->y;
   cmp[2].u = rect->x + rect->width;
   cmp[2].v = rect->y + rect->height;
   cmp[3].u = rect->x;
   cmp[3].v = rect->y + rect->height;
   
   Rox_Uint c = 0;
   for (Rox_Uint idpt = 0; idpt < 4; idpt++)
   {
      Rox_Double dot = cmp[idpt].u * line->a + cmp[idpt].v * line->b + line->c;
      if (dot >= 0.0) c |= (1 << idpt);
   }
   
   if (c == 0 || c == 15) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint id1 = tab1[c];
   Rox_Sint id2 = tab2[c];
   
   Rox_Double la = line->b*e[id1].c-line->c*e[id1].b;
   Rox_Double lb = line->c*e[id1].a-line->a*e[id1].c;
   Rox_Double lc = line->a*e[id1].b-line->b*e[id1].a;

   pt1->u = la / lc;
   pt1->v = lb / lc;
   
   la = line->b*e[id2].c-line->c*e[id2].b;
   lb = line->c*e[id2].a-line->a*e[id2].c;
   lc = line->a*e[id2].b-line->b*e[id2].a;

   pt2->u = la / lc;
   pt2->v = lb / lc;
   
function_terminate:
   return error;
}