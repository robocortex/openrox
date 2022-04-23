//==============================================================================
//
//    OPENROX   : File point2d_print.c
//
//    Contents  : Implementation of point2d display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <stdio.h>
#include "point2d_print.h"

#include <system/errors/errors.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_point2d_float_print ( const Rox_Point2D_Float point2D )
{
    rox_log("point2D = [%f; %f]\n", point2D->u, point2D->v);
    return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_point2d_double_print ( const Rox_Point2D_Double point2D )
{
    rox_log("point2D = [%f; %f]\n", point2D->u, point2D->v);
    return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_point2d_uint_print ( const Rox_Point2D_Uint point2D )
{
    rox_log("point2D = [%d; %d]\n", point2D->u, point2D->v);
    return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_point2d_sint_print ( const Rox_Point2D_Sint point2D )
{
    rox_log("point2D = [%d; %d]\n", point2D->u, point2D->v);
    return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_vector_point2d_double_print ( 
  const Rox_Point2D_Double points2D, 
  const Rox_Sint nbpoints 
)
{   
   rox_log("vector of points 2D double :\n");
   for ( Rox_Sint k=0; k < nbpoints; k++ )
   {    
      rox_log("%f ", points2D[k].u);
   }
   rox_log("\n");

   for ( Rox_Sint k=0; k < nbpoints; k++ )
   {    
      rox_log("%f ", points2D[k].v);
   }
   rox_log("\n");

   return ROX_ERROR_NONE;
}
