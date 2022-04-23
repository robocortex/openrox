//==============================================================================
//
//    OPENROX   : File line2d_print.c
//
//    Contents  : Implementation of line2d display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <stdio.h>
#include "line2d_print.h"
#include <baseproc/geometry/line/line2d_struct.h>

#include <inout/system/print.h>
#include <system/errors/errors.h>

Rox_ErrorCode rox_line2d_normal_print ( const Rox_Line2D_Normal line2d_normal )
{
    rox_log ( "line2d normal [rho, theta] = [%f; %f]\n", line2d_normal->rho, line2d_normal->theta );
    return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_line2d_normal_vector_print ( 
  Rox_Line2D_Normal lines2d_normal, 
  const Rox_Sint nblines 
)
{   
   rox_log("vector of lines 2D double :\n");
   for(Rox_Sint k=0; k < nblines; k++)
   {    
      rox_log("%f ", lines2d_normal[k].rho);
   }
   rox_log("\n");

   for(Rox_Sint k=0; k < nblines; k++)
   {    
      rox_log("%f ", lines2d_normal[k].theta);
   }
   rox_log("\n");

   return ROX_ERROR_NONE;
}
