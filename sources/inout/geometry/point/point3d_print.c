//==============================================================================
//
//    OPENROX   : File point3d_print.c
//
//    Contents  : Implementation of point3d display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <stdio.h>
#include "point3d_print.h"

#include <system/errors/errors.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_point3d_float_print ( const Rox_Point3D_Float point3D )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !point3D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("point3D = [%16.16f; %16.16f; %16.16f]\n", point3D->X, point3D->Y, point3D->Z);

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_double_print ( const Rox_Point3D_Double point3D )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !point3D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("point3D = [%16.16f; %16.16f; %16.16f]\n", point3D->X, point3D->Y, point3D->Z);

function_terminate:
   return error;}

Rox_ErrorCode rox_vector_point3d_double_print ( Rox_Point3D_Double points3D, Rox_Sint nbpoints )
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !points3D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("vector of points 3D double :\n");

   for(Rox_Sint k=0; k < nbpoints; k++)
   {    
      rox_log("%f ", points3D[k].X);
   }
   rox_log("\n");

   for(Rox_Sint k=0; k < nbpoints; k++)
   {    
      rox_log("%f ", points3D[k].Y);
   }
   rox_log("\n");

   for(Rox_Sint k=0; k < nbpoints; k++)
   {    
      rox_log("%f ", points3D[k].Z);
   }
   rox_log("\n");

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_float_vector_print ( Rox_Point3D_Float points3D, Rox_Sint nbpoints )
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !points3D ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("vector of points 3D float :\n");
   for(Rox_Sint k=0; k < nbpoints; k++)
   {    
      rox_log("%f ", points3D[k].X);
   }
   rox_log("\n");

   for(Rox_Sint k=0; k < nbpoints; k++)
   {    
      rox_log("%f ", points3D[k].Y);
   }
   rox_log("\n");

   for(Rox_Sint k=0; k < nbpoints; k++)
   {    
      rox_log("%f ", points3D[k].Z);
   }
   rox_log("\n");

function_terminate:
   return error;
}
