//==============================================================================
//
//    OPENROX   : File rectangle.c
//
//    Contents  : Implementation of rectangle module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "rectangle.h"

#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point2d_struct.h>

#include <inout/system/errors_print.h>
#include <system/errors/errors.h>

Rox_ErrorCode rox_rectangle3d_create_centered_plane_xright_ydown ( 
   Rox_Point3D_Double vertices, 
   const Rox_Double sizex, 
   const Rox_Double sizey 
)
{ 
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!vertices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   vertices[0].X = -sizex/2.0; vertices[0].Y = -sizey/2.0; vertices[0].Z = 0.0;
   vertices[1].X =  sizex/2.0; vertices[1].Y = -sizey/2.0; vertices[1].Z = 0.0;
   vertices[2].X =  sizex/2.0; vertices[2].Y =  sizey/2.0; vertices[2].Z = 0.0;
   vertices[3].X = -sizex/2.0; vertices[3].Y =  sizey/2.0; vertices[3].Z = 0.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_rectangle3d_create_centered_plane_xright_yup ( 
   Rox_Point3D_Double vertices, 
   const Rox_Double sizex, 
   const Rox_Double sizey 
)
{ 
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!vertices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   vertices[0].X = -sizex/2.0; vertices[0].Y = +sizey/2.0; vertices[0].Z = 0.0;
   vertices[1].X = +sizex/2.0; vertices[1].Y = +sizey/2.0; vertices[1].Z = 0.0;
   vertices[2].X = +sizex/2.0; vertices[2].Y = -sizey/2.0; vertices[2].Z = 0.0;
   vertices[3].X = -sizex/2.0; vertices[3].Y = -sizey/2.0; vertices[3].Z = 0.0;

function_terminate:
   return error;
}


Rox_ErrorCode rox_rectangle2d_create_centered_plane_xright_ydown ( 
   Rox_Point2D_Double vertices, 
   const Rox_Double sizex, 
   const Rox_Double sizey 
)
{ 
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!vertices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   vertices[0].u = -sizex/2.0; vertices[0].v = -sizey/2.0;
   vertices[1].u =  sizex/2.0; vertices[1].v = -sizey/2.0;
   vertices[2].u =  sizex/2.0; vertices[2].v =  sizey/2.0;
   vertices[3].u = -sizex/2.0; vertices[3].v =  sizey/2.0;

function_terminate:
   return error;
}



Rox_ErrorCode rox_rectangle2d_create_centered_plane_xright_yup ( 
   Rox_Point2D_Double vertices, 
   const Rox_Double sizex, 
   const Rox_Double sizey 
)
{ 
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!vertices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   vertices[0].u = -sizex/2.0; vertices[0].v = +sizey/2.0;
   vertices[1].u = +sizex/2.0; vertices[1].v = +sizey/2.0;
   vertices[2].u = +sizex/2.0; vertices[2].v = -sizey/2.0;
   vertices[3].u = -sizex/2.0; vertices[3].v = -sizey/2.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_rectangle2d_create_image_coordinates ( 
   Rox_Point2D_Double vertices, 
   const Rox_Double cols, 
   const Rox_Double rows 
)
{ 
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!vertices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   vertices[0].u = -0.5;         vertices[0].v = -0.5;
   vertices[1].u = cols - 0.5;   vertices[1].v = -0.5;
   vertices[2].u = cols - 0.5;   vertices[2].v = rows - 0.5;
   vertices[3].u = -0.5;         vertices[3].v = rows - 0.5;
   
function_terminate:
  return error;
}
