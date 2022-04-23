//==============================================================================
//
//    OPENROX   : File point3d_inverse_projection_from_point2d.c
//
//    Contents  : Implementation of inverse_projection module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "point3d_inverse_projection_from_point2d.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point2d_tools.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/plane/plane_transform.h>

#include <inout/system/errors_print.h>


Rox_ErrorCode rox_point3d_intersection_vector_plane ( 
   Rox_Point3D_Double m, 
   const Rox_Point2D_Double q, 
   const Rox_Plane3D_Double plane3d 
)
{     
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !m )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !plane3d || !q )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double Z = -plane3d->d / (plane3d->a * q->u + plane3d->b * q->v + plane3d->c );

   m->X = Z * q->u; 
   m->Y = Z * q->v; 
   m->Z = Z ; 

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_double_coplanar_inverse_projection_from_point2d_double (
   Rox_Point3D_Double mo,  
   const Rox_Plane3D_Double plane3d_o,
   const Rox_MatUT3 Kc, 
   const Rox_MatSE3 cTo,
   const Rox_Point2D_Double pc, 
   const Rox_Sint nbpts
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double mc = NULL;
   Rox_Point2D_Double qc = NULL;
   Rox_Plane3D_Double_Struct plane3d_c;
   Rox_MatSE3 oTc = NULL;

   if ( !mo )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !plane3d_o || !Kc || !cTo || !pc )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   qc = ( Rox_Point2D_Double ) rox_memory_allocate(sizeof(*qc), nbpts);
   if (!qc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   mc = ( Rox_Point3D_Double ) rox_memory_allocate(sizeof(*mc), nbpts);
   if (!mc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_plane3d_transform ( &plane3d_c, cTo, plane3d_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint k = 0; k < nbpts; k++)
   {
      error = rox_point2d_convert_pixel_double_to_meter_double ( &qc[k], &pc[k], Kc );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_point3d_intersection_vector_plane ( &mc[k], &qc[k], &plane3d_c );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_matse3_new ( &oTc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_inv ( oTc, cTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point3d_double_transform ( mo, oTc, mc, nbpts );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_memory_delete ( qc );
   rox_memory_delete ( mc );
   return error;
}
