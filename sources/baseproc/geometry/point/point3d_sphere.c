//==============================================================================
//
//    OPENROX   : File points3d_sphere.c
//
//    Contents  : Implementation of points3d_sphere module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "point3d_sphere.h"
#include <baseproc/maths/maths_macros.h>
#include <generated/dynvec_triangle_index.h>
#include <baseproc/geometry/point/point3d.h>
#include <generated/dynvec_point3d_double_struct.h>
#include <generated/dynvec_triangle_index_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_tesselation_normalize(Rox_Point3D_Double vertex);

Rox_ErrorCode rox_dynvec_point3d_double_append_unduplicate(Rox_Uint * idx, Rox_DynVec_Point3D_Double ptr, Rox_Point3D_Double data);

Rox_ErrorCode rox_tesselation_midpoint(Rox_Point3D_Double res, Rox_Point3D_Double a, Rox_Point3D_Double b)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!res) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Mid point of a and b ...
   res->X = (a->X + b->X) * 0.5;
   res->Y = (a->Y + b->Y) * 0.5;
   res->Z = (a->Z + b->Z) * 0.5;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tesselation_normalize(Rox_Point3D_Double vertex)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double mag = vertex->X * vertex->X + vertex->Y * vertex->Y + vertex->Z * vertex->Z;

   if (mag < DBL_EPSILON) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   mag = 1.0 / sqrt(mag);
   vertex->X *= mag;
   vertex->Y *= mag;
   vertex->Z *= mag;

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_append_unduplicate (
   Rox_Uint * idx,
   Rox_DynVec_Point3D_Double ptr,
   Rox_Point3D_Double data
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr || !data || !idx) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Is the point a duplicate ?
   Rox_Sint found = -1;
   for (Rox_Uint i = 0; i < ptr->used; i++)
   {
      Rox_Point3D_Double_Struct compare = ptr->data[i];

      if (compare.X == data->X && compare.Y == data->Y && compare.Z == data->Z)
      {
         found = i;
         break;
      }
   }

   // If not duplicate, add
   error = ROX_ERROR_NONE;
   if (found < 0)
   {
      found = ptr->used;
      error = rox_dynvec_point3d_double_append(ptr, data);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Return the true index of the point
   *idx = found;

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_new_from_sphere(Rox_DynVec_Point3D_Double  * obj, Rox_Uint nb_subdivs, Rox_Double maxangle)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Point3D_Double  ret;
   Rox_DynVec_Triangle_Index triangles_cur, triangles_next, triangles_swap;
   Rox_Point3D_Double_Struct  localvert[6];
   Rox_Uint localindices[6] = {0,0,0,0,0,0};
   Rox_Triangle_Index_Struct  localtriangles[4];
   const Rox_Double t = (1+sqrt(5.0))/2;
   const Rox_Double tau = t/sqrt(1+t*t);
   const Rox_Double one = 1/sqrt(1+t*t);
   Rox_Double limz = cos(ROX_PI * maxangle / 180.0);
   Rox_Uint pos;

   // Initial vertices on the isocahedron
   Rox_Point3D_Double_Struct  initial_isocahedron_vertices[] =
   {
      {  tau,  one,    0},
      { -tau,  one,    0},
      { -tau, -one,    0},
      {  tau, -one,    0},
      {  one,   0 ,  tau},
      {  one,   0 , -tau},
      { -one,   0 , -tau},
      { -one,   0 ,  tau},
      {   0 ,  tau,  one},
      {   0 , -tau,  one},
      {   0 , -tau, -one},
      {   0 ,  tau, -one}
   };

   Rox_Triangle_Index_Struct  initial_isocahedron_triangles[] =
   {
      {{ 4, 8, 7 }},
      {{ 4, 7, 9 }},
      {{ 5, 6, 11 }},
      {{ 5, 10, 6 }},
      {{ 0, 4, 3 }},
      {{ 0, 3, 5 }},
      {{ 2, 7, 1 }},
      {{ 2, 1, 6 }},
      {{ 8, 0, 11 }},
      {{ 8, 11, 1 }},
      {{ 9, 10, 3 }},
      {{ 9, 2, 10 }},
      {{ 8, 4, 0 }},
      {{ 11, 0, 5 }},
      {{ 4, 9, 3 }},
      {{ 5, 3, 10 }},
      {{ 7, 8, 1 }},
      {{ 6, 1, 11 }},
      {{ 7, 2, 9 }},
      {{ 6, 10, 2 }}
   };

   *obj = NULL;

   error = rox_dynvec_point3d_double_new(&ret, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_triangle_index_new(&triangles_cur, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_triangle_index_new(&triangles_next, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Add initial vertices
   for ( Rox_Sint idvert = 0; idvert < 12; idvert++)
   {
      rox_dynvec_point3d_double_append(ret, &initial_isocahedron_vertices[idvert]);
   }

   // Add initial triangles
   for ( Rox_Sint idtri = 0; idtri < 20; idtri++)
   {
      rox_dynvec_triangle_index_append(triangles_cur, &initial_isocahedron_triangles[idtri]);
   }

   // Operate n times recursively on triangles
   for (Rox_Uint iter = 0; iter < nb_subdivs; iter++)
   {
      // Reset the next level buffer
      rox_dynvec_triangle_index_reset(triangles_next);

      for (Rox_Uint idtri = 0; idtri < triangles_cur->used; idtri++)
      {
         // Get original vertices
         Rox_Triangle_Index_Struct curtri = triangles_cur->data[idtri];
         localindices[0] = curtri.points[0];
         localindices[1] = curtri.points[1];
         localindices[2] = curtri.points[2];

         localvert[0] = ret->data[localindices[0]];
         localvert[1] = ret->data[localindices[1]];
         localvert[2] = ret->data[localindices[2]];

         // Tesselate vertices
         rox_tesselation_midpoint(&localvert[3], &localvert[0], &localvert[1]);

         rox_tesselation_normalize(&localvert[3]);

         rox_tesselation_midpoint(&localvert[4], &localvert[1], &localvert[2]);

         rox_tesselation_normalize(&localvert[4]);

         rox_tesselation_midpoint(&localvert[5], &localvert[0], &localvert[2]);

         rox_tesselation_normalize(&localvert[5]);

         // Store vertices
         rox_dynvec_point3d_double_append_unduplicate(&localindices[3], ret, &localvert[3]);
         rox_dynvec_point3d_double_append_unduplicate(&localindices[4], ret, &localvert[4]);
         rox_dynvec_point3d_double_append_unduplicate(&localindices[5], ret, &localvert[5]);

         // Create 4 triangles instead of one
         localtriangles[0].points[0] = localindices[0];
         localtriangles[0].points[1] = localindices[3];
         localtriangles[0].points[2] = localindices[5];
         localtriangles[1].points[0] = localindices[3];
         localtriangles[1].points[1] = localindices[1];
         localtriangles[1].points[2] = localindices[4];
         localtriangles[2].points[0] = localindices[5];
         localtriangles[2].points[1] = localindices[4];
         localtriangles[2].points[2] = localindices[2];
         localtriangles[3].points[0] = localindices[5];
         localtriangles[3].points[1] = localindices[3];
         localtriangles[3].points[2] = localindices[4];

         // Store triangles
         rox_dynvec_triangle_index_append(triangles_next, &localtriangles[0]);
         rox_dynvec_triangle_index_append(triangles_next, &localtriangles[1]);
         rox_dynvec_triangle_index_append(triangles_next, &localtriangles[2]);
         rox_dynvec_triangle_index_append(triangles_next, &localtriangles[3]);
      }

      // Swap triangle buffers
      triangles_swap = triangles_cur;
      triangles_cur = triangles_next;
      triangles_next = triangles_swap;
   }

   // Filter out points not respecting the angle
   pos = 0;
   for (Rox_Uint idvert = 0; idvert < ret->used; idvert++)
   {
      if (ret->data[idvert].Z >= limz)
      {
         ret->data[pos] = ret->data[idvert];
         pos++;
      }
   }
   ret->used = pos;

   *obj = ret;

function_terminate:

   rox_dynvec_triangle_index_del(&triangles_cur);
   rox_dynvec_triangle_index_del(&triangles_next);

   return error;
}
