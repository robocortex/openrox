//==============================================================================
//
//    OPENROX   : File quad_detection.c
//
//    Contents  : Implementation of quad_detection module
//                Detection of quadrilaterals in an image
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "quad_detection.h"
#include "quad_detection_struct.h"

#include <generated/dynvec_quad_segment2d_struct.h>
#include <generated/objset_dynvec_orientedimagepoint_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/measures/distance_point_to_segment.h>
#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
#include <baseproc/image/draw/color.h>

#include <core/features/detectors/quad/quad_gradientclusterer_struct.h>
#include <core/features/detectors/quad/quad_segment2d.h>
#include <inout/image/ppm/ppmfile.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

#define MIN_SIDE_SIZE 10
#define MAX_SIDE_SIZE 1000

#define MIN_AREA_SIZE MIN_SIDE_SIZE * MIN_SIDE_SIZE
#define MAX_AREA_SIZE MAX_SIDE_SIZE * MAX_SIDE_SIZE

Rox_ErrorCode rox_dynvec_quad_searchquad (
   Rox_DynVec_Quad quadlist,
   Rox_Quad_Segment2D * path,
   Rox_Uint lvl,
   Rox_Double side_min,
   Rox_Double side_max,
   Rox_Double area_min,
   Rox_Double area_max
);

Rox_Double mod2pi_pos(Rox_Double vin)
{
   Rox_Double twopi_inv = 0.5/ROX_PI;
   Rox_Double twopi = 2.0*ROX_PI;

   Rox_Double q = vin * twopi_inv + 0.5;
   int qi = (int) q;

   return vin - qi*twopi;
}

Rox_Double mod2pi(Rox_Double vin)
{
   Rox_Double v;

   if (vin < 0)
      v = -mod2pi_pos(-vin);
   else
      v = mod2pi_pos(vin);

   return v;
}

Rox_Double mod2pi2(Rox_Double ref, Rox_Double v)
{
   return ref + mod2pi(v-ref);
}

// Rox_Double rox_det_pts(p1, p2)
// {
//    return (p1.u * p2.v - p1.v * p2.u);
// }

Rox_ErrorCode rox_quad_compute_area ( Rox_Double * area, Rox_Quad curquad )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double u1 = curquad->u[0];
   Rox_Double u2 = curquad->u[1];
   Rox_Double u3 = curquad->u[2];
   Rox_Double u4 = curquad->u[3];

   Rox_Double v1 = curquad->v[0];
   Rox_Double v2 = curquad->v[1];
   Rox_Double v3 = curquad->v[2];
   Rox_Double v4 = curquad->v[3];

   // area = 0.5*(|p1 p2| + |p2 p3| + |p3 p4| + |p4 p1| )

   *area = 0.5*fabs((u1*v2-v1*u2)+(u2*v3-v2*u3)+(u3*v4-v3*u4)+(u4*v1-v4*u1));

   return error;
}

Rox_ErrorCode rox_dynvec_quad_searchquad (
   Rox_DynVec_Quad quadlist,
   Rox_Quad_Segment2D * path,
   Rox_Uint lvl,
   Rox_Double side_min,
   Rox_Double side_max,
   Rox_Double area_min,
   Rox_Double area_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   struct Rox_Quad_Struct curquad;

   Rox_Double coordx = 0.0, coordy = 0.0;
   Rox_Double t1 = 0.0, t2 = 0.0, t3 = 0.0, t0 = 0.0, ttheta = 0.0;
   Rox_Double dist = 0.0;


   if (!quadlist || !path) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // did we arrived at the 4th level ?
   if (lvl == 4)
   {
      // 4 segments
      Rox_Bool bad = 0;

      // 5th segment must be the 1st to close
      if (path[4] != path[0])
      { error = ROX_ERROR_NONE; goto function_terminate; }

      // Retrieve the 4 coordinates which best represent the quad
      for ( Rox_Sint i = 0; i < 4; i++)
      {
         if (rox_dynvec_quad_segment2d_intersect(&coordx, &coordy, path[i], path[i+1]) == 0)
         {
            bad = 1;
            break;
         }

         curquad.u[i] = coordx;
         curquad.v[i] = coordy;
      }

      if (bad)
      { error = ROX_ERROR_NONE; goto function_terminate; }

      // Check winding (no strange quads)
      t0 = atan2(curquad.v[1] - curquad.v[0], curquad.u[1] - curquad.u[0]);
      t1 = atan2(curquad.v[2] - curquad.v[1], curquad.u[2] - curquad.u[1]);
      t2 = atan2(curquad.v[3] - curquad.v[2], curquad.u[3] - curquad.u[2]);
      t3 = atan2(curquad.v[0] - curquad.v[3], curquad.u[0] - curquad.u[3]);

      ttheta = mod2pi(t1-t0) + mod2pi(t2-t1) + mod2pi(t3-t2) + mod2pi(t0-t3);

      // Test if the total theta is between -5 and -7 radians
      // Note that 2*pi = 6.2832 radians
      if (ttheta < -7 || ttheta > -5)
      // if (ttheta < 7 || ttheta > 5)
      { error = ROX_ERROR_NONE; goto function_terminate; }

      // Check bounds on area (no too small or too big quads)

      Rox_Double area = 0.0;
      error = rox_quad_compute_area(&area, &curquad);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ((area < area_min) || (area > area_max))
      { error = ROX_ERROR_NONE; goto function_terminate; }

      // Check bounds on size (no too small or too big quads)
      {
         Rox_Double side_size_1 = sqrt(((curquad.v[1] - curquad.v[0])*(curquad.v[1] - curquad.v[0]) + (curquad.u[1] - curquad.u[0])*(curquad.u[1] - curquad.u[0])));
         Rox_Double side_size_2 = sqrt(((curquad.v[2] - curquad.v[1])*(curquad.v[2] - curquad.v[1]) + (curquad.u[2] - curquad.u[1])*(curquad.u[2] - curquad.u[1])));
         Rox_Double side_size_3 = sqrt(((curquad.v[3] - curquad.v[2])*(curquad.v[3] - curquad.v[2]) + (curquad.u[3] - curquad.u[2])*(curquad.u[3] - curquad.u[2])));
         Rox_Double side_size_4 = sqrt(((curquad.v[0] - curquad.v[3])*(curquad.v[0] - curquad.v[3]) + (curquad.u[0] - curquad.u[3])*(curquad.u[0] - curquad.u[3])));
         //Rox_Double diagonal_size_1 = sqrt(((curquad.v[2] - curquad.v[0])*(curquad.v[2] - curquad.v[0]) + (curquad.u[2] - curquad.u[0])*(curquad.u[2] - curquad.u[0])));
         //Rox_Double diagonal_size_2 = sqrt(((curquad.v[3] - curquad.v[1])*(curquad.v[3] - curquad.v[1]) + (curquad.u[3] - curquad.u[1])*(curquad.u[3] - curquad.u[1])));

         if ((side_size_1 < side_min) ||
             (side_size_2 < side_min) ||
             (side_size_3 < side_min) ||
             (side_size_4 < side_min) ||
             (side_size_1 > side_max) ||
             (side_size_2 > side_max) ||
             (side_size_3 > side_max) ||
             (side_size_4 > side_max) )
         {
            error = ROX_ERROR_NONE; goto function_terminate;
         }
      }

      // Check sizes
      //if (((curquad.v[1] - curquad.v[0])*(curquad.v[1] - curquad.v[0]) + (curquad.u[1] - curquad.u[0])*(curquad.u[1] - curquad.u[0])) < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}
      //if (((curquad.v[2] - curquad.v[1])*(curquad.v[2] - curquad.v[1]) + (curquad.u[2] - curquad.u[1])*(curquad.u[2] - curquad.u[1])) < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}
      //if (((curquad.v[3] - curquad.v[2])*(curquad.v[3] - curquad.v[2]) + (curquad.u[3] - curquad.u[2])*(curquad.u[3] - curquad.u[2])) < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}
      //if (((curquad.v[0] - curquad.v[3])*(curquad.v[0] - curquad.v[3]) + (curquad.u[0] - curquad.u[3])*(curquad.u[0] - curquad.u[3])) < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}
      //if (((curquad.v[2] - curquad.v[0])*(curquad.v[2] - curquad.v[0]) + (curquad.u[2] - curquad.u[0])*(curquad.u[2] - curquad.u[0])) < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}
      //if (((curquad.v[3] - curquad.v[1])*(curquad.v[3] - curquad.v[1]) + (curquad.u[3] - curquad.u[1])*(curquad.u[3] - curquad.u[1])) < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}

      // Check distances from each corners to each opposite segments
      dist = rox_distance_point2d_to_segment2d_coordinates(curquad.u[0], curquad.v[0], curquad.u[1], curquad.v[1], curquad.u[2], curquad.v[2]);
      if (dist < MIN_SIDE_SIZE) { error = ROX_ERROR_NONE; goto function_terminate; }

      dist = rox_distance_point2d_to_segment2d_coordinates(curquad.u[0], curquad.v[0], curquad.u[3], curquad.v[3], curquad.u[2], curquad.v[2]);
      if (dist < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}

      dist = rox_distance_point2d_to_segment2d_coordinates(curquad.u[1], curquad.v[1], curquad.u[2], curquad.v[2], curquad.u[3], curquad.v[3]);
      if (dist < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}

      dist = rox_distance_point2d_to_segment2d_coordinates(curquad.u[1], curquad.v[1], curquad.u[3], curquad.v[3], curquad.u[0], curquad.v[0]);
      if (dist < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}

      dist = rox_distance_point2d_to_segment2d_coordinates(curquad.u[2], curquad.v[2], curquad.u[0], curquad.v[0], curquad.u[1], curquad.v[1]);
      if (dist < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}

      dist = rox_distance_point2d_to_segment2d_coordinates(curquad.u[2], curquad.v[2], curquad.u[3], curquad.v[3], curquad.u[0], curquad.v[0]);
      if (dist < MIN_SIDE_SIZE) {error = ROX_ERROR_NONE; goto function_terminate;}

      dist = rox_distance_point2d_to_segment2d_coordinates(curquad.u[3], curquad.v[3], curquad.u[0], curquad.v[0], curquad.u[1], curquad.v[1]);
      if (dist < MIN_SIDE_SIZE) { error = ROX_ERROR_NONE; goto function_terminate;}

      dist = rox_distance_point2d_to_segment2d_coordinates(curquad.u[3], curquad.v[3], curquad.u[1], curquad.v[1], curquad.u[2], curquad.v[2]);
      if (dist < MIN_SIDE_SIZE) { error = ROX_ERROR_NONE; goto function_terminate;}

      rox_dynvec_quad_append(quadlist, &curquad);

      {error = ROX_ERROR_NONE; goto function_terminate;}
   }

   for (Rox_Uint iter = 0; iter < path[lvl]->countchildren; iter++)
   {
      // Force a given arbitrary order
      if (path[lvl]->child[iter]->theta > path[0]->theta) continue;

      path[lvl+1] = path[lvl]->child[iter];

      // Check this branch of the tree
      error = rox_dynvec_quad_searchquad ( quadlist, path, lvl+1, side_min, side_max, area_min, area_max );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_quad_computequads (
   Rox_DynVec_Quad quadlist,
   const Rox_DynVec_Quad_Segment2D seglist,
   const Rox_Double side_min,
   const Rox_Double side_max,
   const Rox_Double area_min,
   const Rox_Double area_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Quad_Segment2D path[5];


   if (!quadlist || !seglist) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_quad_reset(quadlist);
   for (Rox_Uint i = 0; i < seglist->used; i++)
   {
      path[0] = &seglist->data[i];

      // Search for a quad starting at this segment
      error = rox_dynvec_quad_searchquad ( quadlist, path, 0, side_min, side_max, area_min, area_max );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_new (
   Rox_QuadDetector * detector,
   const Rox_Uint imagewidth,
   const Rox_Uint imageheight
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadDetector ret = NULL;

   if (!detector)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *detector = NULL;

   ret = (Rox_QuadDetector) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->clusterer = NULL;
   error = rox_gradientclusterer_new(&ret->clusterer, imagewidth, imageheight);
   ROX_ERROR_CHECK_TERMINATE(error)

   ret->segments = NULL;
   error = rox_dynvec_quad_segment2d_new(&ret->segments, 100);
   ROX_ERROR_CHECK_TERMINATE(error)

   ret->quads = NULL;
   error = rox_dynvec_quad_new(&ret->quads, 100);
   ROX_ERROR_CHECK_TERMINATE(error)

   ret->black_to_white = 1; // by default detect black to white quads

   ret->segment_length_min = 10; // by default ignore segment with length less than 10 pixels

   ret->side_min = MIN_SIDE_SIZE;
   ret->side_max = MAX_SIDE_SIZE;

   ret->area_min = MIN_AREA_SIZE;
   ret->area_max = MAX_AREA_SIZE;

   *detector = ret;

function_terminate:
   if(error) rox_quaddetector_del(&ret);
   return error;
}

Rox_ErrorCode rox_quaddetector_del(Rox_QuadDetector *detector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadDetector todel = NULL;


   if (!detector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *detector;
   *detector = NULL;


   if(!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ROX_ERROR_CHECK(rox_dynvec_quad_del(&todel->quads));
   ROX_ERROR_CHECK(rox_dynvec_quad_segment2d_del(&todel->segments));
   ROX_ERROR_CHECK(rox_gradientclusterer_del(&todel->clusterer));

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_process_image (
   Rox_QuadDetector detector,
   const Rox_Image image,
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !detector || !image || !mask ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Compute group of pixels (small segments)
   error = rox_gradientclusterer_make ( detector->clusterer, image, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute segments
   error = rox_dynvec_quad_segment2d_make ( detector->segments, detector->clusterer->groups, detector->clusterer->nbgroups, detector->black_to_white, detector->segment_length_min ); //, detector->side_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute segments connections (childrens)
   error = rox_dynvec_quad_segment2d_computechildren ( detector->segments );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Find closed quads
   error = rox_dynvec_quad_computequads ( detector->quads, detector->segments, detector->side_min, detector->side_max, detector->area_min, detector->area_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_process_image_ac (
   Rox_QuadDetector detector,
   const Rox_Image image,
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!detector || !image || !mask ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Compute group of pixels (small segments)
   error = rox_gradientclusterer_make_ac ( detector->clusterer, image, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute segments
   error = rox_dynvec_quad_segment2d_make ( detector->segments, detector->clusterer->pointset->data, detector->clusterer->pointset->used, detector->black_to_white, detector->segment_length_min ); // , detector->side_min, detector->side_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute segments connections
   error = rox_dynvec_quad_segment2d_computechildren(detector->segments);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Find closed quads
   error = rox_dynvec_quad_computequads ( detector->quads, detector->segments, detector->side_min, detector->side_max, detector->area_min, detector->area_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_set_quad_color (
   Rox_QuadDetector detector,
   const Rox_Uint black_to_white
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!detector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   detector->black_to_white = black_to_white;

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_set_segment_length_min (
   Rox_QuadDetector detector,
   const Rox_Double segment_length_min
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!detector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   detector->segment_length_min = segment_length_min;

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_get_quad_count (
   Rox_Sint * detected_quads,
   Rox_QuadDetector detector
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!detected_quads || !detector ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *detected_quads = detector->quads->used;

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_get_quad_SL3 (
   Rox_MatSL3 H,
   const Rox_QuadDetector detector,
   const Rox_Uint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct source[4];
   Rox_Point2D_Double_Struct dest[4];


   if ( !H || !detector ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( id > detector->quads->used ) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Build a square
   source[0].u = 0.0;
   source[0].v = 0.0;

   source[1].u = 0.0;
   source[1].v = 1.0;

   source[2].u = 1.0;
   source[2].v = 1.0;

   source[3].u = 1.0;
   source[3].v = 0.0;

   // Get the measured corners of the quad in the image
   dest[0].u = detector->quads->data[id].u[0];
   dest[0].v = detector->quads->data[id].v[0];
   dest[1].u = detector->quads->data[id].u[1];
   dest[1].v = detector->quads->data[id].v[1];
   dest[2].u = detector->quads->data[id].u[2];
   dest[2].v = detector->quads->data[id].v[2];
   dest[3].u = detector->quads->data[id].u[3];
   dest[3].v = detector->quads->data[id].v[3];

   // Compute the homography for a given quad
   error = rox_matsl3_from_4_points_double(H, source, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_get_points (
   Rox_Point2D_Double pts,
   const Rox_QuadDetector detector,
   const Rox_Uint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if(!pts || !detector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (id > detector->quads->used) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pts[0].u = detector->quads->data[id].u[0];
   pts[0].v = detector->quads->data[id].v[0];
   pts[1].u = detector->quads->data[id].u[1];
   pts[1].v = detector->quads->data[id].v[1];
   pts[2].u = detector->quads->data[id].u[2];
   pts[2].v = detector->quads->data[id].v[2];
   pts[3].u = detector->quads->data[id].u[3];
   pts[3].v = detector->quads->data[id].v[3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_set_side_bounds (
   Rox_QuadDetector detector,
   const Rox_Double side_min,
   const Rox_Double side_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!detector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (( side_min < 0 ) || ( side_max < 0 ) || ( side_max < side_min ))
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   detector->side_min = side_min;
   detector->side_max = side_max;

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_set_area_bounds (
   Rox_QuadDetector detector,
   const Rox_Double area_min,
   const Rox_Double area_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!detector) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (( area_min < 0 ) || ( area_max < 0 ) || ( area_max < area_min ))
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   detector->area_min = area_min;
   detector->area_max = area_max;

function_terminate:
   return error;
}

Rox_ErrorCode rox_quaddetector_save_segment2d (
   const Rox_Char * filename,
   const Rox_QuadDetector quaddetector
)
{
   return rox_dynvec_quad_segment2d_save ( filename, quaddetector->segments );
}