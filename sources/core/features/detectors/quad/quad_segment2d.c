//==============================================================================
//
//    OPENROX   : File quad_segment2d.c
//
//    Contents  : Implementation of quad_segment2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "quad_segment2d.h"

#include <string.h>
#include <generated/dynvec_orientedimagepoint_struct.h>
#include <generated/dynvec_quad_segment2d_struct.h>

#include <system/memory/memory.h>

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/line/line2d_struct.h>

#include <baseproc/maths/maths_macros.h>

#include <baseproc/image/draw/draw_line.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

extern Rox_Double mod2pi_pos(Rox_Double vin);
extern Rox_Double mod2pi(Rox_Double vin);

// Add two segments if they are almost on the same line
Rox_ErrorCode rox_dynvec_quad_segment2d_increase (
   Rox_Quad_Segment2D created,
   const Rox_Quad_Segment2D one,
   const Rox_Quad_Segment2D two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!created)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double angle = one->theta;
   Rox_Double refangle = two->theta;

   if (fabs(one->end[0] - two->start[0]) < 2) // the distance is less than 2 pixels
   {
      if (fabs(one->end[1] - two->start[1]) < 2) // the distance is less than 2 pixels
      {
         Rox_Double dist = fabs(atan2(sin(refangle-angle), cos(refangle-angle)));
         if (dist < ROX_PI / 8.0)
         {
            created->start[0] = one->start[0];
            created->start[1] = one->start[1];
            created->end[0] = two->end[0];
            created->end[1] = two->end[1];

            Rox_Double difx = created->end[0] - created->start[0];
            Rox_Double dify = created->end[1] - created->start[1];

            created->theta = one->theta;
            created->length = sqrt(difx*difx+dify*dify);

            created->countchildren = 0;

            error = ROX_ERROR_NONE;
            goto function_terminate;
         }
      }
   }


   error = ROX_ERROR_INVALID_VALUE;
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_pixel_group_least_squares_fit_line2d_parametric (
   Rox_Line2D_Parametric line2d,
   const Rox_DynVec_OrientedImagePoint group
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double mX = 0.0, mY = 0.0, mXX = 0.0, mXY = 0.0, mYY = 0.0, n = 0.0;
   Rox_Double Ex,Ey,Cxx,Cyy,Cxy;
   Rox_Double norm, dotprod;
   Rox_Double phi;

   Rox_Double * px = &line2d->p.u;
   Rox_Double * py = &line2d->p.v;
   Rox_Double * dx = &line2d->d.u;
   Rox_Double * dy = &line2d->d.v;

   // Least square solution to fit a line on the pixel group
   for (Rox_Uint j = 0; j < group->used; j++)
   {
      Rox_Uint ci = group->data[j].i;
      Rox_Uint cj = group->data[j].j;
      Rox_Double  alpha = group->data[j].mag;

      mX  += alpha * (double) cj;
      mY  += alpha * (double) ci;
      mXX += alpha * (double) (cj*cj);
      mXY += alpha * (double) (ci*cj);
      mYY += alpha * (double) (ci*ci);

      n += alpha;
   }

   Ex = mX / n;
   Ey = mY / n;

   Cxx = (mXX / n) - (Ex * Ex);
   Cyy = (mYY / n) - (Ey * Ey);
   Cxy = (mXY / n) - (Ex * Ey);

   // Retrieve line parameters
   phi = 0.5 * atan2(-2.0*Cxy, (Cyy-Cxx));
   *dx = -sin(phi);
   *dy =  cos(phi);

   // Do we need to keep the distance ? can we use the squared distance to avoid sqrt ?
   norm = sqrt( (*dx)*(*dx) + (*dy)*(*dy) );
   *dx /= norm;
   *dy /= norm;

   *px = Ex;
   *py = Ey;

   dotprod = -(*dy) * (*px) + (*dx)* (*py);

   *px = -(*dy)*dotprod;
   *py =  (*dx)*dotprod;

   return error;
}

Rox_Double rox_circular_distance ( Rox_Double theta, Rox_Double phi )
{
   Rox_Double circular_distance = fabs(theta - phi);

   circular_distance = ROX_MIN(circular_distance, fabs(theta - phi + 2*ROX_PI));
   circular_distance = ROX_MIN(circular_distance, fabs(theta - phi - 2*ROX_PI));

   return circular_distance;
}

Rox_ErrorCode rox_pixel_group_compute_quad_segment2d ( Rox_Quad_Segment2D toadd,
   const Rox_Line2D_Parametric line2d,
   const Rox_DynVec_OrientedImagePoint group,
   const Rox_Uint black_to_white,
   const Rox_Double seqment_length_min
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double seg_start[2];
   Rox_Double seg_end[2];
   Rox_Double seg_theta;
   Rox_Double seg_length;

   Rox_Double nx1, ny1, nx2, ny2;
   Rox_Double difx, dify;

   // The origin of the line
   Rox_Double px = line2d->p.u;
   Rox_Double py = line2d->p.v;

   // The tangent vector to the line
   Rox_Double dx = line2d->d.u;
   Rox_Double dy = line2d->d.v;

   // Init min and max coordinates
   Rox_Double mincoord = +DBL_MAX;
   Rox_Double maxcoord = -DBL_MAX;
   Rox_Double coord = 0.0;

   // Retrieve extremas for the segment (points projected on the fitted line parameters)
   for (Rox_Uint j = 0; j < group->used; j++)
   {
      Rox_Uint ci = group->data[j].i;
      Rox_Uint cj = group->data[j].j;

      coord = cj * dx + ci * dy;
      maxcoord = ROX_MAX(coord, maxcoord);
      mincoord = ROX_MIN(coord, mincoord);
   }

   // Segment extremas in 2D space
   nx1 = px + mincoord * dx;
   ny1 = py + mincoord * dy;
   nx2 = px + maxcoord * dx;
   ny2 = py + maxcoord * dy;

   // Segment size
   difx = nx2 - nx1;
   dify = ny2 - ny1;

   // Retrieve segment tangent angle [it is not the angle of the normal !!!]
   seg_theta = atan2 ( dify, difx );
   // The segment normal angle is seg_theta - pi/2 or better: atan2(cos(seg_theta), -sin(seg_theta))

   // Retrieve segment length
   seg_length = sqrt ( difx*difx + dify*dify );

   // If segment is too short or too long, remove
   // if ((seg_llength < seqment_length_min) || (seg_llength > seqment_length_max)) continue;
   // if ((seg_length < 100) || (seg_llength > 300)) continue;

   // A segment of length less than SEGMENT_LENGTH_MIN should be avoided before arriving here ?
   if ( seg_length < seqment_length_min ) return -1;

// #define old

#ifdef old

   Rox_Double noflip = 0.0;
   Rox_Double flip = 0.0;

   // For all pixels, check if their gradient is in the same direction than the line (to find black side)
   for (Rox_Sint j = 0; j < group->used; j++)
   {
      Rox_Double mag   = group->data[j].mag;
      Rox_Double theta = group->data[j].theta;

      // Be careful we compare normal angle theta with tangent angle seg_theta
      Rox_Double err = mod2pi(theta - seg_theta);

      if (black_to_white == 1)
      {
         // Test (err < 0.0) -> to get black to white segments
         if (err < 0.0) noflip += mag;
         else flip += mag;
      }
      else
      {
         // Test (err > 0.0) -> to get white to black segments
         if (err > 0.0) noflip += mag;
         else flip += mag;
      }

   }

   // If needed, flip angle
   if (flip > noflip)
   {
      seg_theta += ROX_PI;
   }

#else
   // New

   Rox_Double normal_angle_mean = 0.0, cn = 0.0, sn = 0.0;
   // For all pixels, check if their gradient is in the same direction than the line (to find black side)
   for (Rox_Uint j = 0; j < group->used; j++)
   {
      Rox_Double theta = group->data[j].theta;

      // Circular mean
      cn += cos(theta);
      sn += sin(theta);
   }
   normal_angle_mean = atan2(sn/group->used, cn/group->used);

   Rox_Double tangent_angle_mean = atan2(cos(normal_angle_mean), -sin(normal_angle_mean));

   Rox_Double c1 = rox_circular_distance ( tangent_angle_mean , seg_theta );
   Rox_Double c2 = rox_circular_distance ( tangent_angle_mean , seg_theta + ROX_PI );
   Rox_Double c3 = rox_circular_distance ( tangent_angle_mean , seg_theta - ROX_PI );

   if      ((c1 <= c2) && (c1 <= c3))
   {
      // seg_theta is ok
   }
   else if ((c2 <= c1) && (c2 <= c3))
   {
      seg_theta = atan2(sin(seg_theta + ROX_PI), cos(seg_theta + ROX_PI));
   }
   else if ((c3 <= c1) && (c3 <= c2))
   {
      seg_theta = atan2(sin(seg_theta - ROX_PI), cos(seg_theta - ROX_PI));
   }

   if ( black_to_white == 0 )
   {
      seg_theta = atan2(sin(seg_theta + ROX_PI), cos(seg_theta + ROX_PI));
   }

#endif

   // Recompute extremas and store segment
   // The points of the segment are stored in a way to represent the tangent vector
   Rox_Double dot = dx*cos(seg_theta) + dy*sin(seg_theta);

   if (dot > 0.0)
   {
      seg_start[0] = nx2;
      seg_start[1] = ny2;
      seg_end[0]   = nx1;
      seg_end[1]   = ny1;
   }
   else
   {
      seg_start[0] = nx1;
      seg_start[1] = ny1;
      seg_end[0]   = nx2;
      seg_end[1]   = ny2;
   }

   // Pass directly the toadd structure to the function : rox_pixel_group_compute_quad_segment2d
   toadd->start[0] = seg_start[0];
   toadd->start[1] = seg_start[1];
   toadd->end[0]   = seg_end[0];
   toadd->end[1]   = seg_end[1];

   toadd->theta = seg_theta;
   toadd->length = seg_length;

   toadd->countchildren = 0;

   return error;
}

Rox_ErrorCode rox_dynvec_quad_segment2d_make (
   Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d,
   Rox_DynVec_OrientedImagePoint * groups,
   Rox_Uint nbgroups,
   Rox_Uint black_to_white, // Set black_to_white to 0 to detect white inside / black outside quads; to 1 to detect black inside / white outside quads
   Rox_Double seqment_length_min // The minimum length in pixels of a segment in order to be considered
) // Rox_Double length_max )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Rox_Double seg_length, seg_theta;
   // Rox_Double seg_start[2], seg_end[2];

   Rox_Quad_Segment2D_Struct toadd;

   Rox_Line2D_Parametric_Struct line2d;


   if (!dynvec_quad_segment2d || !groups)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_quad_segment2d_reset(dynvec_quad_segment2d);

   for (Rox_Uint i = 0; i < nbgroups; i++ )
   {
      if (groups[i] == NULL) continue;

      // For each group compute a line fit
      // The line is represented in parametric form as [px;py] + lammda * [dx;dy]
      error = rox_pixel_group_least_squares_fit_line2d_parametric (&line2d, groups[i]);
      if (error) continue;

      // Compute the quad segment 2D to add
      error = rox_pixel_group_compute_quad_segment2d ( &toadd, &line2d, groups[i], black_to_white, seqment_length_min );
      if (error) continue;

      error = rox_dynvec_quad_segment2d_append(dynvec_quad_segment2d, &toadd);
      if (error) continue;
   }

   Rox_Sint count = dynvec_quad_segment2d->used;

   // TODO : This code is supposed to fuse two segments having the same orientation ???
   // Understand why its add segments instead of replace them
   // Is it better to icrease segments in the previos step ?
   if(1)
   {
      for ( Rox_Sint i = 0; i < count; i++ )
      {
          for ( Rox_Sint j = 0; j < count; j++ )
          {
             if (i == j) continue;

             error = rox_dynvec_quad_segment2d_increase ( &toadd, &dynvec_quad_segment2d->data[i], &dynvec_quad_segment2d->data[j]);
             if (error) continue;

             rox_dynvec_quad_segment2d_append(dynvec_quad_segment2d, &toadd);
          }
          error = ROX_ERROR_NONE;
      }
   }

function_terminate:
   return error;
}

// TODO: This function should use a lower level function that computes the intersection of two segments
// rox_segment2d_intersect_segment2d ( Rox_Double * coordx, Rox_Double * coordy, Rox_Segment2D segment2d_1, Rox_Segment2D segment2d_2 )
Rox_Bool rox_dynvec_quad_segment2d_intersect (
   Rox_Double * coordx,
   Rox_Double * coordy,
   Rox_Quad_Segment2D quad_segment2d1,
   Rox_Quad_Segment2D quad_segment2d2
)
{
   Rox_Double idx, idy, ipx, ipy;
   Rox_Double jdx, jdy, jpx, jpy;

   double m00, m01, m10, m11;
   double i00, i01;
   double b00, b10, det;
   double x00;

   idx = quad_segment2d1->end[0] - quad_segment2d1->start[0];
   idy = quad_segment2d1->end[1] - quad_segment2d1->start[1];
   ipx = quad_segment2d1->start[0];
   ipy = quad_segment2d1->start[1];

   jdx = quad_segment2d2->end[0] - quad_segment2d2->start[0];
   jdy = quad_segment2d2->end[1] - quad_segment2d2->start[1];
   jpx = quad_segment2d2->start[0];
   jpy = quad_segment2d2->start[1];

   m00 = idx;
   m01 =-jdx;
   m10 = idy;
   m11 =-jdy;

   det=m00*m11-m01*m10;

   // Hardcoded threshold !!!
   if (fabs(det) < 0.0000000001)
   {
      return 0;
   }

   if (coordx == NULL) return 1;

   i00 =  m11/det;
   i01 = -m01/det;

   b00 = jpx - ipx;
   b10 = jpy - ipy;

   x00 = i00*b00 + i01*b10;

   *coordx = idx*x00+ipx;
   *coordy = idy*x00+ipy;

   return 1;
}

Rox_ErrorCode rox_dynvec_quad_segment2d_computechildren (
   Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double diftheta = 0.0;
   Rox_Double interx = 0.0, intery = 0.0;
   Rox_Double difx = 0.0, dify = 0.0, parentdist = 0.0, childdist = 0.0, dist = 0.0;
   // Rox_Double dist1 = 0.0;
   Rox_Double dist2 = 0.0;


   if (!dynvec_quad_segment2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Loop through segments
   for (Rox_Uint i = 0; i < dynvec_quad_segment2d->used; i++)
   {
      // Init counter of childrens to 0
      dynvec_quad_segment2d->data[i].countchildren = 0;

      // Loop through all other segments except itself
      for (Rox_Uint j = 0; j < dynvec_quad_segment2d->used; j++)
      {
         if (i==j) continue;

         // TODO : This suppose that the end of segment i and the start of segment j follows
         //        What if the start of segment j is close to the start of segment i ???

         //difx = dynvec_quad_segment2d->data[i].end[0] - dynvec_quad_segment2d->data[j].end[0];
         //dify = dynvec_quad_segment2d->data[i].end[1] - dynvec_quad_segment2d->data[j].end[1];
         //dist1 = sqrt(difx*difx+dify*dify);

         // Compute distance between segments
         difx = dynvec_quad_segment2d->data[i].end[0] - dynvec_quad_segment2d->data[j].start[0];
         dify = dynvec_quad_segment2d->data[i].end[1] - dynvec_quad_segment2d->data[j].start[1];
         dist2 = sqrt(difx*difx+dify*dify);

         // dist = ROX_MIN(dist1, dist2);

         dist = dist2;

         // Keep only if the distance is close enough
         if (dist > dynvec_quad_segment2d->data[i].length * 0.5) continue;

         // should we introduce oriented segments ??? is it already done ???
         // Compute difference of theta
         diftheta = dynvec_quad_segment2d->data[j].theta - dynvec_quad_segment2d->data[i].theta;
         diftheta = mod2pi(diftheta);

         if (diftheta > 0) continue;

         // Retrieve intersection of both segments (may fail if parrallels)
         if (rox_dynvec_quad_segment2d_intersect (&interx, &intery, &dynvec_quad_segment2d->data[i], &dynvec_quad_segment2d->data[j]) == 0) continue;

         // Check if intersection is far away from segments
         difx = interx - dynvec_quad_segment2d->data[i].end[0];
         dify = intery - dynvec_quad_segment2d->data[i].end[1];
         parentdist = sqrt(difx*difx+dify*dify);
         difx = interx - dynvec_quad_segment2d->data[j].start[0];
         dify = intery - dynvec_quad_segment2d->data[j].start[1];
         childdist = sqrt(difx*difx+dify*dify);
         if (ROX_MAX(parentdist, childdist) > dynvec_quad_segment2d->data[i].length) continue;

         // Add this segment as a child
         dynvec_quad_segment2d->data[i].child[dynvec_quad_segment2d->data[i].countchildren] = &dynvec_quad_segment2d->data[j];
         dynvec_quad_segment2d->data[i].countchildren++;

         // Harcoded limit for childrens !!!
         if (dynvec_quad_segment2d->data[i].countchildren == 100)
         {
            break;
         }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_quad_segment2d_draw ( Rox_Image_RGBA image_rgba, Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d, Rox_Uint color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Segment2D segment2d = NULL;

   error = rox_segment2d_new ( &segment2d );

   ROX_ERROR_CHECK_TERMINATE ( error );

   static Rox_Uint addcolor = 0;

   for (Rox_Uint k =0; k < dynvec_quad_segment2d->used; k++)
   {
      Rox_Double u1 = dynvec_quad_segment2d->data[k].start[0];
      Rox_Double v1 = dynvec_quad_segment2d->data[k].start[1];
      Rox_Double u2 = dynvec_quad_segment2d->data[k].end[0];
      Rox_Double v2 = dynvec_quad_segment2d->data[k].end[1];

      error = rox_segment2d_set ( segment2d, u1, v1, u2, v2);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_image_rgba_draw_segment2d ( image_rgba, segment2d, color+addcolor);

      ROX_ERROR_CHECK_TERMINATE ( error );
      addcolor += 100;
   }

function_terminate:
   rox_segment2d_del ( &segment2d );

   return error;
}

Rox_ErrorCode rox_dynvec_quad_segment2d_draw_childrens (
   Rox_Image_RGBA image_rgba,
   Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d,
   Rox_Uint color,
   Rox_Uint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Segment2D segment2d = NULL;

   error = rox_segment2d_new ( &segment2d );

   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint j = 0; j < dynvec_quad_segment2d->data[id].countchildren; j++)
   {
      Rox_Double u1 = dynvec_quad_segment2d->data[id].child[j]->start[0];
      Rox_Double v1 = dynvec_quad_segment2d->data[id].child[j]->start[1];
      Rox_Double u2 = dynvec_quad_segment2d->data[id].child[j]->end[0];
      Rox_Double v2 = dynvec_quad_segment2d->data[id].child[j]->end[1];

      error = rox_segment2d_set ( segment2d, u1, v1, u2, v2);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_image_rgba_draw_segment2d ( image_rgba, segment2d, color);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_segment2d_del ( &segment2d );

   return error;
}

// This function should be in the inout module
Rox_ErrorCode rox_dynvec_quad_segment2d_save (
   const Rox_Char * filename,
   const Rox_DynVec_Quad_Segment2D dynvec_quad_segment2d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   if (!dynvec_quad_segment2d || !filename)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   out = fopen(filename, "w");
   if (!out)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint k = 0; k<dynvec_quad_segment2d->used; k++ )
   {
      Rox_Double u1 = dynvec_quad_segment2d->data[k].start[0];
      Rox_Double v1 = dynvec_quad_segment2d->data[k].start[1];
      Rox_Double u2 = dynvec_quad_segment2d->data[k].end[0];
      Rox_Double v2 = dynvec_quad_segment2d->data[k].end[1];
      fprintf(out, "%f %f %f %f\n", u1, v1, u2, v2);
   }

function_terminate:
   if(out) fclose(out);
   return error;
}


