//==============================================================================
//
//    OPENROX   : File ellipse2d.c
//
//    Contents  : Implementation of ellipse2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#include "ellipse2d.h"

#include <math.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/ellipse/ellipse2d_struct.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>

#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_ellipse2d_new ( Rox_Ellipse2D * ellipse2d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ellipse2D ret = NULL;

   if (!ellipse2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *ellipse2d = NULL;

   ret = (Rox_Ellipse2D) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->xc = 0.0;
   ret->yc = 0.0;
   ret->nxx = 1.0;
   ret->nyy = 1.0;
   ret->nxy = 0.0;

   *ellipse2d = ret;

function_terminate:
   if(error) rox_ellipse2d_del(&ret);
   return error;
}

Rox_ErrorCode rox_ellipse2d_del ( Rox_Ellipse2D * ellipse2d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ellipse2D todel = NULL;

   if (!ellipse2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ellipse2d;
   *ellipse2d = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_set (
   Rox_Ellipse2D ellipse2d,
   const Rox_Double xc,
   const Rox_Double yc,
   const Rox_Double nxx,
   const Rox_Double nyy,
   const Rox_Double nxy
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (nxx*nyy-nxy*nxy <= 0.0)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ellipse2d->xc = xc;
   ellipse2d->yc = yc;
   ellipse2d->nxx = nxx;
   ellipse2d->nyy = nyy;
   ellipse2d->nxy = nxy;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_print(Rox_Ellipse2D ellipse2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("xc  = %12.12f \n", ellipse2d->xc );
   rox_log("yc  = %12.12f \n", ellipse2d->yc );
   rox_log("nxx = %12.12f \n", ellipse2d->nxx);
   rox_log("nyy = %12.12f \n", ellipse2d->nyy);
   rox_log("nxy = %12.12f \n", ellipse2d->nxy);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_get_normal_angle ( Rox_Double * angle, Rox_Ellipse2D ellipse2d, const Rox_Double xe, const Rox_Double ye)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d || !angle)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Retrieve parameters of canonical form
   Rox_Double nxx = ellipse2d->nxx;
   Rox_Double nyy = ellipse2d->nyy;
   Rox_Double nxy = ellipse2d->nxy;
   Rox_Double xc = ellipse2d->xc;
   Rox_Double yc = ellipse2d->yc;

   // Compute the angle that can be used to build the normal vector
   // normal = [cos(angle), sin(angle)] = [nxx*(xe - xc) + nxy*(ye - yc), nxy*(xe - xc) + nyy*(ye - yc)]

   Rox_Double cosa = nxx*(xe - xc) + nxy*(ye - yc);
   Rox_Double sina = nxy*(xe - xc) + nyy*(ye - yc);

   *angle = atan2(sina, cosa);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_get_tangent_angle ( Rox_Double * angle, Rox_Ellipse2D ellipse2d, const Rox_Double xe, const Rox_Double ye)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d || !angle)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Retrieve parameters of canonical form
   Rox_Double nxx = ellipse2d->nxx;
   Rox_Double nyy = ellipse2d->nyy;
   Rox_Double nxy = ellipse2d->nxy;
   Rox_Double xc = ellipse2d->xc;
   Rox_Double yc = ellipse2d->yc;

   // tangent = [cos(angle), sin(angle)] = [nxy*(xc - xe) + nyy*(yc - ye), - nxx*(xc - xe) - nxy*(yc - ye)]

   Rox_Double cosa = + nxy*(xc - xe) + nyy*(yc - ye);
   Rox_Double sina = - nxx*(xc - xe) - nxy*(yc - ye);

   *angle = atan2(sina, cosa);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_convert_parametric_to_canonical ( Rox_Ellipse2D ellipse2d, Rox_Double xc, Rox_Double yc, Rox_Double a, Rox_Double b, Rox_Double phi )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double cos_phi = cos(phi);
   Rox_Double sin_phi = sin(phi);

   // Convert parametric form to canonical form
   ellipse2d->nxx = cos_phi*cos_phi/(a*a) + sin_phi*sin_phi/(b*b);
   ellipse2d->nyy = sin_phi*sin_phi/(a*a) + cos_phi*cos_phi/(b*b);
   ellipse2d->nxy = sin_phi * cos_phi * (1/(a*a)-1/(b*b));

   ellipse2d->xc = xc;
   ellipse2d->yc = yc;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_convert_canonical_to_parametric ( Rox_Double * xc, Rox_Double * yc, Rox_Double * a, Rox_Double * b, Rox_Double * phi, Rox_Ellipse2D ellipse2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Retrieve parameters of canonical form
   Rox_Double nxx = ellipse2d->nxx;
   Rox_Double nyy = ellipse2d->nyy;
   Rox_Double nxy = ellipse2d->nxy;

   // Convert canonical form to parametric form
   *xc = ellipse2d->xc;
   *yc = ellipse2d->yc;

   // *phi = atan(2*nxy/(nxx-nyy))/2;
   *phi = atan2(2*nxy, nxx-nyy)/2;

   *a = 1.0 / sqrt((cos(2*(*phi))*(nxx+nyy)+(nxx-nyy)) / (2*cos(2*(*phi))));
   *b = 1.0 / sqrt((cos(2*(*phi))*(nxx+nyy)-(nxx-nyy)) / (2*cos(2*(*phi))));

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_sample ( Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Ellipse2D ellipse2d, Rox_Double sampling_angle)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !ellipse2d )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double uc = 0.0, vc = 0.0, a = 1.0, b = 1.0, phi = 0.0;
   error = rox_ellipse2d_convert_canonical_to_parametric ( &uc, &vc, &a, &b, &phi, ellipse2d );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_reset(dynvec_point2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Sample points on ellipse
   Rox_Sint nbp = (Rox_Sint) (360.0/sampling_angle);
   for (Rox_Sint k = 0; k < nbp; k++)
   {
      Rox_Point2D_Double_Struct point2D;

      Rox_Double theta = (ROX_PI/180.0) * k * sampling_angle;
      point2D.u = uc + a * cos(theta) * cos(phi) - b * sin(theta) * sin(phi) ;
      point2D.v = vc + b * sin(theta) * cos(phi) + a * cos(theta) * sin(phi) ;

      error = rox_dynvec_point2d_double_append(dynvec_point2d, &point2D);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse_perimeter_ramanujan (
   Rox_Double * perimeter,
   const Rox_Double a,
   const Rox_Double b
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!perimeter)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Ramanujan approximation in paper "Modular Equations and Approximations to π"

   Rox_Double a2 = a*a;
   Rox_Double b2 = b*b;
   Rox_Double ab = a*b;

   *perimeter = ROX_PI*((a+b)+3.0*(a2-2.0*ab+b2)/(10.0*(a+b)+sqrt(a2+14.0*ab+b2)));

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_perimeter ( Rox_Double * perimeter, Rox_Ellipse2D ellipse2d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!perimeter || !ellipse2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double uc = 0.0, vc = 0.0, a = 1.0, b = 1.0, phi = 0.0;
   error = rox_ellipse2d_convert_canonical_to_parametric ( &uc, &vc, &a, &b, &phi, ellipse2d );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ellipse_perimeter_ramanujan(perimeter, a, b);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_ellipse2d_sample_perimeter (
   Rox_DynVec_Point2D_Double dynvec_point2d,
   Rox_Ellipse2D ellipse2d,
   Rox_Double sampling_step
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double uc = 0.0, vc = 0.0, a = 1.0, b = 1.0, phi = 0.0;
   error = rox_ellipse2d_convert_canonical_to_parametric(&uc, &vc, &a, &b, &phi, ellipse2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double perimeter = 0.0;

   error = rox_ellipse_perimeter_ramanujan(&perimeter, a, b);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_reset(dynvec_point2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint nbp = (Rox_Sint) (perimeter/sampling_step);

   //TODO parameter for minimum points ?
   //ellipse can be determined with 5 points, in order to be robust we'd like n/2+1 points = 11
   if (nbp < (2*5+1))
   {
      error = ROX_ERROR_INSUFFICIENT_DATA; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   Rox_Double sampling_angle = 360.0/nbp;

   // Sample points on ellipse
   for (Rox_Sint k = 0; k < nbp; k++)
   {
      Rox_Point2D_Double_Struct point2D;

      Rox_Double theta = (ROX_PI/180.0) * k * sampling_angle;
      point2D.u = uc + a * cos(theta) * cos(phi) - b * sin(theta) * sin(phi) ;
      point2D.v = vc + b * sin(theta) * cos(phi) + a * cos(theta) * sin(phi) ;

      error = rox_dynvec_point2d_double_append(dynvec_point2d, &point2D);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_get_rect_sint (
   Rox_Rect_Sint window,
   Rox_Ellipse2D ellipse2d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ellipse2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double nxx = ellipse2d->nxx;
   Rox_Double nyy = ellipse2d->nyy;
   Rox_Double nxy = ellipse2d->nxy;

   Rox_Double d  = nxx*nyy - nxy*nxy;
   Rox_Double bx = sqrt(nyy/d);
   Rox_Double by = sqrt(nxx/d);

   window->x = (Rox_Sint) (ellipse2d->xc - bx);
   window->y = (Rox_Sint) (ellipse2d->yc - by);
   window->width  = (Rox_Sint) (2*bx);
   window->height = (Rox_Sint) (2*by);

function_terminate:
   return error;
}