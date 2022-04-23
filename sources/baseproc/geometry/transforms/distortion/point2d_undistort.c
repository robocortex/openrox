//==============================================================================
//
//    OPENROX   : File point2d_undistort.c
//
//    Contents  : Implementation of pointundistort module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "point2d_undistort.h"
#include <baseproc/maths/maths_macros.h>
#include <float.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_point2d_double_undistort (
   Rox_Point2D_Double undistorted,
   Rox_Point2D_Double distorted,
   Rox_MatUT3 calib,
   Rox_Array2D_Double distortion)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Uint maxiters = 20;

   Rox_Double xx, xy, yy;
   Rox_Double r2, r4;
   Rox_Double radial;
   Rox_Double tangx, tangy;
   Rox_Double dx, dy;
   Rox_Double solx, soly;

   if (!undistorted || !calib || !distortion) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_double_check_size(calib, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
	error = rox_array2d_double_check_size(distortion, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
	Rox_Double ** dk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** ddist = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &ddist, distortion);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve calibration
   Rox_Double px = dk[0][0];
   Rox_Double py = dk[1][1];
   Rox_Double u0 = dk[0][2];
   Rox_Double v0 = dk[1][2];
   Rox_Double skew = dk[0][1];

   // Inverse of calibration
   Rox_Double ipx = 1.0 / px;
   Rox_Double ipy = 1.0 / py;
   Rox_Double iskew = -skew/(px*py);
   Rox_Double iu0 = (skew*v0 - u0*py)/(px*py);
   Rox_Double iv0 = -v0/py;

   // Get distortion constants
   Rox_Double k1 = ddist[0][0];
   Rox_Double k2 = ddist[1][0];
   Rox_Double k3 = ddist[2][0];
   Rox_Double k4 = ddist[3][0];
   Rox_Double k5 = ddist[4][0];

   // Go from pixel to meters
   Rox_Double y = ipy * distorted->v + iv0;
   Rox_Double x = ipx * distorted->u + iskew * distorted->v + iu0;

   // Store normalized distorted coordinates
   Rox_Double x0 = x;
   Rox_Double y0 = y;

   for (Rox_Uint iter = 0; iter < maxiters; iter++)
   {
      yy = y * y;
      xx = x * x;
      xy = x * y;

      r2 = yy + xx;
      r4 = r2 * r2;

      // Radial distortion
      radial = 1.0f + k1*r2 + k2*r4 + k5*r4*r2;

      // Tangential distortion
      tangx = 2.0f * k3 * xy + k4 * (r2 + 2.0f*xx);
      tangy = k3 * (r2 + 2.0f * yy) + 2.0f * k4 * xy;

      // Difference
      Rox_Double dx = x0 - (x * radial + tangx);
      Rox_Double dy = y0 - (y * radial + tangy);

      // Jacobian of difference wrt x,y
      Rox_Double Jxx = 0.1e1 + 0.3e1 * k1 * x * x + k1 * y * y + 0.5e1 * k2 * pow(x, 0.4e1) + 0.6e1 * k2 * x * x * y * y + k2 * pow(y, 0.4e1) + 0.7e1 * k5 * pow(x, 0.6e1) + 0.15e2 * k5 * pow(x, 0.4e1) * y * y + 0.9e1 * k5 * x * x * pow(y, 0.4e1) + k5 * pow(y, 0.6e1) + 0.2e1 * k3 * y + 0.6e1 * k4 * x;
      Rox_Double Jxy = 0.2e1 * x * k1 * y + 0.4e1 * k2 * y * pow(x, 0.3e1) + 0.4e1 * x * k2 * pow(y, 0.3e1) + 0.6e1 * k5 * y * pow(x, 0.5e1) + 0.12e2 * k5 * pow(y, 0.3e1) * pow(x, 0.3e1) + 0.6e1 * x * k5 * pow(y, 0.5e1) + 0.2e1 * k3 * x + 0.2e1 * k4 * y;
      Rox_Double Jyx = 0.2e1 * x * k1 * y + 0.4e1 * k2 * y * pow(x, 0.3e1) + 0.4e1 * x * k2 * pow(y, 0.3e1) + 0.6e1 * k5 * y * pow(x, 0.5e1) + 0.12e2 * k5 * pow(y, 0.3e1) * pow(x, 0.3e1) + 0.6e1 * x * k5 * pow(y, 0.5e1) + 0.2e1 * k3 * x + 0.2e1 * k4 * y;
      Rox_Double Jyy = 0.1e1 + k1 * x * x + 0.3e1 * k1 * y * y + k2 * pow(x, 0.4e1) + 0.6e1 * k2 * x * x * y * y + 0.5e1 * k2 * pow(y, 0.4e1) + k5 * pow(x, 0.6e1) + 0.9e1 * k5 * pow(x, 0.4e1) * y * y + 0.15e2 * k5 * x * x * pow(y, 0.4e1) + 0.7e1 * k5 * pow(y, 0.6e1) + 0.6e1 * k3 * y + 0.2e1 * k4 * x;

      // Update point
      Rox_Double det = Jxx * Jyy - Jxy*Jyx;
      if (fabs(det) < DBL_EPSILON) det = 1.0;

      Rox_Double ux = (Jyy * dx - Jxy * dy)/det;
      Rox_Double uy = (-Jyx * dx + Jxx * dy)/det;

      x = x + ux;
      y = y + uy;
   }

   // Compute result distance
   yy = y * y;
   xx = x * x;
   xy = x * y;

   r2 = yy + xx;
   r4 = r2 * r2;

   // Radial distortion
   radial = 1.0f + k1*r2 + k2*r4 + k5*r4*r2;

   // Tangential distortion
   tangx = 2.0f * k3 * xy + k4 * (r2 + 2.0f*xx);
   tangy = k3 * (r2 + 2.0f * yy) + 2.0f * k4 * xy;

   // Solution
   solx = x * radial + tangx;
   soly = y * radial + tangy;

   solx = px * solx + skew * soly + u0;
   soly = py * soly + v0;

   // Difference
   dx = solx - distorted->u;
   dy = soly - distorted->v;

   // Go back to normalized coordinates (NOT in pixels)
   undistorted->u = x; //px*x+skew*y+u0;
   undistorted->v = y; //py*y+v0;

   // Check validity
   if (dx*dx + dy*dy > 2.0)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
   }

function_terminate:

   return ROX_ERROR_NONE;
}


Rox_ErrorCode rox_invert_calibration(Rox_Float * fu_i, Rox_Float * fv_i, Rox_Float * cu_i, Rox_Float * cv_i, Rox_Float * su_i, Rox_Float fu, Rox_Float fv, Rox_Float cu, Rox_Float cv, Rox_Float su)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Inverse of calibration
   *fu_i = 1.0f / fu;
   *fv_i = 1.0f / fv;
   *su_i = -su/(fu * fv);
   *cu_i = (su * cv - cu * fv)/(fu * fv);
   *cv_i = -cv/fv;

   return error;
}

Rox_ErrorCode rox_meters_to_pixels(Rox_Float * u, Rox_Float * v, Rox_Float x, Rox_Float y, Rox_Float fu, Rox_Float fv, Rox_Float cu, Rox_Float cv, Rox_Float su)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   *u = fu * x + cu + su * y ;
   *v = fv * y + cv;

   return error;
}

Rox_ErrorCode rox_pixels_to_meters(Rox_Float * x, Rox_Float * y, Rox_Float u, Rox_Float v, Rox_Float fu_i, Rox_Float fv_i, Rox_Float cu_i, Rox_Float cv_i, Rox_Float su_i)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   *x = fu_i * u + cu_i + su_i * v ;
   *y = fv_i * v + cv_i;

   return error;
}

Rox_ErrorCode rox_meters_distortion_brown(Rox_Float * xd, Rox_Float * yd, Rox_Float xu, Rox_Float yu, Rox_Float k1, Rox_Float k2, Rox_Float k3, Rox_Float k4, Rox_Float k5)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // precompute values
   Rox_Float xx = xu * xu;
   Rox_Float xy = xu * yu;
   Rox_Float yy = yu * yu;
   Rox_Float r2 = yy + xx;
   Rox_Float r4 = r2 * r2;

   // Radial distortion
   Rox_Float radial = 1.0f + k1*r2 + k2*r4 + k5*r4*r2;

   // Tangential distortion
   Rox_Float tangx = 2.0f * k3 * xy + k4 * (r2 + 2.0f*xx);
   Rox_Float tangy = k3 * (r2 + 2.0f * yy) + 2.0f * k4 * xy;

   // Apply radial and tangential distortion
   *xd = radial * xu + tangx;
   *yd = radial * yu + tangy;

   return error;
}

Rox_ErrorCode rox_pixels_distortion_brown(Rox_Float ud, Rox_Float vd, Rox_Float uu, Rox_Float vu, Rox_Float fu, Rox_Float fv, Rox_Float cu, Rox_Float cv, Rox_Float su, Rox_Float k1, Rox_Float k2, Rox_Float k3, Rox_Float k4, Rox_Float k5)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Float fu_i=1.0f, fv_i=1.0f, cu_i=0.0f, cv_i=0.0f, su_i=0.0f;
   Rox_Float xu=0.0f, yu=0.0f;
   Rox_Float xd=0.0f, yd=0.0f;

   error = rox_invert_calibration(&fu_i, &fv_i, &cu_i, &cv_i, &su_i, fu, fv, cu, cv, su);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Compute meters coordinates
   error = rox_pixels_to_meters(&xu, &yu, uu, vu, fu_i, fv_i, cu_i, cv_i, su_i);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_meters_distortion_brown(&xd, &yd, xu, yu, k1, k2, k3, k4, k5);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_meters_to_pixels(&ud, &vd, xd, yd, fu, fv, cu, cv, su);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}