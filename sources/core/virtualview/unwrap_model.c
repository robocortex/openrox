//==============================================================================
//
//    OPENROX   : File unwrap_model.c
//
//    Contents  : Implementation of unwrap_model module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "unwrap_model.h"

#include <baseproc/maths/maths_macros.h>

#include <baseproc/geometry/transforms/matsl3/sl3interfrom3dpoints.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_bilinear_omo_uchar_to_uchar/remap_bilinear_omo_uchar_to_uchar.h>
#include <baseproc/image/remap/remap_box_halved/remap_box_halved.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/numeric/array2d_save.h>

static int count = 0;

Rox_ErrorCode rox_unwrap_model (
   Rox_Image * unwrapped,
   Rox_Imask  * unwrapped_mask,
   Rox_Plane3D_Double plane,
   Rox_MatSE3 newpose,
   Rox_MatUT3 newcalib,
   const Rox_Image image_model,
   const Rox_Point3D_Double vertices,
   const Rox_Uint basesize
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double Hinter = NULL, Hinter_inv = NULL;
   Rox_MatSE3 Tinter = NULL;
   Rox_Point2D_Double_Struct planarcoord[4];
   Rox_Double minu, minv, maxu, maxv, dwidth, dheight, nwidth, nheight;
   Rox_MeshGrid2D_Float destmap = NULL;
   Rox_Imask destmask = NULL;
   Rox_Image dest = NULL;
   Rox_Image minified = NULL, bufferminif = NULL;
   Rox_Double zoom, curzoom;

   if( !unwrapped || !unwrapped_mask || !newpose || !newcalib ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!image_model || !vertices )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check input
   error = rox_matse3_check_size ( newpose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( newcalib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get sixe in pixels of image model
   Rox_Sint swidth = 0, sheight = 0;
   error = rox_array2d_uchar_get_size ( &sheight, &swidth, image_model );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create buffers
   error = rox_matsl3_new ( &Hinter );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_new ( &Hinter_inv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &Tinter );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // First decompose the problem :
   // Compute the 3D transformation to the 3d plane (such that the new plane is parralel to the camera plane)
   // Compute the homography Hinter from this new plane to the model.
   // TODO: If Hinter will be always an UT3 matrix it is better to change its type
   // !!! Warning, perhaps need to be optimized for patches with rotation

   error = rox_sl3_from_3d_points_double ( plane, Hinter, Tinter, vertices, swidth, sheight );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_array2d_double_svdinverse(Hinter_inv, Hinter);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_inv ( Hinter_inv, Hinter );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Accessors
   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, Hinter_inv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Basically, we have no information about template calibration, so we look for a plane with good properties and define virtual calibration.
   // Retrieve the 2D coordinates of the template on the new plane using the Hinter_inv homography.

   Rox_Double u_coord = ((double) swidth) - 0.5;
   Rox_Double v_coord = ((double)sheight) - 0.5;

   // Compute 3D coordinates using inverse of Hinter
   Rox_Double u = -0.5 * dh[0][0] - 0.5 * dh[0][1] + dh[0][2];
   Rox_Double v = -0.5 * dh[1][0] - 0.5 * dh[1][1] + dh[1][2];
   Rox_Double w = -0.5 * dh[2][0] - 0.5 * dh[2][1] + dh[2][2];

   planarcoord[0].u = u / w;
   planarcoord[0].v = v / w;

   u = u_coord * dh[0][0] - 0.5 * dh[0][1] + dh[0][2];
   v = u_coord * dh[1][0] - 0.5 * dh[1][1] + dh[1][2];
   w = u_coord * dh[2][0] - 0.5 * dh[2][1] + dh[2][2];
   
   planarcoord[1].u = u / w;
   planarcoord[1].v = v / w;

   u = u_coord * dh[0][0] + v_coord * dh[0][1] + dh[0][2];
   v = u_coord * dh[1][0] + v_coord * dh[1][1] + dh[1][2];
   w = u_coord * dh[2][0] + v_coord * dh[2][1] + dh[2][2];

   planarcoord[2].u = u / w;
   planarcoord[2].v = v / w;

   u = -0.5 * dh[0][0] + v_coord * dh[0][1] + dh[0][2];
   v = -0.5 * dh[1][0] + v_coord * dh[1][1] + dh[1][2];
   w = -0.5 * dh[2][0] + v_coord * dh[2][1] + dh[2][2];

   planarcoord[3].u = u / w;
   planarcoord[3].v = v / w;

   // Compute bounding box
   minu = planarcoord[0].u;
   maxu = planarcoord[0].u;
   minv = planarcoord[0].v;
   maxv = planarcoord[0].v;

   if (planarcoord[1].u < minu) minu = planarcoord[1].u;
   if (planarcoord[1].u > maxu) maxu = planarcoord[1].u;
   if (planarcoord[1].v < minv) minv = planarcoord[1].v;
   if (planarcoord[1].v > maxv) maxv = planarcoord[1].v;
   if (planarcoord[2].u < minu) minu = planarcoord[2].u;
   if (planarcoord[2].u > maxu) maxu = planarcoord[2].u;
   if (planarcoord[2].v < minv) minv = planarcoord[2].v;
   if (planarcoord[2].v > maxv) maxv = planarcoord[2].v;
   if (planarcoord[3].u < minu) minu = planarcoord[3].u;
   if (planarcoord[3].u > maxu) maxu = planarcoord[3].u;
   if (planarcoord[3].v < minv) minv = planarcoord[3].v;
   if (planarcoord[3].v > maxv) maxv = planarcoord[3].v;

   dwidth  = maxu - minu;
   dheight = maxv - minv;

   // Compute optimal destination size
   if (dwidth > dheight)
   {
      nwidth = (int)((double)basesize) * (dwidth / dheight);
      nheight = (int)((double)basesize);
   }
   else
   {
      nheight = (int)((double)basesize) * (dheight / dwidth);
      nwidth  = (int)((double)basesize);
   }

   // Calibration
   Rox_Double fu = (nwidth  / dwidth );
   Rox_Double fv = (nheight / dheight);
   Rox_Double cu = (- 2.0*(nwidth / dwidth  * minu ) - 1.0)/2.0;
   Rox_Double cv = (- 2.0*(nheight/ dheight * minv ) - 1.0)/2.0;

   // Update homography
   dh[0][0] = fu * dh[0][0] + cu * dh[2][0];
   dh[0][1] = fu * dh[0][1] + cu * dh[2][1];
   dh[0][2] = fu * dh[0][2] + cu * dh[2][2];
   dh[1][0] = fv * dh[1][0] + cv * dh[2][0];
   dh[1][1] = fv * dh[1][1] + cv * dh[2][1];
   dh[1][2] = fv * dh[1][2] + cv * dh[2][2];
   dh[2][0] = dh[2][0];
   dh[2][1] = dh[2][1];
   dh[2][2] = dh[2][2];

   // Warp is template_H_plane

   error = rox_array2d_double_svdinverse ( Hinter, Hinter_inv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute zoom of template wrt wanted size
   error = rox_transformtools_homography_get_areazoom ( &zoom, Hinter, (basesize) / 2, (basesize) / 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   zoom = sqrt(zoom);

   // Resize iteratively
   curzoom = 1.0;

   error = rox_array2d_uchar_new ( &minified, sheight, swidth );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_copy ( minified, image_model );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (zoom > 2.0)
   {
      do
      {
         bufferminif = minified;
         zoom /= 2.0;
         curzoom /= 2.0;

         error = rox_array2d_uchar_new(&minified, (Rox_Sint)(((Rox_Double)sheight) * curzoom), (Rox_Sint) (((Rox_Double)swidth) * curzoom));
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_remap_box_nomask_uchar_to_uchar_halved(minified, bufferminif);
         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_array2d_uchar_del(&bufferminif);
      }
      while (zoom > 1.0);
   }

   // Update homography to adapt to unzoomed image
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, Hinter);
   dh[0][0] = curzoom * dh[0][0];
   dh[0][1] = curzoom * dh[0][1];
   dh[0][2] = curzoom * dh[0][2];
   dh[1][0] = curzoom * dh[1][0];
   dh[1][1] = curzoom * dh[1][1];
   dh[1][2] = curzoom * dh[1][2];

   // Create template from minified
   error = rox_meshgrid2d_float_new ( &destmap, (Rox_Sint) nheight, (Rox_Sint) nwidth );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new ( &destmask, (Rox_Sint) nheight, (Rox_Sint) nwidth );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new ( &dest, (Rox_Sint) nheight, (Rox_Sint) nwidth);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_warp_grid_sl3_float ( destmap, Hinter );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_remap_bilinear_omo_uchar_to_uchar ( dest, destmask, minified, destmap ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Store result
   *unwrapped = dest;
   *unwrapped_mask = destmask;

   // Store Tinter
   error = rox_matse3_copy ( newpose, Tinter );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Store calibration
   rox_array2d_double_fillunit  ( newcalib );
   rox_array2d_double_set_value ( newcalib, 0, 0, fu );
   rox_array2d_double_set_value ( newcalib, 1, 1, fv );
   rox_array2d_double_set_value ( newcalib, 0, 2, cu );
   rox_array2d_double_set_value ( newcalib, 1, 2, cv );

   count++;

function_terminate:

   if (error)
   {
      rox_array2d_uint_del(&destmask);
      rox_array2d_uchar_del(&dest);
   }

   rox_array2d_uchar_del(&minified);
   rox_meshgrid2d_float_del(&destmap);
   rox_matsl3_del(&Hinter);
   rox_matsl3_del(&Hinter_inv);
   rox_matse3_del(&Tinter);

   return error;
}
