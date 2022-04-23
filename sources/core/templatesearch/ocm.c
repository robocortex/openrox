//==============================================================================
//
//    OPENROX   : File ocm.h
//
//    Contents  : Implementation of ocm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ocm.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>
#include <baseproc/maths/maths_macros.h>
#include <system/memory/datatypes.h>
#include <generated/dynvec_point2d_sshort_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ocm_cardinal_process(Rox_Double * score
   , Rox_Sshort x, Rox_Sshort y
   , Rox_DynVec_Point2D_Sshort edges, Rox_Array2D_Sshort anglemap_template, Rox_Array2D_Sshort anglemap_image
   , Rox_Array2D_Sshort distancemap, Rox_Double lambda, Rox_Double mean_edges
   , Rox_Sint max_dist, Rox_Double max_angle)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double difangle, denom;

   if (!score || !edges || !anglemap_image || !anglemap_template || !distancemap) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint iwidth = 0, iheight = 0;
   error = rox_array2d_sshort_get_size(&iheight, &iwidth, anglemap_image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sshort_check_size(distancemap, iheight, iwidth);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sshort ** dat = NULL; rox_array2d_sshort_get_data_pointer_to_pointer(&dat, anglemap_template);
   Rox_Sshort ** dai = NULL; rox_array2d_sshort_get_data_pointer_to_pointer(&dai, anglemap_image);
   Rox_Sshort ** dd = NULL; rox_array2d_sshort_get_data_pointer_to_pointer(&dd, distancemap);

   Rox_Sint count = 0;
   for (Rox_Uint idedge = 0; idedge < edges->used; idedge++)
   {
      Rox_Sshort tu = edges->data[idedge].u;
      Rox_Sshort tv = edges->data[idedge].v;
      Rox_Sshort iu = tu + x;
      Rox_Sshort iv = tv + y;

      //test if we're out of the image (needed for non-centered templates) or if distance is too big
      if (iu < 0 || iv < 0 || iu >= iwidth || iv >= iheight || dd[iv][iu] > max_dist)
         continue;

      //test if angle is too big
      difangle = (dat[tv][tu] - dai[iv][iu])/10000.0;//x1000 to avoid storing float when we don't need float precision
      difangle = fmod(difangle, ROX_PI);

      if (difangle > max_angle)
         continue;

      count++;
   }

   denom = (lambda * ((double)edges->used)) + ((1.0-lambda)*mean_edges);
   *score = ((double)count) / denom;

function_terminate:
   return error;
}
