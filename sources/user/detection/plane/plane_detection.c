//==============================================================================
//
//    OPENROX   : File plane_detection.c
//
//    Contents  : Implementation of plane detection module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "plane_detection.h"

#include <system/time/timer.h>
#include <system/memory/array_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/array/conversion/array2d_double_from_uchar.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/meanvar/meanvar.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/image/filter/median/medianfilter.h>
//#include <baseproc/image/remap/remap_bilinear_nomask.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>
#include <baseproc/geometry/transforms/matsl3/sl3fromNpoints.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <core/features/detectors/quad/quad_detection.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>
#include <inout/image/pgm/pgmfile.h>
#include <baseproc/image/convert/roxgray_to_roxrgba.h>
#include <baseproc/image/draw/draw_warp_polygon.h>

#include <baseproc/image/imask/imask.h>
#include <baseproc/image/warp/image_warp_matsl3.h>

Rox_ErrorCode rox_imask_create_thresholding(Rox_Imask mask_out, Rox_Array2D_Float diff_inp, Rox_Imask mask_inp, Rox_Sint threshold);

Rox_ErrorCode rox_plane_detector_from_rectangles(
   Rox_Imask imask1, 
   Rox_Imask imask2, 
   Rox_Point2D_Double pts_rect_1, 
   Rox_Point2D_Double pts_rect_2, 
   Rox_Image image1, 
   Rox_Image image2, 
   Rox_Sint threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Sint rows = 0, cols = 0;
   
   Rox_Image  image_warped1 = NULL;
   Rox_Image  image_warped2 = NULL;
   
   Rox_Imask  imask_warped1 = NULL;
   Rox_Imask  imask_warped2 = NULL;
   
   Rox_MatSL3 homography21 = NULL;
   Rox_MatSL3 homography12 = NULL;
   
   Rox_Array2D_Float difference1 = NULL;
   Rox_Array2D_Float difference2 = NULL;

   error = rox_image_get_rows(&rows, image1);
   ROX_ERROR_CHECK_TERMINATE(error)
    
   error = rox_image_get_cols(&cols, image1);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_image_new(&image_warped1, cols, rows);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_image_new(&image_warped2, cols, rows);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_imask_new(&imask_warped1, cols, rows);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_imask_new(&imask_warped2, cols, rows);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_float_new(&difference1, rows, cols);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_float_new(&difference2, rows, cols);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_matsl3_new(&homography21);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_matsl3_new(&homography12);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_matsl3_from_n_points_double(homography21, pts_rect_2, pts_rect_1, 4);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_matsl3_from_n_points_double(homography12, pts_rect_1, pts_rect_2, 4);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   //rox_matsl3_print(homography12);
   //rox_matsl3_print(homography21);

   error = rox_image_imask_warp_matsl3(image_warped1, imask_warped1, image2, homography12);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_image_imask_warp_matsl3(image_warped2, imask_warped2, image1, homography21);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_float_substract_uchar(difference1, image_warped1, image1);          
   ROX_ERROR_CHECK_TERMINATE(error)
      
   error = rox_array2d_float_substract_uchar(difference2, image_warped2, image2);          
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_imask_create_thresholding(imask1, difference1, imask_warped1, threshold);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_imask_create_thresholding(imask2, difference2, imask_warped2, threshold);
   ROX_ERROR_CHECK_TERMINATE(error)

      // Save results
      //error = rox_imask_save_pgm("imask1.pgm", imask1);
      //ROX_ERROR_CHECK_TERMINATE(error)
    
      //error = rox_imask_save_pgm("imask2.pgm", imask2);
      //ROX_ERROR_CHECK_TERMINATE(error)
      
function_terminate:

   rox_matsl3_del(&homography21);
   rox_matsl3_del(&homography12);
   rox_image_del(&image_warped1);
   rox_image_del(&image_warped2);
   rox_imask_del(&imask_warped1);
   rox_imask_del(&imask_warped2);
   rox_array2d_float_del(&difference1);
   rox_array2d_float_del(&difference2);

	return error;
}

Rox_ErrorCode rox_imask_create_thresholding(Rox_Imask mask_out, Rox_Array2D_Float diff_inp, Rox_Imask mask_inp, Rox_Sint threshold)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Uint ** data_mask_inp = NULL;
   Rox_Uint ** data_mask_out = NULL; 
   
   Rox_Float ** data_diff_inp = NULL;
   rox_array2d_float_get_data_pointer_to_pointer(&data_diff_inp, diff_inp);
   
   Rox_Sint cols = 0, rows = 0;
   
   // reset mask                                                                                                                         
   error = rox_array2d_uint_fillval(mask_out, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_imask_get_cols(&cols, mask_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_imask_get_rows(&rows, mask_inp);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   //error = rox_imask_get_data_pointer_to_pointer(&data_mask_out, mask_out);
   //ROX_ERROR_CHECK_TERMINATE(error)  

   //error = rox_imask_get_data_pointer_to_pointer(&data_mask_inp, mask_inp);
   //ROX_ERROR_CHECK_TERMINATE(error)  
      
   error = rox_array2d_uint_get_data_pointer_to_pointer(&data_mask_out, mask_out);
   error = rox_array2d_uint_get_data_pointer_to_pointer(&data_mask_inp, mask_inp);

   for ( Rox_Sint r = 0; r < rows; r++)
   {
      for ( Rox_Sint c = 0; c < cols; c++)
      {
         if((fabs(data_diff_inp[r][c]) <  (double) threshold) && (data_mask_inp[r][c] != 0))
         {
            data_mask_out[r][c] = ~0;
         }
      }
   }

function_terminate:
   return error;
}