//==============================================================================
//
//    OPENROX   : File image_rgba_draw.c
//
//    Contents  : Implementation of image_rgba_draw module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "image_rgba_draw.h"

#include <string.h>

#include <generated/dynvec_rect_sint_struct.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point3d_double_struct.h>
#include <generated/dynvec_sint_struct.h>

#include <baseproc/geometry/rectangle/rectangle_struct.h>
#include <baseproc/maths/maths_macros.h>

#include <baseproc/image/draw/draw_line.h>
#include <baseproc/image/draw/draw_polygon.h>
#include <baseproc/image/draw/draw_points.h>
#include <baseproc/image/draw/draw_rectangle.h>
#include <baseproc/image/draw/draw_warp_rectangle.h>
#include <baseproc/image/draw/color.h>
#include <baseproc/image/convert/roxgray_to_roxrgba.h>
#include <inout/system/errors_print.h>


Rox_ErrorCode rox_image_rgba_draw_2d_polygon (
   Rox_Image_RGBA image,
   Rox_Point2D_Double  pts,
   Rox_Uint nbpts,
   Rox_Uint color
)
{
   return rox_image_rgba_draw_polygon(image, pts, nbpts, color);
}


Rox_ErrorCode rox_image_rgba_draw_dynvec_rectangle (
   Rox_Image_RGBA image,
   Rox_DynVec_Rect_Sint dynvec_rectangle,
   Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   for (Rox_Uint i = 0; i < dynvec_rectangle->used; ++i)
   {
      error = rox_image_rgba_draw_rectangle(image, &(dynvec_rectangle->data[i]), color);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_image_rgba_draw_projection_frame (
   Rox_Image_RGBA image,
   Rox_Matrix calib,
   Rox_MatSE3 pose,
   Rox_Double size
)
{
   Rox_ErrorCode error = 0;

   Rox_Point3D_Double_Struct pts_x[2];
   Rox_Point3D_Double_Struct pts_y[2];
   Rox_Point3D_Double_Struct pts_z[2];

   // x-axis : blue ; y-axis : green ; z-axis : red
   Rox_Uint color[3] = { (Rox_Uint) ROX_MAKERGBA(0, 0, 255, 255), (Rox_Uint) ROX_MAKERGBA(0, 255, 0, 255), (Rox_Uint) ROX_MAKERGBA(255, 0, 0, 255)};

   // Define 3D pts
   pts_x[0].X =  0.00;   pts_x[0].Y = 0.00; pts_x[0].Z = 0.00;
   pts_x[1].X =  size;   pts_x[1].Y = 0.00; pts_x[1].Z = 0.00;

   pts_y[0].X =  0.00;   pts_y[0].Y = 0.00; pts_y[0].Z = 0.00;
   pts_y[1].X =  0.00;   pts_y[1].Y = size; pts_y[1].Z = 0.00;

   pts_z[0].X =  0.00;   pts_z[0].Y = 0.00; pts_z[0].Z = 0.00;
   pts_z[1].X =  0.00;   pts_z[1].Y = 0.00; pts_z[1].Z = size;

   error = rox_image_rgba_draw_projection_3d_polygon(image, calib, pose, pts_x, 2, color[0]);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_image_rgba_draw_projection_3d_polygon(image, calib, pose, pts_y, 2, color[1]);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_image_rgba_draw_projection_3d_polygon(image, calib, pose, pts_z, 2, color[2]);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
  return error;
}


Rox_ErrorCode rox_image_rgba_draw_matches_new (
  Rox_Image_RGBA         *image,
  Rox_Image                  image1,
  Rox_Image                  image2,
  Rox_DynVec_Point2D_Double  pts1,
  Rox_DynVec_Point2D_Double  pts2,
  Rox_DynVec_Sint            matches,
  Rox_Uint                   matched_color,
  Rox_Uint                   unmatched_color
)
{
   Rox_ErrorCode               error=ROX_ERROR_NONE;
   Rox_Image                   diptych=NULL;
   Rox_Sint                    cols1, rows1, cols2, rows2;
   Rox_DynVec_Point2D_Double   pts2_shift=NULL;

   if ( NULL == image || NULL == image1 || NULL == image2 || NULL == pts1 || NULL == pts2 || NULL == matches )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create big grey image
   error = rox_image_get_size( &rows1, &cols1, image1 );                                        
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_get_size( &rows2, &cols2, image2 );                                        
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_new( &diptych, cols1+cols2, ROX_MAX(rows1, rows2) );                       
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Fill big grey image
   Rox_Uchar ** rowsptr1 = NULL;
   error = rox_image_get_data_pointer_to_pointer( &rowsptr1, image1 );                                          
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** rowsptr2 = NULL; 
   error = rox_image_get_data_pointer_to_pointer( &rowsptr2, image2 );                                          
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** rowsptrd = NULL;
   error = rox_image_get_data_pointer_to_pointer( &rowsptrd, diptych );                                         
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( int ii = 0; ii < rows1; ii++ )
      memcpy( rowsptrd[ ii ], rowsptr1[ ii ], cols1*sizeof(Rox_Uchar) );

   for ( int ii = 0; ii < rows2; ii++ )
      memcpy( &( rowsptrd[ ii ][ cols1 ] ), rowsptr2[ ii ], cols2*sizeof(Rox_Uchar) );

   // Convert grey image to color
   error = rox_image_rgba_new( image, cols1+cols2, ROX_MAX(rows1, rows2) );                  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_roxgray_to_roxrgba( *image, diptych );                                           
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Draw points
   error = rox_image_rgba_draw_dynvec_point2d_double( *image, pts1, unmatched_color );       
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_new( &pts2_shift, ROX_MAX(0, (pts2->used)/10) );           
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_stack( pts2_shift, pts2 );                                 
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint ii = 0; ii < pts2_shift->used; ii++ )
   {
      pts2_shift->data[ ii ].u += cols1;
   }

   error = rox_image_rgba_draw_dynvec_point2d_double( *image, pts2_shift, unmatched_color ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Draw lines
   for (Rox_Uint ii = 0; ii < matches->used; ii++ )
   {
      int jj = matches->data[ ii ];

      if ( -1 == jj )
         continue;

      error = rox_image_rgba_draw_2d_points( *image , &(pts1->data[ ii ])       , 1 , matched_color );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_image_rgba_draw_2d_points( *image , &(pts2_shift->data[ jj ]) , 1 , matched_color );
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Uint color=rand();
      error = rox_image_rgba_draw_line( *image, &pts1->data[ ii ], &pts2_shift->data[ jj ], color );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:

   if ( NULL != pts2_shift ) rox_dynvec_point2d_double_del( &pts2_shift );
   if ( NULL != diptych )    rox_image_del( &diptych );

   return error;
}
