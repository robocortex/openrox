//==============================================================================
//
//    OPENROX   : File rox_example_cam2proj.c
//
//    Contents  : A simple example program for camera-projector calibration
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== HEADERS   ================================================================

#include <stdio.h>
#include <api/openrox.h>

#include <baseproc/array/fill/fillzero.h>
#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/geometry/plane/plane_transform.h>
#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/transforms/matsl3/sl3fromNpoints.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <core/model/model_projector_checkerboard.h>
#include <core/model/model_checkerboard.h>

#include <inout/system/errors_print.h>
#include <inout/image/pgm/pgmfile.h>
#include <inout/numeric/array2d_print.h>

#include <user/calibration/camproj/calibration_camproj_checkerboard.h>
#include <user/identification/checkerboard/ident_checkerboard.h>
#include <user/identification/texture/ident_texture_sl3.h>

#define SUFFIX_PATTERN "calibs/test_camproj/pattern.pgm"
#define SUFFIX_INPUT   "calibs/test_camproj/input.pgm"
#define SUFFIX_OUTPUT  "calibs/test_camproj/output.pgm"

Rox_Sint main( Rox_Void )
{
   Rox_ErrorCode error;

   char* prefix;
   char path_pattern[FILENAME_MAX];
   char path_input[FILENAME_MAX];
   char path_output[FILENAME_MAX];
   Rox_Matrix Kc=NULL, Kp=NULL, pTc=NULL;
   Rox_Double **dKc=NULL, **dKp=NULL, **dpTc=NULL;
   Rox_Array2D_Uchar img_input=NULL;
   Rox_Array2D_Uchar img_pattern=NULL;
   Rox_Array2D_Uchar img_output=NULL;
   Rox_Sint width_input=  0;
   Rox_Sint height_input=  0;
   Rox_Sint width_output=800; // size of output image
   Rox_Sint height_output=600;
   Rox_CheckerCorner_Detector detector;
   Rox_CheckerBoard_Detector  checkerdetector;
   Rox_Model_CheckerBoard     checkerboard_model;
   Rox_Ident_CheckerBoard     checkerboard_detector;
   Rox_CheckerBoard           checker;
   Rox_Sint width;
   Rox_Sint height;
   Rox_Point2D_Double_Struct *detected_grid=NULL;
   Rox_Point2D_Double_Struct *pts2D=NULL;
   Rox_Array2D_Double cGo;
   Rox_Array2D_Double cTo;
   Rox_Double ta, tb, tc, td;
   Rox_Array2D_Double pGc;
   Rox_Array2D_Double cGp;
   Rox_Uchar **dimg_output=NULL;
   Rox_Uchar **dimg_input=NULL;
   Rox_Sint     i=0,         j=0, rows =0, cols =0;
   Rox_Double sizex=0.027, sizey=0.027; //Size of printed square, in meters
   Rox_Point2D_Double_Struct pp, pc;
   Rox_Sint i1, i2, j1, j2;
   Rox_Double pi1, pi2, pj1, pj2;

   // Create the intrinsics matrices 
   error = rox_matrix_new(&Kc, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new(&Kp, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create the extrinsic matrix 
   error = rox_matrix_new(&pTc, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Fill these matrices 
   error = rox_array2d_double_fillzero( Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dKc, Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_array2d_double_fillzero( Kp );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dKp, Kp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero( pTc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dpTc, pTc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   dKc[0][0] = 440.8617;
   dKc[0][2] = 659.8434;
   dKc[1][1] = 438.8332;
   dKc[1][2] = 471.5803;
   dKc[2][2] =   1.0   ;

   dKp[0][0] = 1604.8239;
   dKp[0][2] =  408.5599;
   dKp[1][1] = 1594.4501;
   dKp[1][2] =  603.0901;
   dKp[2][2] =    1.0   ;

   dpTc[0][0] =  0.9998;
   dpTc[0][1] =  0.0115;
   dpTc[0][2] =  0.0138;
   dpTc[0][3] = -0.1479;

   dpTc[1][0] = -0.0119;
   dpTc[1][1] =  0.9995;
   dpTc[1][2] =  0.0275;
   dpTc[1][3] =  0.0254;

   dpTc[2][0] = -0.0135;
   dpTc[2][1] = -0.0277;
   dpTc[2][2] =  0.9995;
   dpTc[2][3] =  0.0380;

   dpTc[3][0] = 0.0000;
   dpTc[3][1] = 0.0000;
   dpTc[3][2] = 0.0000;
   dpTc[3][3] = 1.0000;

   // Read input image and input model 
   prefix = getenv( "ROX_FRAMEWORK_DEVAPPS_DATA" );
   sprintf( path_input , "%s%s", prefix, SUFFIX_INPUT );
   sprintf( path_pattern , "%s%s", prefix, SUFFIX_PATTERN );

   printf(" Input image:          %s \n", path_input );
   printf(" Checkboard to search: %s \n", path_pattern );

   error = rox_image_new_read_pgm( &img_input, path_input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_get_size(&height_input, &width_input, img_input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_new_read_pgm( &img_pattern, path_pattern );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Detect printed pattern 
   // Initialize a corner detector from the pattern image 
   error = rox_image_get_size(&rows, &cols, img_pattern); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkercorner_detector_new( &detector, cols, rows, 0, 6 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkercorner_detector_process(detector, img_pattern);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Detect checkerboard(s) 
   error = rox_checkerboard_detector_new( &checkerdetector );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_checkerboard_detector_process(checkerdetector, detector);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check one checkerboard was found 
   if ( checkerdetector->checkerboards->used == 0 )
   { error = ROX_ERROR_TEMPLATE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   printf(" Detected %d checkerboard%c \n", checkerdetector->checkerboards->used, (checkerdetector->checkerboards->used) > 1 ? 's': ' ' );

   // Get the detected board 
   checker = checkerdetector->checkerboards->data[0];
   error   = rox_checkerboard_check_order( checker, img_pattern );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Build a model of the board 
   error = rox_array2d_point2d_double_get_size(&height, &width, checker->points );
   ROX_ERROR_CHECK_TERMINATE ( error );
     
   printf(" width: %d, height: %d \n", width, height);

   error = rox_model_checkerboard_new( &checkerboard_model );
   ROX_ERROR_CHECK_TERMINATE ( error );
   rox_model_checkerboard_set_template( checkerboard_model, width, height, sizex, sizey );

   // Make a board detector from the built model 
   error = rox_ident_checkerboard_new( &checkerboard_detector );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_checkerboard_set_model( checkerboard_detector, checkerboard_model );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Detect the board in input image 
   detected_grid = (Rox_Point2D_Double_Struct *) rox_memory_allocate( sizeof(Rox_Point2D_Double_Struct), width*height );
   if ( NULL == detected_grid ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ident_checkerboard_make( detected_grid,  checkerboard_detector, img_input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute homography between printed pattern and detected observations 
   pts2D = (Rox_Point2D_Double_Struct *) rox_memory_allocate( sizeof(Rox_Point2D_Double_Struct), width*height );
   if ( NULL == pts2D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( i = 0; i < height; i++ )
      for ( j = 0; j < width; j++ )
      {
         pts2D[i*width+j].u = sizex * j;
         pts2D[i*width+j].v = sizey * i;
      }

   error = rox_array2d_double_new( &cGo, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matsl3_from_n_points_double( cGo, pts2D, detected_grid, width*height );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute camera pose 
   error = rox_array2d_double_new( &cTo, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_transformtools_build_pose_intermodel( cTo, cGo, Kc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the board plane equation in camera reference 
   // The plane/board equation is
   //    z = 0
   //
   // Multiple View Geometry, page 68:
   // if    X' = H.X
   // then π' = H^(-t) . π
   error = rox_plane_transform( &ta,  &tb,  &tc,  &td, cTo, 0.0, 0.0, 1.0, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Form homography between camera and projector 
   error = rox_array2d_double_new( &pGc, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_homography( pGc, pTc, Kp, Kc, ta, tb, tc, td );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute homography between projector and camera 
   error = rox_array2d_double_new( &cGp, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mat3x3_inverse( cGp, pGc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Fill output image 
   error = rox_image_new( &img_output, width_output, height_output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_get_data_pointer_to_pointer( &dimg_output, img_output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_get_data_pointer_to_pointer( &dimg_input, img_input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( i = 0; i < width_output; i++ )
   {
      for ( j = 0; j < height_output; j++ )
      {
         // Apply homography projector to image
         pp.u = (double) i;
         pp.v = (double) j;
         error = rox_point2d_double_homography( &pc, &pp, cGp, 1 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         i1 = (Rox_Uint) pc.u;
         i2 = i1 + 1;
         j1 = (Rox_Uint) pc.v;
         j2 = j1 + 1;

         // Stay within the image borders
         i1 = ( i1 > width_input - 1 ) ? width_input - 1 : i1;
         i2 = ( i2 > width_input - 1 ) ? width_input - 1 : i2;
         i1 = ( i1 < 0 ) ? 0 : i1;
         i2 = ( i2 < 0 ) ? 0 : i2;

         j1 = ( j1 > height_input - 1 ) ? height_input - 1 : j1;
         j2 = ( j2 > height_input - 1 ) ? height_input - 1 : j2;
         j1 = ( j1 < 0 ) ? 0 : j1;
         j2 = ( j2 < 0 ) ? 0 : j2;

         // Select and interpolate the 4 surrounding pixels
         pi1 = 1.0 - ( pc.u - (double) i1 );
         pi2 =       ( pc.u - (double) i1 );
         pj1 = 1.0 - ( pc.v - (double) j1 );
         pj2 =       ( pc.v - (double) j1 );

         // Inverse the gray level
         dimg_output[j][i] = 255 - 
            ( pj1*pi1 * dimg_input[j1][i1] +
              pj1*pi2 * dimg_input[j1][i2] +
              pj2*pi1 * dimg_input[j2][i1] +
              pj2*pi2 * dimg_input[j2][i2] );

      }
   }

   // Write output image to disk
   sprintf( path_output , "%s%s", prefix, SUFFIX_OUTPUT );
   error = rox_image_save_pgm( path_output, img_output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   printf(" Output written to:    %s \n", path_output );

function_terminate:
   rox_matrix_del( &Kc );
   rox_matrix_del( &Kp );
   rox_matrix_del( &pTc );
   rox_image_del( &img_input );
   rox_image_del( &img_pattern );
   rox_image_del( &img_output );

   rox_checkercorner_detector_del( &detector );
   rox_checkerboard_detector_del( &checkerdetector );
   rox_model_checkerboard_del( &checkerboard_model );
   rox_ident_checkerboard_del( &checkerboard_detector );

   rox_memory_delete( detected_grid );
   rox_memory_delete( pts2D );

   rox_array2d_double_del( &cGo );
   rox_array2d_double_del( &cTo );
   rox_array2d_double_del( &pGc );
   rox_array2d_double_del( &cGp );

   return error;
}
