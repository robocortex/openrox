//==============================================================================
//
//    OPENROX   : File points3d_tools.c
//
//    Contents  : Implementation of points3d tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "point3d_tools.h"

#include <math.h>

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point2d_double_struct.h>

#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point2d_struct.h>

#include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>

#include <inout/system/errors_print.h>
#include <inout/system/print.h>
#include <system/errors/errors.h>

//==============================================================================================
// Point3d Double

Rox_ErrorCode rox_point3d_double_set_data ( 
   Rox_Point3D_Double output, 
   const Rox_Double X, 
   const Rox_Double Y, 
   const Rox_Double Z
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !output ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // TODO check inputs
   output->X = X;
   output->Y = Y;
   output->Z = Z;

function_terminate:
   return error;
}


Rox_ErrorCode rox_point3d_double_copy ( 
   Rox_Point3D_Double output, 
   const Rox_Point3D_Double input 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !output || !input ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   output->X = input->X;
   output->Y = input->Y;
   output->Z = input->Z;

function_terminate:
   return error;
}


Rox_ErrorCode rox_point3d_double_distance ( 
   Rox_Double * distance, 
   Rox_Point3D_Double point3d_1, 
   Rox_Point3D_Double point3d_2 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double dX = 0.0, dY = 0.0, dZ = 0.0;

   // Test Inputs
   if ( !distance ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Test Outputs
   if ( !point3d_1  || !point3d_2 ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   dX =  point3d_1->X - point3d_2->X;
   dY =  point3d_1->Y - point3d_2->Y;
   dZ =  point3d_1->Z - point3d_2->Z;

   * distance = sqrt( dX*dX + dY*dY + dZ*dZ );

function_terminate:
   return error;
}


Rox_ErrorCode rox_point3d_double_convert_from_float ( 
   Rox_Point3D_Double output, 
   const Rox_Point3D_Float input 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Test Inputs
   if ( !input ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Test Outputs
   if ( !output ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   output->X = input->X;
   output->Y = input->Y;
   output->Z = input->Z;

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_double_compute_bounding_box ( 
   Rox_Point3D_Double bounding_box, 
   const Rox_Double * points_list,
   const Rox_Sint nb_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !bounding_box || !points_list ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double min_x = points_list[0];
   Rox_Double min_y = points_list[1];
   Rox_Double min_z = points_list[2];

   Rox_Double max_x = points_list[0];
   Rox_Double max_y = points_list[1];
   Rox_Double max_z = points_list[2];

   for ( Rox_Sint k = 1; k < nb_points; k++ )
   {
      if ( points_list[ 3*k + 0 ] <  min_x  ) min_x = points_list[ 3*k + 0 ];
      if ( points_list[ 3*k + 0 ] >  max_x  ) max_x = points_list[ 3*k + 0 ];
      if ( points_list[ 3*k + 1 ] <  min_y  ) min_y = points_list[ 3*k + 1 ];
      if ( points_list[ 3*k + 1 ] >  max_y  ) max_y = points_list[ 3*k + 1 ];
      if ( points_list[ 3*k + 2 ] <  min_z  ) min_z = points_list[ 3*k + 2 ];
      if ( points_list[ 3*k + 2 ] >  max_z  ) max_z = points_list[ 3*k + 2 ];
   }

   // Compute the 8 points of the 3D bounding box
   bounding_box[0].X = min_x; bounding_box[1].X = min_x; bounding_box[2].X = min_x; bounding_box[3].X = min_x;
   bounding_box[0].Y = min_y; bounding_box[1].Y = min_y; bounding_box[2].Y = max_y; bounding_box[3].Y = max_y;
   bounding_box[0].Z = min_z; bounding_box[1].Z = max_z; bounding_box[2].Z = min_z; bounding_box[3].Z = max_z;

   bounding_box[4].X = max_x; bounding_box[5].X = max_x; bounding_box[6].X = max_x; bounding_box[7].X = max_x;
   bounding_box[4].Y = min_y; bounding_box[5].Y = min_y; bounding_box[6].Y = max_y; bounding_box[7].Y = max_y;
   bounding_box[4].Z = min_z; bounding_box[5].Z = max_z; bounding_box[6].Z = min_z; bounding_box[7].Z = max_z;

function_terminate:
   return error;
}

//==============================================================================================
// Point3d Float

Rox_ErrorCode rox_point3d_float_copy (
   Rox_Point3D_Float output, 
   const Rox_Point3D_Float input 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !input ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !output ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   output->X = input->X;
   output->Y = input->Y;
   output->Z = input->Z;

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point3d_float_copy ( 
   Rox_Point3D_Float output, 
   const Rox_Point3D_Float input,
   const Rox_Sint nbp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !input ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !output ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Sint k=0; k<nbp; k++)  
   {
      error = rox_point3d_float_copy ( &output[k], &input[k] ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_float_compute_center ( 
   Rox_Point3D_Float center,
   Rox_Point3D_Float points_list, 
   const Rox_Sint nb_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !center || !points_list ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   if (nb_points < 1)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   center->X = points_list[0].X;
   center->Y = points_list[0].Y;
   center->Z = points_list[0].Z;

   for ( Rox_Sint k = 1; k < nb_points; k++ )
   {
      center->X += points_list[k].X;
      center->Y += points_list[k].Y;
      center->Z += points_list[k].Z;
   }

   center->X /= nb_points;
   center->Y /= nb_points;
   center->Z /= nb_points;

function_terminate:
   return error;
}


Rox_ErrorCode rox_point3d_float_compute_bounding_box ( 
   Rox_Point3D_Float bounding_box, 
   const Rox_Float * points_list,
   const Rox_Sint nb_points
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !bounding_box || !points_list ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Float min_x = points_list[0];
   Rox_Float min_y = points_list[1];
   Rox_Float min_z = points_list[2];

   Rox_Float max_x = points_list[0];
   Rox_Float max_y = points_list[1];
   Rox_Float max_z = points_list[2];

   for ( Rox_Sint k = 1; k < nb_points; k++ )
   {
      if ( points_list[ 3*k + 0 ] <  min_x  ) min_x = points_list[ 3*k + 0 ];
      if ( points_list[ 3*k + 0 ] >  max_x  ) max_x = points_list[ 3*k + 0 ];
      if ( points_list[ 3*k + 1 ] <  min_y  ) min_y = points_list[ 3*k + 1 ];
      if ( points_list[ 3*k + 1 ] >  max_y  ) max_y = points_list[ 3*k + 1 ];
      if ( points_list[ 3*k + 2 ] <  min_z  ) min_z = points_list[ 3*k + 2 ];
      if ( points_list[ 3*k + 2 ] >  max_z  ) max_z = points_list[ 3*k + 2 ];
   }

   // Compute the 8 points of the 3D bounding box
   bounding_box[0].X = min_x; bounding_box[1].X = min_x; bounding_box[2].X = min_x; bounding_box[3].X = min_x;
   bounding_box[0].Y = min_y; bounding_box[1].Y = min_y; bounding_box[2].Y = max_y; bounding_box[3].Y = max_y;
   bounding_box[0].Z = min_z; bounding_box[1].Z = max_z; bounding_box[2].Z = min_z; bounding_box[3].Z = max_z;

   bounding_box[4].X = max_x; bounding_box[5].X = max_x; bounding_box[6].X = max_x; bounding_box[7].X = max_x;
   bounding_box[4].Y = min_y; bounding_box[5].Y = min_y; bounding_box[6].Y = max_y; bounding_box[7].Y = max_y;
   bounding_box[4].Z = min_z; bounding_box[5].Z = max_z; bounding_box[6].Z = min_z; bounding_box[7].Z = max_z;

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_float_shift_scale_float ( 
   float * vertices, 
   const int n_vertices, 
   const float shift[3], 
   const float scale 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !vertices ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( n_vertices < 1 )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( int ii = 0; ii < n_vertices; ii++ )
   {
      // Center scale the vertices
      vertices[ 3*ii + 0 ] = scale * ( vertices[ 3*ii + 0 ] + shift[0] );
      vertices[ 3*ii + 1 ] = scale * ( vertices[ 3*ii + 1 ] + shift[1] );
      vertices[ 3*ii + 2 ] = scale * ( vertices[ 3*ii + 2 ] + shift[2] );
   }

function_terminate:
   return error;  
}

Rox_ErrorCode rox_point3d_float_shift_scale_double ( 
   float * vertices, 
   const int n_vertices, 
   const double shift[3], 
   const double scale 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !vertices ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( n_vertices < 1 )
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Float shift_f[3], scale_f;

   shift_f[0] = (float) shift[0];
   shift_f[1] = (float) shift[1];
   shift_f[2] = (float) shift[2];
   scale_f    = (float) scale;
   
   for ( int ii = 0; ii < n_vertices; ii++ )
   {
      // Center scale the vertices
      vertices[ 3*ii + 0 ] = scale_f * ( vertices[ 3*ii + 0 ] + shift_f[0] );
      vertices[ 3*ii + 1 ] = scale_f * ( vertices[ 3*ii + 1 ] + shift_f[1] );
      vertices[ 3*ii + 2 ] = scale_f * ( vertices[ 3*ii + 2 ] + shift_f[2] );
   }

function_terminate:
   return error;  
}

// should be named _shift instead of _add
Rox_ErrorCode rox_vector_point3d_double_shift (
   Rox_Point3D_Double pts_vec,
   Rox_Sint nbp,
   Rox_Point3D_Double pt_shift
)
{
   Rox_ErrorCode error=ROX_ERROR_NONE;

   if ( !pt_shift || !pts_vec )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }


   for (Rox_Sint k = 0; k < nbp; k++ )
   {
      pts_vec[ k ].X += pt_shift->X;
      pts_vec[ k ].Y += pt_shift->Y;
      pts_vec[ k ].Z += pt_shift->Z;
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_vector_point3d_double_center_normalize (
   Rox_Point3D_Double pts_dst,
   const Rox_Point3D_Double pts_src,
   const Rox_Sint nbp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct center;

   if ( !pts_src || !pts_dst )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_vector_point3d_double_mean ( &center, pts_src, nbp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint k = 0; k < nbp; k++ )
   {
      pts_dst[ k ].X = pts_src[ k ].X;
      pts_dst[ k ].Y = pts_src[ k ].Y;
      pts_dst[ k ].Z = pts_src[ k ].Z;
   }

   center.X = - center.X;
   center.Y = - center.Y;
   center.Z = - center.Z;

   error = rox_vector_point3d_double_shift ( pts_dst, nbp, &center );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_vector_point3d_double_normalize_unit ( pts_dst, nbp );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}



Rox_ErrorCode rox_vector_point3d_double_mean (
   Rox_Point3D_Double pt_mean,
   const Rox_Point3D_Double pts_vec,
   const Rox_Sint nbp
)
{
   Rox_ErrorCode error=ROX_ERROR_NONE;

   if ( !pt_mean || !pts_vec )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pt_mean->X = 0.0;
   pt_mean->Y = 0.0;
   pt_mean->Z = 0.0;

   for ( Rox_Sint k = 0; k < nbp; k++ )
   {
      pt_mean->X += pts_vec[ k ].X;
      pt_mean->Y += pts_vec[ k ].Y;
      pt_mean->Z += pts_vec[ k ].Z;
   }

   pt_mean->X /= (double) nbp;
   pt_mean->Y /= (double) nbp;
   pt_mean->Z /= (double) nbp;

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point3d_double_normalize_unit (
   Rox_Point3D_Double point3D_vector,
   const Rox_Sint nbp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !point3D_vector )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Sint k = 0; k < nbp; k++ )
   {
      Rox_Double X = point3D_vector[k].X;
      Rox_Double Y = point3D_vector[k].Y;
      Rox_Double Z = point3D_vector[k].Z;
      Rox_Double norm = sqrt( X*X+Y*Y+Z*Z );

      point3D_vector[k].X = point3D_vector[k].X / norm;
      point3D_vector[k].Y = point3D_vector[k].Y / norm;
      point3D_vector[k].Z = point3D_vector[k].Z / norm;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point3d_double_set_data ( 
   Rox_Point3D_Double points3D, 
   const Rox_Double * data_points3D, 
   const Rox_Sint numb_points3D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( !data_points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < numb_points3D; i++ )
   {
      points3D[i].X = data_points3D[3*i  ];
      points3D[i].Y = data_points3D[3*i+1];
      points3D[i].Z = data_points3D[3*i+2];
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point3d_double_copy ( 
   Rox_Point3D_Double output, 
   const Rox_Point3D_Double input,
   const Rox_Sint nbp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !input ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !output ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint k=0; k<nbp; k++ )  
   {
      error = rox_point3d_double_copy ( &output[k], &input[k] ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point3d_double_read_txt ( 
   Rox_Point3D_Double output, 
   const Rox_Char * filename 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE * file = fopen( filename, "r" );
   if ( !file )
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Float X, Y, Z;
   if ( fscanf( file, "%f %f %f\n", &X, &Y, &Z ) < 0 )
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // TODO check inputs
   output->X = X;
   output->Y = Y;
   output->Z = Z;

   fclose( file );

function_terminate:
   return error;
}

Rox_ErrorCode rox_vector_point3d_double_check_visibility (
   Rox_Sint * visibility, 
   const Rox_Sint image_rows,
   const Rox_Sint image_cols,
   const Rox_MatUT3 Kc, 
   const Rox_MatSE3 cTo, 
   const Rox_Point3D_Double mo,
   const Rox_Sint nbp
) 
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_DynVec_Point2D_Double pc = NULL;

   error = rox_dynvec_point2d_double_new ( &pc, nbp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Project model corners
   error = rox_point3d_double_transform_project ( pc->data, cTo, Kc, mo, nbp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check they all project within the image borders
   *visibility = 1;
   for ( Rox_Sint index = 0; index < nbp; index++ )
   {
      if ( pc->data[index].u < 0
        || pc->data[index].v < 0
        || pc->data[index].u > (double) ( image_cols - 1 )
        || pc->data[index].v > (double) ( image_rows - 1 ) )
      {
         *visibility = 0;
         goto function_terminate;
      }
   }

function_terminate:
   rox_dynvec_point2d_double_del ( &pc );
   return error;
}
