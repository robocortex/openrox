//============================================================================
//
//    OPENROX   : File model_single_plane.c
//
//    Contents  : Implementation of model_single_plane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "model_single_plane.h"
#include "model_single_plane_struct.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/geometry/plane/plane_3points.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point3d_tools.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/plane/plane_transform.h>

#include <core/virtualview/unwrap_model.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_model_single_plane_new ( Rox_Model_Single_Plane * model_single_plane )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_Single_Plane ret = NULL;

   if ( !model_single_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *model_single_plane = NULL;

   ret = (Rox_Model_Single_Plane) rox_memory_allocate ( sizeof(struct Rox_Model_Single_Plane_Struct), 1 );

   if ( !ret )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->image_template = NULL;
   ret->mask = NULL;

   ret->calibration_template = NULL;
   error = rox_array2d_double_new ( &ret->calibration_template, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->z0_T_o = NULL;
   error = rox_matse3_new ( &ret->z0_T_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->o_T_z0 = NULL;
   error = rox_matse3_new ( &ret->o_T_z0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->z1_T_o = NULL;
   error = rox_matse3_new ( &ret->z1_T_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->o_T_z1 = NULL;
   error = rox_matse3_new ( &ret->o_T_z1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->c_T_o = NULL;
   error = rox_matse3_new ( &ret->c_T_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->c_T_z0 = NULL;
   error = rox_matse3_new ( &ret->c_T_z0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit ( ret->c_T_z0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->c_T_z1 = NULL;
   error = rox_matse3_new ( &ret->c_T_z1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit ( ret->c_T_z1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->calibration_template_inverse = NULL;
   error = rox_array2d_double_new ( &ret->calibration_template_inverse, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit ( ret->calibration_template_inverse );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit ( ret->calibration_template);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Init all vertices to 0
   ret->vertices_ref[0].X = 0;
   ret->vertices_ref[0].Y = 0;
   ret->vertices_ref[0].Z = 0;
   ret->vertices_ref[1].X = 0;
   ret->vertices_ref[1].Y = 0;
   ret->vertices_ref[1].Z = 0;
   ret->vertices_ref[2].X = 0;
   ret->vertices_ref[2].Y = 0;
   ret->vertices_ref[2].Z = 0;
   ret->vertices_ref[3].X = 0;
   ret->vertices_ref[3].Y = 0;
   ret->vertices_ref[3].Z = 0;

   ret->vertices_cur[0].X = 0;
   ret->vertices_cur[0].Y = 0;
   ret->vertices_cur[0].Z = 0;
   ret->vertices_cur[1].X = 0;
   ret->vertices_cur[1].Y = 0;
   ret->vertices_cur[1].Z = 0;
   ret->vertices_cur[2].X = 0;
   ret->vertices_cur[2].Y = 0;
   ret->vertices_cur[2].Z = 0;
   ret->vertices_cur[3].X = 0;
   ret->vertices_cur[3].Y = 0;
   ret->vertices_cur[3].Z = 0;

   ret->is_potentially_visible = 0;

   *model_single_plane = ret;

function_terminate:
   if (error) rox_model_single_plane_del ( &ret );
   return error;
}

Rox_ErrorCode rox_model_single_plane_del ( Rox_Model_Single_Plane * model_single_plane )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Model_Single_Plane todel = NULL;


   if (!model_single_plane)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *model_single_plane;
   *model_single_plane = NULL;


   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_del ( &todel->calibration_template );
   rox_array2d_double_del ( &todel->calibration_template_inverse );
   rox_matse3_del ( &todel->z0_T_o );
   rox_matse3_del ( &todel->z1_T_o );
   rox_matse3_del ( &todel->o_T_z0 );
   rox_matse3_del ( &todel->o_T_z1 );
   rox_matse3_del ( &todel->c_T_o );
   rox_matse3_del ( &todel->c_T_z0 );
   rox_matse3_del ( &todel->c_T_z1 );
   rox_array2d_uchar_del ( &todel->image_template );
   rox_array2d_uint_del ( &todel->mask );
   rox_memory_delete ( todel );

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_single_plane_set_3d_template (
   Rox_Model_Single_Plane model_single_plane,
   const Rox_Image image_template,
   const Rox_Point3D_Double vertices,
   const Rox_Double basesize
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !model_single_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !image_template || !vertices )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Delete previous data
   rox_array2d_uchar_del(&model_single_plane->image_template);
   rox_array2d_uint_del(&model_single_plane->mask);

   // Size of original rectangle

   model_single_plane->sizex = fabs(vertices[0].X-vertices[1].X);
   model_single_plane->sizey = fabs(vertices[1].Y-vertices[2].Y);

   // Copy reference vertices

   error = rox_vector_point3d_double_copy ( model_single_plane->vertices_ref, vertices, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create new data
   // model_single_plane->z0_T_o is the pose that allow to see the plane fronto parallel with size closest to basesize
   error = rox_unwrap_model ( &model_single_plane->image_template, 
                              &model_single_plane->mask, 
                              &model_single_plane->plane_ref, 
                              model_single_plane->z0_T_o, 
                              model_single_plane->calibration_template, 
                              image_template, 
                              vertices, 
                              (const Rox_Uint) basesize
                            );
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_array2d_double_svdinverse ( model_single_plane->calibration_template_inverse, model_single_plane->calibration_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_copy ( model_single_plane->z1_T_o, model_single_plane->z0_T_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Add one to tz
   Rox_Double ** dtz1 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dtz1, model_single_plane->z1_T_o );
   dtz1[2][3] += 1.0;

   error = rox_matse3_inv ( model_single_plane->o_T_z0, model_single_plane->z0_T_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_inv ( model_single_plane->o_T_z1, model_single_plane->z1_T_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_single_plane_set_pose (
   Rox_Model_Single_Plane model_single_plane,
   const Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!model_single_plane)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy ( model_single_plane->c_T_o, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_mulmatmat ( model_single_plane->c_T_z0, model_single_plane->c_T_o, model_single_plane->o_T_z0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_array2d_double_print(model_single_plane->c_T_z0);

   error = rox_matse3_mulmatmat ( model_single_plane->c_T_z1, model_single_plane->c_T_o, model_single_plane->o_T_z1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_array2d_double_print(model_single_plane->c_T_z1);

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_single_plane_get_pose (
   Rox_MatSE3 pose,
   const Rox_Model_Single_Plane model_single_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!model_single_plane)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy ( pose, model_single_plane->c_T_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_single_plane_transform (
   Rox_Model_Single_Plane model_single_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !model_single_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the model in the object frame Fo
   Rox_Point3D_Double m_o = model_single_plane->vertices_ref;

   // Get the model in the camera frame Fc
   Rox_Point3D_Double m_c = model_single_plane->vertices_cur;

   error = rox_point3d_double_transform ( m_c, model_single_plane->c_T_o, m_o, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the plane in the object frame Fo
   Rox_Plane3D_Double plane_o = &model_single_plane->plane_ref;

   // Get the plane in the camera frame Fc
   Rox_Plane3D_Double plane_c = &model_single_plane->plane_cur; 

   // Compute 3D plane in the camera frame Fc
   error = rox_plane3d_transform ( plane_c, model_single_plane->c_T_o, plane_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

if(0)
{
   // This test has been moved into function rox_model_single_plane_check_visibility
   // Check if this plane is potentially visible
   model_single_plane->is_potentially_visible = 0;

   Rox_Double norm = sqrt ( m_c[0].X * m_c[0].X + m_c[0].Y * m_c[0].Y + m_c[0].Z * m_c[0].Z);
   if (norm > DBL_EPSILON)
   {
      // Check dot product (angle with cosinus) between normal to the plane and one corner of the plane
      Rox_Double acosplane = ( m_c[0].X * plane_c->a + m_c[0].Y * plane_c->b + m_c[0].Z * plane_c->c ) / norm;
      if ( acosplane > 0.2 ) model_single_plane->is_potentially_visible = 1;
   }
}

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_single_plane_get_vertices_ref (
   Rox_Point3D_Double vertices,
   const Rox_Model_Single_Plane model_single_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   // Test Inputs
   if ( !model_single_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Test Outputs
   if ( !vertices )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   vertices[0] = model_single_plane->vertices_ref[0];
   vertices[1] = model_single_plane->vertices_ref[1];
   vertices[2] = model_single_plane->vertices_ref[2];
   vertices[3] = model_single_plane->vertices_ref[3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_single_plane_get_vertices_cur (
   Rox_Point3D_Double vertices,
   const Rox_Model_Single_Plane model_single_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   // Test Inputs
   if ( !model_single_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Test Outputs
   if ( !vertices )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   vertices[0] = model_single_plane->vertices_cur[0];
   vertices[1] = model_single_plane->vertices_cur[1];
   vertices[2] = model_single_plane->vertices_cur[2];
   vertices[3] = model_single_plane->vertices_cur[3];

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_single_plane_copy (
   Rox_Model_Single_Plane dst,
   const Rox_Model_Single_Plane src
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Test Inputs
   if ( !src )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Test Outputs
   if ( !dst )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Delete buffers with potentially different size than source
   rox_array2d_uchar_del(&dst->image_template);
   rox_array2d_uint_del(&dst->mask);

   // Create buffers of variable size
   Rox_Sint height = 0, width = 0; 
   error = rox_array2d_uchar_get_size ( &height, &width, src->image_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new ( &dst->image_template, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new ( &dst->mask, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_copy ( dst->image_template, src->image_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_copy(dst->mask, src->mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy ( dst->calibration_template, src->calibration_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy ( dst->calibration_template_inverse, src->calibration_template_inverse );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy ( dst->z0_T_o, src->z0_T_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy ( dst->o_T_z0, src->o_T_z0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy ( dst->c_T_o, src->c_T_o );
   ROX_ERROR_CHECK_TERMINATE ( error );

   dst->is_potentially_visible = src->is_potentially_visible;
   
   dst->vertices_ref[0] = src->vertices_ref[0];
   dst->vertices_ref[1] = src->vertices_ref[1];
   dst->vertices_ref[2] = src->vertices_ref[2];
   dst->vertices_ref[3] = src->vertices_ref[3];

   dst->vertices_cur[0] = src->vertices_cur[0];
   dst->vertices_cur[1] = src->vertices_cur[1];
   dst->vertices_cur[2] = src->vertices_cur[2];
   dst->vertices_cur[3] = src->vertices_cur[3];

   dst->plane_ref = src->plane_ref;
   dst->plane_cur = src->plane_cur;

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_single_plane_set_template_xright_ydown ( 
   Rox_Model_Single_Plane model, 
   const Rox_Image image_template, 
   const Rox_Double sizex, 
   const Rox_Double sizey
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !model )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !image_template ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // If exists delete previous model image
   if (model->image_template) rox_image_del ( &model->image_template );

   // Create new model image and copy input data 
   error = rox_array2d_uchar_new_copy ( &model->image_template, image_template ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set model size in meters
   model->sizex = sizex;
   model->sizey = sizey;

   error = rox_rectangle3d_create_centered_plane_xright_ydown ( model->vertices_ref, sizex, sizey );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_model_single_plane_set_template_xright_yup (
   Rox_Model_Single_Plane model, 
   const Rox_Image image_template, 
   const Rox_Double sizex, 
   const Rox_Double sizey
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !model || !image_template ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // If exists delete previous model image
   if (model->image_template) rox_array2d_uchar_del ( &model->image_template );

   // Create new model image and copy input data 
   error = rox_array2d_uchar_new_copy ( &model->image_template, image_template ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set model size in meters
   model->sizex = sizex;
   model->sizey = sizey;

   error = rox_rectangle3d_create_centered_plane_xright_yup ( model->vertices_ref, sizex, sizey );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_model_single_plane_check_visibility (
   Rox_Model_Single_Plane model_single_plane,
   const Rox_Sint image_rows,
   const Rox_Sint image_cols,
   const Rox_MatUT3 Kc,
   const Rox_MatUT3 cTo
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Check Outputs
   if ( !model_single_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check Inputs
   if ( !Kc || !cTo )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get model points
   Rox_Point3D_Double m_o = model_single_plane->vertices_ref;
   Rox_Point3D_Double m_c = model_single_plane->vertices_cur;

   // Get the plane in the camera frame Fc
   Rox_Plane3D_Double plane_c = &model_single_plane->plane_cur; 

   model_single_plane->is_potentially_visible = 0;

   // Check if this plane is potentially visible
   Rox_Double norm = sqrt ( m_c[0].X * m_c[0].X + m_c[0].Y * m_c[0].Y + m_c[0].Z * m_c[0].Z);
   if (norm > DBL_EPSILON)
   {
      // Check dot product (angle with cosinus) between normal to the plane and one corner of the plane
      Rox_Double acosplane = ( m_c[0].X * plane_c->a + m_c[0].Y * plane_c->b + m_c[0].Z * plane_c->c ) / norm;
      if ( acosplane > 0.2 ) model_single_plane->is_potentially_visible = 1;
   }

   // Check if all vertices are projected in the image
   if ( model_single_plane->is_potentially_visible == 1 )
   {
      error = rox_vector_point3d_double_check_visibility ( &model_single_plane->is_potentially_visible, image_rows, image_cols, Kc, cTo, m_o, 4 );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
