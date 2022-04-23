//==============================================================================
//
//    OPENROX   : File caofile_edges.c
//
//    Contents  : Implementation of add module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "caofile_edges.h"

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <float.h>

#include <generated/dynvec_uint.h>
#include <generated/objset_dynvec_point3d_double.h>

#include <generated/objset_ellipse3d.h>
#include <generated/objset_cylinder3d.h>

#include <system/memory/memory.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point3d_tools.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/determinant/detgl3.h>
#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/crossprod/crossprod.h>
#include <baseproc/array/flip/fliplr.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/decomposition/svdsort.h>
#include <baseproc/array/multiply/mulmatmat.h>

//serialization needed
#include <generated/dynvec_point3d_double_struct.h>

#include <generated/objset_dynvec_point3d_double_struct.h>
#include <generated/objset_ellipse3d_struct.h>
#include <generated/objset_cylinder3d_struct.h>

#include <system/memory/array2d_struct.h>

#include <baseproc/geometry/ellipse/ellipse3d_struct.h>
#include <baseproc/geometry/cylinder/cylinder3d_struct.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

#include <assert.h>

#define NB_FILES_RECURSIVE_MAX   100
#define LINE_MAX_LENGTH          256
#define FILEPATH_MAX_LENGTH      256
//TODO what does that mean, where does this number come from ?
#define MAX_DATA_NBR             100000

#ifdef OPENROX_DISPLAY
   #define CHECK_ERROR_FSCANF(X) if (!X) { error = ROX_ERROR_BAD_IOSTREAM; CHECK_ERROR_LOADCAO(error); }
   #define CHECK_ERROR_FGETS(X) if (X==NULL){error = ROX_ERROR_BAD_IOSTREAM; CHECK_ERROR_LOADCAO(error); }
   #define CHECK_ERROR_LOADCAO(X)\
   {\
      if (X)\
      {\
         if (cao_points)rox_memory_delete(cao_points);\
         if (cao_line_points)rox_memory_delete(cao_line_points);\
         if (segments_points)rox_dynvec_point3d_double_del(&segments_points);\
         if (corners)rox_dynvec_point3d_double_del(&corners);\
         if (segments_points_indices)rox_dynvec_uint_del(&segments_points_indices);\
         if (face_segment_key_vector)rox_dynvec_uint_del(&face_segment_key_vector);\
         assert(0); \
         ROX_ERROR_CHECK_TERMINATE(error)\
      }\
   }
#else //OPENROX_DISPLAY
   #define CHECK_ERROR_FSCANF(X)    if (!X) { error = ROX_ERROR_BAD_IOSTREAM; goto function_terminate; }
   #define CHECK_ERROR_FGETS(X)     if (X==NULL) { error = ROX_ERROR_BAD_IOSTREAM; goto function_terminate; }
   #define CHECK_ERROR_LOADCAO(X)   if (X) { error = ROX_ERROR_BAD_IOSTREAM; goto function_terminate; }
#endif //OPENROX_DISPLAY


#define STRIP_LINE() cur = strstr(cur, "\n");\
cur++;

#define STRIP_PARAMETERS() STRIP_LINE()

#define STRIP_COMMENTS() while (*cur == '#')\
{\
   cur = strstr(cur, "\n");\
   cur++;\
}

#define STRIP_TO_SPACE() while (!isspace(*cur))\
{\
   cur++;\
}

#define STRIP_SPACES() while (isspace(*cur))\
{\
   cur++;\
}

//! Stucture
struct Rox_CaoFile_Edges_Struct
{
   //! Extracted segments
   Rox_ObjSet_DynVec_Point3D_Double   poly_segments;

   //! Extracted ellipses
   Rox_ObjSet_Ellipse3D               ellipses;

   //! Extracted cylinders
   Rox_ObjSet_Cylinder3D              cylinders;

   //! Results of visibility tests
   Rox_DynVec_Point3D_Double          visible_segments;

   //! Results of visibility tests
   Rox_ObjSet_Ellipse3D               visible_ellipses;

   //! Results of visibility tests
   Rox_ObjSet_Cylinder3D              visible_cylinders;
};

#ifdef OPENROX_DISPLAY
Rox_ErrorCode rox_caofile_edges_print(Rox_CaoFile_Edges obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!obj) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }
   ROX_INFO_FORMATTED_DISPLAY("\nrox_caofile_edges_print obj %s\n", "")
   //poly_segments
   ROX_INFO_FORMATTED_DISPLAY("poly_segments->used: %d\n", obj->poly_segments->used);

   for (Rox_Uint id_poly = 0; id_poly < obj->poly_segments->used; id_poly++)
   {
      ROX_INFO_FORMATTED_DISPLAY("poly_segments->data[id_poly]->used: %d\n", obj->poly_segments->data[id_poly]->used);
      for (Rox_Uint id_point = 0; id_point < obj->poly_segments->data[id_poly]->used; id_point++)
      {
         ROX_INFO_FORMATTED_DISPLAY("\t%.2f\t%.2f\t%.2f\n", obj->poly_segments->data[id_poly]->data[id_point].X, obj->poly_segments->data[id_poly]->data[id_point].Y, obj->poly_segments->data[id_poly]->data[id_point].Z);
      }
   }


   //ellipses
   ROX_INFO_FORMATTED_DISPLAY("ellipses->used: %d\n", obj->ellipses->used);
   for (Rox_Uint id_ell = 0; id_ell < obj->ellipses->used; id_ell++)
   {
      ROX_INFO_FORMATTED_DISPLAY("ellipse a: %.2f, b: %.2f, Te:\n", obj->ellipses->data[id_ell]->a, obj->ellipses->data[id_ell]->b);
      rox_array2d_double_print(obj->ellipses->data[id_ell]->Te);
   }

   //cylinders
   ROX_INFO_FORMATTED_DISPLAY("cylinders->used: %d\n", obj->cylinders->used);
   for (Rox_Uint id_cyl = 0; id_cyl < obj->cylinders->used; id_cyl++)
   {
      ROX_INFO_FORMATTED_DISPLAY("cylinders a: %.2f, b: %.2f, h: %.2f, Te:\n", obj->cylinders->data[id_cyl]->a, obj->cylinders->data[id_cyl]->b, obj->cylinders->data[id_cyl]->h);
      rox_array2d_double_print(obj->cylinders->data[id_cyl]->T);
   }

function_terminate:
   return error;
}
#else

Rox_ErrorCode rox_caofile_edges_print(Rox_CaoFile_Edges obj) { return ROX_ERROR_NONE; }
#endif


Rox_ErrorCode rox_caofile_edges_new(Rox_CaoFile_Edges * caofile_edges)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CaoFile_Edges ret = NULL;

   if (!caofile_edges)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   *caofile_edges = NULL;

   ret = (Rox_CaoFile_Edges) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   error = rox_objset_dynvec_point3d_double_new(&ret->poly_segments, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_ellipse3d_new(&ret->ellipses, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_cylinder3d_new(&ret->cylinders, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_new(&ret->visible_segments, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_ellipse3d_new(&ret->visible_ellipses, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_cylinder3d_new(&ret->visible_cylinders, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *caofile_edges = ret;

function_terminate:
   if (error) rox_caofile_edges_del(&ret);

   return error;
}

Rox_ErrorCode rox_caofile_edges_del(Rox_CaoFile_Edges * caofile_edges)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CaoFile_Edges todel;

   if (!caofile_edges)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   todel = *caofile_edges;
   *caofile_edges = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   // Delete structs members

   error = rox_objset_ellipse3d_del(&todel->ellipses);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_ellipse3d_del(&todel->visible_ellipses);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_cylinder3d_del(&todel->cylinders);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_cylinder3d_del(&todel->visible_cylinders);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Delete structs
   error = rox_objset_dynvec_point3d_double_del(&todel->poly_segments);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_del(&todel->visible_segments);
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_memory_delete(todel);

function_terminate:
   return error;
}



//!ignores cao file comments
Rox_ErrorCode rox_cao_strip_comments(FILE * input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint c = 0;

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   do
   {
      //Get a character
      c = getc(input);
      if (c == '#')
      {
         //If comment, skip until next line / EOF
         do
         {
            c = getc(input);
         } while ((c != EOF) && (c != '\n'));
      }
   } while ((c == EOF) || isspace(c));

   // Replace last character in stream
   ungetc(c, input);

function_terminate:
   return error;
}

//!ignores cao file parameters for the current line
Rox_ErrorCode rox_cao_strip_parameters(FILE * input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint c = 0;

   if (!input) { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   //skip caracters until next line / EOF
   do
   {
      c = getc(input);
   } while ((c != EOF) && (c != '\n'));


   // Replace last character in stream
   ungetc(c, input);

function_terminate:
   return error;
}

Rox_ErrorCode add_cylinder(Rox_CaoFile_Edges caofile_edges, Rox_Point3D_Double_Struct p1, Rox_Point3D_Double_Struct p2, const Rox_Double radius)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Cylinder3D cylinder3d = NULL;
   Rox_Double h = 0.0, norm = 0.0, det = 0.0;
   Rox_Array2D_Double n = NULL, n_norm = NULL, n_skew = NULL;
   Rox_Array2D_Double oTe = NULL, R = NULL, T = NULL;
   // Rox_Array2D_Double S = NULL, V = NULL, U = NULL;

   if (!caofile_edges)
   {
      error = ROX_ERROR_NULL_POINTER; goto function_terminate;
   }

   Rox_Array2D_Double x_axis = NULL, y_axis = NULL, z_axis = NULL;
   error = rox_array2d_double_new(&x_axis, 3, 1);              ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(x_axis, 0, 0, 1);      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(x_axis, 1, 0, 0);      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(x_axis, 2, 0, 0);      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&y_axis, 3, 1);              ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(y_axis, 0, 0, 0);      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(y_axis, 1, 0, 1);      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(y_axis, 2, 0, 0);      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&z_axis, 3, 1);              ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(z_axis, 0, 0, 0);      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(z_axis, 1, 0, 0);      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(z_axis, 2, 0, 1);      ROX_ERROR_CHECK_TERMINATE ( error );

   // Prepare matrix results
   error = rox_array2d_double_new(&oTe, 4, 4);                       ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillunit(oTe);                         ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&R, oTe, 0, 0, 3, 3);   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&T, oTe, 0, 3, 3, 1);   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute cylinder heigth h
   error = rox_point3d_double_distance(&h, &p1, &p2);                ROX_ERROR_CHECK_TERMINATE ( error );

   //retrieve translation
   error = rox_array2d_double_set_value(T, 0, 0, (p1.X + p2.X)*0.5);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(T, 1, 0, (p1.Y + p2.Y)*0.5);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(T, 2, 0, (p1.Z + p2.Z)*0.5);  ROX_ERROR_CHECK_TERMINATE ( error );

   //compute normal
   error = rox_array2d_double_new(&n, 3, 1);                      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(n, 0, 0, p2.X - p1.X);    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(n, 1, 0, p2.Y - p1.Y);    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(n, 2, 0, p2.Z - p1.Z);    ROX_ERROR_CHECK_TERMINATE ( error );

   //normalize
   error = rox_array2d_double_new(&n_norm, 3, 1);              ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_norm2(&norm, n);                 ROX_ERROR_CHECK_TERMINATE ( error );
   if (norm == 0) { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   error = rox_array2d_double_scale(n_norm, n, 1.0 / norm);  ROX_ERROR_CHECK_TERMINATE ( error );

   //get skew
   error = rox_array2d_double_new(&n_skew, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_skew_from_vector(n_skew, n_norm);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //function R = build_base(z)
   Rox_Double ** dn = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dn, n_norm );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ax = fabs(acos(dn[0][0]));
   Rox_Double ay = fabs(acos(dn[1][0]));
   Rox_Double az = fabs(acos(dn[2][0]));

   Rox_Array2D_Double res = NULL, res_norm = NULL;

   error = rox_array2d_double_new(&res, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&res_norm, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ((ax >= ay) && (ax >= az))
   {
      error = rox_array2d_double_mulmatmat(res, n_skew, x_axis);  ROX_ERROR_CHECK_TERMINATE ( error );
      // x = skew(z)*[1; 0; 0];
   }
   else if ((ay >= ax) && (ay >= az))
   {
      error = rox_array2d_double_mulmatmat(res, n_skew, y_axis);  ROX_ERROR_CHECK_TERMINATE ( error );
      // x = skew(z)*[0; 1; 0];
   }
   else if ((az >= ax) && (az >= ay))
   {
      error = rox_array2d_double_mulmatmat(res, n_skew, z_axis);  ROX_ERROR_CHECK_TERMINATE ( error );
      // x = skew(z)*[0; 0; 1];
   }
   else { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_norm2(&norm, res);                 ROX_ERROR_CHECK_TERMINATE ( error );

   if (norm == 0)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_scale(res_norm, res, 1.0 / norm);
   ROX_ERROR_CHECK_TERMINATE ( error );
   // x = x / norm(x);

   error = rox_array2d_double_mulmatmat(res, n_skew, res_norm);
   ROX_ERROR_CHECK_TERMINATE ( error );
   // y = skew(z)*x;

   Rox_Double ** x_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &x_data, res_norm );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(R, 0, 0, x_data[0][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 1, 0, x_data[1][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 2, 0, x_data[2][0]);  ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** y_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &y_data, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(R, 0, 1, y_data[0][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 1, 1, y_data[1][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 2, 1, y_data[2][0]);  ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** z_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &z_data, n_norm );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(R, 0, 2, z_data[0][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 1, 2, z_data[1][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 2, 2, z_data[2][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   //R = [x, y, z];

   error = rox_array2d_double_detgl3(&det, R);     ROX_ERROR_CHECK_TERMINATE ( error );
   if (det < 0)
   {
      Rox_Double * row = NULL;
      error = rox_array2d_double_get_data_pointer (&row, R);

      error = rox_array2d_double_set_value(R, 0, 0, -row[0]);  ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_set_value(R, 0, 1, -row[1]);  ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_set_value(R, 0, 2, -row[2]);  ROX_ERROR_CHECK_TERMINATE ( error );
   }
   //   if (det(R) < 0) R(:, 1) = -R(:, 1); end;

   error = rox_cylinder3d_new(&cylinder3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_cylinder3d_set(cylinder3d, radius, radius, h, oTe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //add resulting cylinder to cylinders list our caofile_edgesect
   error = rox_objset_cylinder3d_append(caofile_edges->cylinders, cylinder3d);
   if (error)
   { goto function_terminate; }

function_terminate:
   rox_array2d_double_del(&oTe);
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&T);
   rox_array2d_double_del(&res);
   rox_array2d_double_del(&res_norm);
   rox_array2d_double_del(&n_norm);
   rox_array2d_double_del(&n);
   rox_array2d_double_del(&n_skew);
   rox_array2d_double_del(&x_axis);
   rox_array2d_double_del(&y_axis);
   rox_array2d_double_del(&z_axis);

   return error;
}

// function ellipse3D = convert_ellipse_from_cao(r, mc, m1, m2)
Rox_ErrorCode add_circle(Rox_CaoFile_Edges caofile_edges, const Rox_Point3D_Double_Struct mc, const Rox_Point3D_Double_Struct m1, const Rox_Point3D_Double_Struct m2, const Rox_Double radius)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ellipse3D ellipse3d = NULL;
   Rox_Array2D_Double x = NULL, y = NULL, z = NULL, oTe = NULL, t = NULL, R = NULL;
   Rox_Array2D_Double vector3a = NULL, vector3b = NULL;
   Rox_Double norm = 0;

   if (!caofile_edges)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   // prepare matrix results
   error = rox_array2d_double_new(&oTe, 4, 4);                       ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillunit(oTe);                         ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&R, oTe, 0, 0, 3, 3);   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&t, oTe, 0, 3, 3, 1);   ROX_ERROR_CHECK_TERMINATE ( error );

   // prepare tmp variables
   error = rox_array2d_double_new(&vector3a, 3, 1);       ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&vector3b, 3, 1);       ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&x, 3, 1);             ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&y, 3, 1);             ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&z, 3, 1);             ROX_ERROR_CHECK_TERMINATE ( error );

   // retrieve translation from mc center of points
   error = rox_array2d_double_set_value(t, 0, 0, mc.X);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(t, 1, 0, mc.Y);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(t, 2, 0, mc.Z);  ROX_ERROR_CHECK_TERMINATE ( error );

   //compute x axis in vector3
   error = rox_array2d_double_set_value(vector3a, 0, 0, m1.X - mc.X);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(vector3a, 1, 0, m1.Y - mc.Y);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(vector3a, 2, 0, m1.Z - mc.Z);   ROX_ERROR_CHECK_TERMINATE ( error );
   //normalize x axis from vector3 to x
   error = rox_array2d_double_norm2(&norm, vector3a);  ROX_ERROR_CHECK_TERMINATE ( error );
   if (norm == 0) {      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );   }
   error = rox_array2d_double_scale(x, vector3a, 1.0/norm);  ROX_ERROR_CHECK_TERMINATE ( error );

   //compute z axis starting from other axis in vector3
   error = rox_array2d_double_set_value(vector3a, 0, 0, m2.X - mc.X);   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(vector3a, 1, 0, m2.Y - mc.Y);   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(vector3a, 2, 0, m2.Z - mc.Z);   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_crossprod(vector3b, x, vector3a);         ROX_ERROR_CHECK_TERMINATE ( error );

   //normalize z axis from vector3 to z
   error = rox_array2d_double_norm2(&norm, vector3b);  ROX_ERROR_CHECK_TERMINATE ( error );
   if (norm == 0) { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   error = rox_array2d_double_scale(z, vector3b, 1.0 / norm);  ROX_ERROR_CHECK_TERMINATE ( error );

   //compute y axis from x and z cross product
   error = rox_array2d_double_crossprod(vector3a, z, x);         ROX_ERROR_CHECK_TERMINATE ( error );

   //normalize y axis from vector3 to y
   error = rox_array2d_double_norm2(&norm, vector3a);  ROX_ERROR_CHECK_TERMINATE ( error );
   if (norm == 0)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_scale(y, vector3a, 1.0 / norm);  ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** x_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &x_data, x);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(R, 0, 0, x_data[0][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 1, 0, x_data[1][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 2, 0, x_data[2][0]);  ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** y_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &y_data, y);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(R, 0, 1, y_data[0][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 1, 1, y_data[1][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 2, 1, y_data[2][0]);  ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** z_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &z_data, z);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(R, 0, 2, z_data[0][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 1, 2, z_data[1][0]);  ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value(R, 2, 2, z_data[2][0]);  ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ellipse3d_new(&ellipse3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ellipse3d_set(ellipse3d, radius, radius, oTe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Add resulting ellipse to ellipses list our caofile_edgesect
   error = rox_objset_ellipse3d_append(caofile_edges->ellipses, ellipse3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&oTe);
   rox_array2d_double_del(&vector3a);
   rox_array2d_double_del(&vector3b);
   rox_array2d_double_del(&x);
   rox_array2d_double_del(&y);
   rox_array2d_double_del(&z);
   rox_array2d_double_del(&t);
   rox_array2d_double_del(&R);

   return error;
}

Rox_ErrorCode add_polygon_segments(Rox_CaoFile_Edges caofile_edges, const Rox_DynVec_Point3D_Double corners)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct * corners_data = NULL;
   Rox_DynVec_Point3D_Double corners_without_duplicates = NULL;
   Rox_Uint i = 0, nb_corners = 0;

   if (!caofile_edges || !corners)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   //allocate and get data, allocation will be deleted with caofile_edges
   error = rox_dynvec_point3d_double_new(&corners_without_duplicates, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_get_data_pointer(&corners_data, corners);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //append first element
   error = rox_dynvec_point3d_double_append(corners_without_duplicates, &corners_data[0]);
   if (error)
   {
      if (corners_without_duplicates)rox_dynvec_point3d_double_del(&corners_without_duplicates);
      goto function_terminate;
   }

   //remove duplicates and add elements
   error = rox_dynvec_point3d_double_get_used(&nb_corners, corners);
   if (error)
   {
      if (corners_without_duplicates)rox_dynvec_point3d_double_del(&corners_without_duplicates);
      goto function_terminate;
   }
   for (i = 0; i < nb_corners - 1; i++)
   {
      //organized as 0,1,1,2,2,3,3,4 and we want them as 0,1,2,3,4
      if (fabs(corners_data[i].X - corners_data[i + 1].X) > FLT_EPSILON
         || fabs(corners_data[i].Y - corners_data[i + 1].Y) > FLT_EPSILON
         || fabs(corners_data[i].Z - corners_data[i + 1].Z) > FLT_EPSILON)
      {
         //append to current polygon
         rox_dynvec_point3d_double_append(corners_without_duplicates, &corners_data[i + 1]);
      }
   }

   //add resulting vector to polygon segments of our caofile_edgesect
   error = rox_objset_dynvec_point3d_double_append(caofile_edges->poly_segments, corners_without_duplicates);
   if (error)
   {
      if (corners_without_duplicates) rox_dynvec_point3d_double_del(&corners_without_duplicates);
      goto function_terminate;
   }

function_terminate:
   // Note: allocation of corners_without_duplicates will be deleted with caofile_edges
   return error;
}

Rox_ErrorCode load_cao(Rox_CaoFile_Edges caofile_edges, const Rox_Char * filepath, Rox_Sint* start_id_face, Rox_Char** vector_of_model_filename, Rox_Sint* current_depth)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * cao_file = NULL;
   Rox_Bool header = 0;
   Rox_Sint caoVersion = 0;
   Rox_Char c=0;
   Rox_Char line[LINE_MAX_LENGTH];
   Rox_Char path[FILEPATH_MAX_LENGTH];
   size_t path_length = 0;
   Rox_Char sub_filename[FILEPATH_MAX_LENGTH];
   Rox_Char sub_filepath[FILEPATH_MAX_LENGTH];
   size_t tmp_path_length = 0;
   Rox_Char * tmp_pch = NULL;
   const Rox_Char* prefix = "load";
   Rox_Uint cao_nbr_points = 0;
   Rox_Point3D_Double_Struct* cao_points = NULL;
   Rox_Double x, y, z; // 3D coordinates
   //int i, j;    // image coordinate (used for matching)
   Rox_DynVec_Point3D_Double segments_points = NULL;
   // Rox_Point3D_Double_Struct * segments_points_data = NULL;
   Rox_DynVec_Uint face_segment_key_vector = NULL;
   Rox_Uint face_segment_key_vector_nbr = 0;
   Rox_Uint* face_segment_key_vector_data = NULL;
   Rox_DynVec_Uint segments_points_indices = NULL;
   Rox_Uint segments_points_indices_nbr = 0;
   Rox_Uint* segments_points_indices_data = NULL;
   Rox_Uint cao_nbr_line = 0;
   Rox_Uint *cao_line_points = NULL;
   Rox_Uint index1 = 0, index2 = 0;
   Rox_Sint id_face = 0;
   Rox_Uint cao_nbr_polygon_line = 0;
   Rox_Uint index = 0;
   Rox_Uint nb_line_pol = 0;
   Rox_DynVec_Point3D_Double corners = NULL;
   Rox_Sint    stdio_err_ret = 0;
   Rox_Char*   stdio_ptr_ret = NULL;
   Rox_Uint caoNbCircle = 0;
   Rox_Uint caoNbFacesFromPoints = 0;
   Rox_Uint caoNbCylinders = 0;

   // Store folder path to be able to load relative sub cao filepath
   tmp_pch = (Rox_Char *) strrchr(filepath, '/');
   tmp_path_length = tmp_pch - filepath + 1;
   if (tmp_path_length > FILEPATH_MAX_LENGTH - 1) { error = ROX_ERROR_BAD_IOSTREAM; CHECK_ERROR_LOADCAO(error); }
   memcpy(path, &filepath[0], tmp_path_length);
   path[tmp_path_length] = '\0';
   path_length = tmp_path_length;

   // Open fiel on disk
   cao_file = fopen(filepath, "rb");
   if (!cao_file){ error = ROX_ERROR_BAD_IOSTREAM; CHECK_ERROR_LOADCAO(error); }
   rox_log("Loading cao file: %s\n", filepath);

   // Push our filepath to the end of filepaths backup
   if (*current_depth >= NB_FILES_RECURSIVE_MAX){ error = ROX_ERROR_BAD_SIZE; goto function_terminate; }
   vector_of_model_filename[*current_depth] = (Rox_Char *) rox_memory_allocate(sizeof(Rox_Char), FILEPATH_MAX_LENGTH);
   strcpy(vector_of_model_filename[*current_depth], filepath);
   (*current_depth)++;

   // Extraction of the version (remove empty line and commented ones (comment line begin with the #))
   error = rox_cao_strip_comments(cao_file);
   CHECK_ERROR_LOADCAO(error);

   //////////////////////////Read CAO Version (V1, V2, ...)//////////////////////////
   c = getc(cao_file);
   if (c == 'V')
   {
      stdio_err_ret = fscanf(cao_file, "%d", &caoVersion);
      CHECK_ERROR_FSCANF(stdio_err_ret);

      stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file); // skip the rest of the line
      CHECK_ERROR_FGETS(stdio_ptr_ret);
   }
   else
   {
      rox_log("Error in vpMbTracker::loadCAOModel() -> Bad parameter header file : use V0, V1, ... in  %s", filepath);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   error = rox_cao_strip_comments(cao_file);
   CHECK_ERROR_LOADCAO(error);

   //////////////////////////Read the header part if present//////////////////////////

   c = getc(cao_file);
   ungetc(c, cao_file);
   while (c == 'l' || c == 'L')
   {
      header = 1;
      stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file);
      CHECK_ERROR_FGETS(stdio_ptr_ret);

      if (strncmp(line, prefix, 4*sizeof(Rox_Char)) == 0)
      {
         //prepare sub filepath with current folder
         memcpy(sub_filepath, &path[0], path_length);
         sub_filepath[path_length] = '\0';

         //construct our sub cao filepath
         size_t firstIndex = strcspn(line, "\"");
         tmp_pch = strrchr(line, '\"');
         tmp_path_length = tmp_pch - line + 1;
         tmp_path_length = tmp_path_length - firstIndex - 2;
         memcpy(sub_filename, &line[firstIndex + 1], tmp_path_length);
         sub_filename[tmp_path_length] = '\0';
         if (tmp_path_length + path_length > FILEPATH_MAX_LENGTH-1) { error = ROX_ERROR_BAD_IOSTREAM; goto function_terminate; }
         strncat(sub_filepath, sub_filename, tmp_path_length);

         //test for a cyclic dependency
         for (int i = 0; i < *current_depth; ++i)
         {
            if (strncmp(sub_filepath, vector_of_model_filename[i], FILEPATH_MAX_LENGTH) == 0)
            {
               error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
            }
         }

         //everything is ok, load our sub cao file
         error = load_cao(caofile_edges, sub_filepath, start_id_face, vector_of_model_filename, current_depth);
         CHECK_ERROR_LOADCAO(error);
      }
      else
      {
         header = 0;
      }

      //ignore cao file comments
      error = rox_cao_strip_comments(cao_file);
      CHECK_ERROR_LOADCAO(error);

      c = getc(cao_file);
      ungetc(c, cao_file);
   }

   //////////////////////////Read the point declaration part//////////////////////////
   stdio_err_ret = fscanf(cao_file, "%u", &cao_nbr_points);
   CHECK_ERROR_FSCANF(stdio_err_ret);
   stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file); // skip the rest of the line
   CHECK_ERROR_FGETS(stdio_ptr_ret);

   rox_log("> %u points\n", cao_nbr_points);

   if (cao_nbr_points > MAX_DATA_NBR)
   {
      rox_log("Error, exceed the max number of points in the CAO model in %s\n", filepath);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   if (cao_nbr_points == 0 && !header)
   {
      rox_log("Error, no points are defined in %s\n", filepath);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }
   else if (cao_nbr_points > 0)
   {
      //allocate the coordinates of points
      cao_points = (Rox_Point3D_Double_Struct *) rox_memory_allocate(sizeof(Rox_Point3D_Double_Struct), cao_nbr_points);
      if (cao_points == NULL)
      {
         ROX_INFO_FORMATTED_DISPLAY("Error in allocation of %d objects, reading %s\n", cao_nbr_points, filepath);
         error = ROX_ERROR_NULL_POINTER; CHECK_ERROR_LOADCAO(error);
      }

      //parse the coordinates of points
      for (Rox_Uint k = 0; k < cao_nbr_points; k++)
      {
         //ignore cao file comments
         error = rox_cao_strip_comments(cao_file);
         CHECK_ERROR_LOADCAO(error);

         stdio_err_ret = fscanf(cao_file, "%lf", &x);
         CHECK_ERROR_FSCANF(stdio_err_ret);
         stdio_err_ret = fscanf(cao_file, "%lf", &y);
         CHECK_ERROR_FSCANF(stdio_err_ret);
         stdio_err_ret = fscanf(cao_file, "%lf", &z);
         CHECK_ERROR_FSCANF(stdio_err_ret);

         //FP TODO ? seems unused...
         //if (caoVersion == 2)
         //{
         //   stdio_err_ret = fscanf(cao_file, "%d", &i);
         //   CHECK_ERROR_FSCANF(stdio_err_ret);
         //   stdio_err_ret = fscanf(cao_file, "%d", &j);
         //   CHECK_ERROR_FSCANF(stdio_err_ret);
         //}

         stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file); // skip the rest of the line
         CHECK_ERROR_FGETS(stdio_ptr_ret);

         cao_points[k].X = x;
         cao_points[k].Y = y;
         cao_points[k].Z = z;
      }
   }

   //ignore cao file comments
   error = rox_cao_strip_comments(cao_file);
   CHECK_ERROR_LOADCAO(error);

   //////////////////////////Read the segment declaration part//////////////////////////
   stdio_err_ret = fscanf(cao_file, "%u", &cao_nbr_line);    //file_id >> cao_nbr_line;
   CHECK_ERROR_FSCANF(stdio_err_ret);
   stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file); // skip the rest of the line
   CHECK_ERROR_FGETS(stdio_ptr_ret);

   //FP TODO ?
   //nbLines += cao_nbr_line;
   rox_log("> %d lines\n", cao_nbr_line);

   if (cao_nbr_line > MAX_DATA_NBR)
   {
      rox_log("Error, exceed the max number of lines in the CAO model in %s\n", filepath);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   if (cao_nbr_line > 0)
   {
      cao_line_points = (Rox_Uint *) rox_memory_allocate(sizeof(Rox_Uint), 2 * cao_nbr_line);
      if (cao_line_points == NULL)
      {
         ROX_INFO_FORMATTED_DISPLAY("Error in allocation of %d objects, reading %s\n", 2 * cao_nbr_line, filepath);
         error = ROX_ERROR_NULL_POINTER; CHECK_ERROR_LOADCAO(error);
      }
   }

   //Store in a map the potential segments to add, allocation will be deleted
   error = rox_dynvec_uint_new(&segments_points_indices, 2 + 2 * cao_nbr_line);
   CHECK_ERROR_LOADCAO(error);
   error = rox_dynvec_point3d_double_new(&segments_points, 2 + 2 * cao_nbr_line);
   CHECK_ERROR_LOADCAO(error);

   //Initialization of id_face with start_id_face for dealing with recursive load in header
   id_face = *start_id_face;
   for (Rox_Uint k = 0; k < cao_nbr_line; k++)
   {
      //ignore cao file comments
      error = rox_cao_strip_comments(cao_file);
      CHECK_ERROR_LOADCAO(error);

      stdio_err_ret = fscanf(cao_file, "%u", &index1);    //file_id >> index1;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      stdio_err_ret = fscanf(cao_file, "%u", &index2);    //file_id >> index2;
      CHECK_ERROR_FSCANF(stdio_err_ret);

      //////////////////////////Read the parameter value if present//////////////////////////
      error = rox_cao_strip_parameters(cao_file);
      CHECK_ERROR_LOADCAO(error);
      //Get the end of the line
      //Rox_Char buffer[256];
      //file_id.getline(buffer, 256);
      //std::string end_line(buffer);
      //std::map<std::string, std::string> map_of_params = parseParameters(end_line);
      //std::string segment_name = "";
      ////FP TODO ?
      //Rox_Double minLineLengthThresh = 50.0;// !applyLodSettingInConfig ? minLineLengthThresholdGeneral : 50.0;
      ////FP TODO ?
      //bool useLod = false;// !applyLodSettingInConfig ? useLodGeneral : false;
      //if (map_of_params.find("name") != map_of_params.end())
      //{
      //   segment_name = map_of_params["name"];
      //}
      //if (map_of_params.find("minLineLengthThreshold") != map_of_params.end())
      //{
      //   minLineLengthThresh = std::atof(map_of_params["minLineLengthThreshold"].c_str());
      //}
      //if (map_of_params.find("useLod") != map_of_params.end())
      //{
      //   useLod = parse_boolean(map_of_params["useLod"]);
      //}

      //FP TODO ?
      //segmentInfo.name = segment_name;
      //FP TODO ?
      //segmentInfo.useLod = useLod;
      //FP TODO ?
      //segmentInfo.minLineLengthThresh = minLineLengthThresh;

      cao_line_points[2 * k] = index1;
      cao_line_points[2 * k + 1] = index2;

      if (index1 < cao_nbr_points && index2 < cao_nbr_points)
      {
         //store both indices next to each other in segments_points_indices
         error = rox_dynvec_uint_append(segments_points_indices, &index1);
         CHECK_ERROR_LOADCAO(error);
         error = rox_dynvec_uint_append(segments_points_indices, &index2);
         CHECK_ERROR_LOADCAO(error);

         //store both points next to each other in segments_points
         error = rox_dynvec_point3d_double_append(segments_points, &cao_points[index1]);
         CHECK_ERROR_LOADCAO(error);
         error = rox_dynvec_point3d_double_append(segments_points, &cao_points[index2]);
         CHECK_ERROR_LOADCAO(error);
      }
      else
      {
         rox_log("Error, line %d has wrong coordinates in %s\n", k, filepath);
         error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
      }
   }

   //ignore cao file comments
   error = rox_cao_strip_comments(cao_file);
   CHECK_ERROR_LOADCAO(error);

   //////////////////////////Read the face segment declaration part//////////////////////////
   //Load polygon from the lines extracted earlier (the first point of the line is used)
   stdio_err_ret = fscanf(cao_file, "%u", &cao_nbr_polygon_line);    //file_id >> cao_nbr_polygon_line;
   CHECK_ERROR_FSCANF(stdio_err_ret);
   stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file); // skip the rest of the line
   CHECK_ERROR_FGETS(stdio_ptr_ret);

   //FP TODO ?
   //nbPolygonLines += cao_nbr_polygon_line;
   rox_log("> %u polygon lines\n", cao_nbr_polygon_line);

   if (cao_nbr_polygon_line > MAX_DATA_NBR)
   {
      rox_log("Error, exceed the max number of polygon lines in %s\n", filepath);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   //Store in a vector the indexes of the segments added in the face segment case
   error = rox_dynvec_uint_new(&face_segment_key_vector, 100);
   CHECK_ERROR_LOADCAO(error);

   for (Rox_Uint k = 0; k < cao_nbr_polygon_line; k++)
   {
      nb_line_pol = 0;

      //ignore cao file comments
      error = rox_cao_strip_comments(cao_file);
      CHECK_ERROR_LOADCAO(error);

      //read number of lines in polygon
      stdio_err_ret = fscanf(cao_file, "%u", &nb_line_pol);
      CHECK_ERROR_FSCANF(stdio_err_ret);
      if (nb_line_pol > MAX_DATA_NBR)
      {
         rox_log("Error, exceed the max number of lines in %s\n", filepath);
         error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
      }

      //allocated our container of points 3d coordinates
      error = rox_dynvec_point3d_double_new(&corners, 2 + 2 * nb_line_pol);
      CHECK_ERROR_LOADCAO(error);

      //loop through lines of poly and append coordinates along indices
      for (Rox_Uint n = 0; n < nb_line_pol; n++)
      {
         stdio_err_ret = fscanf(cao_file, "%u", &index);
         CHECK_ERROR_FSCANF(stdio_err_ret);
         if (index >= cao_nbr_line)
         {
            rox_log("Error, exceed the max number of lines in %s\n", filepath);
            error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
         }

         //append faces corners 3d points coordinates next to each other in corners
         error = rox_dynvec_point3d_double_append(corners, &cao_points[cao_line_points[2 * index]]);
         CHECK_ERROR_LOADCAO(error);
         error = rox_dynvec_point3d_double_append(corners, &cao_points[cao_line_points[2 * index + 1]]);
         CHECK_ERROR_LOADCAO(error);

         //append faces corners 3d points indices next to each other in face_segment_key_vector
         error = rox_dynvec_uint_append(face_segment_key_vector, &cao_line_points[2 * index]);
         CHECK_ERROR_LOADCAO(error);
         error = rox_dynvec_uint_append(face_segment_key_vector, &cao_line_points[2 * index + 1]);
         CHECK_ERROR_LOADCAO(error);
      }

      //////////////////////////Read the parameter value if present//////////////////////////
      error = rox_cao_strip_parameters(cao_file);
      CHECK_ERROR_LOADCAO(error);
      //Get the end of the line
      //Rox_Char buffer[256];
      //file_id.getline(buffer, 256);
      //std::string end_line(buffer);
      //std::map<std::string, std::string> map_of_params = parseParameters(end_line);
      //std::string polygon_name = "";
      ////FP TODO ?
      //bool useLod = false;// !applyLodSettingInConfig ? useLodGeneral : false;
      ////FP TODO ?
      //Rox_Double minPolygonAreaThreshold = 2500.0;// !applyLodSettingInConfig ? minPolygonAreaThresholdGeneral : 2500.0;
      //if (map_of_params.find("name") != map_of_params.end())
      //{
      //   polygon_name = map_of_params["name"];
      //}
      //if (map_of_params.find("minPolygonAreaThreshold") != map_of_params.end())
      //{
      //   minPolygonAreaThreshold = std::atof(map_of_params["minPolygonAreaThreshold"].c_str());
      //}
      //if (map_of_params.find("useLod") != map_of_params.end())
      //{
      //   useLod = parse_boolean(map_of_params["useLod"]);
      //}

      //FP TODO ? parameters...
      error = add_polygon_segments(caofile_edges, corners);//  , id_face++, polygon_name, useLod, minPolygonAreaThreshold, minLineLengthThresholdGeneral);
      CHECK_ERROR_LOADCAO(error);

      //FP TODO ?
      //initFaceFromLines(*(faces.getPolygon().back())); // Init from the last polygon that was added

      error = rox_dynvec_point3d_double_del(&corners);
      CHECK_ERROR_LOADCAO(error);
   }

   //Add the segments which were not already added in the face segment case
   error = rox_dynvec_uint_get_used(&segments_points_indices_nbr, segments_points_indices);
   CHECK_ERROR_LOADCAO(error);

   error = rox_dynvec_uint_get_used(&face_segment_key_vector_nbr, face_segment_key_vector);
   CHECK_ERROR_LOADCAO(error);

   error = rox_dynvec_uint_get_data_pointer(&segments_points_indices_data, segments_points_indices);
   CHECK_ERROR_LOADCAO(error);

   error = rox_dynvec_uint_get_data_pointer(&face_segment_key_vector_data, face_segment_key_vector);
   CHECK_ERROR_LOADCAO(error);

   for (Rox_Uint each_index = 0; each_index < segments_points_indices_nbr; each_index += 2)
   {
      Rox_Bool skip = 0;
      //TODO ? this guy seems to have a ARM optimized search for c arrays http://stackoverflow.com/questions/25661925/quickly-find-whether-a-value-is-present-in-a-c-array
      for (Rox_Uint each_key = 0; each_key < face_segment_key_vector_nbr; each_key += 2)
      {
         if (segments_points_indices_data[each_index] == face_segment_key_vector_data[each_key])
         {
            skip = 1;
            break;
         }
      }
      if (!skip)
      {
         //Create a simple segment with two points
         Rox_DynVec_Point3D_Double simple_segment = NULL;
         error = rox_dynvec_point3d_double_new(&simple_segment, 3);
         CHECK_ERROR_LOADCAO(error);
         //store both points next to each other in segments_points
         error = rox_dynvec_point3d_double_append(simple_segment, &cao_points[segments_points_indices_data[each_index]]);
         CHECK_ERROR_LOADCAO(error);
         error = rox_dynvec_point3d_double_append(simple_segment, &cao_points[segments_points_indices_data[each_index+1]]);
         CHECK_ERROR_LOADCAO(error);


         //FP TODO ? parameters...
         error = add_polygon_segments(caofile_edges, simple_segment);// , id_face++, it->second.name, it->second.useLod, minPolygonAreaThresholdGeneral, it->second.minLineLengthThresh);
         CHECK_ERROR_LOADCAO(error);

         error = rox_dynvec_point3d_double_del(&simple_segment);
         CHECK_ERROR_LOADCAO(error);


         //FP TODO ?
         //initFaceFromCorners(*(faces.getPolygon().back())); // Init from the last polygon that was added
      }
   }

   //ignore cao file comments
   error = rox_cao_strip_comments(cao_file);
   CHECK_ERROR_LOADCAO(error);

   //////////////////////////Skip the faces from points part//////////////////////////
   stdio_err_ret = fscanf(cao_file, "%u", &caoNbFacesFromPoints);    //file_id >> caoNbFacesFromPoints;
   CHECK_ERROR_FSCANF(stdio_err_ret);
   rox_log("> %u faces from points\n", caoNbFacesFromPoints);
   stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file); // skip the rest of the line
   CHECK_ERROR_FGETS(stdio_ptr_ret);
   for (size_t i = 0; i < caoNbFacesFromPoints; i++)
   {
      error = rox_cao_strip_comments(cao_file);
      CHECK_ERROR_LOADCAO(error);
      stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file); // skip the rest of the line
      CHECK_ERROR_FGETS(stdio_ptr_ret);
      rox_log("WARNING faces from points are ignored for now, skipping line: %s\n", line);
   }

   //ignore cao file comments
   error = rox_cao_strip_comments(cao_file);
   CHECK_ERROR_LOADCAO(error);

   //////////////////////////Read the Cylinders declaration part//////////////////////////
   stdio_err_ret = fscanf(cao_file, "%u", &caoNbCylinders);    //file_id >> caoNbCylinders;
   CHECK_ERROR_FSCANF(stdio_err_ret);
   stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file); // skip the rest of the line
   CHECK_ERROR_FGETS(stdio_ptr_ret);
   //for (size_t i = 0; i < caoNbCylinders; i++)
   //{
   //   stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file); // skip the rest of the line
   //   CHECK_ERROR_FGETS(stdio_ptr_ret);
   //   rox_log("WARNING cylinders are ignored for now, skipping line: %s\n", line);
   //}

   rox_log("> %u cylinders\n", caoNbCylinders);
   if (caoNbCylinders > MAX_DATA_NBR)
   {
      rox_log("Error, exceed the max number of cylinders in %s\n", filepath);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   for (Rox_Uint k = 0; k < caoNbCylinders; ++k)
   {
      double radius=0;
      Rox_Uint indexP1=0, indexP2=0;

      //ignore cao file comments
      error = rox_cao_strip_comments(cao_file);
      CHECK_ERROR_LOADCAO(error);

      stdio_err_ret = fscanf(cao_file, "%u", &indexP1);    //file_id >> indexP1;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      stdio_err_ret = fscanf(cao_file, "%u", &indexP2);    //file_id >> indexP2;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      stdio_err_ret = fscanf(cao_file, "%lf", &radius);    //file_id >> radius;
      CHECK_ERROR_FSCANF(stdio_err_ret);

      //////////////////////////Read the parameter value if present//////////////////////////
      error = rox_cao_strip_parameters(cao_file);
      CHECK_ERROR_LOADCAO(error);
      //Get the end of the line
      //char buffer[256];
      //fileId.getline(buffer, 256);
      //std::string endLine(buffer);
      //std::map<std::string, std::string> mapOfParams = parseParameters(endLine);

      ////              fileId.ignore(256, '\n'); // skip the rest of the line

      //std::string polygonName = "";
      //bool useLod = !applyLodSettingInConfig ? useLodGeneral : false;
      //double minLineLengthThreshold = !applyLodSettingInConfig ? minLineLengthThresholdGeneral : 50.0;
      //if (mapOfParams.find("name") != mapOfParams.end()) {
      //   polygonName = mapOfParams["name"];
      //}
      //if (mapOfParams.find("minLineLengthThreshold") != mapOfParams.end()) {
      //   minLineLengthThreshold = std::atof(mapOfParams["minLineLengthThreshold"].c_str());
      //}
      //if (mapOfParams.find("useLod") != mapOfParams.end()) {
      //   useLod = parseBoolean(mapOfParams["useLod"]);
      //}

      //int idRevolutionAxis = idFace;
      //addPolygon(caoPoints[indexP1], caoPoints[indexP2], idFace++, polygonName, useLod, minLineLengthThreshold);

      //std::vector<std::vector<vpPoint> > listFaces;
      //createCylinderBBox(caoPoints[indexP1], caoPoints[indexP2], radius, listFaces);
      //addPolygon(listFaces, idFace, polygonName, useLod, minLineLengthThreshold);
      //idFace += 4;

      error = add_cylinder(caofile_edges, cao_points[indexP1], cao_points[indexP2], radius);// , idRevolutionAxis, polygonName);
      CHECK_ERROR_LOADCAO(error);
   }

   //ignore cao file comments
   error = rox_cao_strip_comments(cao_file);
   CHECK_ERROR_LOADCAO(error);

   //////////////////////////Read the Circles declaration part//////////////////////////
   //TODO ?
   //if (fileId.eof())
   //{
   //   // check if not at the end of the file (for old style files)
   //   delete[] caoPoints;
   //   delete[] caoLinePoints;
   //   return;
   //}

   stdio_err_ret = fscanf(cao_file, "%u", &caoNbCircle);    //file_id >> caoNbCircle;
   CHECK_ERROR_FSCANF(stdio_err_ret);
   stdio_ptr_ret = fgets(line, LINE_MAX_LENGTH, cao_file); // skip the rest of the line
   CHECK_ERROR_FGETS(stdio_ptr_ret);

   rox_log("> %u circles\n", caoNbCircle);

   if (caoNbCircle > MAX_DATA_NBR)
   {
      rox_log("Error, exceed the max number of circles in %s\n", filepath);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   //nbCircles += caoNbCircle;
   for (Rox_Uint k = 0; k < caoNbCircle; ++k)
   {
      Rox_Double radius=0;
      Rox_Uint indexP1=0, indexP2=0, indexP3=0;

      error = rox_cao_strip_comments(cao_file);
      CHECK_ERROR_LOADCAO(error);

      stdio_err_ret = fscanf(cao_file, "%lf", &radius);    //file_id >> radius;
      CHECK_ERROR_FSCANF(stdio_err_ret);

      stdio_err_ret = fscanf(cao_file, "%u", &indexP1);    //file_id >> indexP1;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      stdio_err_ret = fscanf(cao_file, "%u", &indexP2);    //file_id >> indexP2;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      stdio_err_ret = fscanf(cao_file, "%u", &indexP3);    //file_id >> indexP3;
      CHECK_ERROR_FSCANF(stdio_err_ret);


      //////////////////////////Read the parameter value if present//////////////////////////
      error = rox_cao_strip_parameters(cao_file);
      CHECK_ERROR_LOADCAO(error);

      //Get the end of the line
      //Rox_Char buffer[256];
      //fileId.getline(buffer, 256);
      //std::string endLine(buffer);
      //std::map<std::string, std::string> mapOfParams = parseParameters(endLine);

      ////              fileId.ignore(256, '\n'); // skip the rest of the line

      //std::string polygonName = "";
      //Rox_Bool useLod = !applyLodSettingInConfig ? useLodGeneral : false;
      //Rox_Double minPolygonAreaThreshold = !applyLodSettingInConfig ? minPolygonAreaThresholdGeneral : 2500.0;
      //if (mapOfParams.find("name") != mapOfParams.end()) {
      //   polygonName = mapOfParams["name"];
      //}
      //if (mapOfParams.find("minPolygonAreaThreshold") != mapOfParams.end()) {
      //   minPolygonAreaThreshold = std::atof(mapOfParams["minPolygonAreaThreshold"].c_str());
      //}
      //if (mapOfParams.find("useLod") != mapOfParams.end()) {
      //   useLod = parseBoolean(mapOfParams["useLod"]);
      //}

      add_circle(caofile_edges, cao_points[indexP1], cao_points[indexP2], cao_points[indexP3], radius);// , idFace, polygonName, useLod, minPolygonAreaThreshold);

      //initCircle(caoPoints[indexP1], caoPoints[indexP2], caoPoints[indexP3], radius, idFace++, polygonName);
   }



   stdio_err_ret = fclose(cao_file);
   if (stdio_err_ret != 0)
   {
      error = ROX_ERROR_BAD_IOSTREAM;
      CHECK_ERROR_LOADCAO(error);
   }

   //set for next iteration
   *start_id_face = id_face;

   //cleanup
   if (cao_points)rox_memory_delete(cao_points);
   if (cao_line_points)rox_memory_delete(cao_line_points);
   if (corners)rox_dynvec_point3d_double_del(&corners);
   if (segments_points)rox_dynvec_point3d_double_del(&segments_points);
   if (segments_points_indices)rox_dynvec_uint_del(&segments_points_indices);
   if (face_segment_key_vector)rox_dynvec_uint_del(&face_segment_key_vector);

function_terminate:
   return error;
}

Rox_ErrorCode rox_caofile_edges_load_cao(Rox_CaoFile_Edges caofile_edges, const Rox_Char * filepath)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   //messy way to handle the recursivity check
   Rox_Char** vector_of_model_filename = NULL;
   Rox_Sint vector_of_model_filename_depth = 0;
   Rox_Sint start_id_face = 0, each_filename = 0;
   // Rox_Uint nbr_polys = 0;
   if (!filepath || !caofile_edges)  { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   vector_of_model_filename = (Rox_Char **) rox_memory_allocate(sizeof(Rox_Char*), NB_FILES_RECURSIVE_MAX);

   error = load_cao(caofile_edges, filepath, &start_id_face, vector_of_model_filename, &vector_of_model_filename_depth);
   if (error)goto function_terminate;

   //Dbg
   //error = rox_objset_dynvec_point3d_double_get_used(&nbr_polys, caofile_edges->poly_segments);
   //if (error)goto function_terminate;

function_terminate:
   //clean up the mess
   if (vector_of_model_filename)
   {
      for (each_filename = 0; each_filename < vector_of_model_filename_depth; ++each_filename)
      {
         rox_memory_delete(vector_of_model_filename[each_filename]);
      }
      rox_memory_delete(vector_of_model_filename);
   }

   return error;
}

Rox_ErrorCode rox_caofile_edges_estimate_segments(Rox_CaoFile_Edges caofile_edges, Rox_Array2D_Double pose
   , Rox_Double fu, Rox_Double fv, Rox_Double cu, Rox_Double cv
   , Rox_Uint minimal_segment_size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!caofile_edges || !pose)
   {
      error = ROX_ERROR_NULL_POINTER; goto function_terminate;
   }

   Rox_Uint nbr_polys = 0, each_poly = 0, nbr_segments = 0;
   Rox_DynVec_Point3D_Double* poly_segments_data = NULL;

   // Reset visible segments
   error = rox_dynvec_point3d_double_reset(caofile_edges->visible_segments);
   if (error) goto function_terminate;

   // get cao file read data
   error = rox_objset_dynvec_point3d_double_get_used(&nbr_polys, caofile_edges->poly_segments);
   if (error) goto function_terminate;

   error  = rox_objset_dynvec_point3d_double_get_data_pointer ( &poly_segments_data, caofile_edges->poly_segments);
   if (error) goto function_terminate;

   // go through all poly_lines and compute visibility from this pose/intrinsics parameters
   for (each_poly = 0; each_poly < nbr_polys; ++each_poly)
   {
      Rox_Uint nbr_points = 0, each_point = 0;
      error = rox_dynvec_point3d_double_get_used(&nbr_points, poly_segments_data[each_poly]);
      if (error) goto function_terminate;

      Rox_Point3D_Double_Struct * points_data = NULL;
      error = rox_dynvec_point3d_double_get_data_pointer(&points_data, poly_segments_data[each_poly]);
      if (error) goto function_terminate;

      //polylines with only two points are always visible
      if (nbr_points == 2)
      {
         error = rox_dynvec_point3d_double_append(caofile_edges->visible_segments, &points_data[0]);
         if (error) goto function_terminate;

         error = rox_dynvec_point3d_double_append(caofile_edges->visible_segments, &points_data[1]);
         if (error) goto function_terminate;
      }
      else //test for polylines made of multiple segments
      {
         Rox_Point3D_Double_Struct faceNormal = { 0.0, 0.0, 0.0 };
         Rox_Point3D_Double_Struct currentVertex = { 0.0, 0.0, 0.0 }, nextVertex = { 0.0, 0.0, 0.0 };
         Rox_Point3D_Double_Struct sum = { 0.0, 0.0, 0.0 };
         Rox_Point3D_Double_Struct e4 = { 0.0, 0.0, 0.0 };
         Rox_Double length = 0.0, dot = 0.0, angle = 0.0;

         //transform each point in camera frame
         Rox_DynVec_Point3D_Double points_transformed = NULL;
         Rox_Point3D_Double_Struct* points_transformed_data = NULL;

         error = rox_dynvec_point3d_double_new(&points_transformed, nbr_points);
         if (error) { rox_dynvec_point3d_double_del(&points_transformed);  goto function_terminate; }

         error = rox_dynvec_point3d_double_get_data_pointer(&points_transformed_data, points_transformed);

         error = rox_point3d_double_transform(points_transformed_data, pose, points_data, nbr_points);
         if (error) { rox_dynvec_point3d_double_del(&points_transformed);  goto function_terminate; }

         //Check visibility from normal
         //Newell's Method for calculating the normal of an arbitrary 3D polygon
         //https://www.opengl.org/wiki/Calculating_a_Surface_Normal
         for (each_point = 0; each_point < nbr_points; ++each_point)
         {
            // Sum it
            sum.X = sum.X + points_transformed_data[each_point].X;
            sum.Y = sum.Y + points_transformed_data[each_point].Y;
            sum.Z = sum.Z + points_transformed_data[each_point].Z;

            // Don't process last
            if (each_point == nbr_points - 1)
               continue;

            //process face normal
            currentVertex = points_transformed_data[each_point];
            nextVertex = points_transformed_data[each_point + 1];

            faceNormal.X += (currentVertex.Y - nextVertex.Y) * (currentVertex.Z + nextVertex.Z);
            faceNormal.Y += (currentVertex.Z - nextVertex.Z) * (currentVertex.X + nextVertex.X);
            faceNormal.Z += (currentVertex.X - nextVertex.X) * (currentVertex.Y + nextVertex.Y);
         }

         //not needed anymore
         rox_dynvec_point3d_double_del(&points_transformed);

         //normalize faceNormal
         length = faceNormal.X*faceNormal.X + faceNormal.Y*faceNormal.Y + faceNormal.Z*faceNormal.Z;

         if (length > 0)
         {
            length = sqrt(length);
            faceNormal.X = faceNormal.X / length;
            faceNormal.Y = faceNormal.Y / length;
            faceNormal.Z = faceNormal.Z / length;
         }
         else
         {
            error = ROX_ERROR_INVALID_VALUE; goto function_terminate;
         }

         if (nbr_points == 0)
         {
            error = ROX_ERROR_BAD_SIZE; goto function_terminate;
         }

         e4.X = -sum.X / nbr_points;
         e4.Y = -sum.Y / nbr_points;
         e4.Z = -sum.Z / nbr_points;

         //normalize e4
         length = e4.X*e4.X + e4.Y*e4.Y + e4.Z*e4.Z;
         if (length > 0)
         {
            length = sqrt(length);
            e4.X = e4.X / length;
            e4.Y = e4.Y / length;
            e4.Z = e4.Z / length;
         }
         else
         {
            error = ROX_ERROR_INVALID_VALUE; goto function_terminate;
         }

         dot = e4.X*faceNormal.X + e4.Y*faceNormal.Y + e4.Z*faceNormal.Z;
         angle = acos(dot);

         if (angle < ROX_PI / 2)
         {
            //std::vector<Segment3D<Rox_Double> > poly_seg;
            //err = it->getSegments(poly_seg);
            //if (err) return err;

            ////Add to results
            //_visibility_results.push_back(std::make_pair(_pose, poly_seg));

            for (each_point = 0; each_point < nbr_points; ++each_point)
            {
               //don't process last
               if (each_point == nbr_points - 1)
                  continue;

               error = rox_dynvec_point3d_double_append(caofile_edges->visible_segments, &points_data[each_point]);
               if (error)goto function_terminate;
               error = rox_dynvec_point3d_double_append(caofile_edges->visible_segments, &points_data[each_point + 1]);
               if (error)goto function_terminate;
            }
         }

      }

   }

   // Dbg
   error = rox_dynvec_point3d_double_get_used(&nbr_segments, caofile_edges->visible_segments);
   if (error) goto function_terminate;

function_terminate:
   return error;
}

Rox_ErrorCode rox_caofile_edges_estimate_all_segments(Rox_CaoFile_Edges caofile_edges)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!caofile_edges)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   Rox_Uint nbr_polys = 0, each_poly = 0, nbr_segments = 0;
   Rox_DynVec_Point3D_Double* poly_segments_data = NULL;

   // Reset visible segments
   error = rox_dynvec_point3d_double_reset(caofile_edges->visible_segments);
   if (error) goto function_terminate;

   // get cao file read data
   error = rox_objset_dynvec_point3d_double_get_used(&nbr_polys, caofile_edges->poly_segments);
   if (error) goto function_terminate;

   error = rox_objset_dynvec_point3d_double_get_data_pointer ( &poly_segments_data, caofile_edges->poly_segments);
   if (error) goto function_terminate;

   // go through all poly_lines and compute visibility from this pose/intrinsics parameters
   for (each_poly = 0; each_poly < nbr_polys; ++each_poly)
   {
      Rox_Uint nbr_points = 0, each_point = 0;
      error = rox_dynvec_point3d_double_get_used(&nbr_points, poly_segments_data[each_poly]);
      if (error) goto function_terminate;

      Rox_Point3D_Double_Struct* points_data = NULL;
      error = rox_dynvec_point3d_double_get_data_pointer(&points_data, poly_segments_data[each_poly]);
      if (error) goto function_terminate;

      //polylines with only two points are always visible
      if (nbr_points == 2)
      {
         error = rox_dynvec_point3d_double_append(caofile_edges->visible_segments, &points_data[0]);
         if (error) goto function_terminate;
         error = rox_dynvec_point3d_double_append(caofile_edges->visible_segments, &points_data[1]);
         if (error) goto function_terminate;
      }
      else //test for polylines made of multiple segments
      {
         for (each_point = 0; each_point < nbr_points; ++each_point)
         {
            //don't process last
            if (each_point == nbr_points - 1)
               continue;

            error = rox_dynvec_point3d_double_append(caofile_edges->visible_segments, &points_data[each_point]);
            if (error) goto function_terminate;
            error = rox_dynvec_point3d_double_append(caofile_edges->visible_segments, &points_data[each_point + 1]);
            if (error) goto function_terminate;
         }
      }
   }

   // Dbg
   error = rox_dynvec_point3d_double_get_used(&nbr_segments, caofile_edges->visible_segments);
   if (error) goto function_terminate;

function_terminate:
   return error;
}

Rox_ErrorCode rox_caofile_edges_estimate_visible_ellipses(Rox_CaoFile_Edges caofile_edges, Rox_Array2D_Double pose, Rox_Double fu, Rox_Double fv, Rox_Double cu, Rox_Double cv, Rox_Uint minimal_segment_size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!caofile_edges || !pose)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   Rox_Uint nbr_ellipses = 0;

   // Reset visible ellipses
   error = rox_objset_ellipse3d_reset(caofile_edges->visible_ellipses);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_ellipse3d_get_used(&nbr_ellipses, caofile_edges->ellipses);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Ellipse3D * ellipses = NULL;
   error = rox_objset_ellipse3d_get_data_pointer(&ellipses, caofile_edges->ellipses);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get cao file read data (without visibility test at the moment)
   for(Rox_Uint i = 0; i<nbr_ellipses; i++)
   {
      Rox_Ellipse3D ellipse3d_toadd = NULL;
      error = rox_ellipse3d_new(&ellipse3d_toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_ellipse3d_copy(ellipse3d_toadd, ellipses[i]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_objset_ellipse3d_append(caofile_edges->visible_ellipses, ellipse3d_toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Dbg
   error = rox_objset_ellipse3d_get_used(&nbr_ellipses, caofile_edges->visible_ellipses);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_caofile_edges_estimate_visible_cylinders(Rox_CaoFile_Edges caofile_edges, Rox_Array2D_Double pose, Rox_Double fu, Rox_Double fv, Rox_Double cu, Rox_Double cv, Rox_Uint minimal_segment_size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!caofile_edges || !pose)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   // Reset visible cylinders
   error = rox_objset_cylinder3d_reset(caofile_edges->visible_cylinders);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint nbr_cylinders = 0;
   error = rox_objset_cylinder3d_get_used(&nbr_cylinders, caofile_edges->cylinders);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Cylinder3D * cylinders = NULL;
   error = rox_objset_cylinder3d_get_data_pointer (&cylinders, caofile_edges->cylinders);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get cao file read data (without visibility test at the moment)
   for(Rox_Uint i = 0; i<nbr_cylinders; i++)
   {
      Rox_Cylinder3D cylinder3d_toadd = NULL;
      error = rox_cylinder3d_new(&cylinder3d_toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_cylinder3d_copy(cylinder3d_toadd, cylinders[i]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_objset_cylinder3d_append(caofile_edges->visible_cylinders, cylinder3d_toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Dbg
   error = rox_objset_cylinder3d_get_used(&nbr_cylinders, caofile_edges->visible_cylinders);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_caofile_edges_get_contours (
   Rox_Segment3D * contours,
   Rox_Uint * csize,
   Rox_CaoFile_Edges caofile_edges
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!contours || !csize || !caofile_edges)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   // Transform poly_points(caofile_edges->visible_segments variables) in segments (contours variable)
   error = rox_dynvec_point3d_double_get_used ( csize, caofile_edges->visible_segments );
   if (error) goto function_terminate;

   if (*csize > 0)
   {
      *csize /= 2;
      // Oh yes, check now or never !
      if ( sizeof(Rox_Segment3D_Struct) == 2 * sizeof(Rox_Point3D_Double_Struct) )
      {
         error = rox_dynvec_point3d_double_get_data_pointer ( (Rox_Point3D_Double *) contours, caofile_edges->visible_segments );
         if (error) goto function_terminate;
      }
      else
      {
         // well, this is ackward, a segment is no longer of the same size as two points, what should we do ?
         ROX_INFO_FORMATTED("Error, Rox_Segment3D_Struct is not the same size as 2 Rox_Point3D_Double_Struct. %s", "");
         error = ROX_ERROR_BAD_SIZE; goto function_terminate;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_caofile_edges_get_visible_ellipses(Rox_Ellipse3D ** ellipses, Rox_Uint * size, Rox_CaoFile_Edges caofile_edges)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //if (!ellipses || !size || !caofile_edges)
   //{ error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   error = rox_objset_ellipse3d_get_data_pointer(ellipses, caofile_edges->visible_ellipses);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_ellipse3d_get_used(size, caofile_edges->visible_ellipses);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_caofile_edges_load_cao_from_string(Rox_CaoFile_Edges obj, const Rox_Char * inline_model_string)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bool header = 0;
   Rox_Sint caoVersion = 0;
   Rox_Char c = 0;
   // size_t tmp_path_length = 0;
   // Rox_Char * tmp_pch = NULL;
   // const Rox_Char* prefix = "load";
   Rox_Uint cao_nbr_points = 0;
   Rox_Point3D_Double_Struct* cao_points = NULL;
   Rox_Double x, y, z; // 3D coordinates
                       //int i, j;    // image coordinate (used for matching)
   Rox_DynVec_Point3D_Double segments_points = NULL;
   // Rox_Point3D_Double_Struct* segments_points_data = NULL;
   Rox_DynVec_Uint face_segment_key_vector = NULL;
   Rox_Uint face_segment_key_vector_nbr = 0;
   Rox_Uint* face_segment_key_vector_data = NULL;
   Rox_DynVec_Uint segments_points_indices = NULL;
   Rox_Uint segments_points_indices_nbr = 0;
   Rox_Uint* segments_points_indices_data = NULL;
   Rox_Uint cao_nbr_line = 0;
   Rox_Uint *cao_line_points = NULL;
   Rox_Uint index1 = 0, index2 = 0;
   Rox_Uint cao_nbr_polygon_line = 0;
   Rox_Uint index = 0;
   Rox_Uint nb_line_pol = 0;
   Rox_DynVec_Point3D_Double corners = NULL;
   Rox_Sint    stdio_err_ret = 0;
   // Rox_Char*   stdio_ptr_ret = NULL;
   Rox_Uint caoNbCircle = 0;
   Rox_Uint caoNbFacesFromPoints = 0;
   Rox_Uint caoNbCylinders = 0;

   if (!obj || !inline_model_string)
   {
      error = ROX_ERROR_NULL_POINTER; CHECK_ERROR_LOADCAO(error);
   }


   const Rox_Char * cur = inline_model_string;
   //////////////////////////Read CAO Version (V1, V2, ...)//////////////////////////

   c = *cur++;
   if (c == 'V')
   {
      stdio_err_ret = sscanf(cur, "%d", &caoVersion);
      CHECK_ERROR_FSCANF(stdio_err_ret);
      STRIP_LINE();
   }
   else
   {
      ROX_INFO_FORMATTED_DISPLAY("Error in vpMbTracker::loadCAOModel() -> Bad parameter header file : use V0, V1, ... in  %s", inline_model_string);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }
   STRIP_COMMENTS();

   //////////////////////////Read the point declaration part//////////////////////////
   stdio_err_ret = sscanf(cur, "%u", &cao_nbr_points);
   CHECK_ERROR_FSCANF(stdio_err_ret);
   STRIP_LINE();

   ROX_INFO_FORMATTED_DISPLAY("> %u points\n", cao_nbr_points);

   if (cao_nbr_points > MAX_DATA_NBR)
   {
      ROX_INFO_FORMATTED_DISPLAY("Error, exceed the max number of points in the CAO model in %s\n", inline_model_string);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   if (cao_nbr_points == 0 && !header)
   {
      ROX_INFO_FORMATTED_DISPLAY("Error, no points are defined in %s\n", inline_model_string);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   //allocate the coordinates of points
   cao_points = (Rox_Point3D_Double_Struct *) rox_memory_allocate(sizeof(Rox_Point3D_Double_Struct), cao_nbr_points);
   if (cao_points == NULL)
   {
      ROX_INFO_FORMATTED_DISPLAY("Error in allocation, reading %s\n", inline_model_string);
      error = ROX_ERROR_NULL_POINTER; CHECK_ERROR_LOADCAO(error);
   }

   //parse the coordinates of points
   for (Rox_Uint k = 0; k < cao_nbr_points; k++)
   {
      STRIP_COMMENTS();
      stdio_err_ret = sscanf(cur, "%lf", &x);
      CHECK_ERROR_FSCANF(stdio_err_ret);
      STRIP_TO_SPACE();
      STRIP_SPACES();

      stdio_err_ret = sscanf(cur, "%lf", &y);
      CHECK_ERROR_FSCANF(stdio_err_ret);
      STRIP_TO_SPACE();
      STRIP_SPACES();

      stdio_err_ret = sscanf(cur, "%lf", &z);
      CHECK_ERROR_FSCANF(stdio_err_ret);

      STRIP_LINE();

      cao_points[k].X = x;
      cao_points[k].Y = y;
      cao_points[k].Z = z;
   }

   STRIP_COMMENTS();

   //////////////////////////Read the segment declaration part//////////////////////////
   stdio_err_ret = sscanf(cur, "%u", &cao_nbr_line);    //file_id >> cao_nbr_line;
   CHECK_ERROR_FSCANF(stdio_err_ret);
   STRIP_LINE();

   ROX_INFO_FORMATTED_DISPLAY("> %d lines\n", cao_nbr_line);

   if (cao_nbr_line > MAX_DATA_NBR)
   {
      ROX_INFO_FORMATTED_DISPLAY("Error, exceed the max number of lines in the CAO model in %s\n", inline_model_string);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   if (cao_nbr_line > 0)
   {
      cao_line_points = (Rox_Uint *) rox_memory_allocate(sizeof(Rox_Uint), 2 * cao_nbr_line);
      if (cao_line_points == NULL)
      {
         ROX_INFO_FORMATTED_DISPLAY("Error in allocation, reading %s\n", inline_model_string);
         error = ROX_ERROR_NULL_POINTER; CHECK_ERROR_LOADCAO(error);
      }
   }

   //Store in a map the potential segments to add, allocation will be deleted
   error = rox_dynvec_uint_new(&segments_points_indices, 2 + 2 * cao_nbr_line);
   CHECK_ERROR_LOADCAO(error);
   error = rox_dynvec_point3d_double_new(&segments_points, 2 + 2 * cao_nbr_line);
   CHECK_ERROR_LOADCAO(error);

   for (Rox_Uint k = 0; k < cao_nbr_line; k++)
   {
      STRIP_COMMENTS();

      stdio_err_ret = sscanf(cur, "%u", &index1);    //file_id >> index1;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      STRIP_TO_SPACE();
      STRIP_SPACES();
      stdio_err_ret = sscanf(cur, "%u", &index2);    //file_id >> index2;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      STRIP_TO_SPACE();
      STRIP_SPACES();

      STRIP_PARAMETERS();

      cao_line_points[2 * k] = index1;
      cao_line_points[2 * k + 1] = index2;

      if (index1 < cao_nbr_points && index2 < cao_nbr_points)
      {
         //store both indices next to each other in segments_points_indices
         error = rox_dynvec_uint_append(segments_points_indices, &index1);
         CHECK_ERROR_LOADCAO(error);
         error = rox_dynvec_uint_append(segments_points_indices, &index2);
         CHECK_ERROR_LOADCAO(error);

         //store both points next to each other in segments_points
         error = rox_dynvec_point3d_double_append(segments_points, &cao_points[index1]);
         CHECK_ERROR_LOADCAO(error);
         error = rox_dynvec_point3d_double_append(segments_points, &cao_points[index2]);
         CHECK_ERROR_LOADCAO(error);
      }
      else
      {
         ROX_INFO_FORMATTED_DISPLAY("Error, line %d has wrong coordinates in %s\n", k, inline_model_string);
         error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
      }
   }

   STRIP_COMMENTS();



   //////////////////////////Read the face segment declaration part//////////////////////////
   //Load polygon from the lines extracted earlier (the first point of the line is used)
   stdio_err_ret = sscanf(cur, "%u", &cao_nbr_polygon_line);    //file_id >> cao_nbr_polygon_line;
   CHECK_ERROR_FSCANF(stdio_err_ret);
   STRIP_LINE();

   ROX_INFO_FORMATTED_DISPLAY("> %u polygon lines\n", cao_nbr_polygon_line);

   if (cao_nbr_polygon_line > MAX_DATA_NBR)
   {
      ROX_INFO_FORMATTED_DISPLAY("Error, exceed the max number of polygon lines in %s\n", inline_model_string);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   //Store in a vector the indexes of the segments added in the face segment case
   error = rox_dynvec_uint_new(&face_segment_key_vector, 100);
   CHECK_ERROR_LOADCAO(error);

   for (Rox_Uint k = 0; k < cao_nbr_polygon_line; k++)
   {
      nb_line_pol = 0;
      STRIP_COMMENTS();

      //read number of lines in polygon
      stdio_err_ret = sscanf(cur, "%u", &nb_line_pol);
      CHECK_ERROR_FSCANF(stdio_err_ret);
      if (nb_line_pol > MAX_DATA_NBR)
      {
         ROX_INFO_FORMATTED_DISPLAY("Error, exceed the max number of lines in %s\n", inline_model_string);
         error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
      }
      STRIP_TO_SPACE();
      STRIP_SPACES();

      //allocated our container of points 3d coordinates
      error = rox_dynvec_point3d_double_new(&corners, 2 + 2 * nb_line_pol);
      CHECK_ERROR_LOADCAO(error);

      //loop through lines of poly and append coordinates along indices
      for (Rox_Uint n = 0; n < nb_line_pol; n++)
      {
         stdio_err_ret = sscanf(cur, "%u", &index);
         CHECK_ERROR_FSCANF(stdio_err_ret);
         if (index >= cao_nbr_line)
         {
            ROX_INFO_FORMATTED_DISPLAY("Error, exceed the max number of lines in %s\n", inline_model_string);
            error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
         }
         STRIP_TO_SPACE();
         STRIP_SPACES();

         //append faces corners 3d points coordinates next to each other in corners
         error = rox_dynvec_point3d_double_append(corners, &cao_points[cao_line_points[2 * index]]);
         CHECK_ERROR_LOADCAO(error);
         error = rox_dynvec_point3d_double_append(corners, &cao_points[cao_line_points[2 * index + 1]]);
         CHECK_ERROR_LOADCAO(error);

         //append faces corners 3d points indices next to each other in face_segment_key_vector
         error = rox_dynvec_uint_append(face_segment_key_vector, &cao_line_points[2 * index]);
         CHECK_ERROR_LOADCAO(error);
         error = rox_dynvec_uint_append(face_segment_key_vector, &cao_line_points[2 * index + 1]);
         CHECK_ERROR_LOADCAO(error);
      }

      //////////////////////////Read the parameter value if present//////////////////////////
      STRIP_PARAMETERS();

      //Add to object
      error = add_polygon_segments(obj, corners);//  , id_face++, polygon_name, useLod, minPolygonAreaThreshold, minLineLengthThresholdGeneral);
      CHECK_ERROR_LOADCAO(error);

      error = rox_dynvec_point3d_double_del(&corners);
      CHECK_ERROR_LOADCAO(error);
   }

   //Add the segments which were not already added in the face segment case
   error = rox_dynvec_uint_get_used(&segments_points_indices_nbr, segments_points_indices);
   CHECK_ERROR_LOADCAO(error);

   error = rox_dynvec_uint_get_used(&face_segment_key_vector_nbr, face_segment_key_vector);
   CHECK_ERROR_LOADCAO(error);

   error = rox_dynvec_uint_get_data_pointer(&segments_points_indices_data, segments_points_indices);
   CHECK_ERROR_LOADCAO(error);

   error = rox_dynvec_uint_get_data_pointer(&face_segment_key_vector_data, face_segment_key_vector);
   CHECK_ERROR_LOADCAO(error);

   for (Rox_Uint each_index = 0; each_index < segments_points_indices_nbr; each_index += 2)
   {
      Rox_Bool skip = 0;
      //TODO ? this guy seems to have a ARM optimized search for c arrays http://stackoverflow.com/questions/25661925/quickly-find-whether-a-value-is-present-in-a-c-array
      for (Rox_Uint each_key = 0; each_key < face_segment_key_vector_nbr; each_key += 2)
      {
         if (segments_points_indices_data[each_index] == face_segment_key_vector_data[each_key])
         {
            skip = 1;
            break;
         }
      }
      if (!skip)
      {
         //Create a simple segment with two points
         Rox_DynVec_Point3D_Double simple_segment = NULL;
         error = rox_dynvec_point3d_double_new(&simple_segment, 3);
         CHECK_ERROR_LOADCAO(error);
         //store both points next to each other in segments_points
         error = rox_dynvec_point3d_double_append(simple_segment, &cao_points[segments_points_indices_data[each_index]]);
         CHECK_ERROR_LOADCAO(error);
         error = rox_dynvec_point3d_double_append(simple_segment, &cao_points[segments_points_indices_data[each_index + 1]]);
         CHECK_ERROR_LOADCAO(error);


         //FP TODO ? parameters...
         error = add_polygon_segments(obj, simple_segment);// , id_face++, it->second.name, it->second.useLod, minPolygonAreaThresholdGeneral, it->second.minLineLengthThresh);
         CHECK_ERROR_LOADCAO(error);

         error = rox_dynvec_point3d_double_del(&simple_segment);
         CHECK_ERROR_LOADCAO(error);


         //FP TODO ?
         //initFaceFromCorners(*(faces.getPolygon().back())); // Init from the last polygon that was added
      }
   }

   STRIP_COMMENTS();

   //////////////////////////Skip the faces from points part//////////////////////////
   stdio_err_ret = sscanf(cur, "%u", &caoNbFacesFromPoints);    //file_id >> caoNbFacesFromPoints;
   CHECK_ERROR_FSCANF(stdio_err_ret);
   ROX_INFO_FORMATTED_DISPLAY("> %u faces from points\n", caoNbFacesFromPoints);
   STRIP_LINE();
   for (size_t i = 0; i < caoNbFacesFromPoints; i++)
   {
      STRIP_COMMENTS();
      STRIP_LINE();
      ROX_INFO_FORMATTED_DISPLAY("WARNING faces from points are ignored for now, skipping line %s\n", "");
   }
   STRIP_COMMENTS();

   //////////////////////////Read the Cylinders declaration part//////////////////////////
   stdio_err_ret = sscanf(cur, "%u", &caoNbCylinders);    //file_id >> caoNbCylinders;
   CHECK_ERROR_FSCANF(stdio_err_ret);
   STRIP_LINE();

   ROX_INFO_FORMATTED_DISPLAY("> %u cylinders\n", caoNbCylinders);
   if (caoNbCylinders > MAX_DATA_NBR)
   {
      ROX_INFO_FORMATTED_DISPLAY("Error, exceed the max number of cylinders in %s\n", inline_model_string);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   for (Rox_Uint k = 0; k < caoNbCylinders; ++k)
   {
      double radius = 0;
      Rox_Uint indexP1 = 0, indexP2 = 0;
      STRIP_COMMENTS();

      stdio_err_ret = sscanf(cur, "%u", &indexP1);    //file_id >> indexP1;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      STRIP_TO_SPACE();
      STRIP_SPACES();
      stdio_err_ret = sscanf(cur, "%u", &indexP2);    //file_id >> indexP2;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      STRIP_TO_SPACE();
      STRIP_SPACES();
      stdio_err_ret = sscanf(cur, "%lf", &radius);    //file_id >> radius;
      CHECK_ERROR_FSCANF(stdio_err_ret);

      //don't strip otherwise expected last value we'll miss the next line break in STRIP_PARAMETERS
      //STRIP_TO_SPACE();
      //STRIP_SPACES();

      //////////////////////////Read the parameter value if present//////////////////////////
      STRIP_PARAMETERS();

      error = add_cylinder(obj, cao_points[indexP1], cao_points[indexP2], radius);// , idRevolutionAxis, polygonName);
      CHECK_ERROR_LOADCAO(error);
   }
   STRIP_COMMENTS();


   //////////////////////////Read the Circles declaration part//////////////////////////
   stdio_err_ret = sscanf(cur, "%u", &caoNbCircle);    //file_id >> caoNbCircle;
   CHECK_ERROR_FSCANF(stdio_err_ret);
   STRIP_LINE();

   ROX_INFO_FORMATTED_DISPLAY("> %u circles\n", caoNbCircle);
   if (caoNbCircle > MAX_DATA_NBR)
   {
      ROX_INFO_FORMATTED_DISPLAY("Error, exceed the max number of circles in %s\n", inline_model_string);
      error = ROX_ERROR_INVALID_VALUE; CHECK_ERROR_LOADCAO(error);
   }

   //nbCircles += caoNbCircle;
   for (Rox_Uint k = 0; k < caoNbCircle; ++k)
   {
      Rox_Double radius = 0;
      Rox_Uint indexP1 = 0, indexP2 = 0, indexP3 = 0;
      STRIP_COMMENTS();

      stdio_err_ret = sscanf(cur, "%lf", &radius);    //file_id >> radius;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      STRIP_TO_SPACE();
      STRIP_SPACES();

      stdio_err_ret = sscanf(cur, "%u", &indexP1);    //file_id >> indexP1;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      STRIP_TO_SPACE();
      STRIP_SPACES();
      stdio_err_ret = sscanf(cur, "%u", &indexP2);    //file_id >> indexP2;
      CHECK_ERROR_FSCANF(stdio_err_ret);
      STRIP_TO_SPACE();
      STRIP_SPACES();
      stdio_err_ret = sscanf(cur, "%u", &indexP3);    //file_id >> indexP3;
      CHECK_ERROR_FSCANF(stdio_err_ret);

      //don't strip otherwise expected last value we'll miss the next line break in STRIP_PARAMETERS
      //STRIP_TO_SPACE();
      //STRIP_SPACES();


      //////////////////////////Read the parameter value if present//////////////////////////
      STRIP_PARAMETERS();

      add_circle(obj, cao_points[indexP1], cao_points[indexP2], cao_points[indexP3], radius);// , idFace, polygonName, useLod, minPolygonAreaThreshold);

                                                                                             //initCircle(caoPoints[indexP1], caoPoints[indexP2], caoPoints[indexP3], radius, idFace++, polygonName);
   }



   //stdio_err_ret = fclose(cao_file);
   //if (stdio_err_ret != 0)
   //{
   //   error = ROX_ERROR_BAD_IOSTREAM;
   //   CHECK_ERROR_LOADCAO(error);
   //}


   //cleanup
   if (cao_points)rox_memory_delete(cao_points);
   if (cao_line_points)rox_memory_delete(cao_line_points);
   if (corners)rox_dynvec_point3d_double_del(&corners);
   if (segments_points)rox_dynvec_point3d_double_del(&segments_points);
   if (segments_points_indices)rox_dynvec_uint_del(&segments_points_indices);
   if (face_segment_key_vector)rox_dynvec_uint_del(&face_segment_key_vector);

function_terminate:
   return error;
}


Rox_ErrorCode rox_caofile_edges_get_visible_cylinders(Rox_Cylinder3D ** cylinders, Rox_Uint * size, Rox_CaoFile_Edges caofile_edges)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //if (!cylinders || !size || !caofile_edges)
   //{ error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   error = rox_objset_cylinder3d_get_data_pointer (cylinders, caofile_edges->visible_cylinders);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_cylinder3d_get_used(size, caofile_edges->visible_cylinders);
   ROX_ERROR_CHECK_TERMINATE ( error );


function_terminate:
   return error;
}

/*
ROX_API Rox_ErrorCode rox_caofile_edges_serialize(char* ser, const Rox_CaoFile_Edges obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0;
   Rox_Uint size = 0;

   if (!ser || !obj) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   //poly_segments
   memcpy(ser + offset, &obj->poly_segments->used, sizeof(obj->poly_segments->used));
   offset += sizeof(obj->poly_segments->used);

   for (Rox_Uint id_poly = 0; id_poly < obj->poly_segments->used; id_poly++)
   {
      memcpy(ser + offset, &obj->poly_segments->data[id_poly]->used, sizeof(obj->poly_segments->data[id_poly]->used));
      offset += sizeof(obj->poly_segments->data[id_poly]->used);

      for (Rox_Uint id_point = 0; id_point < obj->poly_segments->data[id_poly]->used; id_point++)
      {
         memcpy(ser + offset, &obj->poly_segments->data[id_poly]->data[id_point].X, sizeof(obj->poly_segments->data[id_poly]->data[id_point].X));
         offset += sizeof(obj->poly_segments->data[id_poly]->data[id_point].X);
         memcpy(ser + offset, &obj->poly_segments->data[id_poly]->data[id_point].Y, sizeof(obj->poly_segments->data[id_poly]->data[id_point].Y));
         offset += sizeof(obj->poly_segments->data[id_poly]->data[id_point].Y);
         memcpy(ser + offset, &obj->poly_segments->data[id_poly]->data[id_point].Z, sizeof(obj->poly_segments->data[id_poly]->data[id_point].Z));
         offset += sizeof(obj->poly_segments->data[id_poly]->data[id_point].Z);
      }
   }


   //ellipses
   memcpy(ser + offset, &obj->ellipses->used, sizeof(obj->ellipses->used));
   offset += sizeof(obj->ellipses->used);
   for (Rox_Uint id_ell = 0; id_ell < obj->ellipses->used; id_ell++)
   {
      memcpy(ser + offset, &obj->ellipses->data[id_ell]->a, sizeof(obj->ellipses->data[id_ell]->a));
      offset += sizeof(obj->ellipses->data[id_ell]->a);
      memcpy(ser + offset, &obj->ellipses->data[id_ell]->b, sizeof(obj->ellipses->data[id_ell]->b));
      offset += sizeof(obj->ellipses->data[id_ell]->b);

      Rox_Double mat_data[16];
      error = rox_matse3_get_data(mat_data, obj->ellipses->data[id_ell]->Te);
      ROX_ERROR_CHECK_TERMINATE ( error );
      for (Rox_Uint id_mat = 0; id_mat < 16; id_mat++)
      {
         memcpy(ser + offset, &mat_data[id_mat], sizeof(mat_data[id_mat]));
         offset += sizeof(mat_data[id_mat]);
      }
   }

   //cylinders
   memcpy(ser + offset, &obj->cylinders->used, sizeof(obj->cylinders->used));
   offset += sizeof(obj->cylinders->used);
   for (Rox_Uint id_cyl = 0; id_cyl < obj->cylinders->used; id_cyl++)
   {
      memcpy(ser + offset, &obj->cylinders->data[id_cyl]->a, sizeof(obj->cylinders->data[id_cyl]->a));
      offset += sizeof(obj->cylinders->data[id_cyl]->a);
      memcpy(ser + offset, &obj->cylinders->data[id_cyl]->b, sizeof(obj->cylinders->data[id_cyl]->b));
      offset += sizeof(obj->cylinders->data[id_cyl]->b);
      memcpy(ser + offset, &obj->cylinders->data[id_cyl]->h, sizeof(obj->cylinders->data[id_cyl]->h));
      offset += sizeof(obj->cylinders->data[id_cyl]->h);

      Rox_Double mat_data[16];
      error = rox_matse3_get_data(mat_data, obj->cylinders->data[id_cyl]->Te);
      ROX_ERROR_CHECK_TERMINATE ( error );
      for (Rox_Uint id_mat = 0; id_mat < 16; id_mat++)
      {
         memcpy(ser + offset, &mat_data[id_mat], sizeof(mat_data[id_mat]));
         offset += sizeof(mat_data[id_mat]);
      }
   }

   //visible_segments
   memcpy(ser + offset, &obj->visible_segments->used, sizeof(obj->visible_segments->used));
   offset += sizeof(obj->visible_segments->used);
   for (Rox_Uint id_vs = 0; id_vs < obj->visible_segments->used; id_vs++)
   {
      memcpy(ser + offset, &obj->visible_segments->data[id_vs].X, sizeof(obj->visible_segments->data[id_vs].X));
      offset += sizeof(obj->visible_segments->data[id_vs].X);
      memcpy(ser + offset, &obj->visible_segments->data[id_vs].Y, sizeof(obj->visible_segments->data[id_vs].Y));
      offset += sizeof(obj->visible_segments->data[id_vs].Y);
      memcpy(ser + offset, &obj->visible_segments->data[id_vs].Z, sizeof(obj->visible_segments->data[id_vs].Z));
      offset += sizeof(obj->visible_segments->data[id_vs].Z);
   }


   //visible_ellipses
   memcpy(ser + offset, &obj->visible_ellipses->used, sizeof(obj->visible_ellipses->used));
   offset += sizeof(obj->visible_ellipses->used);
   for (Rox_Uint id_ell = 0; id_ell < obj->visible_ellipses->used; id_ell++)
   {
      memcpy(ser + offset, &obj->visible_ellipses->data[id_ell]->a, sizeof(obj->visible_ellipses->data[id_ell]->a));
      offset += sizeof(obj->visible_ellipses->data[id_ell]->a);
      memcpy(ser + offset, &obj->visible_ellipses->data[id_ell]->b, sizeof(obj->visible_ellipses->data[id_ell]->b));
      offset += sizeof(obj->visible_ellipses->data[id_ell]->b);

      Rox_Double mat_data[16];
      error = rox_matse3_get_data(mat_data, obj->visible_ellipses->data[id_ell]->Te);
      ROX_ERROR_CHECK_TERMINATE ( error );
      for (Rox_Uint id_mat = 0; id_mat < 16; id_mat++)
      {
         memcpy(ser + offset, &mat_data[id_mat], sizeof(mat_data[id_mat]));
         offset += sizeof(mat_data[id_mat]);
      }
   }

   //visible_cylinders
   memcpy(ser + offset, &obj->visible_cylinders->used, sizeof(obj->visible_cylinders->used));
   offset += sizeof(obj->visible_cylinders->used);
   for (Rox_Uint id_cyl = 0; id_cyl < obj->visible_cylinders->used; id_cyl++)
   {
      memcpy(ser + offset, &obj->visible_cylinders->data[id_cyl]->a, sizeof(obj->visible_cylinders->data[id_cyl]->a));
      offset += sizeof(obj->visible_cylinders->data[id_cyl]->a);
      memcpy(ser + offset, &obj->visible_cylinders->data[id_cyl]->b, sizeof(obj->visible_cylinders->data[id_cyl]->b));
      offset += sizeof(obj->visible_cylinders->data[id_cyl]->b);
      memcpy(ser + offset, &obj->visible_cylinders->data[id_cyl]->h, sizeof(obj->visible_cylinders->data[id_cyl]->h));
      offset += sizeof(obj->visible_cylinders->data[id_cyl]->h);

      Rox_Double mat_data[16];
      error = rox_matse3_get_data(mat_data, obj->visible_cylinders->data[id_cyl]->Te);
      ROX_ERROR_CHECK_TERMINATE ( error );
      for (Rox_Uint id_mat = 0; id_mat < 16; id_mat++)
      {
         memcpy(ser + offset, &mat_data[id_mat], sizeof(mat_data[id_mat]));
         offset += sizeof(mat_data[id_mat]);
      }
   }


function_terminate:
   return error;
}


ROX_API Rox_ErrorCode rox_caofile_edges_deserialize(Rox_CaoFile_Edges obj, const char * ser)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0, size = 0;

   if (obj == 0 || ser == 0) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   //TODO

function_terminate:
   return error;
}


ROX_API Rox_ErrorCode rox_caofile_edges_get_byte_size(Rox_Uint* size, const Rox_CaoFile_Edges obj)
{

   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint ret = 0, struct_size = 0;

   if (!size || !obj) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   //poly_segments
   ret += sizeof(obj->poly_segments->used);
   for (Rox_Uint id_poly = 0; id_poly < obj->poly_segments->used; id_poly++)
   {
      ret += sizeof(obj->poly_segments->data[id_poly]->used);
      ret += obj->poly_segments->data[id_poly]->used * 3 * sizeof(Rox_Double);
   }
   //ellipses, 2 reals and a matse3 (16 reals)
   ret += sizeof(obj->ellipses->used);
   ret += obj->ellipses->used * 18 * sizeof(Rox_Double);
   //cylinders, 3 reals and a matse3 (16 reals)
   ret += sizeof(obj->cylinders->used);
   ret += obj->cylinders->used * 19 * sizeof(Rox_Double);

   //visible_segments, 3 reals for point3d
   ret += sizeof(obj->visible_segments->used);
   ret += obj->visible_segments->used * 3 * sizeof(Rox_Double);
   //visible_ellipses, 2 reals and a matse3 (16 reals)
   ret += sizeof(obj->visible_ellipses->used);
   ret += obj->visible_ellipses->used * 18 * sizeof(Rox_Double);
   //visible_cylinders, 3 reals and a matse3 (16 reals)
   ret += sizeof(obj->visible_cylinders->used);
   ret += obj->visible_cylinders->used * 19 * sizeof(Rox_Double);

   *size = ret;

function_terminate:
   return error;
}

*/

