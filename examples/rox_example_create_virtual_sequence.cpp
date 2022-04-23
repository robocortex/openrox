//==============================================================================
//
//    OPENROX   : File rox_example_create_virtual_sequence.cpp
//
//    Contents  : A simple example program for creating a virtual sequence.
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== HEADERS   ================================================================

extern "C"
{
   #include <generated/array2d_point2d_float.h>
   #include <generated/array2d_point2d_sshort.h>

   #include <baseproc/maths/maths_macros.h>
   #include <baseproc/array/fill/fillunit.h>
   #include <baseproc/array/inverse/svdinverse.h>
   #include <baseproc/array/multiply/mulmatmat.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
   #include <baseproc/geometry/pixelgrid/meshgrid2d.h>
   #include <baseproc/image/remap/remap_bilinear_nomask_uchar_to_uchar/remap_bilinear_nomask_uchar_to_uchar.h>
   #include <baseproc/image/image.h> 
   #include <inout/numeric/array2d_print.h>
   #include <inout/image/pgm/pgmfile.h>
}

#include <math.h>

#define REFERENCE_IMAGE "/Users/emalis/prog/sequences/astrium_multi/massive.pgm"

int main(int argc, char *argv[])
{
   Rox_Image reference, current;
   Rox_Array2D_Double pose, ipose, H, G, suffix;
   Rox_Array2D_Double calib_reference, calib_camera;
   Rox_ErrorCode err;
   Rox_MeshGrid2D_Float grid = NULL;

   err = rox_array2d_double_new(&calib_reference, 3, 3);
   if (err) return err;

   err = rox_array2d_double_new(&calib_camera, 3, 3);
   if (err) return err;

   err = rox_array2d_double_new(&pose, 4, 4);
   if (err) return err;

   err = rox_array2d_double_new(&ipose, 4, 4);
   if (err) return err;

   err = rox_array2d_double_new(&suffix, 3, 3);
   if (err) return err;

   err = rox_array2d_double_new(&G, 3, 3);
   if (err) return err;

   err = rox_array2d_double_new(&H, 3, 3);
   if (err) return err;

   err = rox_image_new_read_pgm(&reference, REFERENCE_IMAGE);
   if (err) return err;

   err = rox_meshgrid2d_float_new(&grid, 480, 640);
   if (err) return err;

   err = rox_image_new(&current, 640, 480);
   if (err) return err;

   printf("reference loaded\n");

   Rox_Sint width = 0, height = 0;
   
   err = rox_image_get_size(&height, &width, reference);
   if (err) return err;

   rox_transformtools_build_calibration_matrix(calib_reference, 500, 500, 320, 240);
   rox_transformtools_build_calibration_matrix(calib_camera, 500, 500, 320, 240);

   rox_array2d_double_fillunit(suffix);
   rox_array2d_double_set_value(suffix, 0, 2, width / 2);
   rox_array2d_double_set_value(suffix, 1, 2, height / 2);

   for (int idview = 0; idview < 900; idview++)
   {
      rox_matse3_set_axis_angle_translation(pose, 1, 0, 0, ROX_PI/6.0, - idview * 0.02, 0, 0);
      rox_transformtools_build_homography(G, pose, calib_reference, calib_camera, 0, 0, -1, 1);
      rox_array2d_double_svdinverse(H, G);
      rox_array2d_double_mulmatmat(G, suffix, H);
      rox_warp_grid_sl3_float(grid, G);
      rox_remap_bilinear_nomask_uchar_to_uchar(current, reference, grid);

      char filename[FILENAME_MAX];
      sprintf(filename, "output%d.pgm", idview );
      rox_image_save_pgm(filename, current);
   }
  
   return 1;
}
