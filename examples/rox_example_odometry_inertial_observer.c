//==============================================================================
//
//    OPENROX   : File rox_example_odometry_inertial_observer.c
//
//    Contents  : A simple example program for single plane odometry
//                by fusing visual and inertial measures. 
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== HEADERS      =============================================================

#include <api/openrox.h>
#include <stdio.h>

//=== MACROS    ================================================================

// Download the following archive and uncompress it in the "seq" directory:
// http://public.robocortex.com/download/sequences/inertial.zip

#define mod_odometry_file "../seq/inertial/model_odometry.pgm"
#define mod_identse3_file "../seq/inertial/model_identse3.pgm"

#define seq_file "../seq/inertial/image_c0_"
#define mea_file "../seq/inertial/measures.txt"
#define res_file "../res/result_inertial_%03d.ppm"

#define px 448.850881802737945
#define py 450.264204403581687
#define u0 394.306503926447533
#define v0 292.823830574325484

//=== VARIABLES    =============================================================

//=== FUNCTIONS    =============================================================

//=== MAIN PROGRAM =============================================================

Rox_Sint main(Rox_Sint argc, Rox_Char *argv[])
{
   Rox_Error error = 0;
   Rox_Uint major = 0, minor = 0, patch = 0;

   Rox_Double sizex = 0.376;
   Rox_Double sizey = 0.282;

   Rox_Double f_video_acq = 40.0;
   Rox_Double f_inertial  = 200.0;

   Rox_Sint nbimages = 2499;

   Rox_Sint iterations = 0;
   Rox_Char filename[FILENAME_MAX];

   Rox_Sint i = 0;
   Rox_Sint j = 0;

   Rox_Sint is_identified = 0;

   Rox_Double A[3] = {0.0,0.0,0.0};
   Rox_Double W[3] = {0.0,0.0,0.0};
   Rox_Double M[3] = {0.0,0.0,0.0};
   Rox_Double timestamp  = 0.0;
   Rox_Double vTi_data[16] = {0.0, -1.0,  0.0, 0.005,
                            0.0,  0.0, -1.0, 0.030,
                            1.0,  0.0,  0.0, -0.08,
                            0.0,  0.0,  0.0, 1.0};

   Rox_MatSE3 vTi = 0;
   Rox_MatSE3 pTm = 0;
   Rox_MatSE3 vTm_mea = 0;// measued pose
   Rox_MatSE3 vTm_mea_pre = 0; // previous measured pose

   Rox_MatSE3 vTm_pre = 0;
   Rox_Matrix K = 0;

   Rox_Model_Single_Plane model_odometry_single_plane = 0;
   Rox_Model_Single_Plane model_identse3_single_plane = 0;

   // Define ident parameters
   Rox_Ident_Texture_SE3 identifier = 0;

   // Rox_Data
   Rox_Image model_identse3 = 0;
   Rox_Image model_odometry = 0;

   Rox_Odometry_Single_Plane_Params params = 0;
   Rox_Odometry_Single_Plane tracker   = 0;
   Rox_Camera camera = 0;

   // Visualisation
   Rox_Image_RGBA image_rgba = 0;

   Rox_Uint blue  = ROX_MAKERGBA(0, 0 , 255, 255);
   Rox_Uint green = ROX_MAKERGBA(0, 255 , 0, 255);
   Rox_Uint red = ROX_MAKERGBA(255, 0, 0, 255);

   // Rox_Inertial data
   // Measure file
   FILE* measure_file = NULL;

   // Initialize Rox_Inertial structure
   Rox_Inertial inertial = 0;
   Rox_Inertial_Observer observer = 0;

   Rox_Double score = 0.0;

   // Update filter
   Rox_Bool update = 0;

   // Only synchronous observer is demonstrated in this example
   const Rox_Bool synchronous = 1;
   
   iterations = f_inertial / f_video_acq;

   // Get and display the version of Rox AR SDK
   error = rox_get_version(&major, &minor, &patch); 
   if(error) goto function_terminate;

   printf ("ROX AR SDK version %d.%d.%d \n", major, minor, patch);

   // Allocate and init the camera calibration matrix
   error = rox_matrix_new(&K, 3, 3);
   if(error) goto function_terminate;

   error = rox_matrix_build_calibration_matrix(K, px, py, u0, v0);
   if(error) goto function_terminate;

   error = rox_matse3_new(&vTi);
   if(error) goto function_terminate;

   error = rox_matse3_new(&pTm);
   if(error) goto function_terminate;

   error = rox_matse3_new(&vTm_pre);
   if(error) goto function_terminate;

   error = rox_matse3_new(&vTm_mea);
   if(error) goto function_terminate;

   error = rox_matse3_new(&vTm_mea_pre);
   if(error) goto function_terminate;
   
   error = rox_matse3_set_data(vTi, vTi_data);
   if(error) goto function_terminate;

   // Load Model and initialize model
   error = rox_image_new_read_pgm(&model_odometry, mod_odometry_file);
   if(error) goto function_terminate;
   
   error = rox_model_single_plane_new(&model_odometry_single_plane);
   if(error) goto function_terminate;

   error = rox_model_single_plane_set_template_xright_ydown (model_odometry_single_plane, model_odometry, sizex, sizey);
   if(error) goto function_terminate;

   error = rox_image_new_read_pgm(&model_identse3, mod_identse3_file);
   if(error) goto function_terminate;

   error = rox_model_single_plane_new(&model_identse3_single_plane);
   if(error) goto function_terminate;

   error = rox_model_single_plane_set_template_xright_ydown(model_identse3_single_plane, model_identse3, sizex, sizey);
   if(error) goto function_terminate;

   // Odometry
   error = rox_odometry_single_plane_params_new(&params);
   if(error) goto function_terminate;

   error = rox_odometry_single_plane_params_set_prediction_radius(params, 16);
   if(error) goto function_terminate;

   error = rox_odometry_single_plane_new(&tracker, params, model_odometry_single_plane);
   if(error) goto function_terminate;

   error = rox_odometry_single_plane_set_miter(tracker, 16);
   if(error) goto function_terminate;

   error = rox_odometry_single_plane_set_score_thresh(tracker, 0.8);
   if(error) goto function_terminate;

   // Define the ident object
   error = rox_ident_texture_se3_new(&identifier);
   if(error) goto function_terminate;

   error = rox_ident_texture_se3_set_model(identifier, model_identse3_single_plane);
   if(error) goto function_terminate;

   // Load First Image
   sprintf(filename, "%s%06d.pgm", seq_file, i);
   error = rox_camera_new_read_pgm(&camera, filename);
   if(error) goto function_terminate;

   error = rox_camera_set_pinhole_params(camera, px, py, u0, v0);
   if(error) goto function_terminate;

   // Visualisation
   error = rox_image_rgba_new_read_pgm(&image_rgba, filename);
   if(error) goto function_terminate;

   //====================== INIT INERTIAL DATA ==========================
   measure_file = fopen(mea_file, "r");

   error = rox_inertial_new(&inertial, f_inertial);
   if(error) goto function_terminate;

   error = rox_odometry_inertial_observer_new(&observer, vTi, pTm, synchronous);
   if(error) goto function_terminate;

   // Only valid for a synchronous observer
   error = rox_odometry_inertial_observer_set_frequency(observer, f_video_acq);
   if(error) goto function_terminate;
 
   // First detection
   error = rox_ident_texture_se3_make(&is_identified, vTm_mea, identifier, camera);
   if(error) goto function_terminate;

   // Make tracking
   error = rox_odometry_single_plane_set_pose(tracker, vTm_mea);
   if(error) goto function_terminate;

   error = rox_odometry_single_plane_make(tracker, camera);
   if(error) goto function_terminate;

   error = rox_odometry_single_plane_get_score(&score, tracker);
   if(error) goto function_terminate;

   printf("Odometry score at init %f \n", score);

   if(!error)
   {
      // Get camera pose
      error = rox_odometry_single_plane_get_pose(vTm_mea, tracker);
      if(error) goto function_terminate;

      // Initialize observer
      error = rox_odometry_inertial_observer_init(observer, inertial, vTm_mea);
      if(error) goto function_terminate;

      // Draw results
      error = rox_image_rgba_draw_projection_model_single_plane(image_rgba, K, vTm_mea, model_odometry_single_plane, blue);
      if(error) goto function_terminate;
   }

   //======================= START MAIN LOOP   ==========================
   for(i=1; i<nbimages; i++)
   {
      //============== TRACKING ============
      sprintf(filename, "%s%06d.pgm", seq_file, i);
      printf("Reading %s file \r\n", filename);

      error = rox_camera_read_pgm(camera, filename);
      if(error) goto function_terminate;

      // visualisation
      error = rox_image_rgba_read_pgm(image_rgba, filename);
      if(error) goto function_terminate;

      //============== FUSION ============
      for(j = 0; j < iterations; j++)
      {
         // Set Measure
         int res = fscanf(measure_file, "%lf; %lf; %lf; %lf; %lf; %lf; %lf; %lf; %lf; %lf\n ", &A[0], &A[1], &A[2], &W[0], &W[1], &W[2],&M[0], &M[1], &M[2], &timestamp);
         if(res == 0) goto function_terminate;

         error = rox_inertial_set_measure(inertial, A, W, M, timestamp);
         if(error) goto function_terminate;
      }
      
      // ----------- VISUAL AND INERTIAL FUSION ----------------------------
      // Compute the vision prediction
      error = rox_odometry_inertial_observer_compute_prediction(vTm_pre, observer, inertial);
      if(error) goto function_terminate;
      
      // Update the vision measure
      error = rox_odometry_single_plane_set_pose(tracker, vTm_pre);
      if(error) goto function_terminate;

      error = rox_odometry_single_plane_make(tracker, camera);
      if(!error)
      {
         update = 1;
      }
      else
      {
         update = 0;
         printf("Odometry failed \n");
      }

      error = rox_odometry_single_plane_get_score(&score, tracker);
      if(error) goto function_terminate;

      error = rox_odometry_single_plane_get_pose(vTm_mea, tracker);
      if(error) goto function_terminate;

      printf("Odometry score %f \n", score);

      // Update observer state
      error = rox_odometry_inertial_observer_make_corrections(observer, inertial, vTm_mea, update);
      if(error) goto function_terminate;

      // Draw results
      // Draw previous measured pose
      error = rox_image_rgba_draw_projection_model_single_plane(image_rgba, K, vTm_mea_pre, model_odometry_single_plane, red);
      if(error) goto function_terminate;
      
      error = rox_image_rgba_draw_projection_model_single_plane(image_rgba, K, vTm_mea, model_odometry_single_plane, blue);
      if(error) goto function_terminate;

      error = rox_image_rgba_draw_projection_model_single_plane(image_rgba, K, vTm_pre, model_odometry_single_plane, green);
      if(error) goto function_terminate;

      // Save results
      sprintf(filename, res_file, i);
      error = rox_image_rgba_save_ppm(filename, image_rgba);
      if(error) goto function_terminate;
      
      // Copy pose
      error = rox_matse3_copy(vTm_mea_pre, vTm_mea); 
      if(error) goto function_terminate;
   }

function_terminate:

   // Close Files
   if (measure_file) fclose(measure_file);

   // Delete objects and free memory
   rox_matrix_del(&K);
   rox_matse3_del(&vTi);
   rox_matse3_del(&pTm);
   rox_matse3_del(&vTm_mea);
   rox_matse3_del(&vTm_pre);

   rox_image_del(&model_odometry);
   rox_image_del(&model_identse3);

   rox_odometry_single_plane_params_del(&params);
   rox_odometry_single_plane_del(&tracker);
   rox_ident_texture_se3_del(&identifier);

   rox_odometry_inertial_observer_del(&observer);
   rox_inertial_del(&inertial);
   rox_camera_del(&camera);

   rox_image_rgba_del(&image_rgba);

   // Display Error
   rox_error_print(error);

   printf("\nPress return to end the program\n");
   getchar();

   return error;
}

