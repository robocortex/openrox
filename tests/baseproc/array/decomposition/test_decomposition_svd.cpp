//==============================================================================
//
//    OPENROX   : File test_decomposition_svd.cpp
//
//    Contents  : Tests for svd.c
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

//=== INCLUDED HEADERS   =======================================================

#include <openrox_tests.hpp>

extern "C"
{
   #include <baseproc/array/decomposition/svd.h>
   #include <baseproc/geometry/transforms/transform_tools.h>
   #include <inout/system/errors_print.h>
   #include <inout/system/print.h>
   #include <inout/numeric/array2d_print.h>
}

//=== INTERNAL MACROS    =======================================================

ROX_TEST_SUITE_BEGIN(svd)

//=== INTERNAL TYPESDEFS =======================================================

//=== INTERNAL DATATYPES =======================================================

//=== INTERNAL VARIABLES =======================================================

//=== INTERNAL FUNCTDEFS =======================================================

//=== INTERNAL FUNCTIONS =======================================================

//=== EXPORTED FUNCTIONS =======================================================

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_svd_vector)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double data_inp[9] = {-1.0, 0, -0.0001};

   // Rox_Double data_grt[9] = {1024, 0, 0, 0, 1024, 0, 320, 240, 1};
   
   Rox_Array2D_Double S_mes = NULL;
   Rox_Array2D_Double U_mes = NULL;
   Rox_Array2D_Double V_mes = NULL;
   Rox_Array2D_Double m_inp = NULL;

   error = rox_array2d_double_new(&U_mes, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&S_mes, 3, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&V_mes, 1, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&m_inp, 3, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double ** dm_inp = NULL;
   rox_array2d_double_get_data_pointer_to_pointer(&dm_inp, m_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   dm_inp[0][0] = data_inp[0];  dm_inp[1][0] = data_inp[1];  dm_inp[2][0] = data_inp[2];                           

   error = rox_array2d_double_svd(U_mes, S_mes, V_mes, m_inp);
   rox_error_print(error);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Measure: \n");
   rox_array2d_double_print(m_inp);

   error = rox_array2d_double_del(&U_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&S_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&V_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&m_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_svd_matrix2)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Double data_inp[6][5] = {{ 1.544574313838126, 0.006586052166599, 0.771543474849056, 0.002220445121795, 1.597297936246644},
   //                            { 1.372188551329667, 4.799030808103462, 1.322223110616976, 0.262787614123441, 0.249325471382008},
   //                            { 2.911664259578396,-0.355565280933244,-2.020052091695804, 0.048311716005484,-1.262136380732609},
   //                            {-2.485618083164126, 0.162308991329486, 1.756750949735398, 0.094243198625584,-2.527685056526342},
   //                            {-2.342809041582064,-4.381338064155042,-2.299759214019568, 1.025256288200060, 0.998650031556616},
   //                            { 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000}};
   


   Rox_Double data_inp[6][5] = {{ 8.79,  9.93,  9.83, 5.45,  3.16   },
                              { 6.11,  6.91,  5.04, -0.27,  7.98  },
                              {-9.15, -7.93,  4.86, 4.85,  3.01   },
                              {9.57,  1.64,  8.83, 0.74,  5.80    },
                              {-3.49,  4.02,  9.80, 10.00,  4.27  },
                              {9.84,  0.15, -8.99, -6.02, -5.31   }};

/* Standard example : https://software.intel.com/sites/products/documentation/doclib/mkl_sa/11/mkl_lapack_examples/lapacke_dgesvd_row.c.htm
   
   LAPACKE_dgesvd (row-major, high-level) Example Program Results

 Singular values
  27.47  22.64   8.56   5.99   2.01

 Left singular vectors (stored columnwise)
  -0.59   0.26   0.36   0.31   0.23
  -0.40   0.24  -0.22  -0.75  -0.36
  -0.03  -0.60  -0.45   0.23  -0.31
  -0.43   0.24  -0.69   0.33   0.16
  -0.47  -0.35   0.39   0.16  -0.52
   0.29   0.58  -0.02   0.38  -0.65

 Right singular vectors (stored rowwise)
  -0.25  -0.40  -0.69  -0.37  -0.41
   0.81   0.36  -0.25  -0.37  -0.10
  -0.26   0.70  -0.22   0.39  -0.49
   0.40  -0.45   0.25   0.43  -0.62
  -0.22   0.14   0.59  -0.63  -0.44
*/

   Rox_Array2D_Double S_mes = NULL;
   Rox_Array2D_Double U_mes = NULL;
   Rox_Array2D_Double V_mes = NULL;
   Rox_Array2D_Double M_inp = NULL;

   error = rox_array2d_double_new(&U_mes, 6, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&S_mes, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&V_mes, 5, 5);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&M_inp, 6, 5);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double ** dM_inp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dM_inp, M_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for(Rox_Sint i=0; i<6; i++)
   {
      for(Rox_Sint j=0; j<5; j++)
      {
         dM_inp[i][j] = data_inp[i][j];
      }
   }

   error = rox_array2d_double_svd(U_mes, S_mes, V_mes, M_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&U_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&S_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&V_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&M_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_svd_matrix3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double data_inp[6][6] = {{  1.544574313838127, 1.372188551329666,  2.911664259578396, -2.485618083164127, -2.342809041582063, 1.000000000000000},
                              {  0.006586052166599, 4.799030808103464, -0.355565280933245,  0.162308991329486, -4.381338064155043, 1.000000000000000},
                              {  0.771543474849056, 1.322223110616976, -2.020052091695804,  1.756750949735398, -2.299759214019568, 1.000000000000000},
                              {  0.002220445121795, 0.262787614123441,  0.048311716005484,  0.094243198625585,  1.025256288200059, 1.000000000000000},
                              {  1.597297936246644, 0.249325471382007, -1.262136380732608, -2.527685056526342,  0.998650031556615, 1.000000000000000},
                              {  1.597297936246644, 0.249325471382007, -1.262136380732608, -2.527685056526342,  0.998650031556615, 1.000000000000000}};
   
   Rox_Array2D_Double S_mes = NULL;
   Rox_Array2D_Double U_mes = NULL;
   Rox_Array2D_Double V_mes = NULL;
   Rox_Array2D_Double M_inp = NULL;

   error = rox_array2d_double_new(&U_mes, 6, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&S_mes, 6, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&V_mes, 6, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&M_inp, 6, 6);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double ** dM_inp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dM_inp, M_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   for(Rox_Sint i=0; i<6; i++)
   {
      for(Rox_Sint j=0; j<6; j++)
      {
         dM_inp[i][j] = data_inp[i][j];
      }
   }

   error = rox_array2d_double_svd(U_mes, S_mes, V_mes, M_inp);
   rox_error_print(error);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Input: \n");
   rox_array2d_double_print(M_inp);

   rox_log("Measure: \n");
   rox_array2d_double_print(U_mes);
   rox_array2d_double_print(S_mes);
   rox_array2d_double_print(V_mes);

   error = rox_array2d_double_del(&U_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&S_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&V_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&M_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_CASE_DECLARE(rox::OpenROXTest, test_svd_matrix)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double data_inp[9] = {0.9954600404424047, 0.0000000000000000, 0.0951803965236873};

   // Rox_Double data_grt[9] = {1024, 0, 0, 0, 1024, 0, 320, 240, 1};
   
   Rox_Array2D_Double S_mes = NULL;
   Rox_Array2D_Double U_mes = NULL;
   Rox_Array2D_Double V_mes = NULL;
   Rox_Array2D_Double m_inp = NULL;
   Rox_Array2D_Double M_inp = NULL;

   error = rox_array2d_double_new(&U_mes, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&S_mes, 3, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_new(&V_mes, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&m_inp, 3, 1);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_new(&M_inp, 3, 3);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   Rox_Double ** dm_inp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dm_inp, m_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   dm_inp[0][0] = data_inp[0];  dm_inp[1][0] = data_inp[1];  dm_inp[2][0] = data_inp[2];                           

   error = rox_transformtools_skew_from_vector(M_inp, m_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_svd(U_mes, S_mes, V_mes, M_inp);
   rox_error_print(error);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   rox_log("Input: \n");
   rox_array2d_double_print(M_inp);

   rox_log("Measure: \n");
   rox_array2d_double_print(U_mes);
   rox_array2d_double_print(S_mes);
   rox_array2d_double_print(V_mes);
   
   error = rox_array2d_double_del(&U_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&S_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
   
   error = rox_array2d_double_del(&V_mes);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&M_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );

   error = rox_array2d_double_del(&m_inp);
   ROX_TEST_CHECK_EQUAL ( error, ROX_ERROR_NONE );
}

ROX_TEST_SUITE_END()
