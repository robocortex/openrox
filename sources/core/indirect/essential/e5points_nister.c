//==============================================================================
//
//    OPENROX   : File e5points.c
//
//    Contents  : API of e5points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "e5points.h"

#include <system/errors/errors.h>

#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>
#include <baseproc/array/decomposition/qr.h>
#include <baseproc/array/decomposition/gausspivoting.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/nonlin/polynomials.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/transpose/transpose.h>
#include <baseproc/maths/nonlin/polynomials.h>
#include <baseproc/array/determinant/detgl3.h>

#include <inout/system/errors_print.h>

//! Polynom of degree 1 with 4 unknowns
struct Rox_Polynom_P4D1_Struct
{
   //! To be commented
   Rox_Double x;
   //! To be commented
   Rox_Double y;
   //! To be commented
   Rox_Double z;
   //! To be commented
   Rox_Double w;
};

   //! To be commented
struct Rox_Polynom_P4D2_Struct
{
    //! To be commented
   Rox_Double x2;
     //! To be commented
  Rox_Double xy;
    //! To be commented
   Rox_Double xz;
    //! To be commented
   Rox_Double xw;
     //! To be commented
  Rox_Double y2;
     //! To be commented
  Rox_Double yz;
     //! To be commented
  Rox_Double yw;
    //! To be commented
   Rox_Double z2;
     //! To be commented
  Rox_Double zw;
     //! To be commented
  Rox_Double w2;
};

   //! To be commented
struct Rox_Polynom_P4D3_Struct
{
    //! To be commented
  Rox_Double xxx;
    //! To be commented
  Rox_Double xxy;
   //! To be commented
   Rox_Double xxz;
   //! To be commented
   Rox_Double xxw;
    //! To be commented
  Rox_Double xyy;
    //! To be commented
  Rox_Double xyz;
   //! To be commented
   Rox_Double xyw;
   //! To be commented
   Rox_Double xzz;
    //! To be commented
  Rox_Double xzw;
   //! To be commented
   Rox_Double xww;
    //! To be commented
  Rox_Double yyy;
    //! To be commented
  Rox_Double yyz;
    //! To be commented
  Rox_Double yyw;
    //! To be commented
  Rox_Double yzz;
   //! To be commented
   Rox_Double yzw;
   //! To be commented
   Rox_Double yww;
   //! To be commented
   Rox_Double zzz;
   //! To be commented
   Rox_Double zzw;
   //! To be commented
   Rox_Double zww;
    //! To be commented
  Rox_Double www;
};

   //! To be commented
struct Rox_Polynom_P1D3_Struct
{
    //! To be commented
  Rox_Double z[4];
};

   //! To be commented
struct Rox_Polynom_P1D4_Struct
{
   //! To be commented
   Rox_Double z[5];
};

   //! To be commented
struct Rox_Polynom_P1D6_Struct
{
    //! To be commented
  Rox_Double z[7];
};

   //! To be commented
struct Rox_Polynom_P1D7_Struct
{
    //! To be commented
  Rox_Double z[8];
};

   //! To be commented
struct Rox_Polynom_P1D10_Struct
{
    //! To be commented
  Rox_Double z[11];
};

   //! To be commented
typedef struct Rox_Polynom_P4D1_Struct Rox_Polynom_P4D1_Struct;
   //! To be commented
typedef struct Rox_Polynom_P4D2_Struct Rox_Polynom_P4D2_Struct;
   //! To be commented
typedef struct Rox_Polynom_P4D3_Struct Rox_Polynom_P4D3_Struct;
   //! To be commented
typedef struct Rox_Polynom_P4D1_Struct Rox_Polynom_P4D1_Mat33[3][3];
   //! To be commented
typedef struct Rox_Polynom_P4D2_Struct Rox_Polynom_P4D2_Mat33[3][3];
   //! To be commented
typedef struct Rox_Polynom_P4D3_Struct Rox_Polynom_P4D3_Mat33[3][3];
   //! To be commented
typedef struct Rox_Polynom_P1D10_Struct Rox_Polynom_P1D10_Struct;
   //! To be commented
typedef struct Rox_Polynom_P1D7_Struct Rox_Polynom_P1D7_Struct;
   //! To be commented
typedef struct Rox_Polynom_P1D6_Struct Rox_Polynom_P1D6_Struct;
   //! To be commented
typedef struct Rox_Polynom_P1D4_Struct Rox_Polynom_P1D4_Struct;
   //! To be commented
typedef struct Rox_Polynom_P1D3_Struct Rox_Polynom_P1D3_Struct;

Rox_Polynom_P1D10_Struct rox_polynom_p1d10_64_mul(Rox_Polynom_P1D6_Struct left, Rox_Polynom_P1D4_Struct right)
{
   Rox_Polynom_P1D10_Struct ret;

   for ( Rox_Sint i = 0; i < 11; i++) ret.z[i] = 0;

   for ( Rox_Sint i = 0; i < 7; i++)
   {
      for ( Rox_Sint j = 0; j < 5; j++)
      {
         Rox_Uint sum = i + j;
         ret.z[sum] += left.z[i] * right.z[j];
      }
   }

   return ret;
}

Rox_Polynom_P1D10_Struct rox_polynom_p1d10_73_mul(Rox_Polynom_P1D7_Struct left, Rox_Polynom_P1D3_Struct right)
{
   Rox_Polynom_P1D10_Struct ret;

   for ( Rox_Sint i = 0; i < 11; i++) ret.z[i] = 0;

   for ( Rox_Sint i = 0; i < 8; i++)
   {
      for ( Rox_Sint j = 0; j < 4; j++)
      {
         Rox_Uint sum = i + j;
         ret.z[sum] += left.z[i] * right.z[j];
      }
   }

   return ret;
}

Rox_Polynom_P1D10_Struct rox_polynom_p1d10_add(Rox_Polynom_P1D10_Struct left, Rox_Polynom_P1D10_Struct right)
{
   Rox_Polynom_P1D10_Struct ret;

   for ( Rox_Sint i = 0; i < 11; i++) ret.z[i] = left.z[i] + right.z[i];

   return ret;
}

Rox_Polynom_P1D7_Struct rox_polynom_p1d7_mul(Rox_Polynom_P1D4_Struct left, Rox_Polynom_P1D3_Struct right)
{
   Rox_Polynom_P1D7_Struct ret;

   for ( Rox_Sint i = 0; i < 8; i++) ret.z[i] = 0;

   for ( Rox_Sint i = 0; i < 5; i++)
   {
      for ( Rox_Sint j = 0; j < 4; j++)
      {
         Rox_Uint sum = i + j;
         ret.z[sum] += left.z[i] * right.z[j];
      }
   }

   return ret;
}

Rox_Polynom_P1D7_Struct rox_polynom_p1d7_add(Rox_Polynom_P1D7_Struct left, Rox_Polynom_P1D7_Struct right)
{
   Rox_Polynom_P1D7_Struct ret;

   for ( Rox_Sint i = 0; i < 8; i++) ret.z[i] = left.z[i] + right.z[i];

   return ret;
}

Rox_Polynom_P1D7_Struct rox_polynom_p1d7_sub(Rox_Polynom_P1D7_Struct left, Rox_Polynom_P1D7_Struct right)
{
   Rox_Polynom_P1D7_Struct ret;

   for ( Rox_Sint i = 0; i < 8; i++) ret.z[i] = left.z[i] - right.z[i];

   return ret;
}

Rox_Polynom_P1D6_Struct rox_polynom_p1d6_mul(Rox_Polynom_P1D3_Struct left, Rox_Polynom_P1D3_Struct right)
{
   Rox_Polynom_P1D6_Struct ret;

   for ( Rox_Sint i = 0; i < 7; i++) ret.z[i] = 0;

   for ( Rox_Sint i = 0; i < 4; i++)
   {
      for ( Rox_Sint j = 0; j < 4; j++)
      {
         Rox_Uint sum = i + j;
         ret.z[sum] += left.z[i] * right.z[j];
      }
   }

   return ret;
}

Rox_Polynom_P1D6_Struct rox_polynom_p1d6_add(Rox_Polynom_P1D6_Struct left, Rox_Polynom_P1D6_Struct right)
{
   Rox_Polynom_P1D6_Struct ret;

   for ( Rox_Sint i = 0; i < 7; i++) ret.z[i] = left.z[i] + right.z[i];

   return ret;
}

Rox_Polynom_P1D6_Struct rox_polynom_p1d6_sub(Rox_Polynom_P1D6_Struct left, Rox_Polynom_P1D6_Struct right)
{
   Rox_Polynom_P1D6_Struct ret;

   for ( Rox_Sint i = 0; i < 7; i++) ret.z[i] = left.z[i] - right.z[i];

   return ret;
}

Rox_Polynom_P4D2_Struct rox_polynom_p4d1_mul(Rox_Polynom_P4D1_Struct left, Rox_Polynom_P4D1_Struct right)
{
   Rox_Polynom_P4D2_Struct ret;

   ret.x2 = left.x * right.x;
   ret.xy = left.x * right.y + left.y * right.x;
   ret.xz = left.x * right.z + left.z * right.x;
   ret.xw = left.x * right.w + left.w * right.x;
   ret.y2 = left.y * right.y;
   ret.yz = left.y * right.z + left.z * right.y;
   ret.yw = left.y * right.w + left.w * right.y;
   ret.z2 = left.z * right.z;
   ret.zw = left.z * right.w + left.w * right.z;
   ret.w2 = left.w * right.w;

   return ret;
}

Rox_Polynom_P4D2_Struct rox_polynom_p4d2_zero()
{
   Rox_Polynom_P4D2_Struct ret;

   ret.x2 = 0;
   ret.xy = 0;
   ret.xz = 0;
   ret.xw = 0;
   ret.y2 = 0;
   ret.yz = 0;
   ret.yw = 0;
   ret.z2 = 0;
   ret.zw = 0;
   ret.w2 = 0;

   return ret;
}

Rox_Polynom_P4D2_Struct rox_polynom_p4d2_add(Rox_Polynom_P4D2_Struct left, Rox_Polynom_P4D2_Struct right)
{
   Rox_Polynom_P4D2_Struct ret;

   ret.x2 = left.x2 + right.x2;
   ret.xy = left.xy + right.xy;
   ret.xz = left.xz + right.xz;
   ret.xw = left.xw + right.xw;
   ret.y2 = left.y2 + right.y2;
   ret.yz = left.yz + right.yz;
   ret.yw = left.yw + right.yw;
   ret.z2 = left.z2 + right.z2;
   ret.zw = left.zw + right.zw;
   ret.w2 = left.w2 + right.w2;

   return ret;
}

Rox_Polynom_P4D2_Struct rox_polynom_p4d2_scale(Rox_Polynom_P4D2_Struct left, Rox_Double scale)
{
   Rox_Polynom_P4D2_Struct ret;

   ret.x2 = left.x2 * scale;
   ret.xy = left.xy * scale;
   ret.xz = left.xz * scale;
   ret.xw = left.xw * scale;
   ret.y2 = left.y2 * scale;
   ret.yz = left.yz * scale;
   ret.yw = left.yw * scale;
   ret.z2 = left.z2 * scale;
   ret.zw = left.zw * scale;
   ret.w2 = left.w2 * scale;

   return ret;
}

Rox_Polynom_P4D2_Struct rox_polynom_p4d2_sub(Rox_Polynom_P4D2_Struct left, Rox_Polynom_P4D2_Struct right)
{
   Rox_Polynom_P4D2_Struct ret;

   ret.x2 = left.x2 - right.x2;
   ret.xy = left.xy - right.xy;
   ret.xz = left.xz - right.xz;
   ret.xw = left.xw - right.xw;
   ret.y2 = left.y2 - right.y2;
   ret.yz = left.yz - right.yz;
   ret.yw = left.yw - right.yw;
   ret.z2 = left.z2 - right.z2;
   ret.zw = left.zw - right.zw;
   ret.w2 = left.w2 - right.w2;

   return ret;
}

Rox_Polynom_P4D3_Struct rox_polynom_p4d2_mul(Rox_Polynom_P4D2_Struct left, Rox_Polynom_P4D1_Struct right)
{
   Rox_Polynom_P4D3_Struct ret;

   ret.xxx = left.x2 * right.x;
   ret.yyy = left.y2 * right.y;
   ret.zzz = left.z2 * right.z;
   ret.www = left.w2 * right.w;

   ret.xxy = left.x2 * right.y + left.xy * right.x;
   ret.xxz = left.x2 * right.z + left.xz * right.x;
   ret.xxw = left.x2 * right.w + left.xw * right.x;

   ret.xyy = left.y2 * right.x + left.xy * right.y;
   ret.xzz = left.z2 * right.x + left.xz * right.z;
   ret.xww = left.w2 * right.x + left.xw * right.w;

   ret.yyz = left.y2 * right.z + left.yz * right.y;
   ret.yyw = left.y2 * right.w + left.yw * right.y;

   ret.yzz = left.z2 * right.y + left.yz * right.z;
   ret.yww = left.w2 * right.y + left.yw * right.w;

   ret.zzw = left.z2 * right.w + left.zw * right.z;
   ret.zww = left.w2 * right.z + left.zw * right.w;

   ret.xyz = left.xy * right.z + left.xz * right.y + left.yz * right.x;

   ret.xyw = left.xy * right.w + left.xw * right.y + left.yw * right.x;
   ret.xzw = left.xz * right.w + left.xw * right.z + left.zw * right.x;
   ret.yzw = left.yz * right.w + left.yw * right.z + left.zw * right.y;

   return ret;
}

Rox_Polynom_P4D3_Struct rox_polynom_p4d3_add(Rox_Polynom_P4D3_Struct left, Rox_Polynom_P4D3_Struct right)
{
   Rox_Polynom_P4D3_Struct ret;

   ret.xxx = left.xxx + right.xxx;
   ret.yyy = left.yyy + right.yyy;
   ret.zzz = left.zzz + right.zzz;
   ret.www = left.www + right.www;

   ret.xxy = left.xxy + right.xxy;
   ret.xxz = left.xxz + right.xxz;
   ret.xxw = left.xxw + right.xxw;

   ret.xyy = left.xyy + right.xyy;
   ret.xzz = left.xzz + right.xzz;
   ret.xww = left.xww + right.xww;

   ret.yyz = left.yyz + right.yyz;
   ret.yyw = left.yyw + right.yyw;

   ret.yzz = left.yzz + right.yzz;
   ret.yww = left.yww + right.yww;

   ret.zzw = left.zzw + right.zzw;
   ret.zww = left.zww + right.zww;

   ret.xyz = left.xyz + right.xyz;

   ret.xyw = left.xyw + right.xyw;
   ret.xzw = left.xzw + right.xzw;
   ret.yzw = left.yzw + right.yzw;

   return ret;
}

Rox_Polynom_P4D3_Struct rox_polynom_p4d3_zero()
{
   Rox_Polynom_P4D3_Struct ret;

   ret.xxx = 0;
   ret.yyy = 0;
   ret.zzz = 0;
   ret.www = 0;

   ret.xxy = 0;
   ret.xxz = 0;
   ret.xxw = 0;

   ret.xyy = 0;
   ret.xzz = 0;
   ret.xww = 0;

   ret.yyz = 0;
   ret.yyw = 0;

   ret.yzz = 0;
   ret.yww = 0;

   ret.zzw = 0;
   ret.zww = 0;

   ret.xyz = 0;

   ret.xyw = 0;
   ret.xzw = 0;
   ret.yzw = 0;

   return ret;
}

Rox_ErrorCode rox_essential_get_basis (
  Rox_Polynom_P4D1_Mat33 res,
  const Rox_Point2D_Double  ref2D,
  const Rox_Point2D_Double  cur2D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Matrix A = NULL, At = NULL;
   Rox_Double ** dA, **dV;
   Rox_Uint idpt, idrow, i, j;
   Rox_Matrix Q = NULL, R = NULL, U = NULL, S = NULL, V = NULL, P = NULL;

   if (!ref2D || !cur2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //Create buffers
   error = rox_matrix_new(&A, 5, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new(&At, 9, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new(&Q, 9, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new(&R, 9, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new(&P, 5, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new(&U, 9, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new(&S, 5, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new(&V, 5, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Compute A such that A*vec(E)=0
   error = rox_array2d_double_get_data_pointer_to_pointer( &dA, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (idpt = 0; idpt < 5; idpt++)
   {
      dA[idpt][0] = ref2D[idpt].u * cur2D[idpt].u;
      dA[idpt][1] = ref2D[idpt].v * cur2D[idpt].u;
      dA[idpt][2] = cur2D[idpt].u;
      dA[idpt][3] = ref2D[idpt].u * cur2D[idpt].v;
      dA[idpt][4] = ref2D[idpt].v * cur2D[idpt].v;
      dA[idpt][5] = cur2D[idpt].v;
      dA[idpt][6] = ref2D[idpt].u;
      dA[idpt][7] = ref2D[idpt].v;
      dA[idpt][8] = 1.0;
   }

   // Compute the right null space of A.
   // Because A is fat, we compute the left null space of A' which gives the same values
   error = rox_array2d_double_transpose(At, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_qrp(Q, R, P, At);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svd_jacobi(U, S, V, Q, R, P);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Null space is transformed in 4 basis of the essential matrix E1,E2,E3,E4 such that
   // E=E1*x+E2*y+E3*z+E4*w (w is set to 1, x,y,z are unknowns to compute)
   error = rox_array2d_double_get_data_pointer_to_pointer( &dV, U);
   ROX_ERROR_CHECK_TERMINATE ( error );

   idrow = 0;
   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 3; j++)
      {
         res[i][j].x = dV[idrow][5];
         res[i][j].y = dV[idrow][6];
         res[i][j].z = dV[idrow][7];
         res[i][j].w = dV[idrow][8];
         idrow++;
      }
   }

   error = ROX_ERROR_NONE;

function_terminate:

   rox_matrix_del(&A);
   rox_matrix_del(&At);
   rox_matrix_del(&Q);
   rox_matrix_del(&R);
   rox_matrix_del(&U);
   rox_matrix_del(&S);
   rox_matrix_del(&V);
   rox_matrix_del(&P);

   return error;
}

Rox_ErrorCode rox_essential_from_5_points_nister (
  Rox_Matrix Ematrices[10],
  Rox_Uint * nbsolutions,
  const Rox_Point2D_Double  ref2D,
  const Rox_Point2D_Double  cur2D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Polynom_P4D1_Mat33 basis;
   Rox_Polynom_P4D3_Struct det, d1,d2, d3;
   Rox_Polynom_P4D2_Struct trace;
   Rox_Polynom_P4D2_Mat33 EEt, lambda;
   Rox_Polynom_P4D3_Mat33 eigen;
   Rox_Uint i,j,k,l;
   Rox_Matrix A, GA;
   Rox_Double ** dA, **dGA, **dE;
   Rox_Double *rowe, *rowf, *rowg, *rowh, *rowi, *rowj;
   Rox_Polynom_P1D3_Struct B11,B12,B21,B22,B31,B32;
   Rox_Polynom_P1D4_Struct B13,B23,B33;
   Rox_Polynom_P1D7_Struct p1, p2;
   Rox_Polynom_P1D6_Struct p3;
   Rox_Polynom_P1D10_Struct bdet;
   Rox_Double roots[10];
   Rox_Uint nbroots;

   if (!Ematrices || !nbsolutions || !ref2D || !cur2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //Buffers
   A = 0;
   GA = 0;
   *nbsolutions = 0;

   error = rox_array2d_double_new(&A, 10, 20); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&GA, 10, 20); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Retrieve the Basis as a 3*3 matrix of 4th degrees polynoms
   error = rox_essential_get_basis(basis, ref2D, cur2D); ROX_ERROR_CHECK_TERMINATE(error)

   //Compute determinant equation using basis*
   d1 = rox_polynom_p4d2_mul(rox_polynom_p4d2_sub(rox_polynom_p4d1_mul(basis[0][1], basis[1][2]), rox_polynom_p4d1_mul(basis[0][2], basis[1][1])), basis[2][0]);
   d2 = rox_polynom_p4d2_mul(rox_polynom_p4d2_sub(rox_polynom_p4d1_mul(basis[0][2], basis[1][0]), rox_polynom_p4d1_mul(basis[0][0], basis[1][2])), basis[2][1]);
   d3 = rox_polynom_p4d2_mul(rox_polynom_p4d2_sub(rox_polynom_p4d1_mul(basis[0][0], basis[1][1]), rox_polynom_p4d1_mul(basis[0][1], basis[1][0])), basis[2][2]);
   det = rox_polynom_p4d3_add(d1, rox_polynom_p4d3_add(d2, d3));

   // Compute singular values equation using basis

   //E*E'
   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 3; j++)
      {
         EEt[i][j] = rox_polynom_p4d2_zero();

         for (k = 0; k < 3; k++)
         {
            EEt[i][j] = rox_polynom_p4d2_add(EEt[i][j], rox_polynom_p4d1_mul(basis[i][k], basis[j][k]));
         }
      }
   }

   //trace(E*E')
   trace = rox_polynom_p4d2_scale(rox_polynom_p4d2_add(EEt[0][0], rox_polynom_p4d2_add(EEt[1][1], EEt[2][2])), 0.5);

   //EE'-trace(E*E')*I
   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 3; j++)
      {
         if (i == j)
         {
            lambda[i][j] = rox_polynom_p4d2_sub(EEt[i][j], trace);
         }
         else
         {
            lambda[i][j] = EEt[i][j];
         }
      }
   }

   //EE'E - trace(E*E')E
   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 3; j++)
      {
         eigen[i][j] = rox_polynom_p4d3_zero();

         for (k = 0; k < 3; k++)
         {
            eigen[i][j] = rox_polynom_p4d3_add(eigen[i][j], rox_polynom_p4d2_mul(lambda[i][k], basis[k][j]));
         }
      }
   }

   //Fillin matrix
   error = rox_array2d_double_get_data_pointer_to_pointer( &dA, A);

   //Add determinant constraint
   i = 0;
   dA[i][0] = det.xxx;
   dA[i][1] = det.yyy;
   dA[i][2] = det.xxy;
   dA[i][3] = det.xyy;
   dA[i][4] = det.xxz;
   dA[i][5] = det.xxw;
   dA[i][6] = det.yyz;
   dA[i][7] = det.yyw;
   dA[i][8] = det.xyz;
   dA[i][9] = det.xyw;
   dA[i][10] = det.xzz;
   dA[i][11] = det.xzw;
   dA[i][12] = det.xww;
   dA[i][13] = det.yzz;
   dA[i][14] = det.yzw;
   dA[i][15] = det.yww;
   dA[i][16] = det.zzz;
   dA[i][17] = det.zzw;
   dA[i][18] = det.zww;
   dA[i][19] = det.www;

   //Add the 9 eingen values constraints
   i = 1;
   for (k = 0; k < 3; k++)
   {
      for (l = 0; l < 3; l++)
      {
         dA[i][0] = eigen[k][l].xxx;
         dA[i][1] = eigen[k][l].yyy;
         dA[i][2] = eigen[k][l].xxy;
         dA[i][3] = eigen[k][l].xyy;
         dA[i][4] = eigen[k][l].xxz;
         dA[i][5] = eigen[k][l].xxw;
         dA[i][6] = eigen[k][l].yyz;
         dA[i][7] = eigen[k][l].yyw;
         dA[i][8] = eigen[k][l].xyz;
         dA[i][9] = eigen[k][l].xyw;
         dA[i][10] = eigen[k][l].xzz;
         dA[i][11] = eigen[k][l].xzw;
         dA[i][12] = eigen[k][l].xww;
         dA[i][13] = eigen[k][l].yzz;
         dA[i][14] = eigen[k][l].yzw;
         dA[i][15] = eigen[k][l].yww;
         dA[i][16] = eigen[k][l].zzz;
         dA[i][17] = eigen[k][l].zzw;
         dA[i][18] = eigen[k][l].zww;
         dA[i][19] = eigen[k][l].www;

         i++;
      }
   }

   //Transform the A matrix in reduced row echelon form
   error = rox_array2d_double_gauss_pivoting(GA, A); ROX_ERROR_CHECK_TERMINATE ( error );

   //Compute the B matrix using rref(A)
   error = rox_array2d_double_get_data_pointer_to_pointer(&dGA, GA); ROX_ERROR_CHECK_TERMINATE ( error );
   rowe = dGA[4];
   rowf = dGA[5];
   rowg = dGA[6];
   rowh = dGA[7];
   rowi = dGA[8];
   rowj = dGA[9];

   B11.z[3] = -rowf[10];
   B11.z[2] = (rowe[10] - rowf[11]);
   B11.z[1] = (-rowf[12] + rowe[11]);
   B11.z[0] = rowe[12];

   B12.z[3] = -rowf[13];
   B12.z[2] = (rowe[13] - rowf[14]);
   B12.z[1] = (rowe[14] - rowf[15]);
   B12.z[0] = rowe[15];

   B13.z[4] = -rowf[16];
   B13.z[3] = (rowe[16] - rowf[17]);
   B13.z[2] = (rowe[17] - rowf[18]);
   B13.z[1] = (rowe[18] - rowf[19]);
   B13.z[0] = rowe[19];

   B21.z[3] = -rowh[10];
   B21.z[2] = (rowg[10] - rowh[11]);
   B21.z[1] = (-rowh[12] + rowg[11]);
   B21.z[0] = rowg[12];

   B22.z[3] = -rowh[13];
   B22.z[2] = (rowg[13] - rowh[14]);
   B22.z[1] = (rowg[14] - rowh[15]);
   B22.z[0] = rowg[15];

   B23.z[4] = -rowh[16];
   B23.z[3] = (rowg[16] - rowh[17]);
   B23.z[2] = (rowg[17] - rowh[18]);
   B23.z[1] = (rowg[18] - rowh[19]);
   B23.z[0] = rowg[19];

   B31.z[3] = -rowj[10];
   B31.z[2] = (rowi[10] - rowj[11]);
   B31.z[1] = (-rowj[12] + rowi[11]);
   B31.z[0] = rowi[12];

   B32.z[3] = -rowj[13];
   B32.z[2] = (rowi[13] - rowj[14]);
   B32.z[1] = (rowi[14] - rowj[15]);
   B32.z[0] = rowi[15];

   B33.z[4] = -rowj[16];
   B33.z[3] = (rowi[16] - rowj[17]);
   B33.z[2] = (rowi[17] - rowj[18]);
   B33.z[1] = (rowi[18] - rowj[19]);
   B33.z[0] = rowi[19];

   //Compute det(B). The only variable alive is z
   p1 = rox_polynom_p1d7_sub(rox_polynom_p1d7_mul(B23,B12), rox_polynom_p1d7_mul(B13,B22));
   p2 = rox_polynom_p1d7_sub(rox_polynom_p1d7_mul(B13,B21), rox_polynom_p1d7_mul(B23,B11));
   p3 = rox_polynom_p1d6_sub(rox_polynom_p1d6_mul(B11,B22), rox_polynom_p1d6_mul(B12,B21));
   bdet = rox_polynom_p1d10_add(rox_polynom_p1d10_add(rox_polynom_p1d10_73_mul(p1, B31), rox_polynom_p1d10_73_mul(p2, B32)), rox_polynom_p1d10_64_mul(p3, B33));

   //Solution is det(B) = 0
   error = rox_polynomial_roots_sturm(roots, &nbroots, bdet.z, 10); ROX_ERROR_CHECK_TERMINATE ( error );

   for (i = 0; i < nbroots; i++)
   {
      Rox_Double x,y,z,ep1,ep2,ep3;
      error = rox_array2d_double_get_data_pointer_to_pointer( &dE, Ematrices[i]); ROX_ERROR_CHECK_TERMINATE ( error );
      z = roots[i];

      //Given z, estimate x and y
      rox_polynomial_eval(&ep1, p1.z, 7, z);
      rox_polynomial_eval(&ep2, p2.z, 7, z);
      rox_polynomial_eval(&ep3, p3.z, 6, z);
      x = ep1/ep3;
      y = ep2/ep3;

      //Using x,y,z,w=1, compute a possible essential matrix
      for (k = 0; k < 3; k++)
      {
         for (l = 0; l < 3; l++)
         {
            dE[k][l] = x * basis[k][l].x + y * basis[k][l].y + z * basis[k][l].z + basis[k][l].w;
         }
      }
   }

   *nbsolutions = nbroots;

function_terminate:

   rox_matrix_del(&A);
   rox_matrix_del(&GA);

   return error;
}
