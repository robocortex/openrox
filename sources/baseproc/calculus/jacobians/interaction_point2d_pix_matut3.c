//==============================================================================
//
//    OPENROX   : File interaction_point2d_pix_matut3.c
//
//    Contents  : Implementation of interaction_point2d_pix_matut3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "interaction_point2d_pix_matut3.h"

#include <generated/array2d_double.h>
#include <inout/system/errors_print.h>
#include <baseproc/geometry/point/point2d_struct.h>

// NB : cu and cv unused if Kddl > 2 
Rox_ErrorCode rox_jacobian_points_2d_campar (
   Rox_Array2D_Double L, 
   Rox_Point2D_Double pts, 
   Rox_Double cu, 
   Rox_Double cv, 
   Rox_Sint nbp, 
   Rox_Sint Kddl
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !L || !pts )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   switch(Kddl)
   {
      case 1: // unk : f                 
      {
         error = rox_jacobian_points_2d_campar_fu ( L, pts, cu ,cv, nbp ); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }

      case 2: // unk : fu, fv            
      {
         error = rox_jacobian_points_2d_campar_fu_fv(L, pts, cu,cv, nbp); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }

      case 3: // unk : f          cu, cv 
      {
         error = rox_jacobian_points_2d_campar_fu_cu_cv(L, pts, nbp); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }

      case 4: // unk : fu, fv,    cu, cv 
      {
         error = rox_jacobian_points_2d_campar_fu_fv_cu_cv(L, pts, nbp); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }

      case 5:
      {
         error = rox_jacobian_points_2d_campar_fu_fv_cu_cv_s(L, pts, nbp); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }

      default:
      {
         error = ROX_ERROR_INVALID_VALUE;
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }
   }

function_terminate:
   return error;
}

// K = [fu, s, cu; 0, fv, cv; 0, 0, 1] 
Rox_ErrorCode rox_jacobian_points_2d_campar_fu_fv_cu_cv_s (
   Rox_Array2D_Double L, 
   const Rox_Point2D_Double pts, 
   const Rox_Sint nbp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!L || !pts) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(L, 2*nbp, 5); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dL, L);

   for ( Rox_Sint i = 0; i < nbp; i++)
   {
      // 1st row 
      dL[2*i][0] = pts[i].u;
      dL[2*i][1] = pts[i].v;
      dL[2*i][2] = 1.0 ;
      dL[2*i][3] = 0.0 ;
      dL[2*i][4] = 0.0 ;

      // 2nd row 
      dL[2*i+1][0] = 0.0 ;
      dL[2*i+1][1] = 0.0 ;
      dL[2*i+1][2] = 0.0 ;
      dL[2*i+1][3] = pts[i].v;
      dL[2*i+1][4] = 1.0 ;
   }

function_terminate:
   return error;
}

// K = [fu, 0, cu; 0, fv, cv; 0, 0, 1] 
Rox_ErrorCode rox_jacobian_points_2d_campar_fu_fv_cu_cv (
   Rox_Array2D_Double L, 
   const Rox_Point2D_Double pts, 
   const Rox_Sint nbp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !L || !pts )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size (L, 2*nbp, 4 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dL, L );

   for ( Rox_Sint i = 0; i < nbp; i++)
   {
      // 1st row 
      dL[2*i][0] = pts[i].u;
      dL[2*i][1] = 1.0 ;
      dL[2*i][2] = 0.0 ;
      dL[2*i][3] = 0.0 ;

      // 2nd row 
      dL[2*i+1][0] = 0.0 ;
      dL[2*i+1][1] = 0.0 ;
      dL[2*i+1][2] = pts[i].v;
      dL[2*i+1][3] = 1.0 ;
   }

function_terminate:
   return error;
}

// K = [f, 0, cu; 0, f, cv; 0, 0, 1] 
Rox_ErrorCode rox_jacobian_points_2d_campar_fu_cu_cv (
   Rox_Array2D_Double L, 
   Rox_Point2D_Double pts, 
   Rox_Sint nbp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !L || !pts ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size ( L, 2*nbp, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dL, L );

   for ( Rox_Sint i = 0; i < nbp; i++)
   {
      // 1st row 
      dL[2*i][0] = pts[i].u;
      dL[2*i][1] = 1.0 ;
      dL[2*i][2] = 0.0 ;

      // 2nd row 
      dL[2*i+1][0] = pts[i].v;
      dL[2*i+1][1] = 0.0 ;
      dL[2*i+1][2] = 1.0 ;
   }

function_terminate:
   return error;
}

// K = [fu, 0, (cu); 0, fv, (cv); 0, 0, 1] 
Rox_ErrorCode rox_jacobian_points_2d_campar_fu_fv ( 
   Rox_Array2D_Double L, 
   const Rox_Point2D_Double pts, 
   const Rox_Double cu, 
   const Rox_Double cv, 
   const Rox_Sint nbp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!L || !pts) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(L, 2*nbp, 2); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dL, L);

   for ( Rox_Sint i = 0; i < nbp; i++)
   {
      // 1st row 
      dL[2*i][0] = pts[i].u - cu;
      dL[2*i][1] = 0.0 ;

      // 2nd row 
      dL[2*i+1][0] = 0.0 ;
      dL[2*i+1][1] = pts[i].v - cv;
   }

function_terminate:
   return error;
}

// K = [f, 0, (cu); 0, f, (cv); 0, 0, 1] 
Rox_ErrorCode rox_jacobian_points_2d_campar_fu (
   Rox_Array2D_Double L, 
   const Rox_Point2D_Double pts, 
   const Rox_Double cu, 
   const Rox_Double cv, 
   const Rox_Sint nbp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !L || !pts )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(L, 2*nbp, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dL = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dL, L);

   for ( Rox_Sint i = 0; i < nbp; i++)
   {
      // 1st row 
      dL[2*i  ][0] = pts[i].u - cu;

      // 2nd row 
      dL[2*i+1][0] = pts[i].v - cv;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_campar_ddl ( 
   Rox_Array2D_Double xK, 
   const Rox_Array2D_Double xK_ddl, 
   Rox_Double cu, 
   Rox_Double cv, 
   const Rox_Sint ddl
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double fu = 0.0, fv = 0.0, su = 0.0;

   if(!xK || !xK_ddl)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   switch(ddl)
   {
      case 1: // unk : fu               
      {
         error = rox_array2d_double_get_value(&fu, xK_ddl, 0, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 0, 0, +fu); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 1, 0, 0.0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 2, 0, -fu*cu); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 3, 0, +fu); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 4, 0, -fu*cv); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 5, 0, 0.0);
         ROX_ERROR_CHECK_TERMINATE ( error );

         break;
      }
      case 2: // unk : fu, fv           
      {
         error = rox_array2d_double_get_value ( &fu, xK_ddl, 0, 0 ); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_get_value ( &fv, xK_ddl, 1, 0 ); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 0, 0, +fu ); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 1, 0, 0.0 ); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 2, 0, -fu*cu ); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 3, 0, +fv ); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 4, 0, -fv*cv ); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 5, 0, 0.0 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         break;
      }
      case 3: // unk : fu,        cu, cv 
      {
         error = rox_array2d_double_get_value (&fu, xK_ddl, 0, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_get_value (&cu, xK_ddl, 1, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_get_value (&cv, xK_ddl, 2, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 0, 0, fu); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 1, 0, 0.0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 2, 0, cu); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 3, 0, fu); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 4, 0, cv); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value ( xK, 5, 0, 0.0);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }
      case 4: // unk : fu, fv      cu, cv 
      {
         error = rox_array2d_double_get_value(&fu, xK_ddl, 0, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_get_value(&cu, xK_ddl, 1, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_get_value(&fv, xK_ddl, 2, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_get_value(&cv, xK_ddl, 3, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 0, 0, fu); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 1, 0, 0.0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 2, 0, cu); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 3, 0, fv); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 4, 0, cv); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 5, 0, 0.0);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }
      case 5: // unk : fu, fv, cu, cv, su
      {
         error = rox_array2d_double_get_value(&fu, xK_ddl, 0, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_get_value(&su, xK_ddl, 1, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_get_value(&cu, xK_ddl, 2, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_get_value(&fv, xK_ddl, 3, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_get_value(&cv, xK_ddl, 4, 0); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 0, 0, fu); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 1, 0, su); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 2, 0, cu); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 3, 0, fv); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 4, 0, cv); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(xK, 5, 0, 0.0);
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }
      default:
      {
         error = ROX_ERROR_INVALID_VALUE;
         ROX_ERROR_CHECK_TERMINATE ( error );
         break;
      }
   }

function_terminate:
   return error;
}
