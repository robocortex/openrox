//==============================================================================
//
//    OPENROX   : File ellipse_2d_struct.h
//
//    Contents  : Structures of 2D ellipse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ELLIPSE_2D_STRUCT__
#define __OPENROX_ELLIPSE_2D_STRUCT__

#include <system/memory/datatypes.h>

//! \ingroup Geometry
//! \addtogroup Ellipse2D
//! @{

//! 2D ellipse (in normalized or pixel coordinates)
//! Define a 2D ellipse as follows
//! transpose(p-pc)* S * (p-pc) = 1
//! where
//!   p = [x;y]                  % point on the ellipse
//!   pc = [xc;yc]               % center of the ellipse
//!   S = [nxx, nxy; nxy, nyy]   % symmetric matrix
struct Rox_Ellipse2D_Double_Struct
{
   //! The ellipse center coordinate along the x-asis
   Rox_Double xc;

   //! The ellipse center coordinate along the y-asis
   Rox_Double yc;
   
   //! The a11 entry of the summetric matric
   Rox_Double nxx;
      
   //! The a12 entry of the summetric matric
   Rox_Double nyy;
      
   //! The a22 entry of the summetric matric
   Rox_Double nxy;
};

//! 2D ellipse structure
typedef struct Rox_Ellipse2D_Double_Struct Rox_Ellipse2D_Double_Struct;

typedef struct Rox_Ellipse2D_Double_Struct Rox_Ellipse2D_Parametric_Struct;

//! define
#define ROX_TYPE_ELLIPSE2D_DOUBLE (sizeof(struct Rox_Ellipse2D_Double_Struct) << 2)

//! @}

#endif
