//==============================================================================
//
//    OPENROX   : File line2d_struct.h
//
//    Contents  : Structure of line module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE2D_STRUCT__
#define __OPENROX_LINE2D_STRUCT__

#include <system/memory/datatypes.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/plane/plane_struct.h>

//! \ingroup Euclidean_Geometry
//! \defgroup Line Line

//! \addtogroup Line
//! @{

//! Structure of a 2D line in normal form (Hesse normal form) 
struct Rox_Line2D_Normal_Struct
{
   //! Polar coordinates, rho
   Rox_Double rho;

   //! Polar coordinates, theta
   Rox_Double theta;
};

//! Typedef of the structure in normal form
typedef struct Rox_Line2D_Normal_Struct Rox_Line2D_Normal_Struct;

//! Define the size of the structure
#define ROX_TYPE_LINE2D_NORMAL_DOUBLE (sizeof(struct Rox_Line2D_Normal_Struct) << 2)

//! Structure of a 2D line in homogeneous form : l = [a b c];
struct Rox_Line2D_Homogeneous_Struct
{
   //! Homogeneous coordinate a = cos(theta)
   Rox_Double a;

   //! Homogeneous coordinate b = sin(theta)
   Rox_Double b;
   
   //! Homogeneous coordinate c = -rho
   Rox_Double c;
};

//! Typedef of the structure in homogeneous form
typedef struct Rox_Line2D_Homogeneous_Struct Rox_Line2D_Homogeneous_Struct;

//! Define
#define ROX_TYPE_LINE2D_HOMOGENEOUS_DOUBLE (sizeof(struct Rox_Line2D_Homogeneous_Struct) << 2)

//! Structure of a 2D line in parametric form : [p.u;p.v] + lammda * [d.u;d.v]
struct Rox_Line2D_Parametric_Struct
{
   //! Starting point p
   Rox_Point2D_Double_Struct p;
   //! Direction d
   Rox_Point2D_Double_Struct d;
};

//! Typedef of the structure in homogeneous form
typedef struct Rox_Line2D_Parametric_Struct Rox_Line2D_Parametric_Struct;

//! Define
#define ROX_TYPE_LINE2D_PARAMETRIC_DOUBLE (sizeof(struct Rox_Line2D_Parametric_Struct) << 2)

//! @}

#endif
