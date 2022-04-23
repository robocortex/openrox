//==============================================================================
//
//    OPENROX   : File point2d.h
//
//    Contents  : API of point2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINT2D__
#define __OPENROX_POINT2D__

#include <system/memory/datatypes.h>

//! \ingroup Geometry
//! \addtogroup Point2D
//! @{

//! 2D point pointer to structure
typedef struct Rox_Point2D_Double_Struct * Rox_Point2D_Double;
//typedef struct Rox_Vector2D_Double_Struct * Rox_Vector2D_Double;

//! 2D point integer pointer to structure
typedef struct Rox_Point2D_Sint_Struct * Rox_Point2D_Sint;

//! 2D point unsiagned integer pointer to structure
typedef struct Rox_Point2D_Uint_Struct * Rox_Point2D_Uint;

//! 2D point pointer to structure
typedef struct Rox_Point2D_Sshort_Struct * Rox_Point2D_Sshort;

//! 2D point float pointer to structure
typedef struct Rox_Point2D_Float_Struct * Rox_Point2D_Float;

//! define
#define POINT2D_FLOAT_TO_DOUBLE(A,B) {A.u=B.u;A.v=B.v;}

//! @}

#endif // __OPENROX_POINT2D__
