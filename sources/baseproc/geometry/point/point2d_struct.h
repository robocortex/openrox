//==============================================================================
//
//    OPENROX   : File point2d_struct.h
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

#ifndef __OPENROX_POINT2D_STRUCT__
#define __OPENROX_POINT2D_STRUCT__

#include <system/memory/datatypes.h>

//! \ingroup Geometry
//! \addtogroup Point2D
//! @{

//! 2D point
struct Rox_Point2D_Double_Struct
{
   //! The u-axis coordinate in pixel
   Rox_Double u;

   //! The v-axis coordinate in pixel
   Rox_Double v;
};

//! 2D point structure with real coordinates
typedef struct Rox_Point2D_Double_Struct Rox_Point2D_Double_Struct;
//typedef Rox_Point2D_Double_Struct Rox_Vector2D_Double_Struct;

//! define
#define ROX_TYPE_POINT2D_DOUBLE (sizeof(struct Rox_Point2D_Double_Struct) << 2)

//! 2D point integer
struct Rox_Point2D_Sint_Struct
{
   //! The u-axis coordinate in pixel
   Rox_Sint u;

   //! The v-axis coordinate in pixel
   Rox_Sint v;
};

//! 2D point structure with integer coordinates
typedef struct Rox_Point2D_Sint_Struct Rox_Point2D_Sint_Struct;

//! define
#define ROX_TYPE_POINT2D_SINT (sizeof(struct Rox_Point2D_Sint_Struct) << 2)

//! 2D point integer
struct Rox_Point2D_Uint_Struct
{
   //! The u-axis coordinate in pixel
   Rox_Uint u;

   //! The v-axis coordinate in pixel
   Rox_Uint v;
};

//! 2D point structure with integer coordinates
typedef struct Rox_Point2D_Uint_Struct Rox_Point2D_Uint_Struct;

//! define
#define ROX_TYPE_POINT2D_UINT (sizeof(struct Rox_Point2D_Uint_Struct) << 2)

//! 2D point signed short
struct Rox_Point2D_Sshort_Struct
{
   //! The u-axis coordinate in pixel
   Rox_Sshort u;

   //! The v-axis coordinate in pixel
   Rox_Sshort v;
};

//! 2D point structure with short coordinates
typedef struct Rox_Point2D_Sshort_Struct Rox_Point2D_Sshort_Struct;

//! define
#define ROX_TYPE_POINT2D_SSHORT (sizeof(struct Rox_Point2D_Sshort_Struct) << 2)

//! 2D point float
struct Rox_Point2D_Float_Struct
{
   // 2d pixel point
   //! The u-axis coordinate in pixel
   Rox_Float u;
   //! The v-axis coordinate in pixel
   Rox_Float v;
};

//! 2D point structure with float coordinates
typedef struct Rox_Point2D_Float_Struct Rox_Point2D_Float_Struct;

//! define
#define ROX_TYPE_POINT2D_FLOAT (sizeof(struct Rox_Point2D_Float_Struct) << 2)

//! define
#define POINT2D_FLOAT_TO_DOUBLE(A,B) {A.u=B.u;A.v=B.v;}

//! @}

#endif
