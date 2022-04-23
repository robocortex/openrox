//==============================================================================
//
//    OPENROX   : File points_struct.h
//
//    Contents  : API of point_struct module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINTS_STRUCT__
#define __OPENROX_POINTS_STRUCT__

#include <system/memory/datatypes.h>

#ifdef points
//! \ingroup Geometry
//! \addtogroup Point3D
//! @{

//! 3D meters point
struct Rox_Point3D_Double_Struct
{
   //! The x-axis coordinate in meters
   Rox_Double X;

   //! The y-axis coordinate in meters
   Rox_Double Y;

   //! The z-axis coordinate in meters
   Rox_Double Z;
};

//! 3D meters point
struct Rox_Point3D_Sint_Struct
{
   //! The x-axis coordinate in meters
   Rox_Sint X;

   //! The y-axis coordinate in meters
   Rox_Sint Y;

   //! The z-axis coordinate in meters
   Rox_Sint Z;
};

//! 3D meters point structure
typedef struct Rox_Point3D_Sint_Struct Rox_Point3D_Sint_Struct;

//! 3D meters point pointer to structure
typedef struct Rox_Point3D_Sint_Struct * Rox_Point3D_Sint;

//! define
#define ROX_TYPE_POINT3D_SINT (sizeof(struct Rox_Point3D_Sint_Struct) << 2)

//! 3D meters point structure
typedef struct Rox_Point3D_Double_Struct Rox_Point3D_Double_Struct;

//! 3D meters point typedef
typedef struct Rox_Point3D_Double_Struct * Rox_Point3D_Double;

//! define
#define ROX_TYPE_POINT3D_DOUBLE (sizeof(struct Rox_Point3D_Double_Struct) << 2)

//! 3D point float
struct Rox_Point3D_Float_Struct
{
   // 3d meters point
   //! The x-axis coordinate in meters
   Rox_Float X;

   //! The y-axis coordinate in meters
   Rox_Float Y;

   //! The z-axis coordinate in meters
   Rox_Float Z;
};

//! 3D point float structure
typedef struct Rox_Point3D_Float_Struct Rox_Point3D_Float_Struct;

//! 3D point float pointer to strcuture
typedef struct Rox_Point3D_Float_Struct * Rox_Point3D_Float;

//! define
#define ROX_TYPE_POINT3D_FLOAT (sizeof(struct Rox_Point3D_Float_Struct) << 2)

//! define
#define POINT3D_FLOAT_TO_DOUBLE(A,B) {A.X=B.X;A.Y=B.Y;A.Z=B.Z;}

//! @}

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

//! 2D point structure
typedef struct Rox_Point2D_Double_Struct Rox_Point2D_Double_Struct;

//! 2D point pointer to structure
typedef struct Rox_Point2D_Double_Struct * Rox_Point2D_Double;

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

//! 2D point integer typedef
typedef struct Rox_Point2D_Sint_Struct Rox_Point2D_Sint_Struct;

//! 2D point integer pointer to structure
typedef struct Rox_Point2D_Sint_Struct * Rox_Point2D_Sint;

//! define
#define ROX_TYPE_POINT2D_SINT (sizeof(struct Rox_Point2D_Sint_Struct) << 2)

//! 2D point signed short
struct Rox_Point2D_Sshort_Struct
{
   //! The u-axis coordinate in pixel
   Rox_Sshort u;

   //! The v-axis coordinate in pixel
   Rox_Sshort v;
};

//! 2D point typedef
typedef struct Rox_Point2D_Sshort_Struct Rox_Point2D_Sshort_Struct;

//! 2D point pointer to structure
typedef struct Rox_Point2D_Sshort_Struct * Rox_Point2D_Sshort;

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

//! 2D point float structure
typedef struct Rox_Point2D_Float_Struct Rox_Point2D_Float_Struct;

//! 2D point float pointer to structure
typedef struct Rox_Point2D_Float_Struct * Rox_Point2D_Float;

//! define
#define ROX_TYPE_POINT2D_FLOAT (sizeof(struct Rox_Point2D_Float_Struct) << 2)

//! define
#define POINT2D_FLOAT_TO_DOUBLE(A,B) {A.u=B.u;A.v=B.v;}

//! @}

#endif

//! \ingroup Geometry
//! \addtogroup Point5D
//! @{

//! 3D + 2D point float
struct Rox_Point_Float_Struct
{
   // 3d point
   //! The x-axis coordinate in meters
   Rox_Float X;

   //! The y-axis coordinate in meters
   Rox_Float Y;

   //! The z-axis coordinate in meters
   Rox_Float Z;

   // 2d meter point
   //! The x-axis coordinate in meters
   Rox_Float x;

   //! The y-axis coordinate in meters
   Rox_Float y;

   // 2d pixel point
   //! The u-axis coordinate in pixel
   Rox_Float u;

   //! The v-axis coordinate in pixel
   Rox_Float v;
};

//! 3D + 2D point float structure (should be named Rox_Point5D_Float_Struct ?)
typedef struct Rox_Point_Float_Struct Rox_Point_Float_Struct;

//! 3D + 2D point float pointer to structure (should be named Rox_Point5D_Float_Struct ?)
typedef struct Rox_Point_Float_Struct * Rox_Point_Float;

//! define
#define ROX_TYPE_POINT_FLOAT (sizeof(struct Rox_Point_Float_Struct) << 2)

//! 3D + 2D point double
struct Rox_Point_Double_Struct
{
   // 3d point
   //! The x-axis coordinate in meters
   Rox_Double X;

   //! The y-axis coordinate in meters
   Rox_Double Y;

   //! The z-axis coordinate in meters
   Rox_Double Z;

   // 2d meter point
   //! The x-axis coordinate in meters
   Rox_Double x;

   //! The y-axis coordinate in meters
   Rox_Double y;

   // 2d pixel point
   //! The u-axis coordinate in pixel
   Rox_Double u;

   //! The v-axis coordinate in pixel
   Rox_Double v;
};

//! 3D + 2D point double structure (should be named Rox_Point5D_Double_Struct ?)
typedef struct Rox_Point_Double_Struct Rox_Point_Double_Struct;

//! 3D + 2D point double pointer to (should be named Rox_Point5D_Double ?)
typedef struct Rox_Point_Double_Struct * Rox_Point_Double;

//! define
#define ROX_TYPE_POINT_DOUBLE (sizeof(struct Rox_Point_Double_Struct) << 2)

//! @}

#endif
