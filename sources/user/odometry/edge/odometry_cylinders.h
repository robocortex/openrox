//==============================================================================
//
//    OPENROX   : File odometry_cylinders.h
//
//    Contents  : API of odometry_cylinders module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_CYLINDERS__
#define __OPENROX_ODOMETRY_CYLINDERS__

#include <generated/objset_edge_cylinder.h>
#include <generated/objset_edge_cylinder_struct.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/image/image.h>

#include <core/tracking/edge/tracking_cylinder.h>
#include <core/tracking/edge/edge_cylinder_site.h>
#include <core/tracking/edge/edge_cylinder.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Cylinders
//! @{

//! 3D Cylinders based odometry structure
struct Rox_Odometry_Cylinders_Struct
{
   //! odometry estimated pose 
   Rox_MatSE3 pose;

   //! odometry virtual camera calibration 
   Rox_MatUT3 calibration;

   //! Set of cylinders 
   Rox_ObjSet_Edge_Cylinder objset_edge_cylinder;

   //! Moving edge tracker 
   Rox_Tracking_Cylinder tracker;
};

//! CAD Model based odometry structure
typedef struct Rox_Odometry_Cylinders_Struct * Rox_Odometry_Cylinders;

//! Create a new odometry with 3D cylinders model constraint object
//! \param  [out]  odometry_cylinders     a pointer to the newly created object
//! \param  [in ]  search_range           distance in number of pixels to search from each site
//! \param  [in ]  contrast_threshold     contrast threshold to validate a convolution result
//! \return an error code
ROX_API Rox_ErrorCode rox_odometry_cylinders_new (
   Rox_Odometry_Cylinders * odometry_cylinders,
   const Rox_Double fu,
   const Rox_Double fv,
   const Rox_Double cu,
   const Rox_Double cv,
   const Rox_Sint search_range, 
   const Rox_Double contrast_threshold
);

//! Delete an odometry odometry_cylindersect with 3D cylinders model constraint
//! \param  [out]  odometry_cylinders a pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cylinders_del (
   Rox_Odometry_Cylinders * odometry_cylinders
);

//! Append a 3D Cylinder to the set
//! \param  [out]  odometry_cylinders       odometry object
//! \param  [in ]  cylinder                 the 3D cylinder to add in world coordinates
//! \param  [in ]  sampling_step            The sampling step for a cylinder
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cylinders_add_cylinder (
   Rox_Odometry_Cylinders odometry_cylinders, 
   Rox_Cylinder3D cylinder3d, 
   const Rox_Double sampling_step
);

//! Append a 3D Cylinder to the set
//! \param  [out]  odometry_cylinders  odometry object
//! \param  [in ]  image               the image to track into
//! \param  [in ]  max_iters           number of iteration to make vvs estimation
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cylinders_make (
   Rox_Odometry_Cylinders odometry_cylinders, 
   const Rox_Image image, 
   const Rox_Sint max_iters
);

//! Get score
//! \param  [out] score 
//! \param  [in ] odometry_cylinders
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cylinders_get_score (
   Rox_Double * score, 
   const Rox_Odometry_Cylinders odometry_cylinders
);

//! @} 

#endif
