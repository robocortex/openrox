//==============================================================================
//
//    OPENROX   : File odometry_cadmodel.h
//
//    Contents  : API of odometry_cadmodel module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_CADMODEL__
#define __OPENROX_ODOMETRY_CADMODEL__

#include <generated/array2d_double.h>

#include <generated/objset_edge_segment.h>
#include <generated/objset_edge_ellipse.h>
#include <generated/objset_edge_cylinder.h>

#include <generated/objset_edge_segment_struct.h>
#include <generated/objset_edge_ellipse_struct.h>
#include <generated/objset_edge_cylinder_struct.h>

#include <baseproc/image/image.h>
#include <baseproc/maths/linalg/matut3.h>

#include <core/tracking/edge/tracking_segment.h>
#include <core/tracking/edge/edge_segment_site.h>
#include <core/tracking/edge/edge_segment.h>

#include <core/tracking/edge/tracking_ellipse.h>
#include <core/tracking/edge/edge_ellipse_site.h>
#include <core/tracking/edge/edge_ellipse.h>

#include <core/tracking/edge/tracking_cylinder.h>
#include <core/tracking/edge/edge_cylinder_site.h>
#include <core/tracking/edge/edge_cylinder.h>

//! \ingroup Odometry
//! \addtogroup Odometry_CadModel
//! @{

//! CAD Model based odometry structure
struct Rox_Odometry_CadModel_Struct
{
   //! odometry estimated pose 
   Rox_MatSE3 pose;

   //! odometry virtual camera calibration 
   Rox_MatUT3 calibration;

   //! Set of segments 
   Rox_ObjSet_Edge_Segment objset_edge_segment;

   //! Edge tracker 
   Rox_Tracking_Segment tracking_segment;
   
   //! Set of ellipses 
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse;

   //! Ellipse tracker 
   Rox_Tracking_Ellipse tracking_ellipse;

   //! Set of cylinders
   Rox_ObjSet_Edge_Cylinder objset_edge_cylinder;

   //! Cylinder tracker
   Rox_Tracking_Cylinder tracking_cylinder;
};

//! CAD Model based odometry structure
typedef struct Rox_Odometry_CadModel_Struct * Rox_Odometry_CadModel;

//! Create a new odometry with cad model constraint object
//! \param [out] odometry_cadmodel     The pointer to the newly created object
//! \return an error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_new (
   Rox_Odometry_CadModel * odometry_cadmodel,
   const Rox_Double fu,
   const Rox_Double fv,
   const Rox_Double cu,
   const Rox_Double cv,
   const Rox_Sint search_range, 
   const Rox_Double contrast_threshold
);

//! Delete an odometry object with cad model constraint
//! \param [out] odometry_cadmodel     The pointer to the created object
//! \return an error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_del(
   Rox_Odometry_CadModel * odometry_cadmodel
   );

//! Append a 3D Segment to the cadmodel
//! \param [out]  odometry_cadmodel       The odometry object
//! \param [in]   segment                 The 3D segment to add in world coordinates
//! \param [in]   sampling_step           The sampling step for a segment
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_add_segment(
   Rox_Odometry_CadModel odometry_cadmodel, 
   Rox_Segment3D segment, 
   const Rox_Double sampling_step);

//! Append a 3D ellipse to the odometry object
//! \param  [out]  odometry_cadmodel       The odometry object
//! \param  [in]   ellipse                 The 3D ellipse to add in world coordinates
//! \param  [in]   sampling_step           The sampling step for an ellipse
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_add_ellipse(
   Rox_Odometry_CadModel odometry_cadmodel, 
   Rox_Ellipse3D ellipse, 
   const Rox_Double sampling_step);

//! Append a 3D Cylinder to the set
//! \param  [in] odometry_cadmodel         The odometry object
//! \param  [in] cylinder                  The 3D cylinder to add in world coordinates
//! \param  [in] sampling_step             The sampling step for a cylinder
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_add_cylinder(
   Rox_Odometry_CadModel odometry_cadmodel, 
   Rox_Cylinder3D cylinder, 
   const Rox_Double sampling_step);

//! Compute the pose to match the ellipses in the image
//! \param  [out]  odometry_ellipses       The odometry object
//! \param  [in]   image                   The image to track into
//! \param  [in]   max_iters               The number of iteration to make vvs estimation
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_make(
   Rox_Odometry_CadModel odometry_cadmodel, 
   const Rox_Image image, 
   const Rox_Sint max_iters);

//! Compute the global errors vector, in order to build a covariance matrix for instance
//! \param  [out]  res_errors              The resulting errors
//! \param  [in ]  odometry_cadmodel       The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_build_new_errors (
   Rox_Array2D_Double * res_errors, 
   const Rox_Odometry_CadModel odometry_cadmodel
);

//! Compute the global weights matrix, in order to build a covariance matrix for instance
//! \param  [out]  res_weights             The resulting weights
//! \param  [in ]  errors                  Previously computed global errors (result from rox_odometry_cadmodel_build_new_errors for instance)
//! \param  [in ]  odometry_cadmodel       The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_build_new_weights (
   Rox_Array2D_Double * res_weights, 
   const Rox_Array2D_Double errors, 
   const Rox_Odometry_CadModel odometry_cadmodel
);

//! Compute the global interaction matrix, in order to build a covariance matrix for instance
//! \param  [out]  res_interaction         The resulting interaction matrix
//! \param  [in ]  odometry_cadmodel       The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_build_new_interaction_matrix (
   Rox_Array2D_Double * res_interaction, 
   const Rox_Odometry_CadModel odometry_cadmodel
);

//! Compute the global weights matrix, in order to build a covariance matrix for instance
//! \param  [out]  cov                     The resulting covariance matrix pointer (will be allocated)
//! \param  [in ]  pose                    The computed pose matrix
//! \param  [in ]  errors                  The errors vector
//! \param  [in ]  interaction             The interaction matrix
//! \param  [in ]  weights                 The weights vector
//! \param  [in ]  odometry_cadmodel       The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_build_new_covariance (
   Rox_Array2D_Double * cov, 
   const Rox_MatSE3 pose, 
   const Rox_Array2D_Double errors, 
   const Rox_Array2D_Double interaction, 
   const Rox_Array2D_Double weights, 
   const Rox_Odometry_CadModel odometry_cadmodel
);


//! Get the score (the best score is 0)
//! \param  [out] score             The score
//! \param  [in]  odometry_cadmodel The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_cadmodel_get_score (
   Rox_Double * score, 
   const Rox_Odometry_CadModel odometry_cadmodel
);


ROX_API Rox_ErrorCode rox_odometry_cadmodel_get_results (
   Rox_Double * score, 
   const Rox_MatSE3 pose, 
   const Rox_Odometry_CadModel Odometry_CadModel
);

//! @}

#endif
