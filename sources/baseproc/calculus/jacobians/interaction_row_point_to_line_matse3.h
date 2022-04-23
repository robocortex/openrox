//==============================================================================
//
//    OPENROX   : File interaction_row_point_to_line_matse3.h
//
//    Contents  : API of interaction_row_point_to_line_matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INTERACTION_ROW_POINT_TO_LINE_MATSE3__
#define __OPENROX_INTERACTION_ROW_POINT_TO_LINE_MATSE3__

#include <baseproc/geometry/line/line3d.h>
#include <baseproc/geometry/line/line2d.h>
#include <baseproc/maths/linalg/matrix.h>

//! \ingroup Interactions
//! \addtogroup Interaction_MatSE3
//! @{

//! \brief Interaction row of the signed distance point to line relative to SE3
//! \remark 3 dof translation first, then 3 dof rotation
//! \param  [out]  L_row          row of the interaction matrix
//! \param  [in ]  line3D         The farthest plane defining the model 3D line in camera frame
//! \param  [in ]  line2D         The model 2D line (projection in meters of the 3D line in the normlized image plane)
//! \param  [in ]  x              x coordinates of the measured point (in meters)
//! \param  [in ]  y              y coordinates of the measured point (in meters)
ROX_API Rox_ErrorCode rox_interaction_row_point_to_line_matse3 (
   Rox_Matrix L_row, 
   const Rox_Line3D_Planes line3D, 
   const Rox_Line2D_Normal line2D, 
   const Rox_Double x, 
   const Rox_Double y
);

ROX_API Rox_ErrorCode rox_line3d_get_interaction_matrix_point_to_line_parameters ( Rox_Double * lambda_r, Rox_Double * lambda_t, Rox_Line3D_Planes line3d );

// The parameter lambda_rho computed from the 3D line
// The parameter lambda_theta computed from the 3D line
// Rho parameter defining the model 2D line in camera frame
// Theta parameter defining the model 2D line in camera frame
// The x coordinates (in meters) of the measured point
// The y coordinates (in meters) of the measured point
ROX_API Rox_ErrorCode rox_ansi_interaction_row_point_to_line_matse3(
   double* L_row_data,
   double   lr,
   double   lt,
   double   rho,
   double   theta,
   double   x,
   double   y
);


ROX_API Rox_ErrorCode rox_ansi_interaction_rho_theta_matse3(
   double* Lr_data, // The interaction matrix d rho   / dt = Lr * v 
   double* Lt_data, // The interaction matrix d theta / dt = Lt * v 
   double   lr,      // The parameter lambda_rho computed from the 3D line
   double   lt,      // The parameter lambda_theta computed from the 3D line
   double   rho,     // The parameter rho defining the model 2D line in camera frame
   double   ct,      // The parameter cos(theta) defining the model 2D line in camera frame
   double   st       // The parameter cos(theta) defining the model 2D line in camera frame
);
//! @} 

#endif // __OPENROX_INTERACTION_ROW_POINT_TO_LINE_MATSE3__
