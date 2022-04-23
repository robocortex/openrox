//==============================================================================
//
//    OPENROX   : File moving_edge.h
//
//    Contents  : API of moving_edge module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MOVING_EDGE__
#define __OPENROX_MOVING_EDGE__

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_double.h>

#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/image/image.h>

#include <core/tracking/edge/moving_edge_params.h>

//! \ingroup Tracking
//! \addtogroup MovingEdge
//! @{

//! Moving edge 
struct Rox_Moving_Edge_Struct
{
   //! Parameters 
   Rox_Moving_Edge_Params _params;

   //! Coordinates of edge
   Rox_Point2D_Double_Struct _coords;

   //! angle of the edge
   Rox_Double _angle;

   //! Possible sites
   Rox_DynVec_Point2D_Double _sites;

   //! probabilities
   Rox_DynVec_Double _likelihoods;

   //! Gradient norm 
   Rox_Double _norm_gradient;

   //! Convolved result 
   Rox_Double _convolution;

   //! Sign of convolution 
   Rox_Sint _mask_sign;
};

//! Structure
typedef struct Rox_Moving_Edge_Struct * Rox_Moving_Edge;

//! Create Moving Edges object
//! \param [out]  moving_edge       The pointer to the moving edge object
//! \param [in]   params            The parameters for moving edge
//! \return an error code
ROX_API Rox_ErrorCode rox_moving_edge_new(Rox_Moving_Edge * moving_edge, Rox_Moving_Edge_Params params);

//! Delete Moving Edges object
//! \param [out] moving_edge        The pointer to the moving edge object
//! \return an error code
ROX_API Rox_ErrorCode rox_moving_edge_del(Rox_Moving_Edge * moving_edge);

//! Set moving edge current position
//! \param [out] moving_edge        The current moving edge
//! \param [in] u                   The current u coordinate
//! \param [in] v                   The current v coordinate
//! \param [in] alpha               The current alpha coordinate
//! \param [in] prevconvolution     The previous convolution
//! \return an error code
ROX_API Rox_ErrorCode rox_moving_edge_set_coordinates(Rox_Moving_Edge moving_edge, Rox_Double u, Rox_Double v, Rox_Double alpha, Rox_Double prevconvolution);

//! Track moving edge current position
//! \param [out] moving_edge        The current moving edge
//! \param [in] image               The current image
//! \param [in] check_contrast      The
//! \return an error code
ROX_API Rox_ErrorCode rox_moving_edge_track(Rox_Moving_Edge moving_edge, Rox_Image image, Rox_Uint check_contrast);

//! @} 

#endif
