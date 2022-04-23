//==============================================================================
//
//    OPENROX   : File search_edge.h
//
//    Contents  : API of search edge module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SEARCH_EDGE__
#define __OPENROX_SEARCH_EDGE__

#include <generated/array2d_point2d_double.h>
#include <generated/dynvec_point2d_double.h>

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <generated/dynvec_point2d_uint.h>
#include <generated/dynvec_double.h>

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/image/image.h>

//! \ingroup Tracking
//! \addtogroup Search_Edge
//! @{

//! Search Edge Structure 
struct Rox_Search_Edge_Struct
{
   //! Coordinates of edge
   Rox_Point2D_Double_Struct _coords;

   //! Angle model of the edge 
   Rox_Double _angle;

   //! Possible sites
   // Rox_DynVec_Point2D_Double _sites;

   //! probabilities
   // Rox_DynVec_Double _likelihoods;

   //! Gradient norm 
   Rox_Double _norm_gradient;

   //! Convolved result 
   Rox_Double _convolution;

   //! Sign of convolution 
   Rox_Sint _mask_sign;

   //! pixel search range on both sides of segment
   Rox_Sint _search_range;

   //! Scale threshold 
   Rox_Sint _scale_threshold;

   //! Temporary convolutions
   //Rox_Array2D_Point2D_Double _convolutions;

   //! Gradient scale and angle 
   Rox_Array2D_Double _gradient_scale;
   Rox_Array2D_Double _gradient_angle;

   //! Scanline coordinates
   Rox_Array2D_Point2D_Double _scanline;

   //! Scanline coordinates in matrix form

   //! _scanline_u_row = [ u1, u2, ..., un ];
   Rox_Array2D_Double _scanline_u_row;

   //! _scanline_v_row = [ v1, v2, ..., vn ];
   Rox_Array2D_Double _scanline_v_row;
};

//! Structure
typedef struct Rox_Search_Edge_Struct * Rox_Search_Edge;

//! Create Moving Edges object
//! \param  [out]  search_edge           The pointer to the search segment object
//! \param  [in ]  searchrange           The pixel search range on both sides of segment
//! \param  [in ]  threshold_response    The convolutions threshold response
//! \return An error code
ROX_API Rox_ErrorCode rox_search_edge_new (
   Rox_Search_Edge * search_edge, 
   const Rox_Uint searchrange, 
   const Rox_Uint threshold_response);

//! Delete Moving Edges object
//! \param  [out]  search_edge                  The pointer to the search segment object
//! \return An error code
ROX_API Rox_ErrorCode rox_search_edge_del(Rox_Search_Edge * search_edge);

//! Track search segment current position
//! \param  [out] search_edge                  The current search segment object
//! \param  [in]  image                The current image
//! \param  [in]  point                The starting point
//! \return An error code
ROX_API Rox_ErrorCode rox_search_edge_track ( 
   Rox_Search_Edge search_edge, 
   Rox_Image image, 
   Rox_Point2D_Double point
);

ROX_API Rox_ErrorCode rox_search_edge_track_gradient (
   Rox_Search_Edge search_edge,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle,
   const Rox_Point2D_Double point // The starting point of the search
);

ROX_API Rox_ErrorCode rox_scanline ( Rox_Array2D_Point2D_Double scanline, Rox_Point2D_Double point, Rox_Point2D_Double normal ); //, bounds)
ROX_API Rox_ErrorCode rox_scanline_row ( Rox_Array2D_Double scanline_u, Rox_Array2D_Double scanline_v, Rox_Point2D_Double point, Rox_Point2D_Double normal ); //, bounds)
ROX_API Rox_ErrorCode rox_scanline_matrix ( Rox_Array2D_Double scanline_u, Rox_Array2D_Double scanline_v, Rox_Array2D_Double points2d, Rox_Array2D_Double lines2d ); //, bounds)

ROX_API Rox_ErrorCode rox_scan_image_scale_angle ( 
   Rox_Array2D_Double scan_gradient_scale, 
   Rox_Array2D_Double scan_gradient_angle, 
   Rox_Array2D_Point2D_Double scanline, 
   Rox_Image image
);

ROX_API Rox_ErrorCode rox_scan_scale_angle ( 
   Rox_Array2D_Double scan_gradient_scale, 
   Rox_Array2D_Double scan_gradient_angle, 
   Rox_Array2D_Point2D_Double scanline, 
   Rox_Array2D_Uint gradient_scale, 
   Rox_Array2D_Float gradient_angle
);


ROX_API Rox_ErrorCode rox_scan_image_scale_angle_matrix ( 
   Rox_Array2D_Double scan_gradient_scale, 
   Rox_Array2D_Double scan_gradient_angle, 
   Rox_Array2D_Double scanline_u, 
   Rox_Array2D_Double scanline_v, 
   Rox_Image image
);

ROX_API Rox_ErrorCode rox_scan_scale_angle_matrix ( 
   Rox_Array2D_Double scan_gradient_scale, 
   Rox_Array2D_Double scan_gradient_angle, 
   Rox_Array2D_Double scanline_u, 
   Rox_Array2D_Double scanline_v, 
   Rox_Array2D_Uint gradient_scale, 
   Rox_Array2D_Float gradient_angle
);

ROX_API Rox_ErrorCode rox_find_closest_scale_above_threshold_angle_isinrange (Rox_Point2D_Double point_meas, Rox_Array2D_Point2D_Double scanline, Rox_Array2D_Double gradient_scale, Rox_Array2D_Double gradient_angle, Rox_Double scale_threshold, Rox_Double angle_model, Rox_Double angle_range);
ROX_API Rox_ErrorCode rox_find_closest_scale_peak_above_threshold_angle_isinrange (Rox_Point2D_Double point_meas, Rox_Array2D_Point2D_Double scanline, Rox_Array2D_Double gradient_scale, Rox_Array2D_Double gradient_angle, Rox_Double scale_threshold, Rox_Double angle_model, Rox_Double angle_range);

ROX_API Rox_ErrorCode rox_find_closest_scale_above_threshold_angle_isinrange_matrix (Rox_Point2D_Double point_meas, Rox_Array2D_Double scanline_u, Rox_Array2D_Double scanline_v, Rox_Array2D_Double gradient_scale, Rox_Array2D_Double gradient_angle, Rox_Double scale_threshold, Rox_Double angle_model, Rox_Double angle_range);
ROX_API Rox_ErrorCode rox_find_closest_scale_peak_above_threshold_angle_isinrange_row (Rox_Point2D_Double point_meas, Rox_Array2D_Double scanline_u, Rox_Array2D_Double scanline_v, Rox_Array2D_Double gradient_scale, Rox_Array2D_Double gradient_angle, Rox_Double scale_threshold, Rox_Double angle_model, Rox_Double angle_range);

//! @} 

#endif //__OPENROX_SEARCH_EDGE_OBJECT__
