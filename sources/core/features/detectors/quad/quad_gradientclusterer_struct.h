//==============================================================================
//
//    OPENROX   : File quad_gradientclusterer_struct.h
//
//    Contents  : API of quad_gradientclusterer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QUAD_DETECTION_CLUSTERER_STRUCT__
#define __OPENROX_QUAD_DETECTION_CLUSTERER_STRUCT__

#include <generated/array2d_uint.h>
#include <generated/dynvec_quad.h>
#include <generated/dynvec_orientedimagepoint.h>
#include <generated/objset_dynvec_orientedimagepoint.h>

#include <core/features/detectors/edges/edgedraw.h>
#include <core/features/detectors/edges/edgepreproc_gray.h>
#include <core/features/detectors/edges/edgepostproc_ac.h>
#include <core/features/detectors/edges/edgepostproc_normal.h>

#define MINMAG 4161 // Multiply by 128 to get the [0;255] equivalent gradient threshold - best 0.004 - smaller threshold more segments detected 
#define WEIGHT_SCALE 100

#define MAXEDGECOST 0.523598775598299
#define EDGESCALE 190.985931710274357
#define MAGTHRESH 1248480000 // 1200.0 
#define MINSEGSIZE 4

//! Structure 
struct LabelEdge_Struct
{
   //! To be commented 
   Rox_Uint cur;
   
   //! To be commented 
   Rox_Uint assoc;
   
   //! To be commented 
   Rox_Sint cost;
};

//! Structure 
struct LabelNode_Struct
{
   //! To be commented 
   Rox_Uint parent;
   
   //! To be commented 
   Rox_Uint countref;
};

//! Structure 
struct Rox_GradientClusterer_Struct
{
   //! To be commented 
   Rox_Sint width;
   
   //! To be commented 
   Rox_Sint height;
   
   //! To be commented 
   Rox_Uint nbedges;

   //! This is a temporary variable used to compute the image gradient
   //! It may be embedded in the corresponding function
   //! It may also be of type Sshort 
   Rox_Array2D_Uint gmagval;
   
   //! Pointer to Gradient magnitude (square of intensity norm)
   // TODO : change container as   Rox_Array2D_Uint gmag;
   Rox_Uint * gmag;

   //! To be commented 
   Rox_Uint * gmagmin;
   
   //! To be commented 
   Rox_Uint * gmagmax;

   //! Pointer to Gradient angle (in radians)
   // TODO : change container as   Rox_Array2D_Uint gtheta;

   Rox_Float * gtheta;

   //! To be commented 
   Rox_Float * gthetamin;
   
   //! To be commented 
   Rox_Float * gthetamax;

   //! To be commented 
   LabelNode nodes;
   
   //! To be commented 
   LabelEdge edges;
   
   //! To be commented 
   LabelEdge sortededges;
   
   //! To be commented 
   Rox_Uint * edgecounters;

   //! To be commented 
   Rox_DynVec_OrientedImagePoint * groups;
   
   //! To be commented 
   Rox_Uint nbgroups;

   //! To be commented 
   Rox_EdgeDraw edgedraw;
   
   //! To be commented 
   Rox_EdgePreproc_Gray preproc;
   
   //! To be commented 
   Rox_EdgePostproc_Ac postproc;
   
   //! To be commented 
   Rox_EdgePostproc_Normal normals;
   
   //! To be commented 
   Rox_ObjSet_DynVec_OrientedImagePoint pointset;
};

#endif
