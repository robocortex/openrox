//==============================================================================
//
//    OPENROX   : File sdwm.h
//
//    Contents  : API of sdwm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SDWM__
#define __OPENROX_SDWM__

#include <generated/objset_dynvec_rect_sint.h>
#include <generated/objset_dynvec_rect_sint_struct.h>
#include <generated/objset_sdwm_object.h>
#include <generated/objset_sdwm_object_struct.h>

#include <system/memory/datatypes.h>

#include <baseproc/image/pyramid/pyramid_npot_uchar.h>
#include <baseproc/image/image.h>

#include <core/features/detectors/shape/sdwm_object.h>
#include <core/features/descriptors/fpsm/fpsm.h>
#include <core/features/descriptors/fpsm/fpsm_index.h>
#include <core/features/detectors/edges/edgepreproc_gray.h>
#include <core/features/detectors/edges/edgedraw.h>
#include <core/features/detectors/edges/edgepostproc_ac.h>

//! \ingroup Detectors
 //!  \addtogroup SDWM
 //!  @{

//! The Rox_Sdwm_Struct object 
struct Rox_Sdwm_Struct
{
   //! To be commented  
   Rox_Uint nb_levels;
   //! To be commented  
   Rox_Double scale_per_level;
   //! Width of the model template  
   Rox_Sint width_model;
   //! Height of the model template 
   Rox_Sint height_model;
   //! Width of the current image to be processed  
   Rox_Sint width_current;
   //! Height of the current image to be processed 
   Rox_Sint height_current;

   //! Npot (not power of two) image pyramid  
   Rox_Pyramid_Npot_Uchar pyramid;
   //! To be commented  
   Rox_ObjSet_Sdwm_Object objects;
   //! To be commented  
   Rox_ObjSet_DynVec_Rect_Sint results;
   //! To be commented  
   Rox_DynVec_Sint results_indices;
   //! To be commented  
   Rox_DynVec_Sint results_scores;
   //! To be commented  
   Rox_Ulint sum_edges;
   //! To be commented  
   Rox_Uint count_templates;

   //! To be commented  
   Rox_Fpsm fpsm_model;
   //! To be commented  
   Rox_Fpsm * fpsm_currents;
   //! To be commented  
   Rox_Fpsm_Index index;
};

//! Define the pointer of the Rox_Sdwm_Struct 
typedef struct Rox_Sdwm_Struct * Rox_Sdwm;

//! The Rox_Sdwm_Create_Params object to create a new Rox_Sdwm object 
struct Rox_Sdwm_Create_Params_Struct
{ 
   //! The width of the model  
   Rox_Sint width_model;
	//! The height of the model  
	Rox_Sint height_model;
   //! The width of the current image  
   Rox_Sint width_current;
	//! The height of the current image  
	Rox_Sint height_current;
   //! The number of levels in our pyramidal approach  
   Rox_Uint nbr_levels;
   //! The scale to apply to each level of our pyramid  
   Rox_Double scale_per_Level;
   //! The number of distance bins for each ref point  
   Rox_Uint nbr_distances;
   //! The number of angle bins for each ref point  
   Rox_Uint nbr_angles;
   //! The number of reference points in the image  
   Rox_Uint nbr_ref_points;
   //! The minimum number of votes in fpsm to consider a candidate 
   Rox_Uint min_votes;
   //! Gradient difference minimum to consider this point an edge  
   Rox_Uint min_gradient;
};

//! Define the pointer of the Rox_Sdwm_Create_Params_Struct 
typedef struct Rox_Sdwm_Create_Params_Struct * Rox_Sdwm_Create_Params;

//! The Rox_Sdwm_Process_Params object to process a Rox_Sdwm object 
struct Rox_Sdwm_Process_Params_Struct
{ 
   //! The image to search into  
   Rox_Image image;
   //! Max distance difference to consider for the OCM pass  
   Rox_Sint ocm_max_dist;
   //! Max angle difference to consider for the OCM pass  
   Rox_Double ocm_max_angle;
   //! Score minimum to consider to validate a result after the OCM pass  
   Rox_Double ocm_score_min;
   //! Consider non centered templates, ie search outside test image boundaries (ratio length on each side)  
   Rox_Double template_ratio;
   //! NFA minimum value to add edge 
   Rox_Double min_NFA;
   //! The number of gradientsobel pass (blur) needed 0 (no blur) to n  
   Rox_Uint nbr_blur_passes;
   //! The minimum size of a segment to take in account  
   Rox_Uint min_segment_size;
   //! Only take in account straight edge, no turns allowed  
   Rox_Bool straight_edge_only;
   //! 0 to 1 where 1 has no effect, else decreases the score for templates with fewer edges than average  
   Rox_Double ocm_lambda;
   //! 1 to n, step to move working frame  
   Rox_Sint step_search;
   //! Only process last level of pyramid  
   Rox_Bool only_process_last_level;
};

//! Define the pointer of the Rox_Sdwm_Process_Params_Struct 
typedef struct Rox_Sdwm_Process_Params_Struct * Rox_Sdwm_Process_Params;

//! Create sdwm descriptor object
//! \param[out]	obj				The pointer to the object
//! \param[in]	params		   Creation parameters stored in a Rox_Sdwm_Create_Params struct
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_sdwm_new(Rox_Sdwm * obj, Rox_Sdwm_Create_Params params);

//! Delete sdwm object
//! \param[in]	obj 				The pointer to the object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_sdwm_del(Rox_Sdwm * obj);

//! Find shapes in image
//! \param[in]	obj				The object
//! \param[in]	params		   Process parameters stored in a Rox_Sdwm_Process_Params struct
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_sdwm_process(Rox_Sdwm obj, Rox_Sdwm_Process_Params params );

//! Add object to database
//! \param[in]	obj 						The Sdwm object
//! \param[in]	object_toadd			The new object to add
//! \param[in]	min_NFA					NFA minimum value to add edge
//! \param[in]  nbr_blur_passes		The number of gradientsobel pass (blur) needed 0 (no blur) to n
//! \param[in]	min_segment_size		The minimum size of a segment to take in account
//! \param[in]	straight_edge_only	Only take in account straight edge, no turns allowed
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_sdwm_add_object(Rox_Sdwm obj, Rox_Sdwm_Object object_toadd
   , Rox_Double min_NFA, Rox_Uint nbr_blur_passes
   , Rox_Uint min_segment_size, Rox_Bool straight_edge_only);

//! @} 

#endif // __OPENROX_SDWM__

