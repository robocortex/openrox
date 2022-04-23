//==============================================================================
//
//    OPENROX   : File fpsm_index.h
//
//    Contents  : API of fpsm_index module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FPSM_INDEX__
#define __OPENROX_FPSM_INDEX__

#include <system/memory/datatypes.h>

#include <generated/dynvec_fpsm_feature.h>
#include <generated/objset_dynvec_sint.h>
#include <generated/objset_dynvec_fpsm_template.h>
#include <generated/objset_dynvec_sint_struct.h>
#include <generated/objset_dynvec_fpsm_template_struct.h>
#include <generated/dynvec_fpsm_template_struct.h>

//! \addtogroup FPSM
//! @{

//! The Rox_Fpsm_Index_Struct object 
struct Rox_Fpsm_Index_Struct
{
	//! To be commented  
	Rox_Uint nd;
	//! To be commented  
	Rox_Uint ntheta;
	//! To be commented  
	Rox_Uint m;
	//! To be commented  
	Rox_Sint width;
	//! To be commented  
	Rox_Sint height;
	//! To be commented  
	Rox_Uint min_votes;

	//! To be commented  
	Rox_Double maxdist;

	//! To be commented  
	Rox_ObjSet_DynVec_Fpsm_Template index;
	//! To be commented  
	Rox_ObjSet_DynVec_Sint counters;
	//! To be commented  
	Rox_DynVec_Fpsm_Template results;
};

//! Define the pointer of the Rox_Fpsm_Index_Struct 
typedef struct Rox_Fpsm_Index_Struct * Rox_Fpsm_Index;

//! Create fpsm index object
//! \param[out] obj 		The pointer to the object
//! \param[in]	nd 		The number of distance bins
//! \param[in]	ntheta	The number of angle bins
//! \param[in]	m			The number of points per patch
//! \param[in]	min_votes		The minimum votes
//! \param[in]	width		The width of the rectangular patch
//! \param[in]	height		The height of the rectangular patch
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_fpsm_index_new(Rox_Fpsm_Index * obj
	, Rox_Uint nd, Rox_Uint ntheta
	, Rox_Uint m, Rox_Uint min_votes
	, Rox_Sint width, Rox_Sint height);

//! Delete fpsm index object
//! \param[in] obj			The pointer to the object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_fpsm_index_del(Rox_Fpsm_Index * obj);

//! Initialize new index
//! \param[in] obj			The pointer to the object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_fpsm_index_init(Rox_Fpsm_Index obj);

//! Append feature to index
//! \param[in] obj			The pointer to the object
//! \param[in] features	The features
//! \param[in] object_id The object ID
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_fpsm_index_append_object(Rox_Fpsm_Index obj, Rox_DynVec_Fpsm_Feature features, Rox_Uint object_id);

//! Append feature to index
//! \param[in] obj The pointer to the object
//! \param[in] feature	The feature
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_fpsm_index_search(Rox_Fpsm_Index obj, Rox_Fpsm_Feature_Struct * feature);

//! @} 

#endif // __OPENROX_FPSM_INDEX__

