//==============================================================================
//
//    OPENROX   : File sraid_matchresultset.h
//
//    Contents  : API of sraid_matchresultset module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SRAID_MATCH_RESULTSET__
#define __OPENROX_SRAID_MATCH_RESULTSET__

#include <generated/dynvec_uint.h>

//! \addtogroup SRAID
//! @{

//! Match result structure 
struct Rox_SRAID_MatchResult_Struct
{
   //! index of the feature
	Rox_Uint index;
	//! Distance 
	Rox_Uint distance;
};

//! Match result structure 
typedef struct Rox_SRAID_MatchResult_Struct Rox_SRAID_MatchResult_Struct;

//! Set of results 
struct Rox_SRAID_MatchResultSet_Struct
{
   //! Set of results 
	Rox_SRAID_MatchResult_Struct * results;
   //! card of results 
	Rox_Uint count_results;
	//! max card of results 
	Rox_Uint max_results;
};
//! Set of results 
typedef struct Rox_SRAID_MatchResultSet_Struct * Rox_SRAID_MatchResultSet;

//! Create a new sraid result set
//! \param [out] obj pointer to the object created
//! \param [in] maxresults maximum size of the set
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraid_matchresultset_new(Rox_SRAID_MatchResultSet * obj, Rox_Uint maxresults);

//! Delete object result set
//! \param [out] obj set object pointer
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraid_matchresultset_del(Rox_SRAID_MatchResultSet * obj);

//! Delete object result set
//! \param [out] obj set object pointer
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraid_matchresultset_clear(Rox_SRAID_MatchResultSet obj);

//! Check if result set is full
//! \param [out] obj set object
//! \return full or not boolean
//! \todo to be tested
ROX_API Rox_Uint rox_sraid_matchresultset_isfull(Rox_SRAID_MatchResultSet obj);

//! Add a result to result set
//! \param [out] obj set object
//! \param [in] index the matched index
//! \param [in] distance the matched distance
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraid_matchresultset_addresult(Rox_SRAID_MatchResultSet obj, Rox_Uint index, Rox_Uint distance);

//! @} 

#endif
