//==============================================================================
//
//    OPENROX   : File cluster_structure.h
//
//    Contents  : Cluster structure
//
//    Author(s) : R&D department directed by Ezio MALIS
// 
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CLUSTER_STRUCT__
#define __OPENROX_CLUSTER_STRUCT__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus 

// ====== INCLUDED HEADERS   =================================================

// ====== EXPORTED OBJECTS   =================================================

// ====== INTERNAL OBJECTS   =================================================

//! \ingroup Cluster
//! \brief Cluster structure.
struct Rox_Cluster_Struct
{
	//! Number of clusters 
   Rox_Sint  count;
     
	//! Bounds of each cluster 
   Rox_Sint * bounds; 
};

#ifdef __cplusplus
}
#endif // __cplusplus 

#endif // __OPENROX_CLUSTER_STRUCT__ 
