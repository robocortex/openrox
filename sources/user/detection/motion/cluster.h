//==============================================================================
//
//    OPENROX   : File cluster.h
//
//    Contents  : Cluster image module interface
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CLUSTER__
#define __OPENROX_CLUSTER__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus 

// ====== INCLUDED HEADERS   ================================================

#include <baseproc/image/image.h>

// ====== EXPORTED TYPESDEFS ================================================

//! \ingroup Vision
//! \defgroup Cluster Clustering
//! \brief Clustering structures and methods.

//! \ingroup Cluster
//! \brief The pointer on the cluster object.
typedef struct Rox_Cluster_Struct* Rox_Cluster;

// ====== EXPORTED MACROS    =================================================

#define ROX_MAX_CLUSTER 65535

// ====== INTERNAL MACROS    =================================================

#define ROX_MS_UNIFORM_KERNEL(distSqr) ( distSqr <= 1.0f ? 1.0f : 0.0f )
#define ROX_MS_GAUSSIAN_KERNEL(distSqr) ( exp(-distSqr))
#define ROX_MS_EPANECHNIKOV_KERNEL(distSqr) ( distSqr < 1.0f ? 0.75f * (1.0f - distSqr) : 0.0f )

// ====== EXPORTED DATATYPES =================================================

// ====== INTERNAL DATATYPES =================================================

// ====== EXPORTED FUNCTIONS =================================================

//! \ingroup Cluster
//! \brief Instanciate new cluster
//! \param cluster
//! \return Cluster
//!  The instanciated cluster must be deleted using the #rox_cluster_del method.
ROX_API Rox_ErrorCode rox_cluster_new(Rox_Cluster * cluster);

//! \ingroup Cluster
//! \brief Delete cluster Cluster instance
//! \param[in] cluster Cluster instance
//! \return Success
ROX_API Rox_Void rox_cluster_del(Rox_Cluster * cluster);

//! \ingroup Cluster
//! \brief Get number of determined cluster
//! \param[in] count   Number of clusters
//! \param[in] cluster Cluster instance
//! \return Rox_ErrorCode 
ROX_API Rox_ErrorCode rox_cluster_get_count ( Rox_Sint * count, Rox_Cluster cluster);

//! \ingroup Cluster
//! \brief Get bounds of determined cluster
//! \param[in] cluster Cluster instance
//! \return Bounds of clusters Layout: [pos_u1, pos_v1, pos_u2, pos_v2]
ROX_API Rox_Sint * rox_cluster_get_bounds(Rox_Cluster cluster);

//! \ingroup Cluster
//! \brief Determine clusters of binary image with mean shift approach
//! \param[out] cluster Cluster instance
//! \param[in]  image Binary image
//! \param[in]  bandwidth Bandwidth parameter of kernel
//! \return Success
ROX_API Rox_ErrorCode rox_cluster_binary(Rox_Cluster cluster, Rox_Image image, Rox_Sint bandwidth[2]);

// ====== INTERNAL FUNCTIONS =================================================

#ifdef __cplusplus
}
#endif // __cplusplus 

#endif // __OPENROX_CLUSTER__ 
