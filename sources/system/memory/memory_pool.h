//==============================================================================
//
//    OPENROX   : File memory_pool.h
//
//  	Contents  : API of memory module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_MEMORY__
#define __OPENROX_MEMORY__

#ifdef __cplusplus
extern "C" {
#endif

#include <system/arch/compiler.h>
#include <system/memory/datatypes.h>
#include <rox_config.h>
#include <stdlib.h>

//! \ingroup Utils
//! \defgroup System System
//! \brief System functions.

//! \ingroup System
//! @defgroup Memory Memory
//! @{

// Custom memory management functions.
// Use this in place of libc for easier memory management, debugging and statistics

//! Allocate a continuous memory array
//! \param element_size is the size of one element of the array
//! \param element_count is the number of elements allocated
//! \return a pointer to the allocated array or NULL (0) if something failed
ROX_API void * rox_memory_allocate(const Rox_Size element_size, const Rox_Size element_count);

//! Resize a continuous memory array
//! \param pointer the array to resize
//! \param element_size is the size of one element of the array
//! \param element_count is the number of elements allocated
//! \return a pointer to the reallocated array or NULL (0) if something failed
ROX_API void * rox_memory_reallocate(void * pointer, const Rox_Size element_size, const Rox_Size element_count);

//! Tells the system that the given array is not used anymore
//! \param pointer the array to free
ROX_API void rox_memory_delete(void * pointer);

//! Allocate a contiuous memory array with pointer being aligned on a specific number of bytes
//! This kind of aligment is needed for SIMD calls
//! \param aligned_adress the aligned pointer to the allocated array
//! \param element_size the size of one element of the array
//! \param element_count the number of elements allocated
//! \param alignment_bytes alignment size (in bytes) : typically 16 for SSE
//! \return a pointer to the allocated array of NULL (0) if something failed. This pointer should only be used to free the array, data start is given by aligned_adress
ROX_API void * rox_memory_allocate_aligned(void ** aligned_adress, const Rox_Size element_size, const Rox_Size element_count, const unsigned char alignment_bytes);

//! Reallocate (resize) a contiuous memory array with pointer being aligned on a specific number of bytes
//! This kind of aligment is needed for SIMD calls
//! \param aligned_adress the aligned pointer to the allocated array
//! \param oldpointer pointer to the array to resize
//! \param oldaligned aligned pointer to the array to resize
//! \param oldsize size of the array
//! \param element_size the size of one element of the array
//! \param element_count the number of elements allocated
//! \param alignment_bytes alignment size (in bytes) : typically 16 for SSE
//! \return a pointer to the allocated array of NULL (0) if something failed. This pointer should only be used to free the array, data start is given by aligned_adress
ROX_API void * rox_memory_reallocate_aligned(void ** aligned_adress, void * oldpointer, void * oldaligned, const Rox_Size oldsize, const Rox_Size element_size, const Rox_Size element_count, const unsigned char alignment_bytes);

//! @} 

#ifdef __cplusplus
}
#endif

#endif
