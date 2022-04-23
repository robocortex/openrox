//==============================================================================
//
//    OPENROX   : File memory_print.h
//
//  	Contents  : API of memory_print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_MEMORY_PRINT__
#define __OPENROX_MEMORY_PRINT__

#ifdef __cplusplus
extern "C" {
#endif

   #include <system/arch/compiler.h>
   #include <system/memory/datatypes.h>
   #include <generated/config.h>
   #include <stdio.h>

   //! \ingroup Utils
   //! \defgroup InOut System
   //! \brief InOut system functions.

   //! \ingroup InOut System
   //! @defgroup Memory Memory
   //! @{

   // Custom memory display management functions.

   //! Stores info related to an allocation
   //! \param pointer the allocated pointer
   //! \param size size of the allocation
   //! \return an error code
   ROX_API Rox_ErrorCode rox_memory_log_alloc(void *pointer, const Rox_Size size);
   //! Stores info related to an reallocation
   //! \param old_pointer the previous allocated pointer
   //! \param old_pointer the newly allocated pointer
   //! \param size size of the allocation
   //! \return an error code
   ROX_API Rox_ErrorCode rox_memory_log_realloc(void *old_pointer, void *new_pointer, const Rox_Size size);
   //! Stores info related to a pointer free
   //! \param pointer the previously allocated pointer
   //! \return an error code
   ROX_API Rox_ErrorCode rox_memory_log_delete(void *pointer);   
   //! Prints the current state of memory to standard output
   //! \return an error code
   ROX_API Rox_ErrorCode rox_memory_print_summary();


//! @} 

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__OPENROX_memory_print__
