//==============================================================================
//
//    OPENROX   : File array2d.h
//
//    Contents  : API of array2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ARRAY2D__
#define __OPENROX_ARRAY2D__

#ifdef __cplusplus
extern "C" {
#endif

#include <system/memory/array.h>
#include <system/errors/errors.h>

//! \ingroup System
//! @defgroup Array2D Array2D
//! @{

//! Pointer to structure
typedef struct Rox_Array2D_Struct * Rox_Array2D;

//! Create a new 2D array
//! \param  [out]  obj            newly created object pointer
//! \param  [in ]  datatype       data type
//! \param  [in ]  rows           height of array
//! \param  [in ]  cols           width of array
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_new ( Rox_Array2D * obj, Rox_Datatype_Description datatype, Rox_Sint rows, Rox_Sint cols);

//! Delete a 2D array
//! \param  [in ]  ptr            pointer to 2D array to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_del ( Rox_Array2D * ptr);

//! Create a new 2D array containing a pre-allocated buffer
//! \remarks buffer must be aligned on ROX_DEFAULT_ALIGNMENT
//! \param  [out]  obj            newly created object pointer
//! \param  [in ]  datatype       data type
//! \param  [in ]  rows           height of array
//! \param  [in ]  cols           width of array
//! \param  [in ]  buffer         Pre-allocated aligned buffer
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_new_frombuffer (
   Rox_Array2D * obj, 
   Rox_Datatype_Description datatype, 
   const Rox_Sint rows, 
   const Rox_Sint cols, 
   Rox_Void * buffer);

//! Create a view on a parent 2D array. Any change on this view will affect the elements in the parent array !
//! \param  [out]  sub            Destination view
//! \param  [in ]  parent         2D array
//! \param  [in ]  initial_row    First row of the parent array to consider
//! \param  [in ]  initial_col    First col of the parent array to consider
//! \param  [in ]  rows           Number of rows of the parent array to consider
//! \param  [in ]  cols           Number of cols of the parent array to consider
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_new_subarray2d (
   Rox_Array2D * sub, 
   Rox_Array2D parent, 
   Rox_Sint initial_row, 
   Rox_Sint initial_col, 
   Rox_Sint rows, 
   Rox_Sint cols
);

//! Given a view on a parent 2D array and its parent, try to move the view on a different ROI without changing the size of the view. 
//! Any change on this view will affect the elements in the parent array !
//! \param  [out]  sub            destination view
//! \param  [in ]  parent         2D array
//! \param  [in ]  initial_row    first row of the parent array to consider
//! \param  [in ]  initial_col    first col of the parent array to consider
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_subarray2d_shift (
   Rox_Array2D sub, 
   Rox_Array2D parent, 
   Rox_Sint initial_row, 
   Rox_Sint initial_col
);

//! Create a coalesced memory array of row pointers
//! \param  [in ]  ptr_start  pointer to beginning of data
//! \param  [in ]  step       byte count per (virtual) row
//! \param  [in ]  nbrows     rows count
//! \return the list of row pointers
Rox_Void ** rox_array2d_create_rowsptr ( Rox_Void * ptr_start, Rox_Sint step, Rox_Sint nbrows );

//! Delete a memory array of row pointers
//! \param  [in ]  ptr the array to delete
//! \param  [in ]  nbrows the number of rows in this list
//! \return Void
Rox_Void rox_array2d_delete_rowsptr ( Rox_Void ** ptr, Rox_Sint nbrows );

//! Create a coalesced memory array of block pointers
//! For example if rows = 2*k and rows_per_blocks = 2, we create k blocks of pointers where each pointer points on a block of 2 rows
//! If rows_per_blocks = 1, then blocksptr = rowsptr
//! This can be useful if we want to process each block of rows in paraller
//! \param  [in ]  ptr                    the array to use
//! \param  [in ]  rows_per_blocks        rows per blocks (the total number of rows must be a multiple of rows_per_blocks)
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_create_blocksptr ( Rox_Array2D ptr, Rox_Sint rows_per_blocks );

//! Delete a memory array of blocks pointers
//! \param  [in ]  ptr the blocks to delete
//! \param  [in ]  blocks the number of blocks in this list
//! \return Void
ROX_API Rox_Void rox_array2d_delete_blocksptr ( Rox_Void *** ptr, Rox_Sint blocks );

//! Retrieve the list of per row pointers of a 2D array
//! \param  [in ]  array the array to get data from
//! \return the rows pointers
ROX_API Rox_ErrorCode rox_array2d_get_data_pointer_to_pointer ( Rox_Void *** pointer, Rox_Array2D array );

//! Retrieve the list of per row pointers of a 2D array
//! \param  [in ]  array the array to get data from
//! \return the rows pointers
ROX_API Rox_ErrorCode rox_array2d_get_data_pointer ( Rox_Void ** pointer, Rox_Array2D array );

//! Retrieve the list of per blocks pointers of a 2D array
//! \param  [in ]  array the array to get data from
//! \return the rows pointers
ROX_API Rox_ErrorCode rox_array2d_get_blocks_pointer_to_pointer ( Rox_Void **** blocks_pointer, Rox_Array2D array );

//! Get the height of 2D array
//! \param  [in ]  array input array
//! \return The number of rows (0 if error)
ROX_API Rox_ErrorCode rox_array2d_get_rows(Rox_Sint * rows, Rox_Array2D array);

//! Get the width of 2D array
//! \param  [in ]  array          input array
//! \return The number of columns (0 if error)
ROX_API Rox_ErrorCode rox_array2d_get_cols(Rox_Sint * cols, Rox_Array2D array);

//! Get the stride (in bytes) of rows of 2D array
//! \param  [in ]  array          input array
//! \return the width
ROX_API Rox_ErrorCode rox_array2d_get_stride(Rox_Sint * stride, Rox_Array2D array);

//! Get the datatype of 2D array
//! \param  [in ]  array            input array
//! \return the datatype
ROX_API Rox_ErrorCode rox_array2d_get_type(Rox_Datatype_Description * datatype, Rox_Array2D array);

//! Check if two 2D arrays have the same dimension
//! \param  [in ]  array1         input array
//! \param  [in ]  array2         input array
//! \return An Error code 
ROX_API Rox_ErrorCode rox_array2d_match_size(Rox_Array2D array1, Rox_Array2D array2);

//! Check if two 2D arrays have the same dimension and datatype
//! \param  [in ]  array1         input array
//! \param  [in ]  array2         input array
//! \return An Error code 
ROX_API Rox_ErrorCode rox_array2d_match(Rox_Array2D array1, Rox_Array2D array2);

//! Copy an 2D array in another
//! \param  [in]  dest            destination array
//! \param  [in]  source          source array
//! \return An Error code 
ROX_API Rox_ErrorCode rox_array2d_copy(Rox_Array2D dest, Rox_Array2D source);

//! @} 

#ifdef __cplusplus
}
#endif

#endif
