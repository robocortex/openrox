//============================================================================
//
//    OPENROX   : File mask_pgmfile.h
//
//    Contents  : API of mask_pgmfile module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_MASK_PGMFILE__
#define __OPENROX_MASK_PGMFILE__

#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>
#include <stdio.h>

//! \ingroup File
//! \addtogroup PGM_Files
//! \brief Tools to read/ save pgm files
//! @{

//! Create a new mask and fills values using a binary PGM image file
//! \param out the created mask
//! \param path path to mask file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_mask_new_pgm(Rox_Array2D_Uint * out, const char * path);

//! Load mask from pgm file
//! \param in mask to load
//! \param path path to mask file
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_mask_read_pgm(Rox_Array2D_Uint in, const char * path);

//! Save mask to pgm file
//! \param path path to mask file
//! \param in mask to save
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_mask_save_pgm(const char * path, Rox_Array2D_Uint in);

//! @} 

#endif // __OPENROX_MASK_PGMFILE__
