//==============================================================================
//
//    OPENROX   : File ehid_compiler.h
//
//    Contents  : API of ehid_compiler module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_COMPILER__
#define __OPENROX_EHID_COMPILER__

#include <generated/dynvec_double.h>
#include <generated/dynvec_ehid_dbindex.h>
#include <core/features/descriptors/ehid/ehid.h>
#include <core/features/descriptors/ehid/ehid_dbindex.h>
#include <core/features/descriptors/ehid/ehid_database.h>
#include <core/features/descriptors/ehid/ehid_searchtree.h>

//! \addtogroup EHID
//! @{

//! Ehid Compiler object is a pointer to the opaque structure 
typedef struct Rox_Ehid_Compiler_Struct * Rox_Ehid_Compiler;

//! Create compilation object
//! \param [out] ehid_compiler A pointer to the compiler structure created
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_compiler_new(Rox_Ehid_Compiler * ehid_compiler);

//! Delete compilation object
//! \param [out] ehid_compiler a pointer to the compiler structure to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_compiler_del(Rox_Ehid_Compiler * ehid_compiler);

//! Clean compiler from all db
//! \param [in] ehid_compiler the compiler object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_compiler_removealldbs(Rox_Ehid_Compiler ehid_compiler);

//! Append a template (preprocessed) to the database
//! \param [in] ehid_compiler a pointer to the object
//! \param [in] pointslist the list of points & descriptors for the template
//! \param [in] indices the associated indices for each points
//! \param [in] pixel_width the pixel width of the template
//! \param [in] pixel_height the pixel height of the template
//! \param [in] meter_width the meter width of the template
//! \param [in] meter_height the meter height of the template
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_compiler_add_db(Rox_Ehid_Compiler ehid_compiler, Rox_DynVec_Ehid_Point pointslist, Rox_DynVec_Ehid_DbIndex indices, Rox_Double pixel_width, Rox_Double pixel_height, Rox_Double meter_width, Rox_Double meter_height);

//! Execute compilation
//! \param [in] ehid_database A pointer to the database produced
//! \param [in] ehid_compiler The compiler object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_compiler_compile(Rox_Ehid_Database ehid_database, Rox_Ehid_Compiler ehid_compiler);

//! @} 

#endif
