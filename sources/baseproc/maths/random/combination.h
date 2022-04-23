//==============================================================================
//
//    OPENROX   : File combination.h
//
//    Contents  : API of combination module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_COMBINATION__
#define __OPENROX_COMBINATION__

#include <generated/dynvec_uint.h>

//! \ingroup Maths
//! \defgroup Statistics Statistics

//!
//! \ingroup Statistics
//! \addtogroup RANDOM
//! @{

//! A random combination drawer
typedef struct Rox_Combination_Struct * Rox_Combination;

//! Create a new combination object (No repeat, no order)
//! \param [out]  combination    The created object
//! \param [in]      nb_items       The number of existing items in the set (n)
//! \param [in]      nb_draws       The number of draws done for one combination (k)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_combination_new(Rox_Combination * combination, Rox_Uint nb_items, Rox_Uint nb_draws);

//! Delete a combination object (No repeat, no order)
//! \param combination the object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_combination_del(Rox_Combination * combination);

//! Compute a new combination
//! \param combination the object to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_combination_draw(Rox_Combination combination);

//! Display a combination
//! \param combination the object to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_combination_print(Rox_Combination combination);

//! Display a combination draw
//! \param combination the object to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_combination_print_draw(Rox_Combination combination);

//! Display a combination bag
//! \param combination the object to use
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_combination_print_bag(Rox_Combination combination);

//! @}

#endif
