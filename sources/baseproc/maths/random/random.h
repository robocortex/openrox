//==============================================================================
//
//    OPENROX   : File random_generator.h
//
//    Contents  : API of random generator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RANDOM__
#define __OPENROX_RANDOM__

#include <system/memory/memory.h>

#define ROX_RAND_MAX ((1U << 31) - 1)

//! A random combination drawer
typedef struct Rox_Random_Struct * Rox_Random;

ROX_API Rox_ErrorCode rox_random_new(Rox_Random * random);

ROX_API Rox_ErrorCode rox_random_del(Rox_Random * random);

ROX_API Rox_ErrorCode rox_random_set_lcg_seed(Rox_Random random, const Rox_Uint seed);

ROX_API Rox_ErrorCode rox_random_set_lcg_data(Rox_Random random, const Rox_Sint a, const Rox_Sint b, const Rox_Sint m);

ROX_API Rox_ErrorCode rox_random_get_lcg_draw(Rox_Sint * draw, Rox_Random random);

//! Set the seed for rox_rand
//! \param [in] seed    The seed between 1 and ROX_RAND_MAX
//! \todo   To be tested
ROX_API void rox_srand(const Rox_Uint seed);

//! Draw a random number bewteen 0 and ROX_RAND_MAX
//! \return The random number
//! \todo   To be tested
ROX_API int rox_rand(void);

//! @}

#endif
