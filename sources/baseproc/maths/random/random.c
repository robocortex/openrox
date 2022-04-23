//==============================================================================
//
//    OPENROX   : File random_generator.c
//
//    Contents  : Implementation of random generator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "random.h"
#include "random_struct.h"

#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

static int rox_lcg_seed = 1;

void rox_srand(const Rox_Uint seed)
{
   if(seed > 0)
   {
      rox_lcg_seed = seed;
   }
   else
   {
      rox_lcg_seed = 1;
   }
}

int rox_rand(void)
{
   int a = 1103515245;
   int b = 12345;
   int m = ROX_RAND_MAX;
   return rox_lcg_seed = (rox_lcg_seed * a + b) & m;
}

Rox_ErrorCode rox_random_new(Rox_Random * random)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Random ret = NULL;

   if (!random)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_Random) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->seed = rox_lcg_seed;
   // Parameters defined in BSD libc
   // https://rosettacode.org/wiki/Category:BSD_libc
   ret->a = 1103515245;
   ret->b = 12345;
   ret->m = ROX_RAND_MAX;

   *random = ret;

function_terminate:
   if (error) rox_random_del(random);
   return error;
}

Rox_ErrorCode rox_random_set_lcg_data(Rox_Random random, const Rox_Sint a, const Rox_Sint b, const Rox_Sint m)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!random)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   random->a = a;
   random->b = b;
   random->m = m;

function_terminate:
   return error;

}

Rox_ErrorCode rox_random_set_lcg_seed(Rox_Random random, const Rox_Uint seed)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!random)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(seed > 0)
   {
      random->seed = seed;
   }
   else
   {
      random->seed = 1;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_random_lcg_draw(Rox_Random random)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!random)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   random->seed = (random->seed * random->a + random->b) & random->m ;

function_terminate:
   return error;
}

Rox_ErrorCode rox_random_get_lcg_draw(Rox_Sint * draw, Rox_Random random)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!draw || !random)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_random_lcg_draw(random);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *draw = random->seed;

function_terminate:
   return error;
}

Rox_ErrorCode rox_random_del(Rox_Random * random)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Random todel = NULL;

   if (!random)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *random;
   *random = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_memory_delete(todel);

function_terminate:
   return error;
}
