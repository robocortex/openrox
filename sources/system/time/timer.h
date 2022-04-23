//==============================================================================
//
//    OPENROX   : File timer.h
//
//    Contents  : API of timer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TIMER__
#define __OPENROX_TIMER__

#include <system/errors/errors.h>
#include <system/memory/memory.h>
#include <system/memory/datatypes.h>

//! \defgroup Utils Utils
//! \brief Generic macros and utility functions.

//! \ingroup  Utils
//! \defgroup Timer Timer
//! \brief Timer structures and methods.

//! \addtogroup Timer
//! @{

//!    Abstract structure for timer
//!    O/S Dependent

//! object */
typedef struct Rox_Timer_Struct * Rox_Timer;

//! Create a new timer object
//! \param timer a pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_timer_new(Rox_Timer * timer);

//! Delete a new timer object
//! \param timer a pointer to the timer object
//! \return An error code
ROX_API Rox_ErrorCode rox_timer_del(Rox_Timer * timer);

//! Start timer
//! \param timer a pointer to the timer object
//! \return An error code
ROX_API Rox_ErrorCode rox_timer_start(Rox_Timer timer);

//! Stop timer
//! \param timer a pointer to the timer object
//! \return An error code
ROX_API Rox_ErrorCode rox_timer_stop(Rox_Timer timer);

//! Retrieve elapsed time in milliseconds
//! \param milliseconds the number of milliseconds elapsed
//! \param timer the timer object
//! \return An error code
ROX_API Rox_ErrorCode rox_timer_get_elapsed_ms(Rox_Double *milliseconds, Rox_Timer timer);

//! @}

#endif // __OPENROX_TIMER__
