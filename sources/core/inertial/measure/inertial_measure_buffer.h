//============================================================================
//
//    OPENROX   : File inertial_measure_buffer.h
//
//    Contents  : API of inertial_measure_buffer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_INERTIAL_MEASURE_BUFFER__
#define __OPENROX_INERTIAL_MEASURE_BUFFER__

#include <generated/array2d_double.h>
#include <core/inertial/measure/inertial_measure.h>

//! \ingroup Inertial
//! \addtogroup IMU_Buffer
//! @{

//! Define the pointer of the Rox_Imu_Measure_Buffer_Struct
typedef struct Rox_Imu_Measure_Buffer_Struct* Rox_Imu_Measure_Buffer;

//! Constructor for inertial measure buffer structure
//! \param  [in] buffer Pointer to the buffer object
//! \param  [in] length The buffer length
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imu_measure_buffer_new(Rox_Imu_Measure_Buffer *buffer, const Rox_Uint length);

//! Destructor for inertial measure buffer structure
//! \param  [in] buffer object to delete
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imu_measure_buffer_del(Rox_Imu_Measure_Buffer *buffer);

//! Add an inertial measure to the buffer
//! \param  [in] buffer Pointer to the buffer object
//! \param  [in] measure The inertial measure to add
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imu_measure_buffer_queue(Rox_Imu_Measure_Buffer buffer, Rox_Imu_Measure measure);

//! Get the oldest measure in the bufffer
//! \param  [out] measure The copy of the oldest inertial measure
//! \param  [in] buffer Pointer to the buffer object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imu_measure_buffer_dequeue(Rox_Imu_Measure measure, Rox_Imu_Measure_Buffer buffer);

//! Check if the buffer is empty
//! \param  [in] buffer Pointer to the buffer object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imu_measure_buffer_empty(const Rox_Imu_Measure_Buffer buffer);

//! Check if the buffer is full
//! \param  [in] buffer Pointer to the buffer object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_imu_measure_buffer_full(const Rox_Imu_Measure_Buffer buffer);

//! @}

#endif // __OPENROX_INERTIAL_MEASURE_BUFFER__
