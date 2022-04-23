//==============================================================================
//
//    OPENROX   : File inertial_measure_buffer_struct.h
//
//    Contents  : Strcture of inertial_measure_buffer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INERTIAL_MEASURE_BUFFER_STRUCT__
#define __OPENROX_INERTIAL_MEASURE_BUFFER_STRUCT__

#include <generated/array2d_double.h>
#include <core/inertial/measure/inertial_measure.h>

//!	\ingroup Inertial
//!	\addtogroup IMU_Buffer
//!	@{

//! Structure 
struct Rox_Imu_Measure_Buffer_Struct
{
   //! \brief The Inertial_Measure list 
   Rox_Imu_Measure *data;
   //! \brief The buffer length 
   Rox_Uint length;
   //! \brief The first measure position 
   Rox_Uint start_position;
   //! \brief The last measure position	
   Rox_Uint end_position;
   //! \brief The read counter 
   Rox_Uint read_count;
   //! \brief The write counter 
   Rox_Uint write_count;
};

//! @} 

#endif // __OPENROX_INERTIAL_MEASURE_BUFFER_STRUCT__ 
