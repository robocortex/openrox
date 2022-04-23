//==============================================================================
//
//    OPENROX   : File date.h
//
//    Contents  : API of date module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DATE__
#define __OPENROX_DATE__

#include <system/errors/errors.h>
#include <system/memory/memory.h>
#include <system/memory/datatypes.h>
                                                                                                                                                                     
//! \ingroup  Utils                                                                                                                                                          
//! \defgroup Date Date                                                                                                                                                    
//! \brief Date manipulation.                                                                                                                                                           

//! \addtogroup Date
//!   @{

//! Date Structure 
struct Rox_Date_Struct
{
   //! To be commented  
   Rox_Uint _year;
   //! To be commented  
   Rox_Uint _month;
   //! To be commented  
   Rox_Uint _day;
   //! To be commented  
   Rox_Uint _hours;
   //! To be commented  
   Rox_Uint _minutes;
   //! To be commented  
   Rox_Uint _seconds;
   //! To be commented  
   Rox_Uint _milliseconds;
};

//! Abstract structure for date O/S Dependent 
typedef struct Rox_Date_Struct * Rox_Date;

//! Create a new date object
//! \param date a pointer to the created object
//! \return an error code
ROX_API Rox_ErrorCode rox_date_new(Rox_Date * date);

//! Delete a new date object
//! \param date a pointer to the date object
//! \return an error code
ROX_API Rox_Void rox_date_del(Rox_Date * date);

//! Init the date
//! \param date a pointer to the date object
//! \return an error code
ROX_API Rox_ErrorCode rox_date_init(Rox_Date date);

//! Get the year
//! \param year a pointer to the returned year
//! \param date a pointer to the date object
//! \return an error code
ROX_API Rox_ErrorCode rox_date_get_year(Rox_Uint *year, Rox_Date date);

//! Get the month
//! \param month a pointer to the returned month
//! \param date a pointer to the date object
//! \return an error code
ROX_API Rox_ErrorCode rox_date_get_month(Rox_Uint *month, Rox_Date date);

//! Get the day
//! \param day a pointer to the returned day
//! \param date a pointer to the date object
//! \return an error code
ROX_API Rox_ErrorCode rox_date_get_day(Rox_Uint *day, Rox_Date date);

//! Get the hours
//! \param hours a pointer to the returned hours
//! \param date a pointer to the date object
//! \return an error code
ROX_API Rox_ErrorCode rox_date_get_hours(Rox_Uint *hours, Rox_Date date);

//! Get the minutes
//! \param minutes a pointer to the returned minutes
//! \param date a pointer to the date object
//! \return an error code
ROX_API Rox_ErrorCode rox_date_get_minutes(Rox_Uint *minutes, Rox_Date date);

//! Get the seconds
//! \param seconds a pointer to the returned seconds
//! \param date a pointer to the date object
//! \return an error code
ROX_API Rox_ErrorCode rox_date_get_seconds(Rox_Uint *seconds, Rox_Date date);

//! Get the milliseconds
//! \param milliseconds a pointer to the returned milliseconds
//! \param date a pointer to the date object
//! \return an error code
ROX_API Rox_ErrorCode rox_date_get_milliseconds(Rox_Uint *milliseconds, Rox_Date date);

//! @} 

#endif // __OPENROX_DATE__
