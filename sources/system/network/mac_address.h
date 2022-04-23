//==============================================================================
//
//    OPENROX   : File mac_address.h
//
//    Contents  : API of mac_address module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MAC_ADDRESS__
#define __OPENROX_MAC_ADDRESS__

#include <system/errors/errors.h>
#include <system/memory/datatypes.h>

//! addtogroup Network
//! @{

//! Read the mac addresses of the current plateform
//! \param addresses a pointer to the read MAC addresses
//! \return an error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_mac_address_read_decimal(Rox_Char addresses[10][12]);

//! Convert the ascii address into a decimal address, for example F F:F F will be converted to 15 15:15 15
//! \param MAC_decimal a pointer to the decimal address
//! \param MAC_ascii a pointer to the ascii address
//! \return an error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_mac_address_convert_ascii_to_decimal(Rox_Char MAC_decimal[12], Rox_Char MAC_ascii[12]);

//! Convert the decimal address into a ascii address, for example 10 10: 11 11 will be converted to AA:BB
//! \param MAC_ascii a pointer to the ascii address
//! \param MAC_decimal a pointer to the decimal address
//! \return an error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_mac_address_convert_decimal_to_ascii(Rox_Char MAC_ascii[12], Rox_Char MAC_decimal[12]);

//! Get the current MAC address
//! \param MAC_address a pointer to the MAC address
//! \return an error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_get_current_mac_address(Rox_Char mac_address[13]);

//! @} 

#endif // __OPENROX_MAC_ADDRESS__
