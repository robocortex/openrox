//==============================================================================
//
//    OPENROX   : File mac_address.c
//
//    Contents  : Implementation of mac_address module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "mac_address.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_mac_address_convert_ascii_to_decimal(Rox_Char MAC_decimal[12], Rox_Char MAC_ascii[12])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!MAC_decimal || !MAC_ascii) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   for ( Rox_Sint i = 0; i < 12; i++)
   {
      if(MAC_ascii[i] >= '0' && MAC_ascii[i] <= '9')      MAC_decimal[i] = MAC_ascii[i] - 0x30;
      else if(MAC_ascii[i] >= 'a' && MAC_ascii[i] <= 'f') MAC_decimal[i] = MAC_ascii[i] - 0x61 + 10;
      else if(MAC_ascii[i] >= 'A' && MAC_ascii[i] <= 'F') MAC_decimal[i] = MAC_ascii[i] - 0x41 + 10;
      else
      {
         error = ROX_ERROR_INVALID_VALUE;
         ROX_ERROR_CHECK_TERMINATE(error)
      }
   }
  
function_terminate:
   return error;
}

Rox_ErrorCode rox_mac_address_convert_decimal_to_ascii(Rox_Char MAC_ascii[12], Rox_Char MAC_decimal[12])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!MAC_decimal || !MAC_ascii) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   for ( Rox_Sint i = 0; i < 12; i++)
   {
      if(MAC_decimal[i] >= 0 && MAC_decimal[i] <= 9) MAC_ascii[i] = MAC_decimal[i] + 0x30;
      else if(MAC_decimal[i] >= 10 && MAC_decimal[i] <= 15) MAC_ascii[i] = MAC_decimal[i] + 0x57;
      else {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_get_current_mac_address(Rox_Char mac_address[13])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Char addresses[10][12];
   
   if(mac_address == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_mac_address_read_decimal(addresses); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_mac_address_convert_decimal_to_ascii(mac_address, addresses[0]); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}