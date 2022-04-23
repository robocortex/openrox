//==============================================================================
//
//    OPENROX   : File mac_address.h
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
#include <winsock2.h>
#include <iphlpapi.h>
#include <stdio.h>
#include <stdlib.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_mac_address_read_decimal(Rox_Char addresses[10][12])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uchar mac;
   ULONG _error, sizebuf;
   PIP_ADAPTER_ADDRESSES buffer;
   PIP_ADAPTER_ADDRESSES pCurrAddresses;
   int i, count;
   
   if(!addresses) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   count = 0;
   buffer = 0;
   sizebuf = 0;
     
   _error = GetAdaptersAddresses(AF_UNSPEC, GAA_FLAG_SKIP_DNS_SERVER, NULL, buffer, &sizebuf);
   if (_error != ERROR_BUFFER_OVERFLOW) {error = ROX_ERROR_INTERNAL; ROX_ERROR_CHECK_TERMINATE(error)}
   
   buffer = (IP_ADAPTER_ADDRESSES *)HeapAlloc(GetProcessHeap(), 0, sizebuf);
   if (!buffer) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
     
   _error = GetAdaptersAddresses(AF_UNSPEC, GAA_FLAG_SKIP_DNS_SERVER, NULL, buffer, &sizebuf);
   if (_error != NO_ERROR) { HeapFree(GetProcessHeap(), 0, buffer); error = ROX_ERROR_INTERNAL; ROX_ERROR_CHECK_TERMINATE(error)}
   
   pCurrAddresses = buffer;
   while (pCurrAddresses)
   { 
      if (((pCurrAddresses->IfType == IF_TYPE_ETHERNET_CSMACD) || (pCurrAddresses->IfType ==  IF_TYPE_IEEE80211)) && (pCurrAddresses->PhysicalAddressLength == 6)) 
      {         
         for(i = 0; i < 6; i++)
         {
            mac = pCurrAddresses->PhysicalAddress[i];
            addresses[count][2*i+0] = mac / 16;
            addresses[count][2*i+1] = mac % 16;
         }
         count++;
      }
      
      pCurrAddresses = pCurrAddresses->Next;
      if (count == 10) break;
   }
   
   HeapFree(GetProcessHeap(), 0, buffer);
   
function_terminate:
   return error;
}
