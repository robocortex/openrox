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
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IONetworkInterface.h>
#include <IOKit/network/IOEthernetController.h>
#include <inout/system/errors_print.h>
#include <stdio.h>


Rox_ErrorCode rox_mac_address_read_decimal(Rox_Char addresses[10][12])
{
   // Need to add the library CoreFoundation and IOKit to work
   Rox_ErrorCode error = ROX_ERROR_NONE;
   int k, count;
   CFMutableDictionaryRef matchingDict;
   CFMutableDictionaryRef propertyMatchDict;
   kern_return_t kernResult;
   io_object_t intfService;
   io_object_t controllerService;
   io_iterator_t intfIterator;
   UInt8 MACAddress[kIOEthernetAddressSize];
   CFTypeRef MACAddressAsCFData;

   if(addresses == 0) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   matchingDict = IOServiceMatching(kIOEthernetInterfaceClass);
   if (matchingDict == NULL) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   propertyMatchDict = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
   if (propertyMatchDict == NULL)
   {
       CFRelease(matchingDict);
       {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   CFDictionarySetValue(propertyMatchDict, CFSTR(kIOPrimaryInterface), kCFBooleanTrue);
   CFDictionarySetValue(matchingDict, CFSTR(kIOPropertyMatchKey), propertyMatchDict);
   CFRelease(propertyMatchDict);

   kernResult = IOServiceGetMatchingServices(kIOMasterPortDefault, matchingDict, &intfIterator);
   if (kernResult != KERN_SUCCESS)
   {
      IOObjectRelease(intfIterator);
      error = ROX_ERROR_INTERNAL;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   bzero(MACAddress, sizeof(MACAddress));

   count = 0;
   while ((intfService = IOIteratorNext(intfIterator)))
   {
      kernResult = IORegistryEntryGetParentEntry(intfService, kIOServicePlane, &controllerService);
      if (kernResult == KERN_SUCCESS)
      {
          MACAddressAsCFData = IORegistryEntryCreateCFProperty(controllerService, CFSTR(kIOMACAddress), kCFAllocatorDefault, 0);
         if (MACAddressAsCFData)
         {
             CFDataGetBytes(MACAddressAsCFData, CFRangeMake(0, kIOEthernetAddressSize), MACAddress);
             CFRelease(MACAddressAsCFData);

            for(k = 0; k < 6; k++)
            {
               addresses[count][2*k+0] = MACAddress[k] / 16;
               addresses[count][2*k+1] = MACAddress[k] % 16;
            }

            if (count < 10 - 1) count++;
         }
         IOObjectRelease(controllerService);
      }
      IOObjectRelease(intfService);
   }

   IOObjectRelease(intfIterator);
function_terminate:
   return error;
}