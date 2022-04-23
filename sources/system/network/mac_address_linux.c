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
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_mac_address_read_decimal(Rox_Char addresses[10][12])
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar mac;
   struct ifreq ifr;

   if (addresses == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Sint fd = socket(AF_INET, SOCK_DGRAM, 0); 
   if(fd < 0) 
   {error = ROX_ERROR_EXTERNAL; ROX_ERROR_CHECK_TERMINATE(error)}

   ifr.ifr_addr.sa_family = AF_INET;
   strncpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);
   
   Rox_Sint ret = ioctl(fd, SIOCGIFHWADDR, &ifr);
   if(ret < 0)
   {
       // try wlan0 
       strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ-1);
       ret = ioctl(fd, SIOCGIFHWADDR, &ifr);
       if(ret < 0)
       {
           close(fd);
           error = ROX_ERROR_EXTERNAL; 
           ROX_ERROR_CHECK_TERMINATE(error)
       }
   }
   close(fd);

   for ( Rox_Sint i = 0; i < 6; i++)
   {
      mac = (Rox_Uchar)ifr.ifr_hwaddr.sa_data[i];

      addresses[0][2*i+0] = mac / 16;
      addresses[0][2*i+1] = mac % 16;
   }

function_terminate:
   return error;
}
