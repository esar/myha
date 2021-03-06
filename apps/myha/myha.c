/*

   Copyright (C) 2014 Stephen Robinson
  
   This file is part of Myha
  
   Myha is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 2 of the License, or
   (at your option) any later version.
  
   Myha is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this code (see the file names COPYING).  
   If not, see <http://www.gnu.org/licenses/>.
  
*/

#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/rpl/rpl.h"

#include "myha.h"

#include <avr/io.h>
#define TRUE 1
#define FALSE 0

#define PRINTF(format, args...) printf_P(PSTR(format), ##args)

PROCESS(myha_process, "Myha");

static uip_ipaddr_t current_addr;
static uip_ipaddr_t server_addr;
static char node_id[17];


/*---------------------------------------------------------------------------*/

uip_ipaddr_t* myha_get_udp_address()
{
  return &current_addr;
}

uip_ipaddr_t* myha_get_mqtt_address()
{
  return &server_addr;
}

const char* myha_get_node_id()
{
  return node_id;
}

resolv_status_t lookup_server_address(const char* name, uip_ipaddr_t* address)
{
  resolv_status_t status = RESOLV_STATUS_ERROR;

  if(uiplib_ipaddrconv(name, address) == 0)
  {
    uip_ipaddr_t* resolved_addr = NULL;
    
    status = resolv_lookup(name, &resolved_addr);
    if(status == RESOLV_STATUS_UNCACHED || status == RESOLV_STATUS_EXPIRED)
    {
      PRINTF("starting lookup\n");
      resolv_query(name);
      status = RESOLV_STATUS_RESOLVING;
    }
    else if(status == RESOLV_STATUS_CACHED && resolved_addr != NULL)
    {
      PRINTF("lookup succeeded\n");
      uip_ipaddr_copy(address, resolved_addr);
    }
    else if(status == RESOLV_STATUS_RESOLVING)
      PRINTF("still looking up\n");
    else
      PRINTF("lookup failed\n");
  }
  else
    status = RESOLV_STATUS_CACHED;

  return status;
}

PROCESS_THREAD(myha_process, ev, data)
{
  static struct etimer et;
  static int have_addr = FALSE;

  PROCESS_BEGIN();

  memset(&current_addr, 0, sizeof(current_addr));

  snprintf_P(node_id, sizeof(node_id), PSTR("%02x%02x%02x%02x%02x%02x%02x%02x/"), 
             linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
             linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[3],
             linkaddr_node_addr.u8[4], linkaddr_node_addr.u8[5],
             linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

  // Wait for intial IP address
  etimer_set(&et, 1 * CLOCK_SECOND);
  for(;;)
  {
    uip_ipaddr_t* global_addr;

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    if((global_addr = &uip_ds6_get_global(-1)->ipaddr) != NULL)
    {
      int i;
      rpl_dag_t* dag = rpl_get_any_dag();

      if(dag != NULL)
      {
        uip_ipaddr_t new_addr;
//        resolv_status_t resolv_status;

        uip_ipaddr_copy(&new_addr, global_addr);
        memcpy(&new_addr.u8[8], &dag->dag_id.u8[8], sizeof(uip_ipaddr_t) / 2);
        if(!uip_ipaddr_cmp(&new_addr, &current_addr))
        {
          if(have_addr)
          {
            // address has changed
            PRINTF("root address has changed, killing processes...\n");

            // tell all processes to exit
            for(i = 0; myha_autostart_processes[i] != NULL; ++i)
              process_exit(myha_autostart_processes[i]);

            // wait for all processes to exit
            while(TRUE)
            {
              for(i = 0; myha_autostart_processes[i] != NULL; ++i)
                if(process_is_running(myha_autostart_processes[i]))
                  break;
              if(myha_autostart_processes[i] == NULL)
                break;
            }
          }

          uip_ipaddr_copy(&current_addr, &new_addr);
          have_addr = TRUE;

/*
          // lookup the server address
          resolv_status = RESOLV_STATUS_UNCACHED;
          while(resolv_status != RESOLV_STATUS_CACHED)
          {
            //resolv_status = lookup_server_address("myha.local", &server_addr);
            resolv_status = lookup_server_address("bbbb::a:bff:fe0c:d0e", &server_addr);
          
            if(resolv_status == RESOLV_STATUS_RESOLVING)
            {
              PRINTF("myha: waiting for resolv_event_found\n");
              PROCESS_WAIT_EVENT_UNTIL(ev == resolv_event_found);
              PRINTF("myha: wait done\n");
            }
            else if(resolv_status != RESOLV_STATUS_CACHED)
            {
              PRINTF("server address lookup failed\n");
              goto TRY_AGAIN;
            }
          }
*/
          uiplib_ipaddrconv("bbbb::1", &server_addr);

          // start the processes
          PRINTF("starting processes...\n");
          for(i = 0; myha_autostart_processes[i] != NULL; ++i)
            process_start(myha_autostart_processes[i], NULL);

          PRINTF("done, everything should be running.\n");
        }
      }
      else
        PRINTF("no dag yet...\n");
    }
    else
      PRINTF("no global address yet...\n");

TRY_AGAIN:
    etimer_restart(&et);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
