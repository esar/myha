#include <stdio.h>
#include <string.h>

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/rpl/rpl.h"

#include "myha.h"

#include <avr/io.h>
#define TRUE 1
#define FALSE 0

PROCESS(myha_process, "Myha");

static uip_ipaddr_t current_addr;
static uip_ipaddr_t server_addr;


/*---------------------------------------------------------------------------*/

uip_ipaddr_t* myha_get_udp_address()
{
  return &current_addr;
}

uip_ipaddr_t* myha_get_mqtt_address()
{
  return &server_addr;
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
      printf("starting lookup\n");
      resolv_query(name);
      status = RESOLV_STATUS_RESOLVING;
    }
    else if(status == RESOLV_STATUS_CACHED && resolved_addr != NULL)
    {
      printf("lookup succeeded\n");
      uip_ipaddr_copy(address, resolved_addr);
    }
    else if(status == RESOLV_STATUS_RESOLVING)
      printf("still looking up\n");
    else
      printf("lookup failed\n");
  }
  else
    status = RESOLV_STATUS_CACHED;

  return status;
}

PROCESS_THREAD(myha_process, ev, data)
{
  static struct etimer et;
  static uip_ipaddr_t current_addr;
  static uip_ipaddr_t server_addr;
  static int have_addr = FALSE;

  PROCESS_BEGIN();

  memset(&current_addr, 0, sizeof(current_addr));

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
            printf("root address has changed, killing processes...\n");

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
              printf("myha: waiting for resolv_event_found\n");
              PROCESS_WAIT_EVENT_UNTIL(ev == resolv_event_found);
              printf("myha: wait done\n");
            }
            else if(resolv_status != RESOLV_STATUS_CACHED)
            {
              printf("server address lookup failed\n");
              goto TRY_AGAIN;
            }
          }
*/
          uiplib_ipaddrconv("bbbb::1", &server_addr);

          // start the processes
          printf("starting processes...\n");
          for(i = 0; myha_autostart_processes[i] != NULL; ++i)
            process_start(myha_autostart_processes[i], NULL);

          printf("done, everything should be running.\n");
        }
      }
      else
        printf("no dag yet...\n");
    }
    else
      printf("no global address yet...\n");

TRY_AGAIN:
    etimer_restart(&et);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
