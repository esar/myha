/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/rpl/rpl.h"
#include "net/ip/uip.h"

#if UDP_CLIENT_STORE_RADIO_INFO
#include "dev/radio-sensor.h"
#endif

#include <string.h>

#define DEBUG 1
#include "net/ip/uip-debug.h"

#ifndef CETIC_6LBR_UDP_PERIOD
#define CETIC_6LBR_UDP_PERIOD 15
#endif

#ifndef CETIC_6LBR_NODE_INFO_PORT
#define CETIC_6LBR_NODE_INFO_PORT 3000
#endif

#define SEND_INTERVAL    (CETIC_6LBR_UDP_PERIOD * CLOCK_SECOND)
#define MAX_PAYLOAD_LEN    40


extern uip_ds6_prefix_t uip_ds6_prefix_list[];
clock_time_t udp_interval = CETIC_6LBR_UDP_PERIOD * CLOCK_SECOND;

#if PLATFORM_HAS_RADIO && UDP_CLIENT_STORE_RADIO_INFO
int udp_client_lqi = 0;
int udp_client_rssi = 0;
#endif

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(struct uip_udp_conn* conn)
{
  char *str;

  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
    printf("Response from the server: '%s'\n", str);
#if PLATFORM_HAS_RADIO && UDP_CLIENT_STORE_RADIO_INFO
    udp_client_lqi = radio_sensor.value(RADIO_SENSOR_LAST_PACKET);
    udp_client_rssi = radio_sensor.value(RADIO_SENSOR_LAST_VALUE);
#endif
  }
}
/*---------------------------------------------------------------------------*/
char *
add_ipaddr(char * buf, const uip_ipaddr_t *addr)
{
  uint16_t a;
  unsigned int i;
  int f;
  char *p = buf;

  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
        p += sprintf(p, "::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        p += sprintf(p, ":");
      }
      p += sprintf(p, "%04x", a);
    }
  }
  return p;
}


struct uip_udp_conn* connect(uip_ipaddr_t* address)
{
  struct uip_udp_conn* conn;

  PRINTF("UDP-CLIENT: address destination: ");
  PRINT6ADDR(address);
  PRINTF("\n");

  conn = udp_new(address, UIP_HTONS(CETIC_6LBR_NODE_INFO_PORT), NULL);
  if(conn != NULL) 
  {
    PRINTF("Created a connection with the server ");
    PRINT6ADDR(&conn->ripaddr);
    PRINTF(" local/remote port %u/%u\n", UIP_HTONS(conn->lport), UIP_HTONS(conn->rport));
  } 
  else 
  {
    PRINTF("Could not open connection\n");
  }

  return conn;
}

void disconnect(struct uip_udp_conn* conn)
{
  PRINTF("UDP-CLIENT: disconnecting\n");
  uip_udp_remove(conn);
}

void send(struct uip_udp_conn* conn)
{
  int i;
  static int seq_id = 0;
  char buf[MAX_PAYLOAD_LEN];

  PRINTF("Client sending to: ");
  PRINT6ADDR(&conn->ripaddr);

  i = sprintf(buf, "%d | ", ++seq_id);
  rpl_dag_t *dag = rpl_get_any_dag();
  if(dag && dag->instance->def_route) 
    add_ipaddr(buf + i, &dag->instance->def_route->ipaddr);
  else 
    sprintf(buf + i, "(null)");

  PRINTF(" (msg: %s)\n", buf);
  uip_udp_packet_send(conn, buf, strlen(buf));
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer et;
  static uip_ipaddr_t* address;
  static struct uip_udp_conn *conn = NULL;

  PROCESS_BEGIN();
  PRINTF("UDP client process started\n");

    if(ev == PROCESS_EVENT_INIT)
    {
      PRINTF("udp got address\n");
      address = (uip_ipaddr_t*)data;
      PRINT6ADDR(address);
    }

  etimer_set(&et, udp_interval);
  for(;;)
  {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_EXIT)
    {
      disconnect(conn);
      break;
    }
    else if(ev == tcpip_event)
    {
      tcpip_handler(conn);
    }
    else if(etimer_expired(&et))
    {
      if(conn == NULL)
        conn = connect(address);
      if(conn != NULL)
        send(conn);
    }

    etimer_set(&et, udp_interval);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
