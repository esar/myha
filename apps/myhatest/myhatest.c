
#include "contiki.h"
#include "contiki-net.h"

#include "sys/clock.h"
//#include "sys/rtimer.h"
#include "sys/etimer.h"
#include <string.h>
#include <stdio.h>

#include "ip/psock.h"

#include "rako-service.h"
#include "mqtt-service.h"
//#include "version.h"


#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x \n", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])



/*---------------------------------------------------------------------------*/
/* Declare the client process */
PROCESS(mqtt_client_process, "MQTT Client process");


/*---------------------------------------------------------------------------*/
/* Implementation of the client process */
PROCESS_THREAD(mqtt_client_process, ev, data)
{
  static uint8_t in_buffer[64];
  static uint8_t out_buffer[64];
  static uip_ip6addr_t* address;
  static mqtt_connect_info_t connect_info = 
  {
    .client_id = "contiki",
    .username = NULL,
    .password = NULL,
    .will_topic = NULL,
    .will_message = NULL,
    .keepalive = 60,
    .will_qos = 0,
    .will_retain = 0,
    .clean_session = 1,
  };

  static const char* topics[] = 
  {
    "0", "1", "2", "3", "4", "5", NULL
  };

  PROCESS_BEGIN();
    
  PRINTF("Myha started\n");

  if(ev == PROCESS_EVENT_INIT)
  {
    address = (uip_ip6addr_t*)data;

    rako_init();

    //mqtt_init(in_buffer, sizeof(in_buffer), out_buffer, sizeof(out_buffer));
    //mqtt_connect(address, UIP_HTONS(1883), 1, &connect_info);
  }

  for(;;)
  {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_EXIT)
    {
      mqtt_disconnect();
    }
    else if(ev == mqtt_event)
    {
      if(mqtt_event_is_connected(data))
      {
        static int i;

        PRINTF("Myha: MQTT connected\n");

        for(i = 0; topics[i] != NULL; ++i)
        {
          PRINTF("Myha: subscribing: %s...\n", topics[i]);
          mqtt_subscribe(topics[i]);
          PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);
          PRINTF("    done.\n");
        }
      }
      else if(mqtt_event_is_disconnected(data))
      {
        PRINTF("Myha: MQTT disconnected\n");
      }
      else if(mqtt_event_is_publish(data))
      {
        const char* topic = mqtt_event_get_topic(data);
        const char* message = mqtt_event_get_data(data);

        PRINTF("Myha: got publish: %s = %s\n", topic, message);
      }
    }
    else if(ev == rako_recv_event)
    {
      rako_msg_t* msg = (rako_msg_t*)data;
      PRINTF("RAKO: received: raw: %08lx, house: %u, room: %u\n", msg->raw, msg->command.house, msg->command.room);
    }
    else if(ev == rako_sent_event)
    {
      PRINTF("RAKO: sent\n");
    }
  }


/*
  TIFR0 = 0;
  TIMSK0 = 0;
  OCR0A = 64;
  OCR0B = 64;
  TCCR0A = 0xA3;
  TCCR0B = 2;

  OCR1A = 128;
  OCR1B = 128;
  OCR1C = 128;
  TCCR1A = 0xA1;
  TCCR1B = 0x0A;

  OCR3A = 128;
  OCR3B = 128;
  TCCR3A = 0xA1;
  TCCR3B = 0x0A;

  PORTB |= (1 << PORTB7);
  PORTG |= (1 << PORTG5);
  DDRB |= (1 << DDB7); // OC0A is output
  DDRG |= (1 << DDG5); // OC0B is output
  DDRB |= (1 << DDB5); // OC1A is output
  DDRB |= (1 << DDB6); // OC1B is output
  DDRE |= (1 << DDE3); // OC3A is output
  DDRE |= (1 << DDE4); // OC3B is output
*/

  PROCESS_END();
}
