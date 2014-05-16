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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

#include "contiki.h"
#include "contiki-net.h"

#include "mqtt-service.h"
#include "myha.h"
//#include "version.h"

#define SENSOR_ENABLE_PIN (G, 0)

#define DURATION_TICKS_PER_SECOND  (1000000L / 256)
#define HELD_DURATION  (DURATION_TICKS_PER_SECOND / 2)

#define PRINTF(...) printf(__VA_ARGS__)


PROCESS(touch_switch_controller_process, "Touch Switch Controller process");

PROCESS_NAME(myha_process);
PROCESS_NAME(myha_udp_client_process);

AUTOSTART_PROCESSES(&myha_process);
MYHA_AUTOSTART_PROCESSES(&myha_udp_client_process, &touch_switch_controller_process);


static int snprintf_node_topic_P(char* topic, size_t size, const char* format, ...)
{
  va_list ap;
  int len = snprintf_P(topic, size, PSTR("%s/"), myha_get_node_id());

  va_start(ap, format);
  len += vsnprintf_P(topic + len, size - len, format, ap);
  va_end(ap);

  return len;
}

static uint8_t value = 0;
static uint32_t duration = 0;
ISR(TIMER0_OVF_vect)
{
  static uint16_t x = 0;

  // Filter the input to ignore the heart beat pulses and other noise
  // It takes 5 sequential 1's to go from zero to 248
  // It takes 5 sequential 0's to go from 256 to 8
  // We're called roughly 4,000 times per second so a change in state
  // has to hold steady for a little over 1mS to be counted.
  x += ((ACSR & BIT(ACO)) ? 0 : 256);
  x >>= 1;

  if(x >= 248 && value == 0)
  {
    value = 1;
    duration = 0;
    process_poll(&touch_switch_controller_process);
  }
  else if(x <= 8 && value == 1)
  {
    value = 0;
    duration = 0;
    process_poll(&touch_switch_controller_process);
  }
  else
  {
    ++duration;

    if((duration & 0x7f) == 0 && value == 1)
      process_poll(&touch_switch_controller_process);
  }
}


/*---------------------------------------------------------------------------*/
/* Implementation of the client process */
PROCESS_THREAD(touch_switch_controller_process, ev, data)
{
  static uint8_t in_buffer[64];
  static uint8_t out_buffer[64];
  static char topic[64];
  static uip_ip6addr_t* address;
  static mqtt_connect_info_t connect_info = 
  {
    .client_id = "-",
    .username = NULL,
    .password = NULL,
    .will_topic = NULL,
    .will_message = NULL,
    .keepalive = 60,
    .will_qos = 0,
    .will_retain = 0,
    .clean_session = 1,
  };

  PROCESS_BEGIN();
    
  PRINTF("Myha started\n");

  connect_info.client_id = myha_get_node_id();

  if(ev == PROCESS_EVENT_INIT)
  {
    // Timer0: ClockSource = Clk/8 = 1,000,000 ticks/s
    TCCR0B |= BIT(CS01);
    // Timer0: Enable overflow interrupt, triggrs 1,000,000 / 256 = 3906 times/s
    TIMSK0 |= BIT(TOIE0);

    PIN_OUTPUT(SENSOR_ENABLE_PIN);
    PIN_HIGH(SENSOR_ENABLE_PIN);

    address = myha_get_mqtt_address();

    mqtt_init(in_buffer, sizeof(in_buffer), out_buffer, sizeof(out_buffer));
    mqtt_connect(address, UIP_HTONS(1883), 1, &connect_info);
  }

  for(;;)
  {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_EXIT)
    {
      mqtt_disconnect();
    }
    else if(ev == PROCESS_EVENT_POLL)
    {
      char message[4];
      static uint8_t last = 0;
      static uint8_t held = 0;

      if(value != last)
      {
        sprintf_P(topic, PSTR("%s/1/press"), myha_get_node_id());
        sprintf_P(message, PSTR("%u"), value);
        mqtt_publish(topic, message, 0, 0);

        if(held && value == 0)
        {
          held = 0;
          PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);

          sprintf_P(topic, PSTR("%s/1/hold"), myha_get_node_id());
          sprintf_P(message, PSTR("0"));
          mqtt_publish(topic, message, 0, 0);
        }

        last = value;
      }
      if(value == 1 && duration > HELD_DURATION && !held)
      {
        held = 1;
        sprintf_P(topic, PSTR("%s/1/hold"), myha_get_node_id());
        sprintf_P(message, PSTR("1"));
        mqtt_publish(topic, message, 0, 0);
      }
    }
    else if(ev == mqtt_event)
    {
      if(mqtt_event_is_connected(data))
      {
        PRINTF("Myha: MQTT connected\n");

        snprintf_node_topic_P(topic, sizeof(topic), PSTR("1/name"));
        mqtt_publish(topic, "Touch Switch Controller", 0, 0);
        PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);

        snprintf_node_topic_P(topic, sizeof(topic), PSTR("1/control/press/name"));
        mqtt_publish(topic, "Press", 0, 0);
        PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);
        snprintf_node_topic_P(topic, sizeof(topic), PSTR("1/control/press/type"));
        mqtt_publish(topic, "btn-press", 0, 0);

        snprintf_node_topic_P(topic, sizeof(topic), PSTR("1/control/hold/name"));
        mqtt_publish(topic, "Hold", 0, 0);
        PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);
        snprintf_node_topic_P(topic, sizeof(topic), PSTR("1/control/hold/type"));
        mqtt_publish(topic, "btn-hold", 0, 0);

        snprintf_node_topic_P(topic, sizeof(topic), PSTR("+/set/+"));
        PRINTF("Myha: subscribing: %s...\n", topic);
        mqtt_subscribe(topic);
        PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);
        PRINTF("    done.\n");
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
  }

  PROCESS_END();
}
