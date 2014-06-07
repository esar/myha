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

#include "contiki.h"
#include "contiki-net.h"
#include "net/linkaddr.h"

#include "sys/clock.h"
//#include "sys/rtimer.h"
#include "sys/etimer.h"
#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ip/psock.h"

#include "rako-service.h"
#include "mqtt-service.h"
#include "myha.h"
//#include "version.h"


#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x \n", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])


PROCESS(rako_bridge_process, "Rako Bridge process");

PROCESS_NAME(myha_process);
PROCESS_NAME(myha_udp_client_process);

AUTOSTART_PROCESSES(&myha_process);
MYHA_AUTOSTART_PROCESSES(&myha_udp_client_process, &rako_bridge_process);



static char* get_device_topic(char* buffer, int house, int room, int channel, char* name)
{
  sprintf_P(buffer, PSTR("%s/%02x%02x%01x/%s"), myha_get_node_id(),
            house, room, channel, name);
  return buffer;
}

static int split_set_topic(const char* topic, int* house, int* room, int* channel, char** name)
{
  char* p = strchr(topic, '/');
  if(p == NULL)
  {
    PRINTF("Myha: no '/'\n");
    return -1;
  }

  ++p;
  if(sscanf(p, "%2x%2x%1x", house, room, channel) != 3)
  {
    PRINTF("Myha: scanf failed: %s\n", p);
    return -1;
  }

  p += 5;
  if(*p != '/')
  {
    PRINTF("Myha: no '/'\n");
    return -1;
  }

  *name = ++p;
  
  return 0;
}

/*---------------------------------------------------------------------------*/
/* Implementation of the client process */
PROCESS_THREAD(rako_bridge_process, ev, data)
{
  static uint8_t in_buffer[64];
  static uint8_t out_buffer[64];
  static char topic[64];
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

  PROCESS_BEGIN();
    
  PRINTF("Myha started\n");

  if(ev == PROCESS_EVENT_INIT)
  {
    address = myha_get_mqtt_address();

    rako_init();

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
    else if(ev == mqtt_event)
    {
      if(mqtt_event_is_connected(data))
      {
        PRINTF("Myha: MQTT connected\n");

        sprintf_P(topic, PSTR("%s/name"), myha_get_node_id());
        mqtt_publish(topic, "Rako Bridge", 0, 0);
        PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);

        sprintf_P(topic, PSTR("%s/+/set/+"), myha_get_node_id());

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
        int house, room, channel;
        char* name;
        const char* topic = mqtt_event_get_topic(data);
        const char* message = mqtt_event_get_data(data);

        PRINTF("Myha: got publish: %s = %s\n", topic, message);
        if(split_set_topic(topic, &house, &room, &channel, &name) == 0)
        {
          if(strcmp_P(name, PSTR("set/level")) == 0)
          {
            int level = atoi(message);
            rako_send_level(house, room, channel, level);

            PRINTF("Myha: sending level: %u\n", level);
            PROCESS_WAIT_EVENT_UNTIL(ev == rako_sent_event);
            PRINTF("Myha: done\n");
          }
          else if(strcmp_P(name, PSTR("set/scene")) == 0)
          {
            int scene = atoi(message);
            if(scene == 0)
              rako_send_off(house, room, channel);
            else
              rako_send_scene(house, room, channel, scene);

            PRINTF("Myha: sending rako scene command, house: %u, room: %u, channel: %u\n", house, room, channel);
            PROCESS_WAIT_EVENT_UNTIL(ev == rako_sent_event);
            PRINTF("Myha: done\n");
          }
          else
            PRINTF("Myha: not level: %s\n", name);
        }
        else
          PRINTF("Myha: split failed\n");
      }
    }
    else if(ev == rako_cmd_event)
    {
      rako_cmd_t* cmd = (rako_cmd_t*)data;
      
      if(cmd->command == RAKO_CMD_SCENE_SET)
      {
        char message[4];
        get_device_topic(topic, cmd->house, cmd->room, 0, "scene");
        sprintf(message, "%u", cmd->data);
        mqtt_publish(topic, message, 0, 0);
      }
      else if(cmd->command == RAKO_CMD_OFF)
      {
        get_device_topic(topic, cmd->house, cmd->room, 0, "scene");
        mqtt_publish(topic, "0", 0, 0);
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

  PROCESS_END();
}
