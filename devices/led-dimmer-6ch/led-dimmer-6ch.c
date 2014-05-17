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

#define CHAN1_PIN (B, 7)
#define CHAN2_PIN (G, 5)
#define CHAN3_PIN (B, 5)
#define CHAN4_PIN (B, 6)
#define CHAN5_PIN (E, 3)
#define CHAN6_PIN (E, 4)

#define CHAN1_PWM (0, A)
#define CHAN2_PWM (0, B)
#define CHAN3_PWM (1, A)
#define CHAN4_PWM (1, B)
#define CHAN5_PWM (3, A)
#define CHAN6_PWM (3, B)

#define PRINTF(format, args...) printf_P(PSTR(format), ##args)

#define FADE_TIMER_TICKS_PER_SECOND  1000
#define NUM_DIMMER_CHANNELS 6
#define MANUAL_FADE_DELTA 10000

PROCESS(led_dimmer_6ch_process, "LED Dimmer 6 Channel process");

PROCESS_NAME(myha_process);
PROCESS_NAME(myha_udp_client_process);

AUTOSTART_PROCESSES(&myha_process);
MYHA_AUTOSTART_PROCESSES(&myha_udp_client_process, &led_dimmer_6ch_process);

uint8_t profile[64] PROGMEM = 
{
    0,   1,   2,   3,   4,   5,   6,   7,
    9,  10,  11,  13,  14,  16,  18,  19,
   21,  23,  25,  27,  29,  31,  33,  35,
   38,  40,  43,  46,  48,  51,  54,  57,
   60,  64,  67,  71,  75,  79,  83,  87,
   91,  96, 100, 105, 110, 116, 121, 127,
  133, 139, 145, 152, 159, 166, 173, 181,
  189, 197, 206, 214, 224, 233, 243, 254
};

typedef struct fade_state
{
  uint32_t current;
  int32_t  delta;
  uint8_t  target;

} fade_state_t;

static fade_state_t channel_fade_state[NUM_DIMMER_CHANNELS];
static uint8_t channel_on_state;
static uint8_t channel_direction_state;
static uint8_t channel_level[NUM_DIMMER_CHANNELS];


// multiply a 16bit integer by an 8bit integer, producing a 24bit result
static inline uint32_t mul_16_8(uint16_t a, uint8_t b)
{
        uint32_t product;
        asm (
                "mul %A1, %2\n\t"
                "movw %A0, r0\n\t"
                "clr %C0\n\t"
                "clr %D0\n\t"
                "mul %B1, %2\n\t"
                "add %B0, r0\n\t"
                "adc %C0, r1\n\t"
                "clr r1"
                : "=&r" (product)
                : "r" (a), "r" (b));
        return product;
}

uint16_t get_profiled_level(uint16_t x)
{
  uint8_t x_i, x_f, p1, p2;

  // scale 0->255 to 0->63
  x >>= 2;

  // split 8.8 fixed point into integer and fractional parts
  x_i = x >> 8;
  x_f = x & 0xff;

  // lookup two points in the profile
  p1 = pgm_read_byte(&profile[x_i]);
  if(x_i < 63)
    p2 = pgm_read_byte(&profile[x_i + 1]);
  else
    p2 = p1;

  // linearly interpolate between the two points
  // returning an 8.8 fixed point result
  if(p2 > p1)
  {
    uint8_t diff = p2 - p1;
    return (p1 << 8) + (mul_16_8(diff << 8, x_f) >> 8); 
  }
  else
  {
    uint8_t diff = p1 - p2;
    return (p1 << 8) - (mul_16_8(diff << 8, x_f) >> 8);
  }
}

static void fade_timer_enable()
{
  if(!BIT_GET(TIMSK1, OCIE1C))
  {
    OCR1C = 0xff;
    BIT_SET(TIFR1, OCF1C);
    BIT_SET(TIMSK1, OCIE1C);
  }
}

static void fade_timer_disable()
{
  BIT_CLR(TIMSK1, OCIE1C);
}

static void pwm_disable(int channel)
{
  switch(channel)
  {
    case 0: PWM_DISABLE(CHAN1_PWM); PIN_LOW(CHAN1_PIN); break;
    case 1: PWM_DISABLE(CHAN2_PWM); PIN_LOW(CHAN2_PIN); break;
    case 2: PWM_DISABLE(CHAN3_PWM); PIN_LOW(CHAN3_PIN); break;
    case 3: PWM_DISABLE(CHAN4_PWM); PIN_LOW(CHAN4_PIN); break;
    case 4: PWM_DISABLE(CHAN5_PWM); PIN_LOW(CHAN5_PIN); break;
    case 5: PWM_DISABLE(CHAN6_PWM); PIN_LOW(CHAN6_PIN); break;
  }
}

static void pwm_enable(int channel)
{
  switch(channel)
  {
    case 0: PWM_ENABLE(CHAN1_PWM); break;
    case 1: PWM_ENABLE(CHAN2_PWM); break;
    case 2: PWM_ENABLE(CHAN3_PWM); break;
    case 3: PWM_ENABLE(CHAN4_PWM); break;
    case 4: PWM_ENABLE(CHAN5_PWM); break;
    case 5: PWM_ENABLE(CHAN6_PWM); break;
  }
}

static void set_level_fp_8_8(int channel, uint16_t level)
{
  uint8_t last;

  level = get_profiled_level(level) >> 8; //pgm_read_byte(&profile[level >> 2]);
  switch(channel)
  {
    case 0: last = PWM_VALUE(CHAN1_PWM); PWM_VALUE(CHAN1_PWM) = level; break;
    case 1: last = PWM_VALUE(CHAN2_PWM); PWM_VALUE(CHAN2_PWM) = level; break;
    case 2: last = PWM_VALUE(CHAN3_PWM); PWM_VALUE(CHAN3_PWM) = level; break;
    case 3: last = PWM_VALUE(CHAN4_PWM); PWM_VALUE(CHAN4_PWM) = level; break;
    case 4: last = PWM_VALUE(CHAN5_PWM); PWM_VALUE(CHAN5_PWM) = level; break;
    case 5: last = PWM_VALUE(CHAN6_PWM); PWM_VALUE(CHAN6_PWM) = level; break;
    default: last = 0; break;
  }

  if(last == 0 && level != 0)
    pwm_enable(channel);
  else if(level == 0)
    pwm_disable(channel);
}

static void set_level(uint8_t channel, uint8_t level)
{
  set_level_fp_8_8(channel, (uint16_t)level << 8);
}

ISR(TIMER1_COMPC_vect)
{
  static uint8_t tick = 0;
  int i;
  int all_done = 1;

  // Only do anything once every 4 ticks, which is roughly once every 1mS
  // (8MHz (cpu clock) / 8 (timer clock divider) / 256 (output compare) / 4) = 1.024mS
  ++tick;
  if((tick & 3) != 0)
    return;

  for(i = 0; i < NUM_DIMMER_CHANNELS; ++i)
  {
    if(channel_fade_state[i].delta != 0)
    {
      uint8_t new_level;

      channel_fade_state[i].current += channel_fade_state[i].delta;
      new_level = channel_fade_state[i].current >> 16;

      if((channel_fade_state[i].delta > 0 && new_level >= channel_fade_state[i].target) ||
         (channel_fade_state[i].delta < 0 && new_level <= channel_fade_state[i].target))
      {
        channel_fade_state[i].current = (uint32_t)channel_fade_state[i].target << 16;
        channel_fade_state[i].delta = 0;
      }
      else
        all_done = 0;

      set_level_fp_8_8(i, channel_fade_state[i].current >> 8);
    }
  }

  if(all_done)
    fade_timer_disable();
}

static int snprintf_node_topic_P(char* topic, size_t size, const char* format, ...)
{
  va_list ap;
  int len = snprintf_P(topic, size, PSTR("%s/"), myha_get_node_id());

  va_start(ap, format);
  len += vsnprintf_P(topic + len, size - len, format, ap);
  va_end(ap);

  return len;
}

static void calc_fade(int channel)
{
  const long update_time = FADE_TIMER_TICKS_PER_SECOND / 2;
  long distance;

  distance = channel_fade_state[channel].target - (channel_fade_state[channel].current >> 16);
  if(distance < 0)
  {
    distance = 0 - distance;
    channel_fade_state[channel].delta = (distance << 16) / update_time;
    channel_fade_state[channel].delta = 0 - channel_fade_state[channel].delta;
  }
  else
    channel_fade_state[channel].delta = (distance << 16) / update_time;
}

static void fade_to_level(int channel, int level)
{
  channel_fade_state[channel].target = level;
  calc_fade(channel);
  if(channel_fade_state[channel].delta != 0)
    fade_timer_enable();
}

static void process_event_set_level(int device, int max, const char* message)
{
  uint8_t new_level = atoi(message);

  // scale 0->100 to 0->255
  new_level = new_level + new_level + (new_level >> 1);
  if(new_level >= 250)
    new_level = 255;

  fade_timer_disable();
  for(; device < max; ++device)
  {
    channel_level[device] = new_level;
    if(channel_level[device] > 0)
      channel_on_state |= BIT(device);
    channel_fade_state[device].target = new_level;
    calc_fade(device);
  }
  fade_timer_enable();
}

static void process_event_set_state(int device, int max, const char* message)
{
  if(strcmp_P(message, PSTR("on")) == 0)
  {
    for(; device < max; ++device)
      channel_on_state |= BIT(device);
  }
  else if(strcmp_P(message, PSTR("off")) == 0)
  {
    for(; device < max; ++device)
      channel_on_state &= ~BIT(device);
  }
  else if(strcmp_P(message, PSTR("toggle")) == 0)
  {
    for(; device < max; ++device)
      channel_on_state ^= BIT(device);
  }

  fade_timer_disable();
  for(device = 0; device < NUM_DIMMER_CHANNELS; ++device)
  {
    channel_fade_state[device].target = (channel_on_state & BIT(device)) ? channel_level[device] : 0;
    calc_fade(device);
  }
  fade_timer_enable();
}

static void process_event_set_btnhold(int device, int max, const char* message)
{
  if(strcmp_P(message, PSTR("0")) == 0)
  {
    fade_timer_disable();
    for(; device < max; ++device)
    {
      channel_fade_state[device].target = channel_fade_state[device].current >> 16;
      channel_fade_state[device].delta = 0;
      channel_level[device] = channel_fade_state[device].target;
      if(channel_level[device] > 0)
        channel_on_state |= BIT(device);
    }
    fade_timer_enable();
  }
  else
  {
    uint8_t dir;
    if(device == 0 && max == NUM_DIMMER_CHANNELS)
    {
      channel_direction_state ^= BIT(7);
      dir = channel_direction_state & BIT(7);
    }
    else
    {
      channel_direction_state ^= BIT(device);
      dir = channel_direction_state & BIT(device);
    }

    fade_timer_disable();
    if(dir)
    {
      for(; device < max; ++device)
      {
        channel_fade_state[device].target = 255;
        channel_fade_state[device].delta = MANUAL_FADE_DELTA;
      }
    }
    else
    {
      for(; device < max; ++device)
      {
        channel_fade_state[device].target = 0;
        channel_fade_state[device].delta = -MANUAL_FADE_DELTA;
      }
    }
    fade_timer_enable();
  }
}

static void process_local_event(int device, const char* topic, const char* message)
{
  int max = device;

  if(device == 0)
    max = NUM_DIMMER_CHANNELS;
  else
    --device;

  PRINTF("Myha: dev: %u, name: %s\n", device, topic);
  if(strcmp_P(topic, PSTR("set/level")) == 0)
    return process_event_set_level(device, max, message);
  if(strcmp_P(topic, PSTR("set/state")) == 0)
    return process_event_set_state(device, max, message);
  if(strcmp_P(topic, PSTR("set/btnhold")) == 0)
    return process_event_set_btnhold(device, max, message);

  // set/name
  // set/location
  // set/profile
  // set/fade_rate
  // set/scene
  // set/scene/x/level

  // set/config/probe
  // set/control/probe
}

static void process_event(const char* topic, const char* message)
{
  const char* p = strchr(topic, '/');
  if(p - topic == 16 && strncmp(topic, myha_get_node_id(), 16) == 0)
  {
    int device = 0;

    ++p;
    while(*p >= '0' && *p <= '9')
    {
      device = device * 10 + *p - '0';
      ++p;
    }
    if(*p == '/')
      process_local_event(device, p + 1, message);
    return;
  }


  if(strcmp_P(topic, PSTR("0004a30b00195f35/1/press")) == 0)
  {
    if(strcmp_P(message, PSTR("1")) == 0)
      process_local_event(0, "set/state", "toggle");
  }
  else if(strcmp_P(topic, PSTR("0004a30b00195f35/1/hold")) == 0)
  {
    process_local_event(0, "set/btnhold", message);
  }
}

static void hardware_init()
{
  PWM_VALUE(CHAN1_PWM) = 32;
  PWM_VALUE(CHAN2_PWM) = 32;
  PWM_VALUE(CHAN3_PWM) = 32;
  PWM_VALUE(CHAN4_PWM) = 32;
  PWM_VALUE(CHAN5_PWM) = 32;
  PWM_VALUE(CHAN6_PWM) = 32;

  // Timer0: CompareMatchA = ClearOnMatch, CompareMatchB = ClearOnMatch,
  //         WaveformGenerationMode = FastPWM, ClockSelect = Clk/8 
  TCCR0A = BIT(COM0A1) | BIT(COM0B1) | BIT(WGM01) | BIT(WGM00);
  TCCR0B = BIT(CS01);

  // Timer1: CompareMatchA = ClearOnMatch, CompareMatchB = ClearOnMatch,
  //         WaveformGenerationMode = FastPWM, ClockSelect = Clk/8
  TCCR1A = BIT(COM1A1) | BIT(COM1B1) | BIT(WGM10);
  TCCR1B = BIT(WGM12) | BIT(CS11);

  // Timer3: CompareMatchA = ClearOnMatch, CompareMatchB = ClearOnMatch,
  //         WaveformGenerationMode = FastPWM, ClockSelect = Clk/8
  TCCR3A = BIT(COM3A1) | BIT(COM3B1) | BIT(WGM30);
  TCCR3B = BIT(WGM32) | BIT(CS31);

  PIN_OUTPUT(CHAN1_PIN);
  PIN_OUTPUT(CHAN2_PIN);
  PIN_OUTPUT(CHAN3_PIN);
  PIN_OUTPUT(CHAN4_PIN);
  PIN_OUTPUT(CHAN5_PIN);
  PIN_OUTPUT(CHAN6_PIN);
}

/*---------------------------------------------------------------------------*/
/* Implementation of the client process */
PROCESS_THREAD(led_dimmer_6ch_process, ev, data)
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
    
  connect_info.client_id = myha_get_node_id();

  PRINTF("Myha started\n");

  if(ev == PROCESS_EVENT_INIT)
  {
    hardware_init();

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
    else if(ev == mqtt_event)
    {
      if(mqtt_event_is_connected(data))
      {
        PRINTF("Myha: MQTT connected\n");

        snprintf_node_topic_P(topic, sizeof(topic), PSTR("1/name"));
        mqtt_publish(topic, "6 Channel LED Dimmer", 0, 0);
        PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);

        snprintf_node_topic_P(topic, sizeof(topic), PSTR("1/control/level/name"));
        mqtt_publish(topic, "Level", 0, 0);
        PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);

        snprintf_node_topic_P(topic, sizeof(topic), PSTR("1/control/level/type"));
        mqtt_publish(topic, "percent", 0, 0);
        PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);

        sprintf_P(topic, PSTR("0004a30b00195f35/1/press"));
        mqtt_subscribe(topic);
        PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);

        sprintf_P(topic, PSTR("0004a30b00195f35/1/hold"));
        mqtt_subscribe(topic);
        PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);

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
        process_event(topic, message);
      }
    }
  }

  PROCESS_END();
}
