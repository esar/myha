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
#include "colour-space.h"

#define FADE_TIMER_TICKS_PER_SECOND  1000
#define NUM_DIMMER_CHANNELS 6
#define MANUAL_FADE_DELTA 10000
#include "led-dimmer-6ch-eeprom.h"

#define CHAN1_PIN (B, 5)
#define CHAN2_PIN (B, 7)
#define CHAN3_PIN (B, 6)
#define CHAN4_PIN (G, 5)
#define CHAN5_PIN (E, 3)
#define CHAN6_PIN (E, 4)

#define CHAN1_PWM (1, A)
#define CHAN2_PWM (0, A)
#define CHAN3_PWM (1, B)
#define CHAN4_PWM (0, B)
#define CHAN5_PWM (3, A)
#define CHAN6_PWM (3, B)

#define PRINTF(format, args...) printf_P(PSTR(format), ##args)


PROCESS(led_dimmer_6ch_process, "LED Dimmer 6 Channel process");

PROCESS_NAME(myha_process);
PROCESS_NAME(myha_udp_client_process);

AUTOSTART_PROCESSES(&myha_process);
MYHA_AUTOSTART_PROCESSES(&myha_udp_client_process, &led_dimmer_6ch_process);


typedef int (*topic_getter_t)(char* buffer, int buffer_size, int device);
typedef void (*topic_setter_t)(int device, int max, const char* value);
typedef struct topic_handler
{
  const char* topic;
  topic_setter_t set;
  topic_getter_t get;

} topic_handler_t;

typedef struct fade_state
{
  uint32_t current;
  int32_t  delta;
  uint8_t  target;

} fade_state_t;


static void topic_set_level(int device, int max, const char* value);
static int  topic_get_level(char* buffer, int buffer_size, int device);
static void topic_set_state(int device, int max, const char* value);
static int  topic_get_state(char* buffer, int buffer_size, int device);
static void topic_set_hue(int device, int max, const char* value);
static int  topic_get_hue(char* buffer, int buffer_size, int device);
static void topic_set_saturation(int device, int max, const char* value);
static int  topic_get_saturation(char* buffer, int buffer_size, int device);
static void topic_set_brightness(int device, int max, const char* value);
static int  topic_get_brightness(char* buffer, int buffer_size, int device);
static void topic_set_name(int device, int max, const char* value);
static int  topic_get_name(char* buffer, int buffer_size, int device);
static void topic_set_location(int device, int max, const char* value);
static int  topic_get_location(char* buffer, int buffer_size, int device);
static void topic_set_fade_rate(int device, int max, const char* value);
static int  topic_get_fade_rate(char* buffer, int buffer_size, int device);

static const char topic_name_level[]      PROGMEM = "level";
static const char topic_name_state[]      PROGMEM = "state";
static const char topic_name_hue[]        PROGMEM = "hue";
static const char topic_name_saturation[] PROGMEM = "saturation";
static const char topic_name_brightness[] PROGMEM = "brightness";
static const char topic_name_name[]       PROGMEM = "name";
static const char topic_name_location[]   PROGMEM = "location";
static const char topic_name_fade_rate[]  PROGMEM = "fade_rate";
static topic_handler_t topic_handlers[] PROGMEM = 
{
  { topic_name_level,      topic_set_level,      topic_get_level      },
  { topic_name_state,      topic_set_state,      topic_get_state      },
  { topic_name_hue,        topic_set_hue,        topic_get_hue        },
  { topic_name_saturation, topic_set_saturation, topic_get_saturation },
  { topic_name_brightness, topic_set_brightness, topic_get_brightness },
  { topic_name_name,       topic_set_name,       topic_get_name       },
  { topic_name_location,   topic_set_location,   topic_get_location   },
  { topic_name_fade_rate,  topic_set_fade_rate,  topic_get_fade_rate  },
  { NULL }
};

#define TOPIC_LEVEL       0
#define TOPIC_STATE       1
#define TOPIC_HUE         2
#define TOPIC_SATURATION  3
#define TOPIC_BRIGHTNESS  4
#define TOPIC_NAME        5
#define TOPIC_LOCATION    6
#define TOPIC_FADE_RATE   7

#define PUBLISH(channel, topic_index)  do { publish_mask |= BIT(channel); channel_publish_mask[channel] |= BIT(topic_index); } while(0)

static fade_state_t channel_fade_state[NUM_DIMMER_CHANNELS];
static uint8_t channel_on_state = 0xFF;
static uint8_t channel_direction_state = 0;
static uint8_t channel_level[NUM_DIMMER_CHANNELS] = { 32, 32, 32, 32, 32, 32 };

static uint8_t publish_mask;
static uint16_t channel_publish_mask[NUM_DIMMER_CHANNELS];

// calculate log2 of a 16 bit number that is known to be a power of 2
static uint8_t log2_p2_16(uint16_t x)
{
  uint16_t r;
  r  =  (x & 0xAAAA) != 0;
  r |= ((x & 0xCCCC) != 0) << 1;
  r |= ((x & 0xF0F0) != 0) << 2;
  r |= ((x & 0xFF00) != 0) << 3;
  return r;
}

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

uint8_t relative_atoi(uint8_t current_value, const char* new_value)
{
  if(new_value[0] == '+')
    return current_value + atoi(new_value + 1);
  else if(new_value[0] == '-')
    return current_value - atoi(new_value + 1);
  else
    return atoi(new_value);
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
  p1 = eeprom_read_byte(&eeprom.channel[0].profile[x_i]);
  if(x_i < 63)
    p2 = eeprom_read_byte(&eeprom.channel[0].profile[x_i + 1]);
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
  const long update_time = (FADE_TIMER_TICKS_PER_SECOND / 10) * eeprom_read_byte(&eeprom.channel[channel].fade_rate);
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

static void store_level(uint8_t device, uint8_t level)
{
  if(level != channel_level[device])
  {
    channel_level[device] = level;
    PUBLISH(device, TOPIC_LEVEL);

    // TODO: if channel is part of an rgb set then publish
    //       new hue, saturation and brightness
    if(0)
    {
      PUBLISH(device, TOPIC_HUE);
      PUBLISH(device, TOPIC_SATURATION);
      PUBLISH(device, TOPIC_BRIGHTNESS);
    }
  }
}

static void store_state(uint8_t device, uint8_t state)
{
  if(((channel_on_state & BIT(device)) == 0) != (state == 0))
  {
    channel_on_state ^= BIT(device);
    PUBLISH(device, TOPIC_STATE);
  }
}

static void topic_set_level(int device, int max, const char* message)
{
  fade_timer_disable();
  for(; device < max; ++device)
  {
    uint8_t new_level = relative_atoi(channel_level[device], message);
    store_level(device, new_level);
    store_state(device, new_level > 0);
    channel_fade_state[device].target = new_level;
    calc_fade(device);
  }
  fade_timer_enable();
}

static int topic_get_level(char* buffer, int buffer_size, int device)
{
  return snprintf(buffer, buffer_size, "%u", channel_level[device]);
}

static void topic_set_hue(int device, int max, const char* message)
{
  uint8_t i, h, s, v, r, g, b;

  rgb2hsv(channel_level[0], channel_level[1], channel_level[2], &h, &s, &v);
  h = relative_atoi(h, message);
  hsv2rgb(h, s, v, &r, &g, &b);

  store_level(0, r);
  store_level(1, g);
  store_level(2, b);

  fade_timer_disable();
  for(i = 0; i < 3; ++i)
  {
    channel_fade_state[i].target = channel_level[i];
    calc_fade(i);
  }
  fade_timer_enable();
}

static int topic_get_hue(char* buffer, int buffer_size, int device)
{
  uint8_t h, s, v;
  rgb2hsv(channel_level[0], channel_level[1], channel_level[2], &h, &s, &v);
  return snprintf(buffer, buffer_size, "%u", h);
}

static void topic_set_saturation(int device, int max, const char* message)
{
  uint8_t i, h, s, v, r, g, b;

  rgb2hsv(channel_level[0], channel_level[1], channel_level[2], &h, &s, &v);
  s = relative_atoi(s, message);
  hsv2rgb(h, s, v, &r, &g, &b);

  store_level(0, r);
  store_level(1, g);
  store_level(2, b);

  fade_timer_disable();
  for(i = 0; i < 3; ++i)
  {
    channel_fade_state[i].target = channel_level[i];
    calc_fade(i);
  }
  fade_timer_enable();
}

static int topic_get_saturation(char* buffer, int buffer_size, int device)
{
  uint8_t h, s, v;
  rgb2hsv(channel_level[0], channel_level[1], channel_level[2], &h, &s, &v);
  return snprintf(buffer, buffer_size, "%u", s);
}

static void topic_set_brightness(int device, int max, const char* message)
{
  uint8_t i, h, s, v, r, g, b;

  rgb2hsv(channel_level[0], channel_level[1], channel_level[2], &h, &s, &v);
  v = relative_atoi(v, message);
  hsv2rgb(h, s, v, &r, &g, &b);

  store_level(0, r);
  store_level(1, g);
  store_level(2, b);

  fade_timer_disable();
  for(i = 0; i < 3; ++i)
  {
    channel_fade_state[i].target = channel_level[i];
    calc_fade(i);
  }
  fade_timer_enable();
}

static int topic_get_brightness(char* buffer, int buffer_size, int device)
{
  uint8_t h, s, v;
  rgb2hsv(channel_level[0], channel_level[1], channel_level[2], &h, &s, &v);
  return snprintf(buffer, buffer_size, "%u", v);
}

static void topic_set_state(int device, int max, const char* message)
{
  if(strcmp_P(message, PSTR("on")) == 0)
  {
    for(; device < max; ++device)
      store_state(device, 1);
  }
  else if(strcmp_P(message, PSTR("off")) == 0)
  {
    for(; device < max; ++device)
      store_state(device, 0);
  }
  else if(strcmp_P(message, PSTR("toggle")) == 0)
  {
    for(; device < max; ++device)
      store_state(device, !(channel_on_state & BIT(device)));
  }

  fade_timer_disable();
  for(device = 0; device < NUM_DIMMER_CHANNELS; ++device)
  {
    channel_fade_state[device].target = (channel_on_state & BIT(device)) ? channel_level[device] : 0;
    calc_fade(device);
  }
  fade_timer_enable();
}

static int topic_get_state(char* buffer, int buffer_size, int device)
{
  if(channel_on_state & BIT(device))
    return snprintf_P(buffer, buffer_size, PSTR("on"));
  else
    return snprintf_P(buffer, buffer_size, PSTR("off"));
}

static void topic_set_btnhold(int device, int max, const char* message)
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

static void topic_set_name(int device, int max, const char* message)
{
  int length = strlen(message);
  if(length >= sizeof(eeprom.channel[0].name))
    length = sizeof(eeprom.channel[0].name) - 1;

  for(; device < max; ++device)
  {
    eeprom_write_block(message, &eeprom.channel[device].name, length);
    eeprom_write_byte((uint8_t*)&eeprom.channel[device].name[length], 0);
    PUBLISH(device, TOPIC_NAME);
  }
}

static int topic_get_name(char* buffer, int buffer_size, int device)
{
  int len = sizeof(eeprom.channel[device].name);
  if(len >= buffer_size)
    len = buffer_size - 1;
  eeprom_read_block(buffer, eeprom.channel[device].name, len);
  len = strlen(buffer);
  buffer[len] = '\0';
  return len; 
}

static void topic_set_location(int device, int max, const char* message)
{
  int length = strlen(message);
  if(length >= sizeof(eeprom.channel[0].location))
    length = sizeof(eeprom.channel[0].location) - 1;

  for(; device < max; ++device)
  {
    eeprom_write_block(message, &eeprom.channel[device].location, length);
    eeprom_write_byte((uint8_t*)&eeprom.channel[device].location[length], 0);
    PUBLISH(device, TOPIC_LOCATION);
  }
}

static int topic_get_location(char* buffer, int buffer_size, int device)
{
  int len = sizeof(eeprom.channel[device].location);
  if(len >= buffer_size)
    len = buffer_size - 1;
  eeprom_read_block(buffer, eeprom.channel[device].location, len);
  len = strlen(buffer);
  buffer[len] = '\0';
  return len; 
}

static void topic_set_fade_rate(int device, int max, const char* message)
{
  for(; device < max; ++device)
  {
    uint8_t rate = relative_atoi(eeprom_read_byte(&eeprom.channel[device].fade_rate), message);
    eeprom_write_byte(&eeprom.channel[device].fade_rate, rate);
  }
}

static int topic_get_fade_rate(char* buffer, int buffer_size, int device)
{
  return snprintf(buffer, buffer_size, "%u", eeprom_read_byte(&eeprom.channel[device].fade_rate));
}

static void process_local_event(int device, const char* topic, const char* message)
{
  PRINTF("Myha: dev: %u, name: %s\n", device, topic);
  if(strncmp_P(topic, PSTR("set/"), 4) == 0)
  {
    int i;
    int max = device;

    if(device == 0)
      max = NUM_DIMMER_CHANNELS;
    else
      --device;

    topic += 4;

    for(i = 0; i < sizeof(topic_handlers) / sizeof(*topic_handlers); ++i)
    {
      if(strcmp_P(topic, (PGM_P)pgm_read_word(&topic_handlers[i].topic)) == 0)
      {
        PRINTF("setter match\n");
        topic_setter_t setter = (topic_setter_t)pgm_read_word(&topic_handlers[i].set);
        if(setter != NULL)
        {
          PRINTF("calling setter\n");
          setter(device, max, message);
        }
        break;
      }
    }
  }

  // set/profile
  // set/scene
  // set/scene/x/level

  // set/config/probe
  // set/control/probe
}

static int process_publish(char* topic_buffer, int topic_buffer_size,
                           char* message_buffer, int message_buffer_size,
                           int channel, int topic_index)
{
  if(topic_index < sizeof(topic_handlers) / sizeof(*topic_handlers))
  {
    topic_getter_t getter = (topic_getter_t)pgm_read_word(&topic_handlers[topic_index].get);
    if(getter != NULL)
    {
      snprintf_P(topic_buffer, topic_buffer_size, PSTR("%s/%u/%S"),
                 myha_get_node_id(), channel, pgm_read_word(&topic_handlers[topic_index].topic));
      return getter(message_buffer, message_buffer_size, channel);
    }
  }

  return 0;
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
        static int i;

        PRINTF("Myha: MQTT connected\n");

        for(i = 0; i < NUM_DIMMER_CHANNELS; ++i)
        {
          char name[32];
          snprintf_node_topic_P(topic, sizeof(topic), PSTR("%u/name"), i);
          eeprom_read_block(name, &eeprom.channel[i].name[0], sizeof(eeprom.channel[i].name));
          mqtt_publish(topic, name, 0, 0);
          PROCESS_WAIT_EVENT_UNTIL(ev == mqtt_event);
        }

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

      if(publish_mask != 0 && mqtt_ready())
      {
        int channel, i;
        int published_one = 0;

        for(channel = 0; channel < NUM_DIMMER_CHANNELS && !published_one; ++channel)
        {
          if(publish_mask & BIT(channel))
          {
            for(i = 0; i < sizeof(topic_handlers) / sizeof(*topic_handlers) && !published_one; ++i)
            {
              if(channel_publish_mask[channel] & BIT(i))
              {
                char message[32];
                if(process_publish(topic, sizeof(topic), message, sizeof(message), channel, i))
                {
                  PRINTF("publishing %s = %s\n", topic, message);
                  mqtt_publish(topic, message, 0, 0);
                  published_one = 1;
                }

                channel_publish_mask[channel] &= ~BIT(i);
              }
            }

            if(channel_publish_mask == 0)
              publish_mask &= ~BIT(channel);
          }
        }
      }
    }
  }

  PROCESS_END();
}
