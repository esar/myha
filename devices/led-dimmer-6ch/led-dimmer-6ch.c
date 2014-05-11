
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

PROCESS(led_dimmer_6ch_process, "LED Dimmer 6 Channel process");

PROCESS_NAME(myha_process);
PROCESS_NAME(myha_udp_client_process);

AUTOSTART_PROCESSES(&myha_process);
MYHA_AUTOSTART_PROCESSES(&myha_udp_client_process, &led_dimmer_6ch_process);

typedef struct dimmer_channel
{
  uint32_t level;
  int32_t  delta;
  uint8_t  target;

} dimmer_channel_t;

static dimmer_channel_t channels[NUM_DIMMER_CHANNELS];


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

static void set_level(int channel, int level)
{
  uint8_t last;

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
    if(channels[i].delta != 0)
    {
      uint16_t new_level;

      channels[i].level += channels[i].delta;
      new_level = channels[i].level >> 16;

      if((channels[i].delta > 0 && new_level >= channels[i].target) ||
         (channels[i].delta < 0 && new_level <= channels[i].target))
      {
        new_level = channels[i].target;
        channels[i].level = (uint32_t)new_level << 16;
        channels[i].delta = 0;
      }
      else
        all_done = 0;

      set_level(i, new_level);
    }
  }

  if(all_done)
    fade_timer_disable();
}

static int split_set_topic(const char* topic, int* device, char** name)
{
  char* p = strchr(topic, '/');
  if(p == NULL)
  {
    PRINTF("Myha: no /\n");
    return -1;
  }

  ++p;
  *device = 0;
  while(*p >= '0' && *p <= '9')
  {
    *device = *device * 10 + *p - '0';
    ++p;
  }

  if(*p != '/')
  {
    PRINTF("Myha: no /\n");
    return -1;
  }

  *name = ++p;
  
  return 0;
}

static int snprintf_node_topic_P(char* topic, size_t size, const char* format, ...)
{
  va_list ap;
  int len = snprintf_P(topic, size, PSTR("%02x%02x%02x%02x%02x%02x%02x%02x/"), 
                       linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
                       linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[3],
                       linkaddr_node_addr.u8[4], linkaddr_node_addr.u8[5],
                       linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

  va_start(ap, format);
  len += vsnprintf_P(topic + len, size - len, format, ap);
  va_end(ap);

  return len;
}

static void calc_fade(int channel)
{
  const long update_time = FADE_TIMER_TICKS_PER_SECOND / 2;
  long distance;

  distance = channels[channel].target - (channels[channel].level >> 16);
  if(distance < 0)
  {
    distance = 0 - distance;
    channels[channel].delta = (distance << 16) / update_time;
    channels[channel].delta = 0 - channels[channel].delta;
  }
  else
    channels[channel].delta = (distance << 16) / update_time;
}

static void fade_to_level(int channel, int level)
{
  channels[channel].target = level;
  calc_fade(channel);
  if(channels[channel].delta != 0)
    fade_timer_enable();
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
        int device;
        char* name;
        const char* topic = mqtt_event_get_topic(data);
        const char* message = mqtt_event_get_data(data);

        PRINTF("Myha: got publish: %s = %s\n", topic, message);
        if(split_set_topic(topic, &device, &name) == 0)
        {
          PRINTF("Myha: dev: %u, name: %s\n", device, name);
          if(strcmp(name, "set/level") == 0)
          {
            int level = atoi(message) << 1;
            if(level >= 200)
              level = 255;

            if(device == 0)
            {
              int i;
              for(i = 0; i < NUM_DIMMER_CHANNELS; ++i)
                fade_to_level(i, level);
            }
            else if(device >= 1 && device <= 6)
              fade_to_level(device - 1, level);
          }
        }
        else
          PRINTF("Myha: split failed\n");
      }
    }
  }

  PROCESS_END();
}
