
#define MYHA_AUTOSTART_PROCESSES(...)					\
  struct process * const myha_autostart_processes[] = {__VA_ARGS__, NULL}


CLIF extern struct process * const myha_autostart_processes[];

uip_ipaddr_t* myha_get_udp_address();
uip_ipaddr_t* myha_get_mqtt_address();
