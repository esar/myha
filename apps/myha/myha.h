
#define PORTCAT(a,b,c)     a##b##c
#define PIN(port, pin)     PORTCAT(PIN,port,)
#define PINPIN(port, pin)  PORTCAT(PIN,port,pin)
#define PORT(port, pin)    PORTCAT(PORT,port,)
#define PORTPIN(port, pin) PORTCAT(PORT,port,pin)
#define DDR(port, pin)     PORTCAT(DDR,port,)
#define DD(port, pin)      PORTCAT(DD,port,pin)

#define BIT(i)             (1 << (i))
#define BIT_GET(value, i)  ((value) & BIT(i))
#define BIT_SET(value, i)  ((value) |= BIT(i))
#define BIT_CLR(value, i)  ((value) &= ~BIT(i))

#define PIN_GET(x)         BIT_GET(PIN x, PINPIN x)
#define PIN_HIGH(x)        BIT_SET(PORT x, PORTPIN x)
#define PIN_LOW(x)         BIT_CLR(PORT x, PORTPIN x)
#define PIN_OUTPUT(x)      BIT_SET(DDR x, DD x)
#define PIN_INPUT(x)       BIT_CLR(DDR x, DD x)


#define PWMCAT(a,b,c,d)           a##b##c##d
#define PWM_TCCRA(timer, oc)      PWMCAT(TCCR,timer,A,)
#define PWM_OCR(timer, oc)        PWMCAT(OCR,timer,oc,)
#define PWM_MODE_MASK(timer, oc)  (BIT(PWMCAT(COM,timer,oc,1)) | BIT(PWMCAT(COM,timer,oc,0)))
#define PWM_MODE_VAL(timer, oc)   BIT(PWMCAT(COM,timer,oc,1))

#define PWM_VALUE(x)              (PWM_OCR x)
#define PWM_ENABLE(x)             (PWM_TCCRA x) |= (PWM_MODE_VAL x)
#define PWM_DISABLE(x)            (PWM_TCCRA x) &= ~(PWM_MODE_MASK x)


#define MYHA_AUTOSTART_PROCESSES(...)					\
  struct process * const myha_autostart_processes[] = {__VA_ARGS__, NULL}


CLIF extern struct process * const myha_autostart_processes[];

uip_ipaddr_t* myha_get_udp_address();
uip_ipaddr_t* myha_get_mqtt_address();
const char* myha_get_node_id();
