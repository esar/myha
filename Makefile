PROJECT?=myha

# Configuration

RF_CHANNEL?=26
WITH_RPL?=1

# Applications 

WITH_UDPCLIENT?=1

# UDP client configuration

UDP_PERIOD?=5
WITH_UDP_CLIENT_AUTOSTART?=1

# End of user configuration section

all: $(PROJECT)

CONTIKI?=contiki
TARGETDIRS += platform

PROJECTDIR?=.

PROJECTDIRS += $(PROJECTDIR)/apps/udp-client $(PROJECTDIR)/apps/myhatest $(PROJECTDIR)/apps/mqtt-service $(PROJECTDIR)/apps/rako-service

WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS += -DUIP_CONF_IPV6=1

CFLAGS +=-DRF_CHANNEL=$(RF_CHANNEL)

ifneq ($(WITH_RPL),1)
UIP_CONF_RPL=0
CFLAGS += -DUIP_CONF_IPV6_RPL=0
endif

WITH_DELAY_IP=0
ifneq ($(WITH_DELAY_IP), 0)
CFLAGS += -DSLIP_RADIO=1 -DWITH_DELAY_IP=1
endif

ifneq ($(WITH_UDPCLIENT),0)
CFLAGS += -DUDPCLIENT=1
PROJECT_SOURCEFILES += udp-client.c
else
CFLAGS += -DUDPCLIENT=0
endif

PROJECT_SOURCEFILES += myhatest.c mqtt-service.c mqtt-msg.c rako-service.c

ifneq ($(UDP_PERIOD),)
CFLAGS += -DCETIC_6LBR_UDP_PERIOD=$(UDP_PERIOD)
endif

ifneq ($(WITH_UDP_CLIENT_AUTOSTART),0)
CFLAGS += -DUDP_CLIENT_AUTOSTART=1
endif

# Platform specific configuration

MSP430_20BITS=1

SMALL=1

CFLAGS+=-DMSP430_20BITS=$(MSP430_20BITS)

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

include $(CONTIKI)/Makefile.include
