PROJECT?=led-dimmer-6ch

all: $(PROJECT)

APPS = mqtt-service myha myha-udp-client
PROJECT_SOURCEFILES += colour-space.c

led-dimmer-6ch.c: led-dimmer-6ch-eeprom.h

include ../../Makefile.include
