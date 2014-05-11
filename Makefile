DEVICES = rako-bridge                \
          led-dimmer-6ch             \
          touch-switch-controller

all: $(DEVICES)

clean:
	for device in $(DEVICES); do make -C devices/$$device clean; done

.PHONY: $(DEVICES)

$(DEVICES):
	make -C devices/$@

