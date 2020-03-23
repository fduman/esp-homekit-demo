PROGRAM = am2320_temperature_sensor

EXTRA_COMPONENTS = \
	extras/i2c \
	extras/http-parser \
	$(abspath ../../components/esp8266-open-rtos/cJSON) \

FLASH_SIZE ?= 32

include $(SDK_PATH)/common.mk

monitor:
	$(FILTEROUTPUT) --port $(ESPPORT) --baud 115200 --elf $(PROGRAM_OUT)

