SIMPLELINK_WIFI_SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/imports.mak

CC = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"
LNK = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"

SYSCONFIG_GUI_TOOL = $(dir $(SYSCONFIG_TOOL))sysconfig_gui$(suffix $(SYSCONFIG_TOOL))
SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler gcc --product $(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/.metadata/product.json --product $(SIMPLELINK_WIFI_TOOLBOX_INSTALL_DIR)/.metadata/product.json
SYSCFG_GUI_CMD_STUB = $(SYSCONFIG_GUI_TOOL) --compiler gcc --product $(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/.metadata/product.json --product $(SIMPLELINK_WIFI_TOOLBOX_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../../freertos/mqtt_client.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))
SYSCFG_OPT_FILES = $(filter %.opt,$(SYSCFG_FILES))

# Enable verbose output by setting VERBOSE=1
V := @
ifeq ($(VERBOSE), 1)
  V :=
endif

OBJECTS = button_if.obj dns_if.obj led_if.obj mqtt_client_app.obj mqtt_if.obj tcpip_if.obj wifi_if.obj crc.obj nvocmp_cc35xx.obj adaptation_osi_dpl.obj adaptation_osi_filesystem.obj adaptation_uart_term.obj adaptation_syslog.obj freertos_main_freertos.obj syscalls_stubs.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))

NAME = mqtt_client


CFLAGS += -I../.. \
    -I. \
    $(addprefix @,$(SYSCFG_OPT_FILES)) \
    -O3 \
    -DCC35XX \
    -DRAM_CONTAINER \
    -DSNTP_SUPPORT \
    -DNVOCMP_POSIX_MUTEX \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source" \
    -D_REENT_SMALL \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/mbedtls/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/mbedtls/ti/configs" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/mbedtls/ti/port" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/kernel/freertos" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/posix/gcc" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/freertos/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/freertos/portable/GCC/ARM_CM33_NTZ/non_secure" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/net/ble_interface/inc_adapt" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/net/ble_interface/inc_common" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/wifi_platform/cc35xx/inc_common" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/net/wifi_stack/inc_common" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/wifi_host_driver/inc_common" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/wifi_host_driver/inc_adapt" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/tools/TI_Containers" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/lwip/lwip-stack/src/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/lwip/ti_config/lwip-port/osi/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/lwip/lwip-contrib" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/ti_config/nimble-port/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/ti_config/nimble-port/porting/npl/osi/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/nimble-src/nimble/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/nimble-src/nimble/host/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/nimble-src/nimble/host/services/gap/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/nimble-src/nimble/host/services/gatt/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/nimble-src/nimble/host/services/dis/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/nimble-src/nimble/host/util/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/nimble-src/nimble/transport/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/nimble-src/porting/nimble/include" \
    -mcpu=cortex-m33 \
    -mthumb \
    -std=c99 \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -ffunction-sections \
    -fdata-sections \
    -g \
    -gstrict-dwarf \
    -Wall \
    "-I$(GCC_ARMCOMPILER)/arm-none-eabi/include/newlib-nano" \
    "-I$(GCC_ARMCOMPILER)/arm-none-eabi/include"

LFLAGS += -Wl,-T,../../freertos/gcc/linker_GNU.lds \
    "-Wl,-Map,$(NAME).map" \
    -mcpu=cortex-m33 \
    "-L$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source" \
    -fno-lto \
    -Wl,--start-group \
    "$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/net/wifi_stack/lib/gcc/wifi_stack.a" \
    "$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/hostap/lib/gcc/hostap.a" \
    "$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/lwip/lib/gcc/lwip.a" \
    "$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/lib/gcc/nimble.a" \
    "$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/net/ble_interface/lib/gcc/ble_interface.a" \
    "$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/wifi_host_driver/lib/gcc/wifi_host_driver.a" \
    "$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/wifi_platform/cc35xx/lib/gcc/wifi_platform_cc35xx.a" \
    "$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/utils/FWU/lib/gcc/FWU.a" \
    -Wl,--end-group \
    ti_utils_build_linker.cmd.genlibs \
    -mthumb \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -nostartfiles \
    -static \
    -Wl,--gc-sections \
    -lgcc \
    -lc \
    -lm \
    --specs=nano.specs

all: $(NAME).out

.INTERMEDIATE: syscfg
$(SYSCFG_FILES): syscfg
	@ echo generation complete

syscfg: ../../freertos/mqtt_client.syscfg
	@ echo Generating configuration files...
	$(V) $(SYSCFG_CMD_STUB) --output $(@D) $<


# Helpful hint that the user needs to use a standalone SysConfig installation
$(SYSCONFIG_GUI_TOOL):
	$(error $(dir $(SYSCONFIG_TOOL)) does not contain the GUI framework \
        necessary to launch the SysConfig GUI.  Please set SYSCONFIG_TOOL \
        (in your SDK's imports.mak) to a standalone SysConfig installation \
        rather than one inside CCS)

syscfg-gui: ../../freertos/mqtt_client.syscfg $(SYSCONFIG_GUI_TOOL)
	@ echo Opening SysConfig GUI
	$(V) $(SYSCFG_GUI_CMD_STUB) $<


define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	$(V) $(CC) $(CFLAGS) $$< -c -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

button_if.obj: ../../button_if.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

dns_if.obj: ../../dns_if.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

led_if.obj: ../../led_if.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mqtt_client_app.obj: ../../mqtt_client_app.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mqtt_if.obj: ../../mqtt_if.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

tcpip_if.obj: ../../tcpip_if.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

wifi_if.obj: ../../wifi_if.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

crc.obj: ../../crc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

nvocmp_cc35xx.obj: ../../nvocmp_cc35xx.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

adaptation_osi_dpl.obj: ../../adaptation/osi_dpl.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

adaptation_osi_filesystem.obj: ../../adaptation/osi_filesystem.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

adaptation_uart_term.obj: ../../adaptation/uart_term.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

adaptation_syslog.obj: ../../adaptation/syslog.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

freertos_main_freertos.obj: ../../freertos/main_freertos.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

syscalls_stubs.obj: ../../syscalls_stubs.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

$(NAME).out: $(OBJECTS)
	@ echo linking $@
	$(V) $(LNK)  $(OBJECTS)  $(LFLAGS) -o $(NAME).out

clean:
	@ echo Cleaning...
	$(V) $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	$(V) $(RM) $(NAME).out > $(DEVNULL) 2>&1
	$(V) $(RM) $(NAME).map > $(DEVNULL) 2>&1
	$(V) $(RM) $(call SLASH_FIXUP,$(SYSCFG_FILES)) > $(DEVNULL) 2>&1
