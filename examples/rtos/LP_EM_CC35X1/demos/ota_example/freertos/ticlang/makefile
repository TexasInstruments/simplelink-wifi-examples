SIMPLELINK_WIFI_SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/imports.mak

CC = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"
LNK = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"

SYSCONFIG_GUI_TOOL = $(dir $(SYSCONFIG_TOOL))sysconfig_gui$(suffix $(SYSCONFIG_TOOL))
SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler ticlang --product $(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/.metadata/product.json --product $(SIMPLELINK_WIFI_TOOLBOX_INSTALL_DIR)/.metadata/product.json
SYSCFG_GUI_CMD_STUB = $(SYSCONFIG_GUI_TOOL) --compiler ticlang --product $(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/.metadata/product.json --product $(SIMPLELINK_WIFI_TOOLBOX_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../../freertos/ota_example.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))
SYSCFG_OPT_FILES = $(filter %.opt,$(SYSCFG_FILES))

# Enable verbose output by setting VERBOSE=1
V := @
ifeq ($(VERBOSE), 1)
  V :=
endif

OBJECTS = ota_example.obj ota_fwu.obj ota_server.obj ota_wifi.obj dns_if.obj led_if.obj tcpip_if.obj network_mbedtls.obj https_client.obj crc.obj nvocmp_cc35xx.obj adaptation_osi_dpl.obj adaptation_osi_filesystem.obj adaptation_uart_term.obj adaptation_syslog.obj freertos_main_freertos.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))

NAME = ota_example


CFLAGS += -I../.. \
    -I. \
    $(addprefix @,$(SYSCFG_OPT_FILES)) \
    -DTHREADSTACKSIZE=10752 \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/utils/FWU" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/utils/FWU/headers" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/utils/FWU/headers/fwu/1.0/psa" \
    -Ofast \
    -DCC35XX \
    -DNVOCMP_POSIX_MUTEX \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/mbedtls/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/mbedtls/ti/configs" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/mbedtls/ti/port" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/kernel/freertos" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/posix/ticlang" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/freertos/include" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/freertos/portable/GCC/ARM_CM33_NTZ/non_secure" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/net/ble_interface/inc_adapt" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/net/ble_interface/inc_common" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/wifi_platform/cc35xx/inc_common" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/net/wifi_stack/inc_common" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/wifi_host_driver/inc_common" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/wifi_host_driver/inc_adapt" \
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
    -gdwarf-3 \
    -mcpu=cortex-m33 \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -Wall

LFLAGS += "-L$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source" \
    "-Wl,-l$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/lwip/lib/ticlang/lwip.a" \
    "-Wl,-l$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/hostap/lib/ticlang/hostap.a" \
    "-Wl,-l$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/third_party/nimble/lib/ticlang/nimble.a" \
    "-Wl,-l$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/net/wifi_stack/lib/ticlang/wifi_stack.a" \
    "-Wl,-l$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/net/ble_interface/lib/ticlang/ble_interface.a" \
    "-Wl,-l$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/wifi_host_driver/lib/ticlang/wifi_host_driver.a" \
    "-Wl,-l$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/wifi_platform/cc35xx/lib/ticlang/wifi_platform_cc35xx.a" \
    "-Wl,-l$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source/ti/utils/FWU/lib/ticlang/FWU.a" \
    -Wl,--reread_libs \
    ti_utils_build_linker.cmd.genlibs \
    ../../freertos/ticlang/linker.cmd \
    "-Wl,-m,$(NAME).map" \
    -Wl,--rom_model \
    -Wl,--warn_sections \
    "-L$(TICLANG_ARMCOMPILER)/lib" \
    -llibc.a

all: $(NAME).out

.INTERMEDIATE: syscfg
$(SYSCFG_FILES): syscfg
	@ echo generation complete

syscfg: ../../freertos/ota_example.syscfg
	@ echo Generating configuration files...
	$(V) $(SYSCFG_CMD_STUB) --output $(@D) $<


# Helpful hint that the user needs to use a standalone SysConfig installation
$(SYSCONFIG_GUI_TOOL):
	$(error $(dir $(SYSCONFIG_TOOL)) does not contain the GUI framework \
        necessary to launch the SysConfig GUI.  Please set SYSCONFIG_TOOL \
        (in your SDK's imports.mak) to a standalone SysConfig installation \
        rather than one inside CCS)

syscfg-gui: ../../freertos/ota_example.syscfg $(SYSCONFIG_GUI_TOOL)
	@ echo Opening SysConfig GUI
	$(V) $(SYSCFG_GUI_CMD_STUB) $<


define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	$(V) $(CC) $(CFLAGS) -c $$< -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

ota_example.obj: ../../ota_example.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

ota_fwu.obj: ../../ota_fwu.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

ota_server.obj: ../../ota_server.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

ota_wifi.obj: ../../ota_wifi.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

dns_if.obj: ../../dns_if.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

led_if.obj: ../../led_if.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

tcpip_if.obj: ../../tcpip_if.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

network_mbedtls.obj: ../../network_mbedtls.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

https_client.obj: ../../https_client.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

crc.obj: ../../crc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

nvocmp_cc35xx.obj: ../../nvocmp_cc35xx.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

adaptation_osi_dpl.obj: ../../adaptation/osi_dpl.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

adaptation_osi_filesystem.obj: ../../adaptation/osi_filesystem.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

adaptation_uart_term.obj: ../../adaptation/uart_term.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

adaptation_syslog.obj: ../../adaptation/syslog.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

freertos_main_freertos.obj: ../../freertos/main_freertos.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) -c $< -o $@

$(NAME).out: $(OBJECTS)
	@ echo linking $@
	$(V) $(LNK) -Wl,-u,_c_int00 $(OBJECTS)  $(LFLAGS) -o $(NAME).out

clean:
	@ echo Cleaning...
	$(V) $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	$(V) $(RM) $(NAME).out > $(DEVNULL) 2>&1
	$(V) $(RM) $(NAME).map > $(DEVNULL) 2>&1
	$(V) $(RM) $(call SLASH_FIXUP,$(SYSCFG_FILES)) > $(DEVNULL) 2>&1
