SIMPLELINK_WIFI_SDK_INSTALL_DIR ?= $(abspath ../../../../../..)

include $(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/imports.mak

CC = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"
LNK = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"
HEX = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-objcopy"

SYSCONFIG_GUI_TOOL = $(dir $(SYSCONFIG_TOOL))sysconfig_gui$(suffix $(SYSCONFIG_TOOL))
SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler gcc --product $(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/.metadata/product.json --product $(SIMPLELINK_WIFI_TOOLBOX_INSTALL_DIR)/.metadata/product.json
SYSCFG_GUI_CMD_STUB = $(SYSCONFIG_GUI_TOOL) --compiler gcc --product $(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/.metadata/product.json --product $(SIMPLELINK_WIFI_TOOLBOX_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../nortos/bl3.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))
SYSCFG_OPT_FILES = $(filter %.opt,$(SYSCFG_FILES))

# Enable verbose output by setting VERBOSE=1
V := @
ifeq ($(VERBOSE), 1)
  V :=
endif

OBJECTS = main.obj boot_flow.obj hooks.obj image_auth.obj boot_report.obj handoff.obj uart.obj gpio.obj mcuboot_src_bootutil_img_hash.obj mcuboot_src_bootutil_keys.obj mcuboot_src_fault_injection_hardening.obj mcuboot_src_image_validate.obj mcuboot_src_security_cnt.obj mcuboot_src_tlv.obj mcuboot_ti_src_flash_map_backend.obj mcuboot_ti_src_bootutil_find_key.obj mcuboot_ti_src_bootutil_verify_sig.obj mcuboot_ti_src_bootutil_memequal.obj mbedtls_library_asn1parse.obj mbedtls_library_bignum.obj mbedtls_library_bignum_core.obj mbedtls_library_bignum_mod.obj mbedtls_library_bignum_mod_raw.obj mbedtls_library_constant_time.obj mbedtls_library_ecdsa.obj mbedtls_library_ecp.obj mbedtls_library_ecp_curves.obj mbedtls_library_ecp_curves_new.obj mbedtls_library_platform.obj mbedtls_library_platform_util.obj mbedtls_library_sha256.obj xmem_XMEMWFF3_nortos.obj xmem_flash_FlashWFF3_nortos.obj startup_gcc.obj syscall_stubs.obj syscall_stubs_specific.obj stdlib_override.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))

NAME = bl3


DFLAGS += -gstrict-dwarf

CFLAGS += -I.. \
    -I. \
    $(addprefix @,$(SYSCFG_OPT_FILES)) \
    -I../mcuboot \
    -I../mcuboot/src \
    -I../mcuboot/ti/include \
    -I../mbedtls/include \
    -DMBEDTLS_CONFIG_FILE=\"mbedtls_config_bl3.h\" \
    -O3 \
    -DCC35XX \
    -DDeviceFamily_CC35XX \
    -DNVOCMP_POSIX_MUTEX \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/kernel/nortos" \
    "-I$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/kernel/nortos/posix" \
    -mcpu=cortex-m33 \
    -mthumb \
    -std=c99 \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -ffunction-sections \
    -fdata-sections \
    -g \
    -Wall \
    "-I$(GCC_ARMCOMPILER)/arm-none-eabi/include/newlib-nano" \
    "-I$(GCC_ARMCOMPILER)/arm-none-eabi/include"


ifneq ($(CONFIG),RELEASE)
# Add debug flags by default (excluded only when CONFIG=RELEASE)
CFLAGS += $(DFLAGS)
endif

LFLAGS += -Wl,-T,../nortos/gcc/linker_bl3_GNU.lds \
    "-Wl,-Map,$(NAME).map" \
    -mcpu=cortex-m33 \
    "-L$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/source" \
    -Wl,--wrap=memcpy \
    -Wl,--wrap=memset \
    -fno-lto \
    -eresetISR \
    "-L$(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/kernel/nortos" \
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
    --specs=nano.specs \
    --specs=nosys.specs

all: $(NAME).bin

.INTERMEDIATE: syscfg
$(SYSCFG_FILES): syscfg
	@ echo generation complete

syscfg: ../nortos/bl3.syscfg
	@ echo Generating configuration files...
	$(V) $(SYSCFG_CMD_STUB) --output $(@D) $<


# Helpful hint that the user needs to use a standalone SysConfig installation
$(SYSCONFIG_GUI_TOOL):
	$(error $(dir $(SYSCONFIG_TOOL)) does not contain the GUI framework \
        necessary to launch the SysConfig GUI.  Please set SYSCONFIG_TOOL \
        (in your SDK's imports.mak) to a standalone SysConfig installation \
        rather than one inside CCS)

syscfg-gui: ../nortos/bl3.syscfg $(SYSCONFIG_GUI_TOOL)
	@ echo Opening SysConfig GUI
	$(V) $(SYSCFG_GUI_CMD_STUB) $<


define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	$(V) $(CC) $(CFLAGS) $$< -c -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

main.obj: ../main.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

boot_flow.obj: ../boot_flow.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

hooks.obj: ../hooks.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

image_auth.obj: ../image_auth.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

boot_report.obj: ../boot_report.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

handoff.obj: ../handoff.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

uart.obj: ../uart.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

gpio.obj: ../gpio.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mcuboot_src_bootutil_img_hash.obj: ../mcuboot/src/bootutil_img_hash.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mcuboot_src_bootutil_keys.obj: ../mcuboot/src/bootutil_keys.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mcuboot_src_fault_injection_hardening.obj: ../mcuboot/src/fault_injection_hardening.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mcuboot_src_image_validate.obj: ../mcuboot/src/image_validate.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mcuboot_src_security_cnt.obj: ../mcuboot/src/security_cnt.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mcuboot_src_tlv.obj: ../mcuboot/src/tlv.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mcuboot_ti_src_flash_map_backend.obj: ../mcuboot/ti/src/flash_map_backend.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mcuboot_ti_src_bootutil_find_key.obj: ../mcuboot/ti/src/bootutil_find_key.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mcuboot_ti_src_bootutil_verify_sig.obj: ../mcuboot/ti/src/bootutil_verify_sig.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mcuboot_ti_src_bootutil_memequal.obj: ../mcuboot/ti/src/bootutil_memequal.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_asn1parse.obj: ../mbedtls/library/asn1parse.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_bignum.obj: ../mbedtls/library/bignum.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_bignum_core.obj: ../mbedtls/library/bignum_core.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_bignum_mod.obj: ../mbedtls/library/bignum_mod.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_bignum_mod_raw.obj: ../mbedtls/library/bignum_mod_raw.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_constant_time.obj: ../mbedtls/library/constant_time.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_ecdsa.obj: ../mbedtls/library/ecdsa.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_ecp.obj: ../mbedtls/library/ecp.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_ecp_curves.obj: ../mbedtls/library/ecp_curves.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_ecp_curves_new.obj: ../mbedtls/library/ecp_curves_new.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_platform.obj: ../mbedtls/library/platform.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_platform_util.obj: ../mbedtls/library/platform_util.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

mbedtls_library_sha256.obj: ../mbedtls/library/sha256.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

xmem_XMEMWFF3_nortos.obj: ../xmem/XMEMWFF3_nortos.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

xmem_flash_FlashWFF3_nortos.obj: ../xmem/flash/FlashWFF3_nortos.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

startup_gcc.obj: ../startup_gcc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

syscall_stubs.obj: ../syscall_stubs.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

syscall_stubs_specific.obj: ../syscall_stubs_specific.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

stdlib_override.obj: ../stdlib_override.c $(SYSCFG_H_FILES)
	@ echo Building $@
	$(V) $(CC) $(CFLAGS) $< -c -o $@

$(NAME).out: $(OBJECTS)
	@ echo linking $@
	$(V) $(LNK)  $(OBJECTS)  $(LFLAGS) -o $(NAME).out

$(NAME).bin: $(NAME).out
	@ echo building $@
	$(V) $(HEX) -O binary $? $@

clean:
	@ echo Cleaning...
	$(V) $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	$(V) $(RM) $(NAME).out > $(DEVNULL) 2>&1
	$(V) $(RM) $(NAME).map > $(DEVNULL) 2>&1
	$(V) $(RM) $(NAME).bin > $(DEVNULL) 2>&1
	$(V) $(RM) $(call SLASH_FIXUP,$(SYSCFG_FILES)) > $(DEVNULL) 2>&1
