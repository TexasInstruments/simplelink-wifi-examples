SIMPLELINK_WIFI_SDK_INSTALL_DIR ?= $(abspath ../../..)
include $(SIMPLELINK_WIFI_SDK_INSTALL_DIR)/imports.mak

# Find all subdirectories
FREERTOS-GCC-SUBDIRS = $(wildcard */*/freertos/gcc/.)
FREERTOS-TICLANG-SUBDIRS = $(wildcard */*/freertos/ticlang/.)

FREERTOS-SUBDIRS =
ifneq ($(GCC_ARMCOMPILER),)
	FREERTOS-SUBDIRS += $(FREERTOS-GCC-SUBDIRS)
endif
ifneq ($(TICLANG_ARMCOMPILER),)
	FREERTOS-SUBDIRS += $(FREERTOS-TICLANG-SUBDIRS)
endif

.PHONY: freertos $(FREERTOS-SUBDIRS)

all: freertos

freertos: $(FREERTOS-SUBDIRS)

$(FREERTOS-SUBDIRS):
	@ $(MAKE) -C $@

CLEAN-FREERTOS-SUBDIRS = $(addprefix clean-,$(FREERTOS-SUBDIRS))

.PHONY: clean-freertos $(CLEAN-FREERTOS-SUBDIRS)

clean: clean-freertos

clean-freertos: $(CLEAN-FREERTOS-SUBDIRS)

$(CLEAN-FREERTOS-SUBDIRS):
	@ $(MAKE) -C $(subst clean-,,$@) clean
