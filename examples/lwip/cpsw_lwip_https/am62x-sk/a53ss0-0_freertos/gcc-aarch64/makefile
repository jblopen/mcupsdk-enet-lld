#
# Auto generated makefile
#

export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

CG_TOOL_ROOT=$(CGT_GCC_AARCH64_PATH)

LD=$(CG_TOOL_ROOT)/bin/aarch64-none-elf-ld
CC=$(CG_TOOL_ROOT)/bin/aarch64-none-elf-gcc
LNK=$(CG_TOOL_ROOT)/bin/aarch64-none-elf-gcc
STRIP=$(CG_TOOL_ROOT)/bin/aarch64-none-elf-strip
OBJCOPY=$(CG_TOOL_ROOT)/bin/aarch64-none-elf-objcopy
ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release
ConfigName:=$(PROFILE)

OUTNAME:=cpsw_lwip_https.$(PROFILE).out

BOOTIMAGE_PATH=$(abspath .)
BOOTIMAGE_NAME:=cpsw_lwip_https.$(PROFILE).appimage
BOOTIMAGE_NAME_XIP:=cpsw_lwip_https.$(PROFILE).appimage_xip
BOOTIMAGE_NAME_SIGNED:=cpsw_lwip_https.$(PROFILE).appimage.signed
BOOTIMAGE_RPRC_NAME:=cpsw_lwip_https.$(PROFILE).rprc
BOOTIMAGE_RPRC_NAME_XIP:=cpsw_lwip_https.$(PROFILE).rprc_xip
BOOTIMAGE_RPRC_NAME_TMP:=cpsw_lwip_https.$(PROFILE).rprc_tmp
BOOTIMAGE_NAME_HS:=cpsw_lwip_https.$(PROFILE).appimage.hs
BOOTIMAGE_NAME_HS_FS:=cpsw_lwip_https.$(PROFILE).appimage.hs_fs
TARGETS := $(BOOTIMAGE_NAME)
ifeq ($(DEVICE_TYPE), HS)
   TARGETS += $(BOOTIMAGE_NAME_HS)
endif
ATF_LOAD_ADDR:=0x80000000
ATF_OUTNAME:=bl31.$(PROFILE).out
ATF_RPRC_NAME:=bl31.$(PROFILE).rprc
ATF_BIN_NAME:=bl31.bin
ATF_BUILD_PATH:=$(MCU_PLUS_SDK_PATH)/source/atf/trusted-firmware-a/build/k3/lite/$(PROFILE)

FILES_common := \
	altcp_tls_mbedtls_ti.c \
	httpd.c \
	main.c \
	app_main.c \
	app_cpswconfighandler.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \
    ti_enet_config.c \
    ti_enet_open_close.c \
    ti_enet_soc.c \
    ti_enet_lwipif.c \

FILES_PATH_common = \
	.. \
	../../.. \
	generated \

INCLUDES_common := \
	-I${MCU_PLUS_SDK_PATH}/source \
	-I${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include \
	-I${MCU_PLUS_SDK_PATH}/source/board/ethphy/port \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/GCC/ARM_CA53 \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am62x/a53 \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/utils \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/V3 \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/core \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am62x \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4 \
	-I${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include \
	-I${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include \
	-I${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include \
	-I${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwipif/inc \
	-I${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib \
	-I${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/apps/altcp_tls \
	-I${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am62x \
	-I${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls/include \
	-I${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/mbedtls_ti \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/dpl/ \
	-Igenerated \

DEFINES_common := \
	-DSOC_AM62X \
	-DMBEDTLS_CONFIG_FILE=\"alt_config.h\" \

CFLAGS_common := \
	-mcpu=cortex-a53+fp+simd \
	-mabi=lp64 \
	-mcmodel=large \
	-mstrict-align \
	-mfix-cortex-a53-835769 \
	-mfix-cortex-a53-843419 \
	-Wall \
	-Werror \
	-g \
	-Wno-int-to-pointer-cast \
	-Wno-pointer-to-int-cast \
	-Wno-unused-but-set-variable \
	-fdata-sections \
	-ffunction-sections \
	-Wno-unused-function \

CFLAGS_release := \
	-O2 \
	-flto \

CFLAGS_debug := \
	-D_DEBUG_=1 \

LNK_FILES_common = \
	linker.cmd \

LIBS_PATH_common = \
	-Lgenerated \
	-L${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib \
	-L${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	-L${MCU_PLUS_SDK_PATH}/source/board/lib \
	-L${MCU_PLUS_SDK_PATH}/source/networking/enet/lib \
	-L${MCU_PLUS_SDK_PATH}/source/networking/lwip/lib \
	-L${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/lib \
	-L$(CGT_GCC_AARCH64_PATH)/lib \

LIBS_common = \
	-l:freertos.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	-l:drivers.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	-l:board.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	-l:enet-cpsw.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	-l:lwipif-cpsw-freertos.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	-l:lwip-freertos.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	-l:lwip-contrib-freertos.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	-l:mbedtls.am62x.a53.gcc-aarch64.${ConfigName}.lib \

LFLAGS_common = \
	-Wl,-static \
	-Wl,--gc-sections \
	-Wl,--build-id=none \
	-lstdc++ \
	-lgcc \
	-lm \
	-lc \
	-lrdimon \

LNKOPTFLAGS_release = \
	       -flto \

LIBS_NAME = \
	freertos.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	drivers.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	board.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	enet-cpsw.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	lwipif-cpsw-freertos.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	lwip-freertos.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	lwip-contrib-freertos.am62x.a53.gcc-aarch64.${ConfigName}.lib \
	mbedtls.am62x.a53.gcc-aarch64.${ConfigName}.lib \

LIBS_PATH_NAME = \
	generated \
	${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/source/board/lib \
	${MCU_PLUS_SDK_PATH}/source/networking/enet/lib \
	${MCU_PLUS_SDK_PATH}/source/networking/lwip/lib \
	${MCU_PLUS_SDK_PATH}/source/networking/mbedtls_library/lib \
	$(CGT_GCC_AARCH64_PATH)/lib \

FILES := $(FILES_common) $(FILES_$(PROFILE))
ASMFILES := $(ASMFILES_common) $(ASMFILES_$(PROFILE))
FILES_PATH := $(FILES_PATH_common) $(FILES_PATH_$(PROFILE))
CFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE))
DEFINES := $(DEFINES_common) $(DEFINES_$(PROFILE))
INCLUDES := $(INCLUDES_common) $(INCLUDE_$(PROFILE))
LIBS := $(LIBS_common) $(LIBS_$(PROFILE))
LIBS_PATH := $(LIBS_PATH_common) $(LIBS_PATH_$(PROFILE))
LFLAGS := $(LFLAGS_common) $(LFLAGS_$(PROFILE))
LNK_FILES := $(LNK_FILES_common) $(LNK_FILES_$(PROFILE))

OBJDIR := obj/$(PROFILE)/
OBJS := $(FILES:%.c=%.obj)
OBJS += $(ASMFILES:%.S=%.obj)
DEPS := $(FILES:%.c=%.d)

vpath %.obj $(OBJDIR)
vpath %.c $(FILES_PATH)
vpath %.S $(FILES_PATH)
vpath %.lib $(LIBS_PATH_NAME)
vpath %.a $(LIBS_PATH_NAME)

$(OBJDIR)/%.obj %.obj: %.c
	@echo  Compiling: am62x:a53ss0-0:freertos:gcc-aarch64 $(OUTNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -MMD -MT $@ -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: am62x:a53ss0-0:freertos:gcc-aarch64 $(LIBNAME): $<
	$(CC) -c -x assembler-with-cpp $(CFLAGS) $(INCLUDES) $(DEFINES) -o $(OBJDIR)$@ $<

all: $(TARGETS)

SYSCFG_GEN_FILES=generated/ti_drivers_config.c generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=generated/ti_drivers_open_close.c generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=generated/ti_dpl_config.c generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=generated/ti_pinmux_config.c generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=generated/ti_board_config.c generated/ti_board_config.h
SYSCFG_GEN_FILES+=generated/ti_board_open_close.c generated/ti_board_open_close.h
SYSCFG_GEN_FILES+=generated/ti_enet_config.c generated/ti_enet_config.h
SYSCFG_GEN_FILES+=generated/ti_enet_open_close.c generated/ti_enet_open_close.h
SYSCFG_GEN_FILES+=generated/ti_enet_soc.c
SYSCFG_GEN_FILES+=generated/ti_enet_lwipif.c generated/ti_enet_lwipif.h

$(OUTNAME): syscfg $(SYSCFG_GEN_FILES) $(OBJS) $(LNK_FILES) $(LIBS_NAME)
	@echo  .
	@echo  Linking: am62x:a53ss0-0:freertos:gcc-aarch64 $@ ...
	$(LNK) $(LIBS_PATH) -Wl,-Map=$(OUTNAME:%.out=%.map) -o $@ -Wl,--start-group $(LFLAGS) $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) -Wl,--end-group -Wl,-T,$(LNK_FILES)
	@echo  Linking: am62x:a53ss0-0:freertos:gcc-aarch64 $@ Done !!!
	@echo  .

clean:
	@echo  Cleaning: am62x:a53ss0-0:freertos:gcc-aarch64 $(OUTNAME) ...
	$(RMDIR) $(OBJDIR)
	$(RM) $(OUTNAME)
	$(RM) $(BOOTIMAGE_NAME)
	$(RM) $(BOOTIMAGE_NAME_XIP)
	$(RM) $(BOOTIMAGE_NAME_SIGNED)
	$(RM) $(BOOTIMAGE_NAME_HS)
	$(RM) $(BOOTIMAGE_NAME_HS_FS)
	$(RM) $(BOOTIMAGE_RPRC_NAME)
	$(RM) $(BOOTIMAGE_RPRC_NAME_XIP)
	$(RMDIR) generated/

scrub:
	@echo  Scrubing: am62x:a53ss0-0:freertos:gcc-aarch64 cpsw_lwip_https ...
	$(RMDIR) obj
ifeq ($(OS),Windows_NT)
	$(RM) \*.out
	$(RM) \*.map
	$(RM) \*.appimage*
	$(RM) \*.rprc*
	$(RM) \*.tiimage*
	$(RM) \*.bin
else
	$(RM) *.out
	$(RM) *.map
	$(RM) *.appimage*
	$(RM) *.rprc*
	$(RM) *.tiimage*
	$(RM) *.bin
endif
	$(RMDIR) generated

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	$(MKDIR) $@

.NOTPARALLEL:

.INTERMEDIATE: syscfg
$(SYSCFG_GEN_FILES): syscfg

syscfg: ../example.syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --device AM62x --context a53ss0-0 --part Default --package ALW --output generated/ ../example.syscfg

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device AM62x --context a53ss0-0 --part Default --package ALW --output generated/  ../example.syscfg

#
# Generation of boot image which can be loaded by Secondary Boot Loader (SBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif
ifeq ($(OS),Windows_NT)
  BOOTIMAGE_CERT_GEN_CMD=powershell -executionpolicy unrestricted -command $(MCU_PLUS_SDK_PATH)/tools/boot/signing/x509CertificateGen.ps1
else
  BOOTIMAGE_CERT_GEN_CMD=$(MCU_PLUS_SDK_PATH)/tools/boot/signing/x509CertificateGen.sh
endif
BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt

BOOTIMAGE_CERT_KEY=$(APP_SIGNING_KEY)

BOOTIMAGE_CORE_ID_a53ss0-0 = 0
BOOTIMAGE_CORE_ID_a53ss0-1 = 1
BOOTIMAGE_CORE_ID_a53ss1-0 = 2
BOOTIMAGE_CORE_ID_a53ss1-1 = 3
BOOTIMAGE_CORE_ID_r5fss0-0 = 4
BOOTIMAGE_CORE_ID_m4fss0-0 = 5
BOOTIMAGE_CORE_ID_hsm-m4fss0-0 = 6
BOOTIMAGE_CORE_ID_load_only = 21
SBL_RUN_ADDRESS=0x43C00000
SBL_DEV_ID=55

MULTI_CORE_IMAGE_GEN = $(SYSCFG_NODE) $(MCU_PLUS_SDK_PATH)/tools/boot/multicoreImageGen/multicoreImageGen.js
OUTRPRC_CMD = $(SYSCFG_NODE) $(MCU_PLUS_SDK_PATH)/tools/boot/out2rprc/elf2rprc.js
APP_IMAGE_SIGN_CMD = $(MCU_PLUS_SDK_PATH)/tools/boot/signing/appimage_x509_cert_gen.py

ifeq ($(OS),Windows_NT)
  XIPGEN_CMD=$(MCU_PLUS_SDK_PATH)/tools/boot/xipGen/xipGen.exe
else
  XIPGEN_CMD=$(MCU_PLUS_SDK_PATH)/tools/boot/xipGen/xipGen.out
endif

ifeq ($(ATF_INTEGRATED_BOOT), yes)
MULTI_CORE_IMAGE_PARAMS = \
	$(ATF_RPRC_NAME)@$(BOOTIMAGE_CORE_ID_a53ss0-0) \
	$(BOOTIMAGE_RPRC_NAME)@$(BOOTIMAGE_CORE_ID_load_only) \

else
MULTI_CORE_IMAGE_PARAMS = \
	$(BOOTIMAGE_RPRC_NAME)@$(BOOTIMAGE_CORE_ID_a53ss0-0) \

endif

MULTI_CORE_IMAGE_PARAMS_XIP = \
	$(BOOTIMAGE_RPRC_NAME_XIP)@$(BOOTIMAGE_CORE_ID_a53ss0-0) \

$(BOOTIMAGE_NAME): $(OUTNAME)
	@echo  Boot image: am62x:a53ss0-0:freertos:gcc-aarch64 $(BOOTIMAGE_PATH)/$@ ...
ifneq ($(OS),Windows_NT)
	$(CHMOD) a+x $(XIPGEN_CMD)
endif
ifeq ($(ATF_INTEGRATED_BOOT), yes)
	$(LD) -b binary -A aarch64 --oformat elf64-littleaarch64 --section-start=.data=$(ATF_LOAD_ADDR) -e $(ATF_LOAD_ADDR) $(ATF_BUILD_PATH)/$(ATF_BIN_NAME) -o $(ATF_OUTNAME)
	$(OUTRPRC_CMD) $(ATF_OUTNAME) $(SW_VERSION)>> $(BOOTIMAGE_TEMP_OUT_FILE)
endif
	$(OUTRPRC_CMD) $(OUTNAME) $(SW_VERSION)>> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(COPY) $(BOOTIMAGE_RPRC_NAME) $(BOOTIMAGE_RPRC_NAME_TMP)
	$(RM) $(BOOTIMAGE_RPRC_NAME)
	$(XIPGEN_CMD) -i $(BOOTIMAGE_RPRC_NAME_TMP) -o $(BOOTIMAGE_RPRC_NAME) -x $(BOOTIMAGE_RPRC_NAME_XIP) --flash-start-addr 0x60000000 -v > $(BOOTIMAGE_TEMP_OUT_FILE)
	$(MULTI_CORE_IMAGE_GEN) --devID $(SBL_DEV_ID) --out $(BOOTIMAGE_NAME) $(MULTI_CORE_IMAGE_PARAMS) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(MULTI_CORE_IMAGE_GEN) --devID $(SBL_DEV_ID) --out $(BOOTIMAGE_NAME_XIP) $(MULTI_CORE_IMAGE_PARAMS_XIP) >> $(BOOTIMAGE_TEMP_OUT_FILE)
# Sign the appimage for HS-FS using appimage signing script
    #
    #   Authentication type 0 with load address 0x84000000
    #
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --authtype 0  --loadaddr 84000000 --key $(APP_SIGNING_KEY) --output $(BOOTIMAGE_NAME_HS_FS) --keyversion $(VERSION)
	$(RM) $(BOOTIMAGE_RPRC_NAME_TMP)
	$(RM) $(BOOTIMAGE_TEMP_OUT_FILE)
ifeq ($(ATF_INTEGRATED_BOOT), yes)
	$(RM) $(ATF_OUTNAME)
	$(RM) $(ATF_RPRC_NAME)
endif
	@echo  Boot image: am62x:a53ss0-0:freertos:gcc-aarch64 $(BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .
	@echo  Boot image: am62x:a53ss0-0:freertos:gcc-aarch64 $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_HS_FS) Done !!!
	@echo  .

$(BOOTIMAGE_NAME_HS): $(BOOTIMAGE_NAME)
ifeq ($(DEVICE_TYPE), HS)
# Sign the appimage using appimage signing script
ifeq ($(ENC_ENABLED),no)
	@echo Boot image signing: Encryption is disabled.
    #
    #   Authentication type 0 with load address 0x84000000
    #
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --authtype 0  --loadaddr 84000000 --key $(APP_SIGNING_KEY) --output $(BOOTIMAGE_NAME_HS) --keyversion $(VERSION)
else
	@echo Boot image signing: Encryption is enabled.
    #
    #   Authentication type 0 with load address 0x84000000
    #
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --authtype 0  --loadaddr 84000000 --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --output $(BOOTIMAGE_NAME_HS) --keyversion $(VERSION)
	$(RM) $(BOOTIMAGE_NAME)-enc
endif
	@echo  Boot image: am62x:a53ss0-0:freertos:gcc-aarch64 $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_HS) Done !!!
	@echo  .
endif
-include $(addprefix $(OBJDIR)/, $(DEPS))
