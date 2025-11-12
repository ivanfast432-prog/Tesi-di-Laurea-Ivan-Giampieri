PROJECT_NAME     := EMGyro2
TARGETS          := nrf52840_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := C:/msys64/opt/nRF5_SDK_17.1.0
PROJ_DIR := ../../..

$(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
  LINKER_SCRIPT  := nrf52840_softdevice.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52840.c \
  $(SDK_ROOT)/modules/nrfx/soc/nrfx_atomic.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/atomic_flags/nrf_atflags.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_init.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_rng.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_rng_mbedtls.c \
  \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_rng.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_rng.c \
  $(SDK_ROOT)/components/libraries/crc16/crc16.c \
  $(SDK_ROOT)/components/libraries/queue/nrf_queue.c \
  \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_aead.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_aes.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_aes_shared.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_ecc.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_ecdh.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_ecdsa.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_eddsa.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_error.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_hash.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_hkdf.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_hmac.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_init.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_rng.c \
  $(SDK_ROOT)/components/libraries/crypto/nrf_crypto_shared.c \
  \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_aes.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_aes_aead.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_chacha_poly_aead.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_ecc.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_ecdh.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_ecdsa.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_eddsa.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_hash.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_hmac.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_init.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_mutex.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_rng.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310/cc310_backend_shared.c \
  \
  $(SDK_ROOT)/components/libraries/crypto/backend/oberon/oberon_backend_chacha_poly_aead.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/oberon/oberon_backend_ecc.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/oberon/oberon_backend_ecdh.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/oberon/oberon_backend_ecdsa.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/oberon/oberon_backend_eddsa.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/oberon/oberon_backend_hash.c \
  $(SDK_ROOT)/components/libraries/crypto/backend/oberon/oberon_backend_hmac.c \
  \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_twim.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spim.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_saadc.c \
  $(PROJ_DIR)/main.c \
  $(PROJ_DIR)/bluetooth.c \
  $(PROJ_DIR)/gyro.c \
  $(PROJ_DIR)/emg.c \
  $(PROJ_DIR)/ble_bas.c \
  $(PROJ_DIR)/ble_emg.c \
  $(PROJ_DIR)/ble_gyro.c \
  $(PROJ_DIR)/ble_system.c \
  $(PROJ_DIR)/power.c \
  $(PROJ_DIR)/usb.c \
  \
  $(SDK_ROOT)/components/ble/peer_manager/auth_status_tracker.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatt_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatts_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/id_manager.c \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr/nrf_ble_qwr.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_data_storage.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_database.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_id.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_manager_handler.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_buffer.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_dispatcher.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_manager.c \
  $(SDK_ROOT)/external/utf_converter/utf.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_sd.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis/ble_dis.c \
  \
  $(PROJ_DIR)/libFLAC/bitmath.c \
  $(PROJ_DIR)/libFLAC/bitreader.c \
  $(PROJ_DIR)/libFLAC/bitwriter.c \
  $(PROJ_DIR)/libFLAC/cpu.c \
  $(PROJ_DIR)/libFLAC/crc.c \
  $(PROJ_DIR)/libFLAC/fixed.c \
  $(PROJ_DIR)/libFLAC/float.c \
  $(PROJ_DIR)/libFLAC/format.c \
  $(PROJ_DIR)/libFLAC/lpc.c \
  $(PROJ_DIR)/libFLAC/md5.c \
  $(PROJ_DIR)/libFLAC/memory.c \
  $(PROJ_DIR)/libFLAC/stream_encoder.c \
  $(PROJ_DIR)/libFLAC/stream_encoder_framing.c \
  $(PROJ_DIR)/libFLAC/window.c \
  \
  \
  \
  $(SDK_ROOT)/components/libraries/usbd/app_usbd.c \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm/app_usbd_cdc_acm.c \
  $(SDK_ROOT)/components/libraries/usbd/app_usbd_core.c \
  $(SDK_ROOT)/components/libraries/usbd/app_usbd_serial_num.c \
  $(SDK_ROOT)/components/libraries/usbd/app_usbd_string_desc.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_usbd.c \
  \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_power.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_power.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  \
  \
  \

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(PROJ_DIR) \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/libraries/scheduler \
  ../config \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/ringbuf \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/components/libraries/mutex \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/components/libraries/atomic_flags \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/libraries/memobj \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/components/libraries/log/src \
  \
  \
  \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc \
  \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/stack_info \
  \
  $(SDK_ROOT)/components/libraries/crypto/backend/micro_ecc \
  $(SDK_ROOT)/components/libraries/crypto/backend/oberon \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_hw \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310_bl \
  $(SDK_ROOT)/external/nrf_tls/mbedtls/nrf_crypto/config \
  $(SDK_ROOT)/components/libraries/crypto/backend/optiga \
  $(SDK_ROOT)/components/libraries/crypto/backend/nrf_sw \
  $(SDK_ROOT)/components/libraries/crypto/backend/mbedtls \
  $(SDK_ROOT)/components/libraries/crypto/backend/cc310 \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components/libraries/crypto/backend/cifra \
  $(SDK_ROOT)/external/nrf_cc310/include \
  \
  \
  $(SDK_ROOT)/external/utf_converter \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/softdevice/s140/headers \
  $(SDK_ROOT)/components/softdevice/s140/headers/nrf52 \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  \
  $(PROJ_DIR)/libFLAC/include \

# Libraries common to all targets
LIB_FILES += \
$(SDK_ROOT)/external/nrf_cc310/lib/cortex-m4/hard-float/libnrf_cc310_0.9.13.a \
$(SDK_ROOT)/external/nrf_oberon/lib/cortex-m4/hard-float/liboberon_3.0.8.a \

# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_CUSTOM
#CFLAGS += -DUSE_FAKE_EMG_DATA
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DNRF_SD_BLE_API_VERSION=6
CFLAGS += -DS140
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRFX_SAADC_API_V2
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
#crypto library configuration
CFLAGS += -DMBEDTLS_CONFIG_FILE=\"nrf_crypto_mbedtls_config.h\"
CFLAGS += -DNRF_CRYPTO_MAX_INSTANCE_COUNT=1
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums
# for FLAC:
CFLAGS += -DHAVE_CONFIG_H

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=6
ASMFLAGS += -DS140
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DSWI_DISABLE0
#crypto library configuration
ASMFLAGS += -DNRF_CRYPTO_MAX_INSTANCE_COUNT=1

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

nrf52840_xxaa: CFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: CFLAGS += -D__STACK_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52840_xxaa firmware

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52840_xxaa
	@echo		flash_softdevice
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase firmware

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex
	nrfutil settings generate --family NRF52840 --bl-settings-version 2 \
  --application $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex \
  --application-version 1 \
  --bootloader-version  2 \
  $(OUTPUT_DIRECTORY)/nrf52840_xxaa_blcfg.hex
	mergehex -o $(OUTPUT_DIRECTORY)/nrf52840_xxaa_merge.hex -m $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex $(OUTPUT_DIRECTORY)/nrf52840_xxaa_blcfg.hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/nrf52840_xxaa_merge.hex --sectorerase --debugreset

# Flash softdevice
SD_FILE := $(wildcard $(SDK_ROOT)/components/softdevice/s140/hex/*.hex)
flash_softdevice:
	@echo Flashing: $(notdir $(SD_FILE))
	nrfjprog -f nrf52 --program $(SD_FILE) --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

# Generate firmware update file
firmware: $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex $(PROJ_DIR)/../keys/priv.pem
	@nrfutil pkg generate --sd-req 0x100 \
		--application $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex \
		--application-version 2 \
		--hw-version 52 \
		--key-file $(PROJ_DIR)/../keys/priv.pem \
		$(OUTPUT_DIRECTORY)/nrf52840_xxaa.zip

SDK_CONFIG_FILE := ../config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
