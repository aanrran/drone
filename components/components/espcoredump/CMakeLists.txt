﻿set(srcs "src/core_dump_common.c"
         "src/core_dump_flash.c"
         "src/core_dump_uart.c"
         "src/core_dump_elf.c"
         "src/core_dump_binary.c"
         "src/core_dump_sha.c"
         "src/core_dump_crc.c")

set(includes "include")
set(priv_includes "include_core_dump")

idf_build_get_property(target IDF_TARGET)

if(CONFIG_IDF_TARGET_ARCH_XTENSA)
  list(APPEND srcs "src/port/xtensa/core_dump_port.c")
  list(APPEND includes "include/port/xtensa")
  list(APPEND priv_includes "include_core_dump/port/xtensa")
elseif(CONFIG_IDF_TARGET_ARCH_RISCV)
  list(APPEND srcs "src/port/riscv/core_dump_port.c")
  list(APPEND includes "include/port/riscv")
  list(APPEND priv_includes "include_core_dump/port/riscv")
endif()

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ${includes}
                    PRIV_INCLUDE_DIRS ${priv_includes}
                    LDFRAGMENTS linker.lf
                    PRIV_REQUIRES esp_partition spi_flash bootloader_support mbedtls esp_rom soc esp_system driver)

if(CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF)
  target_link_libraries(${COMPONENT_LIB} PRIVATE idf::esp_app_format)
endif()
