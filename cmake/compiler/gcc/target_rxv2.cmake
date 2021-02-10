# SPDX-License-Identifier: Apache-2.0


if(CONFIG_RXV2)
  list(APPEND TOOLCHAIN_C_FLAGS  -mcpu=${GCC_M_CPU})
  list(APPEND TOOLCHAIN_C_FLAGS  -misa=v2 -mlittle-endian-data -ffunction-sections -fdata-sections)
#  list(APPEND TOOLCHAIN_C_FLAGS  -fno-leading-underscore)
  
  list(APPEND TOOLCHAIN_LD_FLAGS -mcpu=${GCC_M_CPU})
  list(APPEND TOOLCHAIN_LD_FLAGS -misa=v2 -mlittle-endian-data)
endif()

