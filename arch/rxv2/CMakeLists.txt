# Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(core)
  set_property(GLOBAL PROPERTY PROPERTY_OUTPUT_FORMAT elf32-rx-le) # needed for e.g. objcopy
  set_property(GLOBAL PROPERTY PROPERTY_OUTPUT_ARCH "v2")

