# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  switch.S
  cpu_idle.c
  irq_manage.c
  reset.S
  thread.c
  vects.c
  ofsm.c
)

zephyr_library_sources_ifdef(CONFIG_IRQ_OFFLOAD irq_offload.c)
zephyr_library_sources_ifdef(CONFIG_RXV2_MPU core_mpu.c)
