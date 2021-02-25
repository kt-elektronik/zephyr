# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  cache.c
#  core.c # do we need this as the architecture do not have a file called like this?
  cpu_idle.c
#  fatal.c # check if: what to do (already located in kernel module)
  irq_manage.c
  prep_c.c
  reset.S
  reboot.c
  swap.c
  thread.c # check if: requires adaptation (already located in kernel module)
  vects.c
  inthandler.c
)

zephyr_library_sources_ifdef(CONFIG_IRQ_OFFLOAD irq_offload.c)
zephyr_library_sources_ifdef(CONFIG_RXV2_MPU core_mpu.c)