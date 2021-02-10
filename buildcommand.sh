#!/bin/bash
# Build skript for testing progress of zephyr rx65 porting

export ZEPHYR_TOOLCHAIN_VARIANT=rxgcc

rm -rf ~/.cache/zephyr/

west build -p always -b rsk_rx65n_2mb samples/basic/blinky # -- -G "Eclipse CDT4 - Ninja"
