# Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
# SPDX-License-Identifier: Apache-2.0

# options after "--tool-opt=" are directly passed to the tool. So instead of "--iface=JTAG" you could also write "--tool-opt=-if JTAG"
board_runner_args(renesasfp "--device=RX65x")
board_runner_args(renesasfp "--interface=fine")

include(${ZEPHYR_BASE}/boards/common/renesasfp.board.cmake)
