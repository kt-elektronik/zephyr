# SPDX-License-Identifier: Apache-2.0

board_runner_args(renesasfp "--device=RX65x")

# options after "--tool-opt=" are directly passed to the tool. So instead of "--iface=JTAG" you could also write "--tool-opt=-if JTAG"
board_runner_args(jlink "--device=R5F565NE" "--iface=JTAG" "--speed=1000" "--tool-opt=-jtagconf -1,-1 -autoconnect 1" )
board_runner_args(jlink "--use-mot")

include(${ZEPHYR_BASE}/boards/common/renesasfp.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

