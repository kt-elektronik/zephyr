# SPDX-License-Identifier: Apache-2.0

board_set_flasher_ifnset(jlink)
board_set_debugger_ifnset(jlink)

if(DEFINED CONFIG_HAS_FLASH_LOAD_OFFSET)
    board_finalize_runner_args(jlink "--dt-flash=y")
else()
    board_finalize_runner_args(jlink "--dt-flash=n")
endif()
