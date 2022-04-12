# Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
#
# SPDX-License-Identifier: Apache-2.0
'''Runner for flashing with Renesas Flash Programmer'''
# https://www.renesas.com/eu/en/software-tool/renesas-flash-programmer-programming-gui

import os
import sys
import re

from subprocess import CalledProcessError
from runners.core import ZephyrBinaryRunner, RunnerCaps

DEFAULT_RFP_EXE = 'rfp-cli'

class RenesasFPRunner(ZephyrBinaryRunner):
    '''Runnner front-end for Renesas Flash Programmer.'''

    def __init__(self, cfg, device, id_code, tool=None, interface='uart', attempts=5):
        super().__init__(cfg)
        self.mot_name = cfg.mot_file
        self.device = device
        self.id_code = id_code
        self.tool = tool
        self.interface = interface
        self.attempts = attempts

    @classmethod
    def name(cls):
        return 'renesasfp'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'})

    @classmethod
    def do_add_parser(cls, parser):
        # Required:
        parser.add_argument('--device', required=True, help='device name')

        # Optional:
        parser.add_argument('--idcode', default='ffffffffffffffffffffffffffffffff',
                            help='Id code to use (default: ffffffffffffffffffffffffffffffff).')
        parser.add_argument('--tool', default=None,
                            help='tool to use for flashing (default: auto detect)')
        parser.add_argument('--interface', default='uart',
                            help='debugger communication type: "uart" (default) or "fine"')
        parser.add_argument('--attempts', default=5,
                            help='number of attempts to flash the device (default: 5)')

    @classmethod
    def do_create(cls, cfg, args):
        return RenesasFPRunner(cfg, args.device, args.idcode, tool=args.tool,
                                interface=args.interface, attempts=args.attempts)

    def get_tools(self):
        '''automatically detect connected tools to use with the RFP'''
        cmd = [DEFAULT_RFP_EXE, '-d', self.device, '-lt']
        output = self.check_output(cmd)
        output = output.decode(sys.getdefaultencoding())
        in_tool_section = False
        self.tools = []
        tool_type = None
        for line in output.split('\n'):
            if not in_tool_section:
                in_tool_section = in_tool_section or re.search('\[Tool\]', line)
            else:
                if len(line) > 0 and line[0] != ' ':
                    tool_type = line.split(' ')[0]
                elif len(line) > 2 and line[:2] == '  ':
                    self.tools.append((tool_type, line[2:-1]))

    def attempt_call(self, cmd):
        ''' attempt to call a command repeatedly if it fails. This is necessary
        as rfp-cli can randomly fail every once in a while'''
        attempts_remaining = self.attempts
        while (attempts_remaining > 0):
            try:
                self.check_call(cmd)
                # flashing successful, no more attempts needed
                attempts_remaining = 0
            except CalledProcessError as e:
                attempts_remaining = attempts_remaining - 1
                self.logger.debug(f'"{" ".join(cmd)}" returns code {e.returncode}, {attempts_remaining} attempts remaining')
                if attempts_remaining == 0:
                    # no more attempts remaining, raise error to let the calling process know
                    raise e

    def flash(self, **kwargs):
        if self.mot_name is None or not os.path.isfile(self.mot_name):
            raise ValueError('Cannot flash; no mot ({}) file found'.format(self.mot_name))

        if self.tool is None:
            self.get_tools()
            if len(self.tools) == 0:
                raise ValueError(f'No compatible tools found')
            elif len(self.tools) != 1:
                self.logger.error("Multiple tools found:")
                for tool in self.tools:
                    self.logger.error(f'  {tool[0]}:{tool[1]}')
                raise ValueError(f'Found more than one compatible tool, please specify, which to use for flashing')

            self.tool = f'{self.tools[0][0]}:{self.tools[0][1]}'

        cmd = [DEFAULT_RFP_EXE,
               '-d', self.device,
               '-t', self.tool,
               '-if', self.interface,
               '-auth', 'id', self.id_code,
               '-run',
               '-a', self.mot_name]

        self.logger.debug(f'RFP command: "{" ".join(cmd)}"')

        self.attempt_call(cmd)

    def do_run(self, command, **kwargs):
        if command == 'flash':
            self.flash(**kwargs)
        else:
            raise ValueError('Can only flash at this point')
