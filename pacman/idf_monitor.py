#!/usr/bin/env python
#
# esp-idf serial output monitor tool. Does some helpful things:
# - Looks up hex addresses in ELF file with addr2line
# - Reset ESP32 via serial RTS line (Ctrl-T Ctrl-R)
# - Run "make flash" (Ctrl-T Ctrl-F)
# - Run "make app-flash" (Ctrl-T Ctrl-A)
# - If gdbstub output is detected, gdb is automatically loaded
#
# Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Contains elements taken from miniterm "Very simple serial terminal" which
# is part of pySerial. https://github.com/pyserial/pyserial
# (C)2002-2015 Chris Liechti <cliechti@gmx.net>
#
# Originally released under BSD-3-Clause license.
#
from __future__ import print_function, division
import subprocess
import argparse
import codecs
import re
import os

try:
    from platformio.util import get_serialports
except ImportError:
    def get_serialports(*args, **kwargs):
        return []

try:
    import queue
except ImportError:
    import Queue as queue
import time
import sys
import serial
import serial.tools.miniterm as miniterm
import threading
import ctypes

key_description = miniterm.key_description


def to_ctr_key(character):
    # return '\\x%02x' % (ord(character.upper()) - ord('@'))
    return chr(ord(character.upper()) - ord('@'))


def ask_for_port(ports_to_list=None):
    """\
    Show a list of ports and ask the user for a choice. To make selection
    easier on systems with long device names, also allow the input of an
    index.
    """
    if ports_to_list is None:
        ports_to_list = get_serialports(filter_hwid=False)
    sys.stderr.write('\n--- Available ports:\n')
    ports = []
    # for n, (port, desc, hwid) in enumerate(sorted(comports()), 1):
    for n, p in enumerate(ports_to_list, 1):
        port = p.get('port')
        description = p.get('description')
        port = port.encode('utf-8')
        description = description.encode('utf-8')
        sys.stderr.write('--- {:2}: {:30} {}\n'.format(n, port, description))
        ports.append(p.get('port'))
    while True:
        port = raw_input('--- Enter port index or full name: ')
        try:
            index = int(port) - 1
            if not 0 <= index < len(ports):
                sys.stderr.write('--- Invalid index!\n')
                continue
        except ValueError:
            pass
        else:
            port = ports[index]
        return port


def get_port():
    ports = get_serialports(filter_hwid=True)
    to_ignore = list()
    for item in ports:
        if 'USB VID:PID=1A86:7523 LOCATION=20-13' in item['hwid'] and '/dev/cu.wchusbserial14d0' in item['port'] and 'USB2.0-Serial' in item['description']:
            sys.stderr.write('-i %s\n' % item)
            to_ignore.append(item)
        elif "VID:PID" in item['hwid']:
            sys.stderr.write('-> %s\n' % item)
        else:
            sys.stderr.write('%s\n' % item)
    # print comports()
    for item in to_ignore:
        ports.remove(item)
    port = None
    if len(ports) == 1:
        port = ports[0].get('port')
    if len(ports) > 1:
        port = ask_for_port()
    return port


# Control-key characters
CTRL_A = '\x01'
CTRL_B = '\x02'
CTRL_F = '\x06'
CTRL_H = '\x08'
CTRL_R = '\x12'
CTRL_T = '\x14'
CTRL_D = '\x04'
CTRL_C = '\x03'
CTRL_X = '\x18'
CTRL_O = '\x09'
CTRL_U = '\x15'
CTRL_P = to_ctr_key('P')
CTRL_G = to_ctr_key('G')
CTRL_RBRACKET = '\x1d'  # Ctrl+]
EXIT_KEY = CTRL_X

# ANSI terminal codes
ANSI_RED = '\033[1;31m'
ANSI_YELLOW = '\033[0;33m'
ANSI_NORMAL = '\033[0m'


def color_print(message, color):
    """ Print a message to stderr with colored highlighting """
    sys.stderr.write("%s%s%s\n" % (color, message,  ANSI_NORMAL))


def yellow_print(message):
    color_print(message, ANSI_YELLOW)


def red_print(message):
    color_print(message, ANSI_RED)


__version__ = "1.0"

# Tags for tuples in queues
TAG_KEY = 0
TAG_SERIAL = 1

# regex matches an potential PC value (0x4xxxxxxx)
MATCH_PCADDR = re.compile(r'0x4[0-9a-f]{7}', re.IGNORECASE)

DEFAULT_TOOLCHAIN_PREFIX = "xtensa-esp32-elf-"

pio_toolchain = os.path.join("platformio", "packages", "toolchain-xtensa32", "bin")
if os.path.exists(pio_toolchain):
    DEFAULT_TOOLCHAIN_PREFIX = os.path.join(pio_toolchain, "xtensa-esp32-elf-")

DEFAULT_ELF_FILE = None

pio_elf_file = os.path.join('.pioenvs', 'esp32dev', 'firmware.elf')
if os.path.exists(pio_elf_file):
    DEFAULT_ELF_FILE = pio_elf_file
else:
    idf_build_dir = 'build'
    if os.path.exists(idf_build_dir):
        from glob import glob
        l = glob(os.path.join(idf_build_dir, '*.elf'))
        if len(l) == 1:
            DEFAULT_ELF_FILE = l[0]


class StoppableThread(object):
    """
    Provide a Thread-like class which can be 'cancelled' via a subclass-provided
    cancellation method.

    Can be started and stopped multiple times.

    Isn't an instance of type Thread because Python Thread objects can only be run once
    """
    def __init__(self):
        self._thread = None

    @property
    def alive(self):
        """
        Is 'alive' whenever the internal thread object exists
        """
        return self._thread is not None

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run_outer)
            self._thread.start()

    def _cancel(self):
        pass # override to provide cancellation functionality

    def run(self):
        pass # override for the main thread behaviour

    def _run_outer(self):
        try:
            self.run()
        finally:
            self._thread = None

    def stop(self):
        if self._thread is not None:
            old_thread = self._thread
            self._thread = None
            self._cancel()
            old_thread.join()

class ConsoleReader(StoppableThread):
    """ Read input keys from the console and push them to the queue,
    until stopped.
    """
    def __init__(self, console, event_queue):
        super(ConsoleReader, self).__init__()
        self.console = console
        self.event_queue = event_queue

    def run(self):
        self.console.setup()
        try:
            while self.alive:
                try:
                    if os.name == 'nt':
                        # Windows kludge: because the console.cancel() method doesn't
                        # seem to work to unblock getkey() on the Windows implementation.
                        #
                        # So we only call getkey() if we know there's a key waiting for us.
                        import msvcrt
                        while not msvcrt.kbhit() and self.alive:
                            time.sleep(0.1)
                        if not self.alive:
                            break
                    c = self.console.getkey()
                except KeyboardInterrupt:
                    c = '\x03'
                if c is not None:
                    self.event_queue.put((TAG_KEY, c), False)
        finally:
            self.console.cleanup()

    def _cancel(self):
        if hasattr(self.console, "cancel"):
            self.console.cancel()
        elif os.name == 'posix':
            # this is the way cancel() is implemented in pyserial 3.1 or newer,
            # older pyserial doesn't have this method, hence this hack.
            #
            # on Windows there is a different (also hacky) fix, applied above.
            import fcntl, termios
            fcntl.ioctl(self.console.fd, termios.TIOCSTI, b'\0')

class SerialReader(StoppableThread):
    """ Read serial data from the serial port and push to the
    event queue, until stopped.
    """
    def __init__(self, serial, event_queue):
        super(SerialReader, self).__init__()
        self.baud = serial.baudrate
        self.serial = serial
        self.event_queue = event_queue
        if not hasattr(self.serial, 'cancel_read'):
            # enable timeout for checking alive flag,
            # if cancel_read not available
            self.serial.timeout = 0.25

    def run(self):
        def _connect():
            if not self.serial.is_open:
                self.serial.baudrate = self.baud
                self.serial.rts = True  # Force an RTS reset on open
                self.serial.open()
                self.serial.rts = False
        _connect()
        try:
            while self.alive:
                try:
                    data = self.serial.read(self.serial.in_waiting or 1)
                    if len(data):
                        self.event_queue.put((TAG_SERIAL, data), False)
                except serial.SerialException as err:
                    self.serial.close()
                    time.sleep(1)
                    _connect()
                except IOError as err:
                    self.serial.close()
                    time.sleep(1)
                    _connect()
        finally:
            self.serial.close()

    def _cancel(self):
        if hasattr(self.serial, 'cancel_read'):
            try:
                self.serial.cancel_read()
            except:
                pass


class Monitor(object):
    """
    Monitor application main class.

    This was originally derived from miniterm.Miniterm, but it turned out to be easier to write from scratch for this
    purpose.

    Main difference is that all event processing happens in the main thread, not the worker threads.
    """
    def __init__(self, serial_instance, elf_file, make="make", toolchain_prefix=DEFAULT_TOOLCHAIN_PREFIX, eol="CRLF"):
        super(Monitor, self).__init__()
        self.event_queue = queue.Queue()
        self.console = miniterm.Console()
        if os.name == 'nt':
            sys.stderr = ANSIColorConverter(sys.stderr)
            self.console.output = ANSIColorConverter(self.console.output)
            self.console.byte_output = ANSIColorConverter(self.console.byte_output)

        self.serial = serial_instance
        self.console_reader = ConsoleReader(self.console, self.event_queue)
        self.serial_reader = SerialReader(self.serial, self.event_queue)
        self.elf_file = elf_file
        self.make = make
        self.toolchain_prefix = toolchain_prefix
        self.menu_key = CTRL_T
        self.exit_key = EXIT_KEY

        self.translate_eol = {
            "CRLF": lambda c: c.replace(b"\n", b"\r\n"),
            "CR":   lambda c: c.replace(b"\n", b"\r"),
            "LF":   lambda c: c.replace(b"\r", b"\n"),
        }[eol]

        # internal state
        self._pressed_menu_key = False
        self._read_line = b""
        self._gdb_buffer = b""

    def main_loop(self):
        self.console_reader.start()
        self.serial_reader.start()
        try:
            while self.console_reader.alive and self.serial_reader.alive:
                (event_tag, data) = self.event_queue.get()
                if event_tag == TAG_KEY:
                    self.handle_key(data)
                elif event_tag == TAG_SERIAL:
                    self.handle_serial_input(data)
                else:
                    raise RuntimeError("Bad event data %r" % ((event_tag,data),))
        finally:
            try:
                self.console_reader.stop()
                self.serial_reader.stop()
            except:
                pass
            sys.stderr.write(ANSI_NORMAL + "\n")

    def handle_key(self, key):
        if self._pressed_menu_key:
            self.handle_menu_key(key)
            self._pressed_menu_key = False
        elif key == self.menu_key:
            self._pressed_menu_key = True
        elif key == self.exit_key:
            self.console_reader.stop()
            self.serial_reader.stop()
        else:
            try:
                key = self.translate_eol(key)
                self.serial.write(codecs.encode(key))
            except serial.SerialException:
                pass # this shouldn't happen, but sometimes port has closed in serial thread

    def handle_serial_input(self, data):
        # this may need to be made more efficient, as it pushes out a byte
        # at a time to the console
        for b in data:
            self.console.write_bytes(b)
            if b == b'\n':  # end of line
                self.handle_serial_input_line(self._read_line.strip())
                self._read_line = b""
            else:
                self._read_line += b
            self.check_gdbstub_trigger(b)

    def handle_serial_input_line(self, line):
        for m in re.finditer(MATCH_PCADDR, line):
            self.lookup_pc_address(m.group())

    def handle_menu_key(self, c):
        if c == self.exit_key or c == self.menu_key:  # send verbatim
            self.serial.write(codecs.encode(c))
        elif c in [ CTRL_H, 'h', 'H', '?' ]:
            red_print(self.get_help_text())
        elif c == CTRL_R:  # Reset device via RTS
            self.serial.setRTS(True)
            time.sleep(0.2)
            self.serial.setRTS(False)
        elif c == CTRL_G:
            self.serial.rts = not self.serial.rts
        elif c == CTRL_D:  # Send DTR
            self.serial.setDTR(True)
            time.sleep(0.2)
            self.serial.setDTR(False)
        elif c == CTRL_F:  # Recompile & upload
            self.run_make("flash")
        elif c == CTRL_A:  # Recompile & upload app only
            self.run_make("app-flash")
        elif c == CTRL_P:
            self.run_make("pio-upload")
            time.sleep(1)
        elif c == CTRL_U:
            # self.run_make("pio-ota-arduino", pause_serial=False)
            self.run_make("pio-plot")
        else:
            h = '0x%02X' % ord(c)
            # ascii_code = ord(c)
            # if ascii_code < 32:
            #     h = ord('@') + ascii_code
            # else:
            #     h = repr(c)
            red_print('--- unknown menu character {} {}--'.format(key_description(c), h))

    def get_help_text(self):
        return """
--- idf_monitor ({version}) - ESP-IDF monitor tool
--- based on miniterm from pySerial
---
--- {exit:8} Exit program
--- {menu:8} Menu escape key, followed by:
--- Menu keys:
---    {menu:7} Send the menu character itself to remote
---    {exit:7} Send the exit character itself to remote
---    {reset:7} Reset target board via RTS line
---    {rts:7} Toggle RTS line
---    {dtr:7} Instant Toggle DTR line
---    {make:7} Run 'make flash' to build & flash
---    {appmake:7} Run 'make app-flash to build & flash app
---    {piouploadmake:7} Run 'make pio-upload to build & flash app
---    {piootauploadmake:7} Run 'make pio-ota-arduino to build & flash ota app
""".format(version=__version__,
           exit=key_description(self.exit_key),
           menu=key_description(self.menu_key),
           reset=key_description(CTRL_R),
           rts=key_description(CTRL_G),
           dtr=key_description(CTRL_D),
           make=key_description(CTRL_F),
           appmake=key_description(CTRL_A),
           piouploadmake=key_description(CTRL_P),
           piootauploadmake=key_description(CTRL_U)
           )

    def __enter__(self):
        """ Use 'with self' to temporarily disable monitoring behaviour """
        self.serial_reader.stop()
        self.console_reader.stop()

    def __exit__(self, *args, **kwargs):
        """ Use 'with self' to temporarily disable monitoring behaviour """
        self.console_reader.start()
        self.serial_reader.start()

    def prompt_next_action(self, reason):
        self.console.setup()  # set up console to trap input characters
        try:
            red_print("""
--- {}
--- Press {} to exit monitor.
--- Press {} to run 'make flash'.
--- Press {} to run 'make app-flash'.
--- Press {} to run 'make pio-upload'.
--- Press {} to run 'make pio-ota-arduino'.
--- Press any other key to resume monitor (resets target).""".format(reason,
                                                                     key_description(self.exit_key),
                                                                     key_description(CTRL_F),
                                                                     key_description(CTRL_A),
                                                                     key_description(CTRL_P),
                                                                     key_description(CTRL_U)
                                                                     ))
            k = CTRL_T  # ignore CTRL-T here, so people can muscle-memory Ctrl-T Ctrl-F, etc.
            while k == CTRL_T:
                k = self.console.getkey()
        finally:
            self.console.cleanup()
        if k == self.exit_key:
            self.event_queue.put((TAG_KEY, k))
        elif k in [ CTRL_F, CTRL_A, CTRL_P, CTRL_U]:
            self.event_queue.put((TAG_KEY, self.menu_key))
            self.event_queue.put((TAG_KEY, k))

    def run_make(self, target, pause_serial=True):
        def _run_make():
            yellow_print("Running make %s..." % target)
            p = subprocess.Popen([self.make, target ])
            try:
                p.wait()
            except KeyboardInterrupt:
                p.wait()
            if p.returncode != 0:
                self.prompt_next_action("Build failed")
            
        if pause_serial:
            with self:
                _run_make()
        else:
            _run_make()

    def lookup_pc_address(self, pc_addr):
        try:
            translation = subprocess.check_output(
                ["%saddr2line" % self.toolchain_prefix,
                 "-pfia", "-e", self.elf_file, pc_addr],
                cwd=".")
            if not "?? ??:0" in translation:
                yellow_print(translation)
        except Exception as err:
            pass

    def check_gdbstub_trigger(self, c):
        self._gdb_buffer = self._gdb_buffer[-6:] + c  # keep the last 7 characters seen
        m = re.match(b"\\$(T..)#(..)", self._gdb_buffer) # look for a gdb "reason" for a break
        if m is not None:
            try:
                chsum = sum(ord(p) for p in m.group(1)) & 0xFF
                calc_chsum = int(m.group(2), 16)
            except ValueError:
                return  # payload wasn't valid hex digits
            if chsum == calc_chsum:
                self.run_gdb()
            else:
                red_print("Malformed gdb message... calculated checksum %02x received %02x" % (chsum, calc_chsum))


    def run_gdb(self):
        with self:  # disable console control
            sys.stderr.write(ANSI_NORMAL)
            try:
                subprocess.call(["%sgdb" % self.toolchain_prefix,
                                "-ex", "set serial baud %d" % self.serial.baudrate,
                                "-ex", "target remote %s" % self.serial.port,
                                "-ex", "interrupt",  # monitor has already parsed the first 'reason' command, need a second
                                self.elf_file], cwd=".")
            except KeyboardInterrupt:
                pass  # happens on Windows, maybe other OSes
            self.prompt_next_action("gdb exited")

def main():
    parser = argparse.ArgumentParser("idf_monitor - a serial output monitor for esp-idf")

    parser.add_argument(
        '--port', '-p',
        help='Serial port device',
        default=os.environ.get('ESPTOOL_PORT', None)
    )

    parser.add_argument(
        '--baud', '-b',
        help='Serial port baud rate',
        type=int,
        default=os.environ.get('MONITOR_BAUD', 115200))

    parser.add_argument(
        '--make', '-m',
        help='Command to run make',
        type=str, default='make')

    parser.add_argument(
        '--toolchain-prefix',
        help="Triplet prefix to add before cross-toolchain names",
        default=DEFAULT_TOOLCHAIN_PREFIX)

    parser.add_argument(
        "--eol",
        choices=['CR', 'LF', 'CRLF'],
        type=lambda c: c.upper(),
        help="End of line to use when sending to the serial port",
        default='CRLF')

    parser.add_argument(
        '--elf_file',
        help='ELF file of application',
        type=argparse.FileType('r'),
        default=DEFAULT_ELF_FILE)

    args = parser.parse_args()
    if args.port is None:
        args.port = get_port()
    if args.port is None:
        for p in ["/dev/cu.SLAB_USBtoUART", "/dev/ttyUSB0"]:
            if os.path.exists(p):
                args.port = p
                break
    if args.port.startswith("/dev/tty."):
        args.port = args.port.replace("/dev/tty.", "/dev/cu.")
        yellow_print("--- WARNING: Serial ports accessed as /dev/tty.* will hang gdb if launched.")
        yellow_print("--- Using %s instead..." % args.port)

    os.environ['UPLOAD_PORT'] = args.port

    serial_instance = serial.serial_for_url(args.port, args.baud,
                                            do_not_open=True)
    serial_instance.dtr = False
    serial_instance.rts = False

    elf_file_name = None
    if args.elf_file is not None:
        args.elf_file.close()  # don't need this as a file
        elf_file_name = args.elf_file.name

    # remove the parallel jobserver arguments from MAKEFLAGS, as any
    # parent make is only running 1 job (monitor), so we can re-spawn
    # all of the child makes we need (the -j argument remains part of
    # MAKEFLAGS)
    try:
        makeflags = os.environ["MAKEFLAGS"]
        makeflags = re.sub(r"--jobserver[^ =]*=[0-9,]+ ?", "", makeflags)
        os.environ["MAKEFLAGS"] = makeflags
    except KeyError:
        pass  # not running a make jobserver

    monitor = Monitor(serial_instance, elf_file_name, args.make, eol=args.eol)

    yellow_print('--- idf_monitor on {p.name} {p.baudrate} ---'.format(
        p=serial_instance))
    yellow_print('--- Quit: {} | Menu: {} | Help: {} followed by {} ---'.format(
        key_description(monitor.exit_key),
        key_description(monitor.menu_key),
        key_description(monitor.menu_key),
        key_description(CTRL_H)))

    monitor.main_loop()


if os.name == 'nt':
    # Windows console stuff

    STD_OUTPUT_HANDLE = -11
    STD_ERROR_HANDLE = -12

    # wincon.h values
    FOREGROUND_INTENSITY = 8
    FOREGROUND_GREY = 7

    # matches the ANSI color change sequences that IDF sends
    RE_ANSI_COLOR = re.compile(b'\033\\[([01]);3([0-7])m')

    # list mapping the 8 ANSI colors (the indexes) to Windows Console colors
    ANSI_TO_WINDOWS_COLOR = [ 0, 4, 2, 6, 1, 5, 3, 7 ]

    GetStdHandle = ctypes.windll.kernel32.GetStdHandle
    SetConsoleTextAttribute = ctypes.windll.kernel32.SetConsoleTextAttribute

    class ANSIColorConverter(object):
        """Class to wrap a file-like output stream, intercept ANSI color codes,
        and convert them into calls to Windows SetConsoleTextAttribute.

        Doesn't support all ANSI terminal code escape sequences, only the sequences IDF uses.

        Ironically, in Windows this console output is normally wrapped by winpty which will then detect the console text
        color changes and convert these back to ANSI color codes for MSYS' terminal to display. However this is the
        least-bad working solution, as winpty doesn't support any "passthrough" mode for raw output.
        """

        def __init__(self, output):
            self.output = output
            self.handle = GetStdHandle(STD_ERROR_HANDLE if self.output == sys.stderr else STD_OUTPUT_HANDLE)
            self.matched = b''

        def write(self, data):
            for b in data:
                l = len(self.matched)
                if b == '\033':  # ESC
                    self.matched = b
                elif (l == 1 and b == '[') or (1 < l < 7):
                    self.matched += b
                    if self.matched == ANSI_NORMAL:  # reset console
                        SetConsoleTextAttribute(self.handle, FOREGROUND_GREY)
                        self.matched = b''
                    elif len(self.matched) == 7:     # could be an ANSI sequence
                        m = re.match(RE_ANSI_COLOR, self.matched)
                        if m is not None:
                            color = ANSI_TO_WINDOWS_COLOR[int(m.group(2))]
                            if m.group(1) == b'1':
                                color |= FOREGROUND_INTENSITY
                            SetConsoleTextAttribute(self.handle, color)
                        else:
                            self.output.write(self.matched) # not an ANSI color code, display verbatim
                        self.matched = b''
                else:
                    self.output.write(b)
                    self.matched = b''

        def flush(self):
            self.output.flush()


if __name__ == "__main__":
    main()
