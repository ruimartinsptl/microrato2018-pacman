import os
import sys
import signal
import socket
import platform

from drawnow import *
from time import sleep
import matplotlib.pyplot as plt

# https://hardsoftlucid.wordpress.com/various-stuff/realtime-plotting/
# http://arduino-er.blogspot.pt/2015/04/python-to-plot-graph-of-serial-data.html


def readlines(sock, recv_buffer=4096, delim='\n'):
    # https://synack.me/blog/using-python-tcp-sockets
    _buffer = ''
    _data = True
    while _data:
        _data = sock.recv(recv_buffer)
        _buffer += _data

        while _buffer.find(delim) != -1:
            line, _buffer = _buffer.split('\n', 1)
            yield line
    return


def main():
    global sock, f
    # Connect the socket to the port on the server given by the caller
    if len(sys.argv) > 1:
        server_name = sys.argv[1]
        if len(sys.argv) > 2:
            server_port = int(sys.argv[2])
        else:
            server_port = 8000
    else:
        server_name = 'localhost'
        server_port = 8000

    values = []

    plt.ion()

    max_values = 250

    def plot_values():
        plt.title('Telnet Values')
        plt.grid(True)
        plt.ylabel('Values')
        # https://matplotlib.org/devdocs/api/_as_gen/matplotlib.pyplot.plot.html#matplotlib.pyplot.plot
        plt.plot(values, 'b.-', label='values', linewidth=.5)
        plt.legend(loc='upper right')

    server_address = (server_name, server_port)

    def connect():
        # Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        sock.settimeout(5.0)
        print 'connecting to %s port %s' % server_address

        try:
            sock.connect(server_address)
            sock.settimeout(30.0)
            print 'connected'
            return sock
        except socket.timeout as err:
            sleep(0.5)
            print 'connection timeout', err
        except socket.error as err:
            sleep(5)
            print 'connection error', err
        return None

    while True:
        try:
            sock = connect()
            if sock is None:
                continue
            for l in readlines(sock):
                # print l
                if ',' in l:
                    t, v = l.split(',')
                else:
                    v = l
                # print t, v
                try:
                    v = float(v)
                    values.append(v)
                    if len(values) > max_values:
                        values.pop(0)

                    drawnow(plot_values)
                except ValueError as err:
                    print 'Parse error', err
        except socket.timeout as err:
            sleep(0.5)
            print 'read timeout', err
        except socket.error as err:
            sleep(5)
            print 'read error', err
        finally:
            if 'sock' in globals() and sock is not None:
                try:
                    sock.close()
                except Exception as err:
                    pass


def close():
    global sock, f
    if 'sock' in globals() and sock is not None:
        sock.close()
        sock = None
    if 'f' in globals() and f is not None:
        f.stop()
        f = None
    sys.exit()


def receive_signal(signum, frame):
    print 'Caught signal %s, exiting.' % str(signum)
    close()


if __name__ == '__main__':
    # http://stackoverflow.com/a/5669030
    plat_sys = platform.system()
    if 'Windows' not in plat_sys:
        for i in ['SIGTERM', 'SIGINT', 'SIGQUIT', 'SIGHUP']:
            signum = getattr(signal, i)
            signal.signal(signum, receive_signal)
        main()
    else:
        try:
            main()
        except KeyboardInterrupt:
            print 'Caught KeyboardInterrupt, exiting.'
            close()
