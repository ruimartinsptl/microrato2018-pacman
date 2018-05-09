# RuntimeError: Python is not installed as a framework. The Mac OS X backend will not be able to function correctly if
# Python is not installed as a framework. See the Python documentation for more information on installing Python as a
# framework on Mac OS X. Please either reinstall Python as a framework, or try one of the other backends. If you are
# using (Ana)Conda please install python.app and replace the use of 'python' with 'pythonw'. See 'Working with
# Matplotlib on OSX' in the Matplotlib FAQ for more information.
# SOLUTION: https://stackoverflow.com/questions/21784641/installation-issue-with-matplotlib-python?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa

from __future__ import print_function
import serial
import argparse
from time import sleep

from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pylab import *


# plot class
class AnalogPlot:
    # constr
    def __init__(self, strPort, boudrate, maxLen):
        # open serial port
        self.ser = serial.Serial(strPort, boudrate)

        self.a0 = deque([0.0] * maxLen)
        self.a1 = deque([0.0] * maxLen)
        self.a2 = deque([0.0] * maxLen)
        self.a3 = deque([0.0] * maxLen)
        self.a4 = deque([0.0] * maxLen)
        self.a5 = deque([0.0] * maxLen)
        self.a6 = deque([0.0] * maxLen)
        self.a7 = deque([0.0] * maxLen)
        self.a8 = deque([0.0] * maxLen)
        self.a9 = deque([0.0] * maxLen)
        self.maxLen = maxLen

    # add to buffer
    def addToBuf(self, buf, val):
        if len(buf) < self.maxLen:
            buf.append(val)
        else:
            buf.pop()
            buf.appendleft(val)

    # add data
    def add(self, data):
        assert(len(data) == 10)
        self.addToBuf(self.a0, data[0])
        self.addToBuf(self.a1, data[1])
        self.addToBuf(self.a2, data[2])
        self.addToBuf(self.a3, data[3])
        self.addToBuf(self.a4, data[4])
        self.addToBuf(self.a5, data[5])
        self.addToBuf(self.a6, data[6])
        self.addToBuf(self.a7, data[7])
        self.addToBuf(self.a8, data[8])
        self.addToBuf(self.a9, data[9])

    # update plot
    def update(self, frameNum, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9):
        try:
            line = self.ser.readline()
            if '-->' in line[0:3]:
                data = [float(val) for val in line[4:].split()]
                # print data
                if len(data) == 10:
                    self.add(data)
                    a0.set_data(range(self.maxLen), self.a0)
                    a1.set_data(range(self.maxLen), self.a1)
                    a2.set_data(range(self.maxLen), self.a2)
                    a3.set_data(range(self.maxLen), self.a3)
                    a4.set_data(range(self.maxLen), self.a4)
                    a5.set_data(range(self.maxLen), self.a5)
                    a6.set_data(range(self.maxLen), self.a6)
                    a7.set_data(range(self.maxLen), self.a7)
                    a8.set_data(range(self.maxLen), self.a8)
                    a9.set_data(range(self.maxLen), self.a9)
        except KeyboardInterrupt:
            print('exiting')

        return a0,

        # clean up
    def close(self):
        # close serial
        self.ser.flush()
        self.ser.close()


# main() function
def main():
    parser = argparse.ArgumentParser(description="Micro rato 2018")
    parser.add_argument('-p', '--port', dest='port', required=False, default='/dev/cu.usbmodem1421')
    parser.add_argument('-b', '--boudrate', dest='boudrate', required=False, default=115200, type=int)
    args = parser.parse_args()
    ser = serial.Serial(args.port, args.boudrate)
    analogPlot = AnalogPlot(args.port, args.boudrate, 100)

    # while True:
    #     print(ser.readline(), end='')  # Python3

    # set up animation

    # Sent for figure
    font = {'size': 9}
    matplotlib.rc('font', **font)

    fig = plt.figure()
    fig.suptitle("RAW VALUES", fontsize=12)
    # p1 = fig.add_subplot(221)
    ax = plt.axes(xlim=(0, 100), ylim=(0, 1023))

    # ax01 = subplot2grid((2, 2), (0, 0))
    # ax02 = subplot2grid((2, 2), (0, 1))
    # ax03 = subplot2grid((2, 2), (1, 0), colspan=2, rowspan=1)
    # ax04 = ax03.twinx()

    a0, = ax.plot([], [])
    a1, = ax.plot([], [])
    a2, = ax.plot([], [])
    a3, = ax.plot([], [])
    a4, = ax.plot([], [])
    a5, = ax.plot([], [])
    a6, = ax.plot([], [])
    a7, = ax.plot([], [])
    a8, = ax.plot([], [])
    a9, = ax.plot([], [])
    anim = animation.FuncAnimation(fig, analogPlot.update,
                                   fargs=(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9),
                                   interval=10)

    # show plot
    plt.show()

    # clean up
    analogPlot.close()

    print('exiting.')


# call main
if __name__ == '__main__':
    main()
