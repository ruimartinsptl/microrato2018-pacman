# RuntimeError: Python is not installed as a framework. The Mac OS X backend will not be able to function correctly if
# Python is not installed as a framework. See the Python documentation for more information on installing Python as a
# framework on Mac OS X. Please either reinstall Python as a framework, or try one of the other backends. If you are
# using (Ana)Conda please install python.app and replace the use of 'python' with 'pythonw'. See 'Working with
# Matplotlib on OSX' in the Matplotlib FAQ for more information.
# SOLUTION: https://stackoverflow.com/questions/21784641/installation-issue-with-matplotlib-python?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa

from __future__ import print_function
import serial
import argparse
from random import randint
from time import sleep

from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pylab import *

mouse_status = {
    1: 'STATE_MOUSE_WAITTING_TO_START',
    2: 'STATE_MOUSE_AVOIDING_COLISION',
    3: 'STATE_MOUSE_WALKING',
    4: 'STATE_MOUSE_ON_CHEESE',
    5: 'STATE_MOUSE_ALL_DONE',
    6: 'STATE_MOUSE_ABORTED',
    7: 'STATE_MOUSE_TIMEOUT',
    8: 'STATE_MOUSE_SEARCH_BEACON'
}

# def translate(sensor_val, in_from, in_to, out_from, out_to):
#     out_range = out_to - out_from
#     in_range = in_to - in_from
#     in_val = sensor_val - in_from
#     val=(float(in_val)/in_range)*out_range
#     out_val = out_from+val
#     return out_val


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


# plot class
class AnalogPlot:
    # constr
    def __init__(self, strPort, boudrate, maxLen):
        # open serial port
        self.ser = serial.Serial(strPort, boudrate)

        self.floor_sensor_value = deque([0.0] * maxLen)
        self.flor_sensor_is_black = deque([0.0] * maxLen)
        self.dist_left = deque([0.0] * maxLen)
        self.dist_front_left = deque([0.0] * maxLen)
        self.dist_front_right = deque([0.0] * maxLen)
        self.dist_right = deque([0.0] * maxLen)
        self.beacon_angle = deque([0.0] * maxLen)
        self.emtpy = deque([0.0] * maxLen)
        # self.a8 = deque([0.0] * maxLen)
        # self.a9 = deque([0.0] * maxLen)
        self.maxLen = maxLen

    # add to buffer
    def addToBuf(self, buf, val):
        if len(buf) < self.maxLen:
            buf.appendleft(val)
        else:
            buf.popleft()
            buf.append(val)

    def add_dist(self, dist):
        if dist == 0 or dist > 200:
            dist = 200
        return translate(dist, 0, 200.0, 0, 100.0)

    def add_angle(self, angle):
        if angle >= 360:
            angle = 0
        if angle < 0:
            angle = 400
        return translate(angle, 0, 360.0, 0, 100.0)

    # add data
    def add(self, data):
        assert (len(data) == 11)
        self.addToBuf(self.floor_sensor_value, translate(data[0], 0, 1024.0, 0, 100.0))
        self.addToBuf(self.flor_sensor_is_black, 25.0 if data[1] == 1 else 75)
        self.addToBuf(self.dist_left, self.add_dist(data[2]))
        self.addToBuf(self.dist_front_left, self.add_dist(data[2]))
        self.addToBuf(self.dist_front_right, self.add_dist(data[4]))
        self.addToBuf(self.dist_right, self.add_dist(data[5]))
        self.addToBuf(self.beacon_angle, self.add_angle(data[6]))
        # self.addToBuf(self.emtpy, data[7])
        self.addToBuf(self.emtpy, 0)
        # self.addToBuf(self.a8, data[8])
        # self.addToBuf(self.a9, data[9])


    @staticmethod
    def cast_value(val):
        try:
            return float(val)
        except ValueError as err:
            return val

    # update plot
    def update(self, frameNum, p011, p012, p013, p014, p015, p016, p017, p018, p021, p031):
        try:
            # line = self.ser.readline()
            line = self.ser.read(self.ser.inWaiting())
            # print(line, end='')
            line = self.ser.readline()
            if '--> ' in line[0:4]:
                data = [self.cast_value(val) for val in line[4:].split()]
                print(mouse_status[int(data[9])].ljust(30), end='')
                print(line, end='')
                # print data
                if len(data) == 11:
                    self.add(data)
                    p011.set_data(range(self.maxLen), self.floor_sensor_value)
                    p012.set_data(range(self.maxLen), self.flor_sensor_is_black)
                    p013.set_data(range(self.maxLen), self.dist_left)
                    p014.set_data(range(self.maxLen), self.dist_front_left)
                    p015.set_data(range(self.maxLen), self.dist_front_right)
                    p016.set_data(range(self.maxLen), self.dist_right)
                    p017.set_data(range(self.maxLen), self.beacon_angle)
                    p018.set_data(range(self.maxLen), self.emtpy)

                    # for p in p021:
                    #     p.set_height(randint(0, 100))
                    for p, y in zip(p021, [self.dist_left, self.dist_front_left, self.dist_front_right, self.dist_right]):
                        # p.set_height(y[-1])
                        # p.set_height(translate(y[-1], 0, 200, 0, 100))
                        p.set_height(y[-1])

                    v = int(translate(self.beacon_angle[-1], 0, 100, 0, 400))
                    # v = randint(0, 359)
                    for p in p031:
                        p.set_height(0)
                    if 0 <= v <= 359:
                        p031[v].set_height(100)
            else:
                print(line, end='')

        except KeyboardInterrupt:
            print('key pressed to exit')

        return p011,

        # clean up

    def close(self):
        # close serial
        self.ser.flush()
        self.ser.close()


# main() function
def main():
    parser = argparse.ArgumentParser(description="Micro rato 2018")
    # parser.add_argument('-p', '--port', dest='port', required=False, default='/dev/cu.usbmodem1421')
    parser.add_argument('-p', '--port', dest='port', required=False, default='/dev/cu.usbmodem1411')
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

    fig = plt.figure(num=0, figsize=(12, 8), dpi=100)
    fig.suptitle("Valores dos sensores", fontsize=12)
    # p1 = fig.add_subplot(221)
    # ax = plt.axes(xlim=(0, 100), ylim=(0, 1023))

    ax01 = subplot2grid((2, 4), (0, 0), colspan=2, rowspan=2)

    ax02 = subplot2grid((2, 4), (0, 2), polar=True)
    ax02.set_theta_zero_location('N', offset=0)
    ax02.set_theta_direction(-1)
    ax02.set_thetamin(-90)
    ax02.set_thetamax(90)
    ax02.set_rorigin(-2.5)

    ax03 = subplot2grid((2, 4), (0, 3), polar=True)
    ax03.set_theta_zero_location('W', offset=0)
    ax03.set_theta_direction(-1)
    ax03.set_thetamin(-180)
    ax03.set_thetamax(180)
    ax03.set_rorigin(-25)

    ax04 = subplot2grid((2, 4), (1, 2))
    # ax04 = ax03.twinx()

    ax01.set_title('raw data')
    ax02.set_title('distance')
    ax03.set_title('beacon')
    ax04.set_title('floor')

    # set y-limits
    # ax01.set_ylim(0, 1023)
    ax01.set_ylim(0, 110)
    # ax02.set_ylim(-6,6)
    # ax03.set_ylim(-0,5)
    # ax04.set_ylim(-10,10)

    # sex x-limits
    ax01.set_xlim(0, 110)
    # ax02.set_xlim(0,5.0)
    # ax03.set_xlim(0,5.0)
    # ax04.set_xlim(0,5.0)

    # Turn on grids
    # ax01.grid(True)
    # ax02.grid(True)
    # ax03.grid(True)
    ax04.grid(True)

    # set label names
    # ax01.set_xlabel("x")
    # ax01.set_ylabel("py")
    # ax02.set_xlabel("t")
    # ax02.set_ylabel("vy")
    # ax03.set_xlabel("t")
    # ax03.set_ylabel("py")
    # ax04.set_ylabel("vy")

    # set plots
    p011, = ax01.plot([], [], 'r-', label="floor")
    p012, = ax01.plot([], [], 'r-', label="is_black")
    p013, = ax01.plot([], [], 'g-', label="L")
    p014, = ax01.plot([], [], 'b-', label="FL")
    p015, = ax01.plot([], [], 'y-', label="FR")
    p016, = ax01.plot([], [], 'c-', label="R")
    p017, = ax01.plot([], [], 'b-', label="bf")
    p018, = ax01.plot([], [], 'b-', label="bb")

    # p021, = ax02.plot([], [], 'b-', label="yv1")
    N = 4
    theta = np.arange(0.0-np.pi/4, 2*np.pi-np.pi/4, 2*np.pi/N)
    theta = [-np.pi/3, -np.pi/2/3, np.pi/2/3, np.pi/3]
    # radii = [80,50,75, 30]
    radii = [0] * N
    radii[-1] = 100
    width = np.pi/4*np.random.rand(N)
    width = 0.3
    p021 = ax02.bar(theta, radii, width=width, bottom=0.0)

    # p022, = ax02.plot([], [], 'g-', label="yv2")

    N = 360
    theta = np.arange(0.0, 2*np.pi, 2*np.pi/N)
    # radii = [0, 0, 100, 0, 0, 0]
    radii = [0] * N
    radii[-1] = 100
    # width = np.pi/4*np.random.rand(N)
    width = 0.1
    p031 = ax03.bar(theta, radii, width=width, bottom=0.0)

    p041, = ax03.plot([], [], 'b-', label="yp1")
    p042, = ax04.plot([], [], 'g-', label="yv1")

    # set lagends
    ax01.legend(
        [p011, p012, p013, p014, p015, p016, p017, p018],
        [p011.get_label(), p012.get_label(), p013.get_label(), p014.get_label(), p015.get_label(), p016.get_label(), p017.get_label(), p018.get_label()]
    )
    # ax02.legend([p021,p022], [p021.get_label(),p022.get_label()])
    # ax03.legend([p031,p032], [p031.get_label(),p032.get_label()])

    anim = animation.FuncAnimation(fig, analogPlot.update,
                                   fargs=(p011, p012, p013, p014, p015, p016, p017, p018, p021, p031),
                                   interval=100)

    # show plot
    plt.show()

    # clean up
    analogPlot.close()

    print('exiting.')


# call main
if __name__ == '__main__':
    main()
