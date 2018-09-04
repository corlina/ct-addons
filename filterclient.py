from __future__ import print_function
import argparse
import socket
import struct
import time
import threading
import numpy as np
from math import atan2, sqrt, degrees
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.animation as animation
from matplotlib import pyplot as plt


class MotionTracker(object):
    def __init__(self, time_term, read_interval, bufsize=20):
        self.rot_decay = float(time_term) / (time_term + read_interval)
        self.bufsize = bufsize
        self.dt = read_interval
        # lateral x,y,z - coordinates
        self.tracked_lx, self.tracked_ly, self.tracked_lz = 0.0, 0.0, 0.0
        # angles (radians) with x,y,z axes - orientation
        self.tracked_ax, self.tracked_ay, self.tracked_az = 0.0, 90.0, 90.0

        # filter state
        # self._acc_y, self._acc_y, self._acc_z =
        self._gyro_buffer = []
        self._gyro_mom_x, self._gyro_mom_y, self._gyro_mom_z = 0.0, 0.0, 0.0
        self._gyro_int_x, self._gyro_int_y, self._gyro_int_z = 0.0, 0.0, 0.0

        # CALIBRATION:
        self._calibration_state = False
        # orientation of gravity vector in initial state
        self._grav_ax, self._grav_ay, self._grav_az = 0.0, 0.0, 0.0
        # gyroscope output in motionless state - use as offsets
        self._gyro_offs_x, self._gyro_offs_y, self._gyro_offs_z = 0.0, 0.0, 0.0
        self._calibration_sums = None
        self._calibration_n = 0

    def start_calibration(self):
        self._calibration_state = True
        self._calibration_sums = [0, 0, 0, 0, 0, 0]
        self._calibration_n = 0

    def finish_calibration(self):
        self._calibration_state = False
        calib_means = [x / self._calibration_n for x in self._calibration_sums]
        (self._grav_ax, self._grav_ay, self._grav_az,
         self._gyro_offs_x, self._gyro_offs_y, self._gyro_offs_z) = calib_means
        self._gyro_buffer = self._gyro_buffer[:self.bufsize]
        bs = len(self._gyro_buffer)
        gyro_sums = map(sum, zip(*self._gyro_buffer))
        self._gyro_mom_x, self._gyro_mom_y, self._gyro_mom_z = [x/bs for x in gyro_sums]

        # reset tracked position, make it new normal
        self.tracked_lx, self.tracked_ly, self.tracked_lz = 0.0, 0.0, 0.0
        self.tracked_ax, self.tracked_ay, self.tracked_az = 0.0, 90.0, 90.0

    def add_data(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
        if self._calibration_state:
            data_tuple = acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
            for i in range(len(data_tuple)):
                self._calibration_sums[i] += data_tuple[i]
            self._calibration_n += 1
            self._gyro_buffer.append((gyro_x, gyro_y, gyro_z))
        else:
            bs = len(self._gyro_buffer)
            gx, gy, gz = self._gyro_buffer.pop(0)
            self._gyro_buffer.append((gyro_x, gyro_y, gyro_z))
            self._gyro_mom_x += (gyro_x - gx) / bs
            self._gyro_mom_y += (gyro_y - gy) / bs
            self._gyro_mom_z += (gyro_z - gz) / bs
            self._gyro_int_x = self._gyro_mom_x
            self._gyro_int_y = self._gyro_mom_y
            self._gyro_int_z = self._gyro_mom_z
            k0 = self.rot_decay
            k1 = 1 - k0

            x_rot = get_x_angle(acc_x, acc_y, acc_z) - get_x_angle(self._grav_ax, self._grav_ay, self._grav_az)
            y_rot = get_y_angle(acc_x, acc_y, acc_z) - get_y_angle(self._grav_ax, self._grav_ay, self._grav_az)
            z_rot = get_z_angle(acc_x, acc_y, acc_z) - get_z_angle(self._grav_ax, self._grav_ay, self._grav_az)
            self.tracked_ax = k0 * (self.tracked_ax + self._gyro_int_x) + k1 * x_rot
            self.tracked_ay = k0 * (self.tracked_ay + self._gyro_int_y) + k1 * y_rot
            self.tracked_az = k0 * (self.tracked_az + self._gyro_int_z) + k1 * z_rot

    @property
    def angles(self):
        return self.tracked_ax, self.tracked_ay, self.tracked_az


def dist(*args):
    return sqrt(sum(x*x for x in args))


def get_y_angle(x, y, z):
    radians = atan2(x, dist(y, z))
    return -degrees(radians)


def get_x_angle(x, y, z):
    radians = atan2(y, dist(x, z))
    return degrees(radians)


def get_z_angle(x, y, z):
    radians = atan2(z, dist(x, y))
    return -degrees(radians)


def stream_from_socket(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))

    packet_len = struct.calcsize('ffffff')

    while True:
        packet = sock.recv(packet_len)
        yield struct.unpack('ffffff', packet)


def stream_from_file(filename, dt):
    data = np.loadtxt(filename)[:, 1:]
    for item in data:
        yield tuple(item)
        time.sleep(dt)


def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument('host')
    # parser.add_argument('port', type=int)
    # opts = parser.parse_args()

    dt = 0.011
    bufsize = 2000

    streamer = stream_from_file('sensor-3.dat', dt)

    tracker = MotionTracker(0.5, dt)

    print('Calibrating...')
    tracker.start_calibration()
    for _ in range(200):
        item = next(streamer)
        tracker.add_data(*item)
    tracker.finish_calibration()
    print('Calibration done')

    lock = threading.Lock()
    buffers = [[] for _ in range(9)]
    labels = 'accel_X accel_Y accel_Z ' \
             'gyro_X gyro_Y gyro_Z ' \
             'Angle_X Angle_Y Angle_Z'.split()
    lines = {}
    fig = plt.gcf()

    def data_update():
        for item in streamer:
            tracker.add_data(*item)
            item = tuple(item)
            angles = tuple(tracker.angles)
            allitem = item + angles
            with lock:
                for i in range(9):
                    buf = buffers[i]
                    buf.append(allitem[i])
                    if len(buf) > bufsize:
                        buf.pop(0)
    update_thread = threading.Thread(target=data_update)
    update_thread.daemon = True
    update_thread.start()

    init_params = [
        (311, [0, 1, 2], (-30, 30)),
        (312, [3, 4, 5], (-120, 120)),
        (313, [6, 7, 8], (-3000, 3000)),
    ]

    for subidx, indices, (ymin, ymax) in init_params:
        ax = plt.subplot(subidx)
        ax.set_xlim(0, bufsize+100)
        ax.set_ylim(ymin, ymax)
        for i in indices:
            [line] = plt.plot([], label=labels[i].replace('_', ' '))
            lines[i] = line
        leg = plt.legend(loc='lower left', shadow=True, fancybox=True)
        leg.get_frame().set_alpha(0.5)

    def init_func():
        for buf in buffers:
            del buf[:]
        fig.canvas.draw()
        return tuple(lines.values())

    def dummy_source():
        while update_thread.isAlive():
            yield buffers

    def update_func(_):
        _buffers = buffers
        for i in range(9):
            buf = _buffers[i]
            line = lines[i]
            with lock:
                buf = buf[:]
                line.set_data(
                    range(bufsize-len(buf), bufsize),
                    buf
                )
        return tuple(lines.values())

    ani = animation.FuncAnimation(fig, update_func, dummy_source,
                                  repeat=False, init_func=init_func)
    plt.show()

if __name__ == '__main__':
    main()
