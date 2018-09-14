from math import atan2, sqrt, degrees, radians


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

    X_OFFS = 0.41999999999999993
    Z_OFFS = -1.1099999999999994
    Y_OFFS = 0.2549999999999999

    def finish_calibration(self):
        self._calibration_state = False
        calib_means = [x / self._calibration_n for x in self._calibration_sums]
        (self._grav_ax, self._grav_ay, self._grav_az,
         self._gyro_offs_x, self._gyro_offs_y, self._gyro_offs_z) = calib_means

        self._grav_ax -= self.X_OFFS
        self._grav_ay -= self.Y_OFFS
        self._grav_az -= self.Z_OFFS

        self._gyro_buffer = self._gyro_buffer[-self.bufsize:]
        bs = len(self._gyro_buffer)
        gyro_sums = map(sum, zip(*self._gyro_buffer))
        self._gyro_mom_x, self._gyro_mom_y, self._gyro_mom_z = [x/bs for x in gyro_sums]

        # reset tracked position, make it new normal
        self.tracked_lx, self.tracked_ly, self.tracked_lz = 0.0, 0.0, 0.0
        self.tracked_ax, self.tracked_ay, self.tracked_az = 0.0, 0.0, 0.0

    def add_data(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
        if self._calibration_state:
            data_tuple = acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
            for i in range(len(data_tuple)):
                self._calibration_sums[i] += data_tuple[i]
            self._calibration_n += 1
            self._gyro_buffer.append((gyro_x, gyro_y, gyro_z))
        else:

            gyro_x -= self._gyro_offs_x
            gyro_y -= self._gyro_offs_y
            gyro_z -= self._gyro_offs_z
            acc_x -= self.X_OFFS
            acc_y -= self.Y_OFFS
            acc_z -= self.Z_OFFS

            bs = len(self._gyro_buffer)
            gx, gy, gz = self._gyro_buffer.pop(0)
            self._gyro_buffer.append((gyro_x, gyro_y, gyro_z))
            self._gyro_mom_x += (gyro_x - gx) / bs
            self._gyro_mom_y += (gyro_y - gy) / bs
            self._gyro_mom_z += (gyro_z - gz) / bs
            dt = self.dt
            self._gyro_int_x += self._gyro_mom_x * dt
            # self._gyro_int_x = self._gyro_mom_x
            self._gyro_int_y = self._gyro_mom_y
            self._gyro_int_z = self._gyro_mom_z
            k0 = self.rot_decay
            k1 = 1 - k0

            x_rot = get_x_angle(acc_x, acc_y, acc_z) - get_x_angle(self._grav_ax, self._grav_ay, self._grav_az)
            y_rot = get_y_angle(acc_x, acc_y, acc_z) - get_y_angle(self._grav_ax, self._grav_ay, self._grav_az)
            z_rot = get_z_angle(acc_x, acc_y, acc_z) - get_z_angle(self._grav_ax, self._grav_ay, self._grav_az)
            self.tracked_ax = k0 * (self.tracked_ax + self._gyro_mom_x * dt) + k1 * x_rot
            self.tracked_ay = k0 * (self.tracked_ay + self._gyro_mom_y * dt) + k1 * y_rot
            self.tracked_az = k0 * (self.tracked_az + self._gyro_mom_z * dt) + k1 * z_rot

    @property
    def angles(self):
        return self.tracked_ax, self.tracked_ay, self.tracked_az

    @property
    def coordinates(self):
        return 0.0, 0.0, 0.0


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



