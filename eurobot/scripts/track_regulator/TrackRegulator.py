# Point is always the x, y coordinate of object and anlge of rotation of its frame
import numpy as np
import time

def cvt_local2global(local_point, sc_point):
    point = np.zeros(3)
    x, y, a = local_point
    X, Y, A = sc_point
    point[0] = x * np.cos(A) - y * np.sin(A) + X
    point[1] = x * np.sin(A) + y * np.cos(A) + Y
    point[2] = a + A
    return point


def cvt_global2local(global_point, sc_point):
    point = np.zeros(3)
    x, y, a = global_point
    X, Y, A = sc_point
    point[0] = x * np.cos(A) + y * np.sin(A) - X * np.cos(A) - Y * np.sin(A)
    point[1] = -x * np.sin(A) + y * np.cos(A) + X * np.sin(A) - Y * np.cos(A)
    point[2] = a - A
    return point

class PID_regulator(object):
    def __init__(self, k_p, k_d, k_i, max_response, max_integral):
        self.k_p = k_p
        self.k_d = k_d
        self.k_i = k_i
        self.max_response = max_response
        self.max_integral = max_integral
        self.target = 0
        self.error_i = 0
        self.prev_error = 0
        self.prev_time = time.time()

    def set_target(self, target):
        self.target = target
    
    def regulate(self, feedback, skale):
        dt = time.time() - self.prev_time
        self.prev_time = time.time()

        error_p = self.target - feedback
        
        self.error_i += error_p * dt
        error_i = self.error_i

        error_d = (error_p - self.prev_error) / dt

        self.prev_error = error_p

        self.error_i = max(-self.max_integral, self.error_i)
        self.error_i = min(self.max_integral, self.error_i)
        print(error_p, error_d, error_i, self.k_p, self.k_d, self.k_i)
        response = error_p * self.k_p + error_i * self.k_i + error_d * self.k_d
        response = max(-self.max_response, response)
        response = min(self.max_response, response)
        return response * skale

class TrackRegulator(object):
    def __init__(self):
        self.MAX_VELOCITY = 0.2
        self.MAX_ROTATION = 1
        self.MIN_VELOCITY = 0.002
        self.MIN_ROTATION = 0.01
        self.NORM_ANGLE = 3.14 / 8
        self.NORM_DISTANCE = 100
        self.PERP_NORM_DISTANCE = 50
        self.PERP_MAX_RATE = 1
        self.MIN_DIST = 3
        self.MIN_ANGLE = 0.02
        self.T_PERP = self.PERP_NORM_DISTANCE / 1000. / self.MAX_VELOCITY
        self.T_ROTATION = self.NORM_ANGLE / 1. / self.MAX_ROTATION
        self.target_point = np.zeros(3)
        self.is_rotate = False
        self.is_move_forward = False
        self.is_moving = False

    def start_rotate(self, point):
        print("Start rotate")
        da = (self.target_point[2] - point[2]) % (2 * np.pi)
        if da < np.pi:
            self.rotation_diraction = 1
            self.dangle = da
        else:
            self.rotation_diraction = -1
            self.dangle = 2 * np.pi - da
        self.dangle -= self.MIN_ANGLE
        self.start_angle = point[2]
        self.is_rotate = True

    def start_move_forward(self, point):
        print("Start move forward")
        self.is_rotate = False
        self.is_move_forward = True
        self.distance = np.sum((self.target_point[:2] - point[:2]) ** 2) ** 0.5 - self.MIN_DIST

        self.start_to_target_point = np.zeros(3)
        self.start_to_target_point[:2] = point[:2]
        dp = self.target_point[:2] - point[:2]
        self.start_to_target_point[2] = np.arctan2(dp[1], dp[0])
        self.pid_perp = PID_regulator(1. / 1000 / self.T_PERP, 1. / 100000, 1. / 1000 / self.T_PERP ** 2, self.MAX_VELOCITY, self.MAX_VELOCITY * self.T_PERP)
        self.pid_rot = PID_regulator(1 / self.T_ROTATION, 1. / 100, 1 / self.T_ROTATION ** 2, self.MAX_ROTATION, self.MAX_ROTATION * self.T_ROTATION)
    def start_move(self, target_point, point):
        self.is_moving = True
        print("Start move")
        self.target_point = target_point
        self.start_rotate(point)

    def rotate(self, point):
        da = (point[2] - self.start_angle) % (2 * np.pi)
        print(da, self.dangle)
        if self.rotation_diraction == -1:
            da = 2 * np.pi - da
        if da > 3 * np.pi / 2:
            da = da - 2 * np.pi
        if da >= self.dangle:
            print("Stop rotate")
            self.start_move_forward(point)
            return np.zeros(3)
        elif da <= 0:
            v_angle = 0
        elif da > self.dangle - self.NORM_ANGLE:
            v_angle = np.sqrt((self.dangle - da) / self.NORM_ANGLE) * self.MAX_ROTATION
        elif da < self.NORM_ANGLE:
            v_angle = np.sqrt(da / self.NORM_ANGLE) * self.MAX_ROTATION
        else:
            v_angle = self.MAX_ROTATION
        v_angle += self.MIN_ROTATION
        print("ROTATE v_angle = %f da = %f target_angle = %f" % (v_angle, da, self.dangle))
        return np.array([0, 0, self.rotation_diraction * (v_angle + self.MIN_ROTATION)])

    def move(self, point):
        point_in_target_system = cvt_global2local(point, self.start_to_target_point)
        dx = point_in_target_system[0]
        if dx > self.distance:
            print("Stop move forward")
            print("Stop move")
            self.is_move_forward = False
            self.is_moving = False
            return np.zeros(3)
        elif dx > self.distance - self.NORM_DISTANCE:
            v = np.sqrt((self.distance - dx) / self.NORM_DISTANCE) * self.MAX_VELOCITY
        elif dx < self.NORM_DISTANCE and dx > 0:
            v = np.sqrt(dx / self.NORM_DISTANCE) * self.MAX_VELOCITY
        elif dx <= 0:
            v = 0
        else:
            v = self.MAX_VELOCITY
        v += self.MIN_VELOCITY
        
        dy = point_in_target_system[1]
        # v_perp = - v * dy / self.PERP_NORM_DISTANCE
        # v_perp = min(v_perp, self.MAX_VELOCITY)
        # v_perp = max(v_perp, -self.MAX_VELOCITY)
        v_perp = self.pid_perp.regulate(dy, v / self.MAX_VELOCITY)

        da = (point[2] - self.target_point[2]) % (2 * np.pi)
        if da > np.pi:
            da = da - 2 * np.pi
        # w = - da / self.NORM_ANGLE * self.MAX_ROTATION * v / self.MAX_VELOCITY
        # w = min(w, self.MAX_ROTATION)
        # w = max(w, -self.MAX_ROTATION)

        w = self.pid_rot.regulate(da, v / self.MAX_VELOCITY)
        print("dy = %f, v_perp = %f, da = %f, w = %f" %(dy, v_perp, da, w)) 
        v_x, v_y, _ = cvt_local2global((v, v_perp, 0), (0, 0, self.start_to_target_point[2] - point[2]))
        print("MOVE FORWARD v_x, v_y = (%f, %f) dist = %f target_dist = %f" % (v_x, v_y, dx, self.distance))
        return np.array([v_x, v_y, w])

    def regulate(self, point):
        if self.is_moving:
            if self.is_rotate:
                return self.rotate(point)
            elif self.is_move_forward:
                return self.move(point)
        else:
            return np.zeros(3)
