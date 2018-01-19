# Point is always the x, y coordinate of object and anlge of rotation of its frame
import numpy as np

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


class TrackRegulator(object):
    def __init__(self):
        self.MAX_VELOCITY = 0.2
        self.MAX_ROTATION = 1
        self.MIN_VELOCITY = 0.02
        self.MIN_ROTATION = 0.10
        self.NORM_ANGLE = 3.14 / 8
        self.NORM_DISTANCE = 50
        self.PERP_NORM_DISTANCE = 50
        self.PERP_MAX_RATE = 1
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
        self.start_angle = point[2]
        self.is_rotate = True

    def start_move_forward(self, point):
        print("Start move forward")
        self.is_rotate = False
        self.is_move_forward = True
        self.distance = np.sum((self.target_point[:2] - point[:2]) ** 2) ** 0.5

        self.start_to_target_point = np.zeros(3)
        self.start_to_target_point[:2] = point[:2]
        dp = self.target_point[:2] - point[:2]
        self.start_to_target_point[2] = np.arctan2(dp[1], dp[0])

    def start_move(self, target_point, point):
        self.is_moving = True
        print("Start move")
        self.target_point = target_point
        self.start_rotate(point)

    def rotate(self, point):
        da = (point[2] - self.start_angle) % (2 * np.pi)
        if self.rotation_diraction == -1:
            da = 2 * np.pi - da

        if da >= self.dangle:
            print("Stop rotate")
            self.start_move_forward(point)
            return np.zeros(3)
        elif da > self.dangle - self.NORM_ANGLE:
            v_angle = np.sqrt((self.dangle - da) / self.NORM_ANGLE) * self.MAX_ROTATION
        elif da < self.NORM_ANGLE:
            v_angle = np.sqrt(da / self.NORM_ANGLE) * self.MAX_ROTATION
        else:
            v_angle = self.MAX_ROTATION
        v_angle += self.MIN_ROTATION
        print("ROTATE v_angle = %f da = %f" % (v_angle, da))
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
            v = np.sqrt(dx) / self.NORM_DISTANCE * self.MAX_VELOCITY
        elif dx < 0:
            v = 0
        else:
            v = self.MAX_VELOCITY
        v += self.MIN_VELOCITY
        
        dy = point_in_target_system[1]
        v_perp = - v * dy / self.PERP_NORM_DISTANCE * self.PERP_MAX_RATE
        v_perp = min(v_perp, self.MAX_VELOCITY)
        v_perp = max(v_perp, -self.MAX_VELOCITY)
        w = - (point[2] - self.target_point[2]) / self.NORM_ANGLE * self.MAX_ROTATION * v / self.MAX_VELOCITY
        v_x, v_y, _ = cvt_local2global((v, v_perp, 0), (0, 0, self.start_to_target_point[2] - point[2]))
        # v_x = v * np.cos(self.start_to_target_point[2] - point[2])
        # v_y = v * np.sin(self.start_to_target_point[2] - point[2])
        print("MOVE FORWARD v_x, v_y = (%f, %f) dist = %f target_dist = %f" % (v_x, v_y, dx, self.distance))
        return np.array([v_x, v_y, w])

    def regulate(self, point):
        if self.is_rotate:
            return self.rotate(point)
        elif self.is_move_forward:
            return self.move(point)
        else:
            return np.zeros(3)
