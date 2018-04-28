#!/usr/bin/env python
import numpy as np


def cvt_local2global(local_point, src_point):
    x, y, a = local_point.T
    X, Y, A = src_point.T
    x1 = x * np.cos(A) - y * np.sin(A) + X
    y1 = x * np.sin(A) + y * np.cos(A) + Y
    a1 = (a + A) % (2 * np.pi)
    return np.array([x1, y1, a1]).T


def cvt_global2local(global_point, src_point):
    x1, y1, a1 = global_point.T
    X, Y, A = src_point.T
    x = x1 * np.cos(A) + y1 * np.sin(A) - X * np.cos(A) - Y * np.sin(A)
    y = -x1 * np.sin(A) + y1 * np.cos(A) + X * np.sin(A) - Y * np.cos(A)
    a = (a1 - A) % (2 * np.pi)
    return np.array([x, y, a]).T


def find_src(global_point, local_point):
    x, y, a = local_point.T
    x1, y1, a1 = global_point.T
    A = (a1 - a) % (2 * np.pi)
    X = x1 - x * np.cos(A) + y * np.sin(A)
    Y = y1 - x * np.sin(A) - y * np.cos(A)
    return np.array([X, Y, A]).T

# Dimensions of the playing field
WORLD_X = 3000
WORLD_Y = 2000
WORLD_BORDER = 22
BEAC_R = 96.0 / 2
BEAC_L = 100.0
BEAC_BORDER = 22.0

ORANGE_BEACONS = np.array([[WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y / 2.],
                           [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y - BEAC_L / 2.],
                           [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), BEAC_L / 2.]])

GREEN_BEACONS = np.array([[-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y / 2.],
                          [WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y - BEAC_L / 2.],
                          [WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., BEAC_L / 2.]])

# parameters of lidar

LIDAR_DELTA_ANGLE = (np.pi / 180) / 4
LIDAR_START_ANGLE = -(np.pi / 2 + np.pi / 4)


class ParticleFilter:
    def __init__(self, particles_num=500, sense_noise=50, distance_noise=5, angle_noise=0.02, start_x=293, start_y=425,
                 start_angle=3 * np.pi / 2, color='orange', max_itens=3500.0, max_dist=3700.0):

        self.start_coords = np.array([start_x, start_y, start_angle])

        if color == 'orange':
            self.beacons = ORANGE_BEACONS
        else:
            self.beacons = GREEN_BEACONS

        self.particles_num = particles_num
        self.sense_noise = sense_noise
        self.distance_noise = distance_noise
        self.angle_noise = angle_noise
        self.max_itens = max_itens
        self.max_dist = max_dist
        self.last = (start_x, start_y, start_angle)

        # Create Particles
        x = np.random.normal(start_x, distance_noise, particles_num)
        y = np.random.normal(start_y, distance_noise, particles_num)
        angle = np.random.normal(start_angle, angle_noise, particles_num) % (2 * np.pi)
        self.particles = np.array([x, y, angle]).T
        self.landmarks = [[], []]

    @staticmethod
    def gaus(x, mu=0, sigma=1):
        """calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma"""
        return np.exp(- ((x - mu) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

    @staticmethod
    def p_trans(a, d):
        x_beac = d * np.cos(a)
        y_beac = d * np.sin(a)
        return x_beac, y_beac

    def localisation(self, delta_coords, lidar_data):
        self.move_particles([delta_coords[0], delta_coords[1], delta_coords[2]])
        self.particles = self.particle_sense(lidar_data, self.particles)
        main_robot = self.calculate_main()
        self.last = main_robot
        return main_robot

    def particle_sense(self, scan, particles):
        angle, distance = self.get_landmarks(scan)
        x_coords, y_coords = self.p_trans(angle, distance)
        self.landmarks = np.array([x_coords, y_coords])
        weights = self.weights(x_coords, y_coords, particles)
        particles = particles[self.resample(weights), :]
        return particles

    def move_particles(self, delta):  # delta = [dx,dy,d_rot]
        x_noise = np.random.normal(0, self.distance_noise, self.particles_num)
        y_noise = np.random.normal(0, self.distance_noise, self.particles_num)
        angle_noise = np.random.normal(0, self.angle_noise, self.particles_num)
        self.particles[:, 0] = self.particles[:, 0] + delta[0] + x_noise
        self.particles[:, 1] = self.particles[:, 1] + delta[1] + y_noise
        self.particles[:, 2] = (self.particles[:, 2] + delta[2] + angle_noise) % (2 * np.pi)

    def resample(self, weights):
        """ according to weights """
        n = self.particles_num
        weigths = np.array(weights)
        indices = []
        C = np.append([0.], np.cumsum(weigths))
        j = 0
        u0 = (np.random.rand() + np.arange(n)) / n
        for u in u0:
            while j < len(C) and u > C[j]:
                j += 1
            indices += [j - 1]
        return indices

    def calculate_main(self):
        x = np.mean(self.particles[:, 0])
        y = np.mean(self.particles[:, 1])
        zero_elem = self.particles[0, 2]
        # this helps if particles angles are close to 0 or 2*pi
        temporary = ((self.particles[:, 2] - zero_elem + np.pi) % (2.0 * np.pi)) + zero_elem - np.pi
        angle = np.mean(temporary)
        return np.array((x, y, angle))

    def weights(self, x_beac, y_beac, particles):
        """Calculate particle weights based on their pose and landmards"""
        # BEACONS: from global BEACONS to particles local: (X, Y) - Nx3x2 matrices, N - number of particles
        # determines 3 beacon positions (x,y) for every particle in it's local coords
        res = self.beacons[np.newaxis, :, :] - particles[:, np.newaxis, :2]
        X = (res[:, :, 0] * np.cos(particles[:, 2])[:, np.newaxis]
             + res[:, :, 1] * np.sin(particles[:, 2])[:, np.newaxis])
        Y = (-res[:, :, 0] * np.sin(particles[:, 2])[:, np.newaxis]
             + res[:, :, 1] * np.cos(particles[:, 2])[:, np.newaxis])
        beacon = np.concatenate((X[:, :, np.newaxis], Y[:, :, np.newaxis]), axis=2)

        # beacon = beacons are in local coordinates of particles. 
        # distance from theoretical beacons to detected beacons from scan (x_beac, y_beac)
        # ln1, ln2, ln3: NxM (M - number of detected beacons from scan)
        ln1 = np.abs(np.sqrt((beacon[:, np.newaxis, 0, 0] - x_beac[np.newaxis, :]) ** 2
                             + (beacon[:, np.newaxis, 0, 1] - y_beac[np.newaxis, :]) ** 2) - BEAC_R)
        ln2 = np.abs(np.sqrt((beacon[:, np.newaxis, 1, 0] - x_beac[np.newaxis, :]) ** 2
                             + (beacon[:, np.newaxis, 1, 1] - y_beac[np.newaxis, :]) ** 2) - BEAC_R)
        ln3 = np.abs(np.sqrt((beacon[:, np.newaxis, 2, 0] - x_beac[np.newaxis, :]) ** 2
                             + (beacon[:, np.newaxis, 2, 1] - y_beac[np.newaxis, :]) ** 2) - BEAC_R)

        # lns are differences in theoretical and detected beacon data from lidar
        # ln1,ln2,ln3 are correct computed OK)

        # get minimal distance for each particle, its detected beacons to theoretical beacons
        errors = np.minimum(ln1, np.minimum(ln2, ln3))
        # too far real beacons go away: non valid
        limit_err = errors > BEAC_DIST_THRES

        # find map from detected beacon to closest real (valid distance need)
        error_l1 = np.logical_and(np.equal(errors, ln1), ~limit_err)
        error_l2 = np.logical_and(np.equal(errors, ln2), ~limit_err)
        error_l3 = np.logical_and(np.equal(errors, ln3), ~limit_err)
        # bool error_li: sum to get number of particles for each of 3 beacons
        err_l1 = np.sum(error_l1, axis=-1)
        err_l2 = np.sum(error_l2, axis=-1)
        err_l3 = np.sum(error_l3, axis=-1)

        # find sum of errors near 3 beacons for each particle: beacon_error_sum Nx3
        beacon_error_sum = np.ones([particles.shape[0], 3], dtype=np.float) * 1000
        ind = np.where(err_l1)[0]
        if ind.size:
            beacon_error_sum[ind, 0] = np.sum(np.where(error_l1, errors, 0), axis=-1)[ind] / err_l1[ind]
        ind = np.where(err_l2)[0]
        if ind.size:
            beacon_error_sum[ind, 1] = np.sum(np.where(error_l2, errors, 0), axis=-1)[ind] / err_l2[ind]
        ind = np.where(err_l3)[0]
        if ind.size:
            beacon_error_sum[ind, 2] = np.sum(np.where(error_l3, errors, 0), axis=-1)[ind] / err_l3[ind]
        weights = self.gaus(np.mean(beacon_error_sum, axis=1), mu=0, sigma=self.sense_noise)

        if np.sum(weights) > 0:
            weights /= np.sum(weights)
        else:
            weights = np.ones(particles.shape[0], dtype=np.float) / particles.shape[0]
        return weights

    def get_landmarks(self, scan):
        """Returns filtrated lidar data"""
        ind = np.where(np.logical_and(scan[:, 1] > self.max_itens, scan[:, 0] < self.max_dist))[0]
        angles = (LIDAR_DELTA_ANGLE * ind + LIDAR_START_ANGLE) % (2 * np.pi)
        distances = scan[ind, 0]
        return angles, distances
