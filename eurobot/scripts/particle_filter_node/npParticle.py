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
    def __init__(self, particles_num=500, sense_noise=50, distance_noise=5, angle_noise=0.02,
                 start_x=293, start_y=425, start_angle=3 * np.pi / 2, color='orange', min_intens=3500.0,
                 max_dist=3700.0, beac_dist_thresh=200, k_angle=2, num_is_near_thresh=0.1, dist_offset=15):

        self.start_coords = np.array([start_x, start_y, start_angle])
        self.color = color
        if color == 'orange':
            self.beacons = ORANGE_BEACONS
        else:
            self.beacons = GREEN_BEACONS

        self.particles_num = particles_num
        self.sense_noise = sense_noise
        self.distance_noise = distance_noise
        self.angle_noise = angle_noise
        self.min_intens = min_intens
        self.max_dist = max_dist
        self.last = (start_x, start_y, start_angle)
        self.beac_dist_thresh = beac_dist_thresh
        self.k_angle = k_angle
        self.num_is_near_thresh = num_is_near_thresh
        self.dist_offset = dist_offset
        
        # Create Particles
        x = np.random.normal(start_x, distance_noise, particles_num)
        y = np.random.normal(start_y, distance_noise, particles_num)
        angle = np.random.normal(start_angle, angle_noise, particles_num) % (2 * np.pi)
        self.particles = np.array([x, y, angle]).T
        self.landmarks = [[], []]
        self.cost_function = []
        self.best_particles = {}

        self.min_cost_function = 0

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
        return main_robot

    def particle_sense(self, scan, particles):
        angle, distance = self.get_landmarks(scan)
        x_coords, y_coords = self.p_trans(angle, distance)
        self.landmarks = np.array([x_coords, y_coords]).T
        weights = self.weights(self.landmarks, particles)
        inds = self.resample(weights)
        self.min_cost_function = np.mean(self.cost_function)
        particles = particles[inds, :]
        return particles

    def move_particles(self, delta):  # delta = [dx,dy,d_rot]
        x_noise = np.random.normal(0, self.distance_noise, self.particles_num)
        y_noise = np.random.normal(0, self.distance_noise, self.particles_num)
        angle_noise = np.random.normal(0, self.angle_noise, self.particles_num)
        noise = np.array([x_noise, y_noise, angle_noise]).T
        move_point = delta + noise
        self.particles = cvt_local2global(move_point, self.particles)
        self.particles[self.particles[:, 1] > 2000 - 120, 1] = 2000 - 120
        self.particles[self.particles[:, 1] < 0, 1] = 0
        self.particles[self.particles[:, 0] > 3000 - 120, 0] = 3000 - 120
        self.particles[self.particles[:, 0] < 120, 0] = 120

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

    def weights(self, landmarks, particles):
        """Calculate particle weights based on their pose and landmarks"""
        # determines 3 beacon positions (x,y) for every particle in it's local coords
        res = self.beacons[np.newaxis, :, :] - particles[:, np.newaxis, :2]
        X = (res[:, :, 0] * np.cos(particles[:, 2])[:, np.newaxis]
             + res[:, :, 1] * np.sin(particles[:, 2])[:, np.newaxis])
        Y = (-res[:, :, 0] * np.sin(particles[:, 2])[:, np.newaxis]
             + res[:, :, 1] * np.cos(particles[:, 2])[:, np.newaxis])
        beacons = np.concatenate((X[:, :, np.newaxis], Y[:, :, np.newaxis]), axis=2)

        # find closest beacons to landmark
        dist_from_beacon = np.linalg.norm(beacons[:, np.newaxis, :, :] -
                                          landmarks[np.newaxis, :, np.newaxis, :], axis=3)
        ind_closest_beacon = np.argmin(dist_from_beacon, axis=2)
        closest_beacons = beacons[np.arange(beacons.shape[0])[:, np.newaxis], ind_closest_beacon]

        # Calculate cos of angle between landmark, beacon and particle
        scalar_product = np.sum((closest_beacons - particles[:, np.newaxis, :2]) *
                                (closest_beacons - landmarks[np.newaxis, :, :2]), axis=2)
        dist_from_closest_beacon_to_particle = np.linalg.norm(closest_beacons - particles[:, np.newaxis, :2], axis=2)
        dist_from_closest_beacon_to_landmark = np.linalg.norm(closest_beacons - landmarks[np.newaxis, :, :2], axis=2)
        cos_landmarks = scalar_product / np.where(dist_from_closest_beacon_to_landmark,
                                                  dist_from_closest_beacon_to_landmark, 1) / np.where(
            dist_from_closest_beacon_to_particle, dist_from_closest_beacon_to_particle, 1)

        # Calculate errors of position of landmarks
        errors = (dist_from_closest_beacon_to_landmark - BEAC_R) ** 2 + self.k_angle * (1 - cos_landmarks) ** 2

        # too far real beacons go away: non valid
        is_near = dist_from_closest_beacon_to_landmark < self.beac_dist_thresh
        is_near_sum = np.sum(is_near, axis=0)
        is_near_or = (is_near_sum > is_near.shape[0] * self.num_is_near_thresh)
        num_good_landmarks = np.sum(is_near_or)
        sum_errors = np.sum(errors * is_near_or[np.newaxis, :], axis=1)
        if num_good_landmarks:
            self.cost_function = np.sqrt(sum_errors) / num_good_landmarks
        else:
            self.cost_function = np.ones(sum_errors.shape[0]) * 1000
        weights = self.gaus(self.cost_function, mu=0, sigma=self.sense_noise)
        if np.sum(weights) > 0:
            weights /= np.sum(weights)
        else:
            weights = np.ones(particles.shape[0], dtype=np.float) / particles.shape[0]

        self.best_particles = {}
        best_particles_inds = np.argsort(self.cost_function)[:10]
        self.best_particles["particles"] = particles[best_particles_inds]
        self.best_particles["cost_function"] = self.cost_function[best_particles_inds]
        self.best_particles["num_good_landmarks"] = num_good_landmarks
        self.best_particles["weights"] = weights[best_particles_inds]
        return weights

    def get_landmarks(self, scan):
        """Returns filtrated lidar data"""
        ind = np.where(np.logical_and(scan[:, 1] > self.min_intens, scan[:, 0] < self.max_dist))[0]
        angles = (LIDAR_DELTA_ANGLE * ind + LIDAR_START_ANGLE) % (2 * np.pi)
        distances = scan[ind, 0]
        return angles, distances - self.dist_offset
