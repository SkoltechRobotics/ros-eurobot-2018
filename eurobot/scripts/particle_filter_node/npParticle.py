#!/usr/bin/env python
import numpy as np
import time
import logging
import math
from sklearn.cluster import DBSCAN
from scipy.optimize import least_squares, basinhopping


# Dimensions of the playing field
WORLD_X = 3000
WORLD_Y = 2000
WORLD_BORDER = 22
BEAC_R = 44
BEAC_L = 100
BEAC_BORDER = 20

calibrate_times = 0
beacon_storage = []

# ideal beacon coords for color == 'orange'
BEACONS = np.array([[WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y / 2.],
                    [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y - BEAC_L / 2.],
                    [-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), BEAC_L / 2.]])

# parameters of lidar
BEAC_DIST_THRES = 200
LIDAR_DELTA_ANGLE = (np.pi / 180) / 4
LIDAR_START_ANGLE = np.pi + np.pi / 4 # relatively to the robot
class ParticleFilter:
    def __init__(self, particles=500, sense_noise=50, distance_noise=5, angle_noise=0.02, in_x=293, in_y=425,
                 in_angle=3 * np.pi / 2, color='orange', max_itens=3500.0, max_dist=3700.0, reset_factor=10.0):
        global BEACONS
        if color == 'green':
            BEACONS = np.array([[-(WORLD_BORDER + BEAC_BORDER + BEAC_L / 2.), WORLD_Y / 2.],
                                [WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., WORLD_Y - BEAC_L / 2.],
                                [WORLD_X + WORLD_BORDER + BEAC_BORDER + BEAC_L / 2., BEAC_L / 2.]])
        global beacons
        beacons = BEACONS

        #stamp = time.time()
        self.particles_num = particles
        self.sense_noise = sense_noise
        self.distance_noise = distance_noise
        self.angle_noise = angle_noise
        self.warning = False
        self.last = (in_x, in_y, in_angle)
        # Create Particles
        x = np.random.normal(in_x, distance_noise, particles)
        y = np.random.normal(in_y, distance_noise, particles)
        orient = np.random.normal(in_angle, angle_noise, particles) % (2 * np.pi)
        self.particles = np.array([x, y, orient]).T  # instead of np.vstack((x,y,orient)).T
        #logging.info('initialize time: ' + str(time.time() - stamp))
        # Added Andrei for debug
        self.debug_info = []
        self.start_time = time.time()

        self.max_itens = max_itens
        self.max_dist = max_dist

        self.landmarks = [[], []]

        self.reset_factor = reset_factor
        self.reset = False

    @staticmethod
    def gaus(x, mu=0, sigma=1):
        """calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma"""
        return np.exp(- ((x - mu) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

    def move_particles(self, delta):  # delta = [dx,dy,d_rot]  mode="npy"
        # stamp = time.time()
        x_noise = np.random.normal(0, self.distance_noise, self.particles_num)
        y_noise = np.random.normal(0, self.distance_noise, self.particles_num)
        angle_noise = np.random.normal(0, self.angle_noise, self.particles_num)
        self.particles[:, 0] = self.particles[:, 0] + delta[0] + x_noise
        self.particles[:, 1] = self.particles[:, 1] + delta[1] + y_noise
        self.particles[:, 2] = (self.particles[:, 2] + delta[2] + angle_noise) % (2 * math.pi)
        # START instead of
        # NOT FASTER! NOT RIGHT((
        # # self.particles + noise + delta:
        # # noise - Nx3 : N - num particles, (x_noise, y_noise, angle_noise)
        # self.particles += (np.random.multivariate_normal(mean=np.array([0, 0, 0]),
        #                                     cov=np.diag(np.array([self.distance_noise,
        #                                                    self.distance_noise,
        #                                                    self.angle_noise])),
        #                                     size=(self.particles_num))
        #                    + np.array([delta]))
        # self.particles[:, 2] %= 2 * np.pi
        # END instead of
        # logging.info('Particle Move time: ' + str(time.time() - stamp))

    def resample(self, weights):
        """ according to weights """
        # OLD START
        # n = self.particles_num
        # indices = []
        # C = [0.] + [sum(weights[:i + 1]) for i in range(n)]
        # u0, j = np.random.rand(), 0
        # for u in [(u0 + i) / n for i in range(n)]:
        # START instead of
        n = self.particles_num
        weigths = np.array(weights)
        indices = []
        C = np.append([0.], np.cumsum(weigths))  # [0.] + [sum(weights[:i + 1]) for i in range(n)]
        j = 0
        u0 = (np.random.rand() + np.arange(n)) / n
        for u in u0:  # [(u0 + i) / n for i in range(n)
            # END instead of
            while j < len(C) and u > C[j]:
                j += 1
            indices += [j - 1]
        return indices

    def calculate_main(self):
        #stamp = time.time()
        x = np.mean(self.particles[:, 0])
        y = np.mean(self.particles[:, 1])
        zero_elem = self.particles[0, 2]
        # this helps if particles angles are close to 0 or 2*pi
        temporary = ((self.particles[:, 2] - zero_elem + np.pi) % (2.0 * np.pi)) + zero_elem - np.pi
        orient = np.mean(temporary)
        answer = (x, y, orient)
        #logging.info('main_calculation time' + str(time.time() - stamp))
        #logging.info("Particle Filter coordinates: " + str(answer))
        return answer

    def particle_sense(self, scan, particles):
        #stamp = time.time()
        angle, distance = self.get_landmarks(scan)
        x_coords, y_coords = self.p_trans(angle, distance)
        self.landmarks = np.array([x_coords, y_coords])
        weights = self.weights(x_coords, y_coords, particles)
        # correct if lost:
        if self.warning:
            return
            # x = np.random.normal(self.last[0], 150, self.particles_num)
            # y = np.random.normal(self.last[1], 150, self.particles_num)
            # orient = np.random.normal(self.last[2], np.pi, self.particles_num) % (2 * np.pi)
            # self.particles = np.array([x, y, orient]).T  # instead of np.vstack((x,y,orient)).T
            # self.warning = False
            # logging.info('particle_sense time :' + str(time.time() - stamp) + " points: " + str(len(x_coords)))
            # return self.particles
        particles = particles[self.resample(weights), :]
        #logging.info('particle_sense time :' + str(time.time() - stamp) + " points: " + str(len(x_coords)))
        global beacon_storage
        global beacon_storage
        return particles

    def weights(self, x_beac, y_beac, particles):
        """Calculate particle weights based on their pose and landmards"""
        # TODO check ICP implementation
        # BEACONS: from global BEACONS to particles local: (X, Y) - Nx3x2 matrices, N - number of particles
        # determines 3 beacon positions (x,y) for every particle in it's local coords
        res = BEACONS[np.newaxis, :, :] - particles[:, np.newaxis, :2]
        X = (-res[:, :, 0] * np.sin(particles[:, 2])[:, np.newaxis]
             + res[:, :, 1] * np.cos(particles[:, 2])[:, np.newaxis])
        Y = (-res[:, :, 0] * np.cos(particles[:, 2])[:, np.newaxis]
             - res[:, :, 1] * np.sin(particles[:, 2])[:, np.newaxis])
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
        # weights of particles are estimated via errors got from scan of beacons and theoretical beacons location
        # median version
        # weights = self.gaus(np.median(beacon_error_sum, axis=1),mu=0, sigma=self.sense_noise)
        # mean version
        weights = self.gaus(np.mean(beacon_error_sum, axis=1), mu=0, sigma=self.sense_noise)
        # check weights
        #if self.warning is False and np.sum(weights) < self.gaus(self.sense_noise * 15.0, mu=0, sigma=self.sense_noise) * particles.shape[0]:
            #logging.info("Dangerous Situation")
            # self.warning = True

        if np.sum(weights) > 0:
            weights /= np.sum(weights)
        else:
            weights = np.ones(particles.shape[0], dtype=np.float) / particles.shape[0]
        return weights
        # TODO try use median instead mean
        # TODO if odometry works very bad and weights are small use only lidar

    def localisation(self, delta_coords, lidar_data):

        # beacon calibration
        global calibrate_times
        if calibrate_times > 0:
            beac = self.find_beacons(lidar_data, 0)
            calibrate_times -= 1
            global beacon_storage
            beacon_storage.append(beac)
            return self.last

        if self.reset is True:
            # reset particles from scratch according to reset flag
            successfully, coords  = self.evaluate_coords(lidar_data)
            if successfully: # TODO behavior in case of failing to reset
                self.reset_particles(coords)
            self.reset = False
        else:
            # move particles according to estimated movemen
            self.move_particles([delta_coords[0], delta_coords[1], delta_coords[2]])
        # add approximation
        self.particles = self.particle_sense(lidar_data, self.particles)

        # if self.warning:
        #    print "Finding place"
        #    temp_num = self.particles_num
        #    self.particles_num = 5000
        #    x = np.random.uniform(self.last[0]-200,self.last[0]+ 200, self.particles_num)
        #    y = np.random.uniform(self.last[1]-200,self.last[1]+ 200, self.particles_num)
        #    orient = np.random.uniform(self.last[2]-np.pi/2,self.last[2]+ np.pi/2, self.particles_num) % (2 * np.pi)
        #    self.particles = np.array([x, y, orient]).T  # instead of np.vstack((x,y,orient)).T
        #    temp_sense = self.sense_noise
        #    self.sense_noise = 25
        #    lidar_data = get_raw()
        #    self.particle_sense(lidar_data)
        #    lidar_data = get_raw()
        #    self.particle_sense(lidar_data)
        #    self.move_particles([0, 0, 0])
        #    lidar_data = get_raw()
        #    self.particle_sense(lidar_data)
        #    self.move_particles([0, 0, 0])
        #    self.sense_noise = temp_sense
        #    lidar_data = get_raw()
        #    self.particle_sense(lidar_data)
        #    self.warning = False

        #    main_robot = self.calculate_main()
        #    self.particles_num = temp_num
        #    x = np.random.normal(main_robot[0], 100, self.particles_num)
        #    y = np.random.normal(main_robot[1], 100, self.particles_num)
        #    orient = np.random.normal(main_robot[2], np.pi/2, self.particles_num)
        #    self.particles = np.array([x, y, orient]).T
        #    lidar_data = get_raw()
        #    self.particle_sense(lidar_data)

        main_robot = self.calculate_main()
        self.last = main_robot
        return main_robot

    # help functions

    def get_landmarks(self, scan):
        """Returns filtrated lidar data"""
        ind = np.where(np.logical_and(scan[:, 1] > self.max_itens, scan[:, 0] < self.max_dist))[0]
        # array of angles/distances, for which condition is met
        angles = LIDAR_DELTA_ANGLE * ind
        distances = scan[ind, 0]
        # logging.info('scan preproccesing time: ' + str(time.time() - stamp))
        return (angles + LIDAR_START_ANGLE) % (2 * np.pi), distances

    @staticmethod
    def p_trans(a, d):
        x_beac = d * np.cos(a)
        y_beac = d * np.sin(a)
        return x_beac, y_beac

    def start_over(self):
        self.reset = True

    def get_center(self, landmarks):
        def fun(point, landmarks):
                return np.sum((landmarks - point) ** 2, axis=1) ** .5 - BEAC_R
        med = np.median(landmarks, axis=0)
        dist = np.sum(med ** 2) ** .5
        center_by_med = med + BEAC_R * np.array([med[0] / dist, med[1] / dist])
        center = least_squares(fun, center_by_med, args=[landmarks])
        return center

    def evaluate_coords(self, scan):
        # landmarks
        angles, distances = self.get_landmarks(scan)
        angles = (angles + np.pi / 2) % (2 * np.pi)
        x_landm, y_landm = self.p_trans(angles, distances)
        landmarks = np.array([x_landm, y_landm]).T

        # clustering
        db = DBSCAN(eps=40, min_samples=7).fit(landmarks)
        labels = db.labels_
        unique_labels = set(labels)

        # beacon centers in LIDAR frame
        centers = []
        for l in unique_labels:
            if l == -1:
                # noise
                continue

            class_member_mask = (labels == l)

            center = self.get_center(landmarks[class_member_mask])
            centers.append(center.x)

        if len(centers) < 2 or len(centers) > 3:
            return False, []
        else:
            found = False
            if len(centers) == 2:
                if np.abs(np.linalg.norm(centers[0] - centers[1]) - (WORLD_Y - BEAC_L)) < BEAC_L / 2:
                    A = centers[1]
                    B = centers[0]
                    found = True
            elif len(centers) == 3:
                # find beacons A and B
                for i in range(3):
                    j = (i + 1) % 3
                    if np.abs(np.linalg.norm(centers[i] - centers[j]) - (WORLD_Y - BEAC_L)) < BEAC_L / 2:
                        A = centers[j]
                        B = centers[i]
                        found = True
                        break
            if found == False:
                return False, []
            R = np.array([0, 0])
            AR = np.linalg.norm(A - R)
            cosRAB = np.sum((B - A) * (R - A)) / np.linalg.norm(B - A) / np.linalg.norm(R - A)
            sinRAB = (1 - cosRAB ** 2) ** .5
            x = AR * sinRAB - WORLD_BORDER - BEAC_BORDER - BEAC_L / 2
            y = AR * cosRAB + BEAC_L / 2

            eY = np.array([0, 1])
            cosa = np.sum(eY * (B - A)) / np.linalg.norm(B - A)
            a = np.arccos(cosa)
            if (B - A)[0] < 0:
                a = 2 * np.pi - a
            
            return True, np.array([x, y, a])

    def reset_particles(self, coords):
        inp_x, inp_y, inp_a = coords
        x = np.random.normal(inp_x, self.reset_factor * self.distance_noise, self.particles.shape[0])
        y = np.random.normal(inp_y, self.reset_factor * self.distance_noise, self.particles.shape[0])
        orient = np.random.normal(inp_a, self.reset_factor * self.angle_noise, self.particles.shape[0]) % (2 * np.pi)
        self.particles = np.array([x, y, orient]).T 

    def find_beacons(self, scan, pose_index):
        """ Determine beacon coords from scan. """
        if pose_index == 0:
            a = self.last[2]
            pose = np.array([214, 158, a])
        else:
            return False

        beac = []
        
        # landmarks
        angles, distances = self.get_landmarks(scan)
        angles = (angles + np.pi / 2) % (2 * np.pi)
        x_landm, y_landm = self.p_trans(angles, distances)
        landmarks = np.array([x_landm, y_landm]).T

        # clustering
        db = DBSCAN(eps=40, min_samples=5).fit(landmarks)
        labels = db.labels_
        unique_labels = set(labels)

        # beacon centers in LIDAR frame
        centers = []
        for l in unique_labels:
            if l == -1:
                # noise
                continue

            class_member_mask = (labels == l)

            center = self.get_center(landmarks[class_member_mask])
            centers.append(center.x)

        #print 'beacons in LIDAR frame:', centers
        
        # beacon centers in world frame
        centers = self.cvt_local2global(centers, pose)
       
        #print 'beacons in world frame:',  centers

        for beacon in BEACONS:
            distances = np.linalg.norm(centers - beacon, axis=1)
            closest = np.argmin(distances)
            if distances[closest] < BEAC_L:
                beac.append(centers[closest])
            else:
                beac.append([np.nan, np.nan])
        #print 'beac:', beac
        return beac


    def cvt_local2global(self, points, pose):
        """ Transformation of list of points from local to global frame by given robot pose in global frame."""
        angle = pose[2]
        M = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        ans = [np.matmul(M, point.reshape((2,1))).reshape(2) + pose[:2] for point in points]
        return ans


    def clean_storage(self):
        global beacon_storage
        beacon_storage = []

    def calibrate_beacons(self):
        global calibrate_times
        calibrate_times = 30

    def set_beacons(self):
        global beacons, beacon_storage
        mean = np.nanmean(beacon_storage, axis=0)
        if mean.shape != (3, 2) or np.any(np.isnan(mean)):
            return False, []
        else:
            return True, mean
