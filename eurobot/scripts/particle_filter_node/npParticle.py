#!/usr/bin/env python
import numpy as np
import time
import logging
import math

# Dimensions of the playing field
WORLD_X = 3000
WORLD_Y = 2000
WORLD_BORDER = 16 # in rools it's 22!
BEAC_R = 44
BEAC_L = 100
BEAC_BORDER = 20

# for color == 'orange'
BEACONS = np.array([[WORLD_X+WORLD_BORDER+BEAC_BORDER+BEAC_L/2., WORLD_Y/2.], 
                [-(WORLD_BORDER+BEAC_BORDER+BEAC_L/2.), WORLD_Y-BEAC_L/2.], 
                [-(WORLD_BORDER+BEAC_BORDER+BEAC_L/2.), BEAC_L/2.]])

# parametres of lidar
BEAC_DIST_THRES = 200

class ParticleFilter:
    def __init__(self, particles=500, sense_noise=50, distance_noise=30, angle_noise=0.02, in_x=293, in_y=425, in_angle=3*np.pi/2, color='orange', max_itens=3500.0, max_dist=3700.0):
        global BEACONS
        if(color == 'green'):
            BEACONS = np.array([[-(WORLD_BORDER+BEAC_BORDER+BEAC_L/2.), WORLD_Y/2.], 
                            [WORLD_X+WORLD_BORDER+BEAC_BORDER+BEAC_L/2., WORLD_Y-BEAC_L/2.], 
                            [WORLD_X+WORLD_BORDER+BEAC_BORDER+BEAC_L/2., BEAC_L/2.]])

        stamp = time.time()
        self.particles_num = particles
        self.sense_noise = sense_noise
        self.distance_noise = distance_noise
        self.angle_noise = angle_noise
        self.warning = False
        self.last = (in_x,in_y,in_angle)
        # Create Particles
        x = np.random.normal(in_x, distance_noise, particles)
        y = np.random.normal(in_y, distance_noise, particles)
        orient = np.random.normal(in_angle, angle_noise, particles) % (2 * np.pi)
        self.particles = np.array([x, y, orient]).T  # instead of np.vstack((x,y,orient)).T
        logging.info('initialize time: '+str(time.time()-stamp))
        # Added Andrei for debug
        self.debug_info = []
        self.start_time = time.time()

        self.max_itens=max_itens
        self.max_dist=max_dist

        self.landmarks = [[],[]]
        
    def gaus(self, x, mu=0, sigma=1):
        """calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma"""
        return np.exp(- ((x-mu) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

    def move_particles(self, delta, mode="npy"): # delta = [dx,dy,d_rot]
        stamp = time.time()
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
        #logging.info('Particle Move time: ' + str(time.time() - stamp))

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
        C = np.append([0.], np.cumsum(weigths))# [0.] + [sum(weights[:i + 1]) for i in range(n)]
        j = 0
        u0 = (np.random.rand() + np.arange(n))/n
        for u in u0: #[(u0 + i) / n for i in range(n)
        # END intsead of
            while j < len(C) and u > C[j]:
                j += 1
            indices += [j - 1]
        return indices

    def calculate_main(self):
        stamp = time.time()
        x = np.mean(self.particles[:, 0])
        y = np.mean(self.particles[:, 1])
        zero_elem = self.particles[0, 2]
        # this helps if particles angles are close to 0 or 2*pi
        temporary = ((self.particles[:, 2]-zero_elem+np.pi) % (2.0 * np.pi))+zero_elem-np.pi
        orient = np.mean(temporary)
        answer = (x, y, orient)
        logging.info('main_calculation time' + str(time.time() - stamp))
        logging.info("Particle Filter coordinates: "+str(answer))
        return answer

    def particle_sense(self, scan):
        stamp = time.time()
        angle, distance = self.get_landmarks(scan)
        x_coords, y_coords = self.p_trans(angle,distance)
        self.landmarks = np.array([x_coords,y_coords])
        weights = self.weights(x_coords,y_coords)
        # correct if lost:
        if self.warning:
            return
            x = np.random.normal(self.last[0], 150, self.particles_num)
            y = np.random.normal(self.last[1], 150, self.particles_num)
            orient = np.random.normal(self.last[2], np.pi, self.particles_num) % (2 * np.pi)
            self.particles = np.array([x, y, orient]).T  # instead of np.vstack((x,y,orient)).T
            self.warning = False
            logging.info('particle_sense time :' + str(time.time() - stamp) + " points: " + str(len(x_coords)))
            return self.particles
        self.particles = self.particles[self.resample(weights), :]
        logging.info('particle_sense time :' + str(time.time() - stamp)+" points: "+str(len(x_coords)))
        return self.particles

    def weights(self, x_beac, y_beac):
        """Calculate particle weight based on its pose and lidar data"""
        # TODO check ICP implementation
        # BEACONS: from global BEACONS to particles local: (X, Y) - Nx3x2 matrices, N - number of particles
        # determines 3 beacon positions (x,y) for every particle in it's local coords
        res = BEACONS[np.newaxis, :, :] - self.particles[:, np.newaxis, :2]
        X = ( res[:,:,0]*np.cos(self.particles[:,2])[:, np.newaxis] 
              + res[:,:,1]*np.sin(self.particles[:,2])[:, np.newaxis])
        Y = ( -res[:, :, 0] * np.sin(self.particles[:, 2])[:, np.newaxis] 
              + res[:, :, 1] * np.cos(self.particles[:, 2])[:, np.newaxis]) 
        beacon = np.concatenate((X[:, :, np.newaxis], Y[:, :, np.newaxis]), axis=2)
        
        # beacon = beacons are in local coordinates of particles. 
        # distance from theoretical beacons to detected beacons from scan (x_beac, y_beac)
        # ln1, ln2, ln3: NxM (M - number of detected beacons from scan)
        ln1 = np.abs(np.sqrt((beacon[:, np.newaxis, 0, 0] - x_beac[np.newaxis, :])**2
                             + (beacon[:, np.newaxis, 0, 1] - y_beac[np.newaxis, :])**2) - BEAC_R)
        ln2 = np.abs(np.sqrt((beacon[:, np.newaxis, 1, 0] - x_beac[np.newaxis, :])**2
                             + (beacon[:, np.newaxis, 1, 1] - y_beac[np.newaxis, :])**2) - BEAC_R)
        ln3 = np.abs(np.sqrt((beacon[:, np.newaxis, 2, 0] - x_beac[np.newaxis, :])**2
                             + (beacon[:, np.newaxis, 2, 1] - y_beac[np.newaxis, :])**2) - BEAC_R)

        # lns are differences in theoretical and detected beacon data from lidar
        # ln1,ln2,ln3 are correct computed OK)

        # get minimal distance for each particle, its detected beacons to theoretical beacons
        errors = np.minimum(ln1, np.minimum(ln2, ln3))
        # too far real beacons go away: non valid
        limit_err = errors > BEAC_DIST_THRES

        # find map from detected beacon to closest real (valid distance neede)
        error_l1 = np.logical_and(np.equal(errors, ln1), ~limit_err)
        error_l2 = np.logical_and(np.equal(errors, ln2), ~limit_err)
        error_l3 = np.logical_and(np.equal(errors, ln3), ~limit_err)
        # bool error_li: sum to get number of particles for each of 3 beacons
        err_l1 = np.sum(error_l1, axis=-1)
        err_l2 = np.sum(error_l2, axis=-1)
        err_l3 = np.sum(error_l3, axis=-1)

        # find sum of errors near 3 beacons for each particle: beacon_error_sum Nx3
        beacon_error_sum = np.ones([self.particles_num, 3], dtype=np.float)*1000
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
        #weights = self.gaus(np.median(beacon_error_sum, axis=1),mu=0, sigma=self.sense_noise)
        # mean version
        weights = self.gaus(np.mean(beacon_error_sum, axis=1),mu=0, sigma=self.sense_noise)
        # check weights
        if self.warning == False and np.sum(weights)<self.gaus(self.sense_noise*15.0,mu =0,sigma= self.sense_noise)*self.particles_num:
            logging.info("Dangerous Situation")
            #self.warning = True


        if np.sum(weights) > 0:
            weights /= np.sum(weights)
        else:
            weights = np.ones(self.particles_num, dtype=np.float)/self.particles_num
        return weights
        # TODO try use median instead mean
        # TODO if odometry works very bad and weights are small use only lidar

    def localisation(self, delta_coords, lidar_data):
        tmstmp = time.time() - self.start_time
        self.move_particles([delta_coords[0], delta_coords[1], delta_coords[2]])
        # add aproximation
        self.particle_sense(lidar_data)
        if self.warning:
            print "Finding place"
            temp_num = self.particles_num
            self.particles_num = 5000
            x = np.random.uniform(self.last[0]-200,self.last[0]+ 200, self.particles_num)
            y = np.random.uniform(self.last[1]-200,self.last[1]+ 200, self.particles_num)
            orient = np.random.uniform(self.last[2]-np.pi/2,self.last[2]+ np.pi/2, self.particles_num) % (2 * np.pi)
            self.particles = np.array([x, y, orient]).T  # instead of np.vstack((x,y,orient)).T
            temp_sense = self.sense_noise
            self.sense_noise = 25
            lidar_data = get_raw()
            self.particle_sense(lidar_data)
            lidar_data = get_raw()
            self.particle_sense(lidar_data)
            self.move_particles([0, 0, 0])
            lidar_data = get_raw()
            self.particle_sense(lidar_data)
            self.move_particles([0, 0, 0])
            self.sense_noise = temp_sense
            lidar_data = get_raw()
            self.particle_sense(lidar_data)
            self.warning = False

            main_robot = self.calculate_main()
            self.particles_num = temp_num
            x = np.random.normal(main_robot[0], 100, self.particles_num)
            y = np.random.normal(main_robot[1], 100, self.particles_num)
            orient = np.random.normal(main_robot[2], np.pi/2, self.particles_num)
            self.particles = np.array([x, y, orient]).T
            lidar_data = get_raw()
            self.particle_sense(lidar_data)

        main_robot = self.calculate_main()
        self.last = main_robot
        return main_robot

# help functions

    def get_landmarks(self, scan):
        """Returns filtrated lidar data"""
        stamp = time.time()
        # array of indexes, where condition is met:
        ind = np.where(np.logical_and(scan[:, 1] > self.max_itens, scan[:, 0] < self.max_dist))[0]
        # array of angles/distances, for which condition is met
        angles = np.pi / 4 / 180 * ind
        distances = scan[ind, 0]
        #logging.info('scan preproccesing time: ' + str(time.time() - stamp))
        return (angles + np.pi * 5/ 4) % (2 * np.pi), distances


    def p_trans(self, agl, pit):
        #x_beac = pit*np.cos(agl) # multiply by minus in our robot
        #y_beac = pit*np.sin(agl)
        x_beac = pit*np.cos(agl)
        y_beac = pit*np.sin(agl)
        return x_beac,y_beac




