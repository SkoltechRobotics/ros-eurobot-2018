import numpy as np
import cv2


def euler_angles_to_rotation_matrix(theta):
    R_x = np.array([[1, 0, 0], [0, np.cos(theta[0]), -np.sin(theta[0])],
                    [0, np.sin(theta[0]),
                     np.cos(theta[0])]])
    R_y = np.array([[np.cos(theta[1]), 0,
                     np.sin(theta[1])], [0, 1, 0],
                    [-np.sin(theta[1]), 0,
                     np.cos(theta[1])]])
    R_z = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0],
                    [np.sin(theta[2]), np.cos(theta[2]), 0], [0, 0, 1]])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


K = np.array([[543.107811968841, 0.0, 630.7227631662455], [0.0, 540.0320855865099, 486.5749791589516], [0.0, 0.0, 1.0]])
D = np.array([[-0.05195582161040964], [-0.009477421485536482], [0.010893416415594874], [-0.005379345567357744]])
rotation_matrix = euler_angles_to_rotation_matrix((3 * np.pi / 4 + 0.2, 0, 0))

camera_position = np.array([[1500, 0, 1000]]).T
t = -rotation_matrix.dot(camera_position)
M = np.concatenate((rotation_matrix, t), axis=1)

T = np.array([[7.72128582e-01, -4.89107631e-02, 1.25099533e+02],
              [-5.58640771e-02, 7.94615984e-01, 1.35127960e+02],
              [-9.73771967e-05, -4.64651494e-05, 1.00000000e+00]])

L = np.array([[2.34, 0, 0], [0, -2.08, 2000], [0, 0, 0], [0, 0, 1]])
CAMERA_MATRIX_0 = T.dot(np.linalg.inv(M.dot(L)) * 10 ** 3)

L = np.array([[2.34, 0, 0], [0, -2.08, 2000], [0, 0, 510], [0, 0, 1]])
CAMERA_MATRIX_45 = T.dot(np.linalg.inv(M.dot(L)) * 10 ** 3)


def find_enemy(img):
    img_1 = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)[:, :, 2]
    thr_val = np.max(img_1) * 0.95
    _, thr = cv2.threshold(img_1, thr_val, 255, cv2.THRESH_BINARY)
    dist = cv2.distanceTransform(thr, distanceType=cv2.DIST_L1, maskSize=3)
    min_ind = np.unravel_index(np.argmax(dist), dist.shape)
    return min_ind
