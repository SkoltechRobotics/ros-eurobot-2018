import cv2
import skimage.filters
import numpy as np
from numba import njit
from EnemiesRecognition import K, D

K1 = K.copy()
K1[:2] /= 4
K2 = K1.copy()
K2[:2, :2] /= 2


@njit
def edge_cost_func(edge, params):
    BETA = 15.
    N = (params[1] - params[0]) * (params[3] - params[2])
    n = np.sum(edge[params[0]:params[1], params[2]:params[3]])
    return N - n * BETA


@njit
def get_new_params(old_params, j, k):
    params = old_params.copy()
    params[j] -= (1 - 2 * (j % 2)) * k
    diff_params = old_params.copy()
    diff_params[2 * (j // 2)], diff_params[2 * (j // 2) + 1] = \
        min(old_params[j], params[j]), max(old_params[j], params[j])
    return diff_params, params


@njit
def local_search(init_params, edge):
    params = init_params.copy()
    for i in range(200):
        f = False
        for k in [3, 2, 1, -1]:
            for j in range(4):
                diff_params, new_params = get_new_params(params, j, k)
                cost = edge_cost_func(edge, diff_params)
                if (k > 0 and cost > 0) or (k < 0 and cost < 0):
                    params = new_params
                    f = True
                    break
            if f:
                break
        if not f:
            break
    return params


def search_cube(img):

    img3 = np.zeros_like(img, dtype=np.float32)
    for c in range(3):
        img3[:, :, c] = skimage.filters.gaussian(img[:, :, c], sigma=2)
    img3 = (255 * img3).astype(np.uint8)

    edge = 0
    for c in range(3):
        edge = edge + skimage.filters.sobel(img3[:, :, c])
    edge = edge ** 0.8
    init_params = np.array([90, 130, 140, 180])
    params = local_search(init_params, edge)
    return params, edge
