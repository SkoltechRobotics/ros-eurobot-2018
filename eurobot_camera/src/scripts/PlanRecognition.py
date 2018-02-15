import cv2
import numpy as np
import skimage.color
import skimage.exposure
import skimage.segmentation
import skimage.future
import skimage.filters
import itertools


def permutations(array, n=None, perm=None):
    if perm is None:
        perm = []
    if n is None:
        n = len(array)
    if n == 0:
        yield perm
    else:
        for i in range(len(array)):
            for x in permutations(array[:i] + array[i + 1:], n - 1, perm + [array[i]]):
                yield x


def all_plans():
    for x in permutations(list(range(5)), 3):
        yield x


PLANS = list(all_plans())
# COLORS = np.array([[0, 124, 176], [208, 93, 40], [14, 14, 16], [97, 153, 59],
#                    [247, 181, 0]], dtype=np.uint8)

COLORS = np.array([[0, 123, 176], [208, 90, 40], [28, 28, 32], [96, 153, 59], [247, 181, 0]], dtype=np.uint8)
HSV_COLORS = cv2.cvtColor(np.array([COLORS]), cv2.COLOR_RGB2HSV)[0]

LABELS = ['blue', 'orange', 'black', 'green', 'yellow']
STEP = 10


def img_transformation(img, c=-0.5, **kwargs):
    # k_size = int(STEP)
    # clh_img = cv2.medianBlur(img, k_size + k_size % 2 - 1)
    # clh_img = cv2.cvtColor(clh_img, cv2.COLOR_RGB2HSV)
    # clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(STEP, STEP))
    # clh_img[:, :, 2] = clahe.apply(clh_img[:, :, 2])
    # clh_img = cv2.cvtColor(clh_img, cv2.COLOR_HSV2RGB)

    # img2 = clh_img
    # img2[:, :, 1] = skimage.exposure.adjust_sigmoid(img2[:, :, 1], 0.5, 10)
    # img2[:, :, 2] = skimage.exposure.adjust_sigmoid(img2[:, :, 2], np.asscalar(np.mean(img2[:, :, 1] / 255)), 10)

    # laplace_img = clh_img
    # laplace_img = c * cv2.Laplacian(laplace_img / 255., cv2.CV_64F) + laplace_img / 255.
    # laplace_img = (255 * laplace_img.clip(0, 1)).astype(np.uint8)
    # output_img = laplace_img

    output_img = (skimage.filters.gaussian(img, 2) * 255).astype(np.uint8)
    return output_img


def _weight_mean_color(graph, src, dst, n):
    """Callback to handle merging nodes by recomputing mean color.

    The method expects that the mean color of `dst` is already computed.

    Parameters
    ----------
    graph : RAG
        The graph under consideration.
    src, dst : int
        The vertices in `graph` to be merged.
    n : int
        A neighbor of `src` or `dst` or both.

    Returns
    -------
    data : dict
        A dictionary with the `"weight"` attribute set as the absolute
        difference of the mean color between node `dst` and `n`.
    """

    diff = graph.node[dst]['mean color'] - graph.node[n]['mean color']
    diff = np.linalg.norm(diff)
    return {'weight': diff}


def merge_mean_color(graph, src, dst):
    """Callback called before merging two nodes of a mean color distance graph.

    This method computes the mean color of `dst`.

    Parameters
    ----------
    graph : RAG
        The graph under consideration.
    src, dst : int
        The vertices in `graph` to be merged.
    """
    graph.node[dst]['total color'] += graph.node[src]['total color']
    graph.node[dst]['pixel count'] += graph.node[src]['pixel count']
    graph.node[dst]['mean color'] = (graph.node[dst]['total color'] /
                                     graph.node[dst]['pixel count'])


def rag(input_img, **kwargs):
    thresh = kwargs.pop("thresh", 20)
    compactness = kwargs.pop("compactness", 10)

    # s_cutoff = kwargs.pop("s_cutoff", 0.25)
    # s_gain = kwargs.pop("s_gain", 10)
    #
    # v_cutoff = kwargs.pop("v_cutoff", 0.25)
    # v_gain = kwargs.pop("v_gain", 10)

    h, w = input_img.shape[0:2]
    labels = skimage.segmentation.slic(input_img, compactness=compactness, n_segments=h * w // 50)

    g = skimage.future.graph.rag_mean_color(input_img, labels)

    labels2 = skimage.future.graph.merge_hierarchical(labels, g, thresh=thresh, rag_copy=False,
                                                      in_place_merge=True,
                                                      merge_func=merge_mean_color,
                                                      weight_func=_weight_mean_color)
    rag_img = skimage.color.label2rgb(labels2, input_img, kind='avg')
    # img2 = cv2.cvtColor(rag_img, cv2.COLOR_RGB2HSV)
    # img2[:, :, 1] = skimage.exposure.adjust_sigmoid(img2[:, :, 1], s_cutoff, s_gain)
    # img2[:, :, 2] = skimage.exposure.adjust_sigmoid(img2[:, :, 2], v_cutoff, v_gain)
    # rag_img = cv2.cvtColor(img2, cv2.COLOR_HSV2RGB)
    return rag_img, labels2


def get_distances_skimage(input_img, kl=2, kp=1, **kwargs):
    lab_COLORS = skimage.color.rgb2lab(COLORS[np.newaxis, :, :])[0]
    lab_img = skimage.color.rgb2lab(input_img)
    dist_array = skimage.color.deltaE_cmc(lab_img[:, :, np.newaxis], lab_COLORS[np.newaxis, np.newaxis, :],
                                          kl, kp)
    return dist_array


def determ_color(color, s_cutoff=0.25, v_cutoff=0.25, s_gain=20, v_gain=20, **kwargs):
    img2 = np.array([[color.astype(np.uint8)]])
    img2 = cv2.cvtColor(img2, cv2.COLOR_RGB2HSV)
    img2[:, :, 1] = skimage.exposure.adjust_sigmoid(img2[:, :, 1], s_cutoff, s_gain)
    img2[:, :, 2] = skimage.exposure.adjust_sigmoid(img2[:, :, 2], v_cutoff, v_gain)
    img2 = cv2.cvtColor(img2, cv2.COLOR_HSV2RGB)
    return np.argmin(get_distances_skimage(np.array(img2))[0, 0])


def find_colors(img, **kwargs):
    img = img_transformation(img)

    img = rag(img, **kwargs)[0]
    full_dist_array = get_distances_skimage(img, **kwargs)
    height, width = img.shape[0] // STEP, img.shape[1] // STEP
    dist_array = np.zeros((height, width, COLORS.shape[0]))
    for i in range(height):
        for j in range(width):
            img_ = full_dist_array[STEP * i:STEP * i + STEP, STEP * j:STEP * j + STEP, :]
            dist_array[i, j, :] = np.mean(np.mean(img_, axis=0), axis=0)

    plan_dist_array = np.zeros((height, width - 4, len(PLANS)))

    for i, plan in enumerate(PLANS):
        plan_dist_array[:, :, i] = sum(
            [np.roll(dist_array[:, :, plan[x]] ** 2, -2 * x, axis=1)[:, :-4] for x in range(3)])

    best_plan = int(np.unravel_index(np.argmin(plan_dist_array), (height, width - 4, len(PLANS)))[2])
    ind = np.unravel_index(np.argmin(plan_dist_array), (height, width - 4, len(PLANS)))[0:2]
    return PLANS[best_plan], ind


def find_colors_geom(img, k1=1, k2=1, kr=1, **kwargs):
    img = img_transformation(img, **kwargs)

    labels = rag(img, **kwargs)[1]
    n = np.max(labels) + 1
    xc = np.zeros(n)
    yc = np.zeros(n)
    xr = np.zeros(n)
    yr = np.zeros(n)

    h, w = labels.shape[0:2]
    x = np.arange(w)
    y = np.arange(h)

    for i in range(n):
        m = np.count_nonzero(labels == i)
        xc[i] = np.sum(x[np.newaxis, :] * (labels == i)) / m
        yc[i] = np.sum(y[:, np.newaxis] * (labels == i)) / m

        xr[i] = (np.sum(((x[np.newaxis, :] - xc[i]) * (labels == i)) ** 2) / m) ** 0.5
        yr[i] = (np.sum(((y[:, np.newaxis] - yc[i]) * (labels == i)) ** 2) / m) ** 0.5

    r = STEP / np.sqrt(3) / kr
    cost_function = np.zeros((n, n, n))
    for i, j, k in itertools.product(range(n), range(n), range(n)):
        if (i != j) and (j != k) and (k != i) and (xc[k] >= xc[j]) and (xc[j] >= xc[i]):
            cost_function[i, j, k] = \
                k1 * ((xr[i] - r) ** 2 + (yr[i] - r) ** 2 + (xr[j] - r) ** 2 + (yr[j] - r) ** 2 + (xr[k] - r) ** 2 + (
                        yr[k] - r) ** 2) + \
                k2 * ((yc[i] - yc[j]) ** 2 + (xc[j] - xc[i] - STEP * 2) ** 2 + (yc[j] - yc[k]) ** 2 + (
                        xc[k] - xc[j] - STEP * 2) ** 2 + (yc[k] - yc[i]) ** 2 + (xc[k] - xc[i] - STEP * 4) ** 2)
        else:
            cost_function[i, j, k] = np.inf

    ind = np.unravel_index(np.argmin(cost_function), (n, n, n))

    # print(xc[list(ind)])
    # print(yc[list(ind)])
    best_plan = [0, 0, 0]
    colors = np.zeros((3, 3), dtype=np.uint8)
    for j, i in enumerate(ind):
        m = np.count_nonzero(labels == i)
        colors[j] = np.sum(np.sum(img.astype(np.float32) * (labels == i)[:, :, np.newaxis], axis=0), axis=0) / m
        best_plan[j] = determ_color(colors[j], **kwargs)

    return best_plan, colors, [xc[list(ind)], yc[list(ind)]]
