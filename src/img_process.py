# -- coding: utf-8 --

'''
图像处理的一些函数。输出图像和卷积核，返回计算得到的img。
返回的img里只有255和0两种元素
'''

import copy
import cv2
import numpy as np


def dilate(img_param, kernel):
    """
    膨胀操作，返回的img里只有255和0两种元素
    :param img_param: numpy.array类型
    :param kernel: numpy.array类型，支持np.ones生成的矩阵，其他矩阵可能不支持，未测试
    :return:np.array
    """
    img = copy.deepcopy(img_param)
    new_img = copy.deepcopy(img)
    h, w = img.shape
    pad_width = len(kernel) // 2
    img = np.pad(img, (pad_width, pad_width), 'edge')
    for i in range(0, h):
        for j in range(0, w):
            new_img[i, j] = 255 if np.any(kernel * img[i:i + 2 * pad_width + 1, j:j + 2 * pad_width + 1]) else 0
    return new_img


def erode(img_param, kernel):
    """
    腐蚀操作，返回的img里只有255和0两种元素
    :param img: np.array
    :param kernel: np.ones
    :return: np.array
    """
    img = copy.deepcopy(img_param)
    new_img = copy.deepcopy(img)

    h, w = img.shape
    pad_width = len(kernel) // 2

    img = np.pad(img, (pad_width, pad_width), 'edge')
    for i in range(0, h):
        for j in range(0, w):
            new_img[i, j] = 255 if np.all((kernel * img[i:i + 2 * pad_width + 1, j:j + 2 * pad_width + 1])) else 0

    return new_img


def morph_open(img, kernel):
    """
    开运算。先腐蚀后膨胀。返回的img里只有255和0两种元素
    :param img: np.array
    :param kernel: np.ones
    :return: np.array
    """
    return dilate(erode(img, kernel), kernel)


def morph_close(img, kernel):
    """
    闭运算。先膨胀后腐蚀。返回的img里只有255和0两种元素
    :param img: np.array
    :param kernel: np.ones
    :return: np.array
    """
    return erode(dilate(img, kernel), kernel)


def morph_top_hat(img, kernel):
    """
    高帽运算。原始img - 开运算得到的img。返回的img里只有255和0两种元素
    :param img: np.array
    :param kernel: np.ones
    :return: np.array
    """
    np.putmask(img, img > 0, 255)  # 将img大于0的值都设置为255
    ret_img = img - morph_open(img, kernel)
    np.putmask(ret_img, ret_img < 0, 0)
    return ret_img


def morph_black_hat(img, kernel):
    """
    黑帽运算。原始img - 闭运算得到的img。返回的img里只有255和0两种元素
    :param img: np.array
    :param kernel: np.ones
    :return: np.array
    """
    np.putmask(img, img > 0, 255)  # 将img大于0的值都设置为255
    ret_img = img - morph_close(img, kernel)
    np.putmask(ret_img, ret_img < 0, 0)
    return ret_img


def morph_top_hat_reverse(img, kernel):
    """
    反高帽运算。开运算得到的img - 原始img。返回的img里只有255和0两种元素
    :param img: np.array
    :param kernel: np.ones
    :return: np.array
    """
    np.putmask(img, img > 0, 255)  # 将img大于0的值都设置为255
    ret_img = -img + morph_open(img, kernel)
    np.putmask(ret_img, ret_img < 0, 0)
    return ret_img


def morph_black_hat_reverse(img, kernel):
    """
    反黑帽运算。闭运算得到的img - 原始img。返回的img里只有255和0两种元素
    :param img: np.array
    :param kernel: np.ones
    :return:
    """
    np.putmask(img, img > 0, 255)  # 将img大于0的值都设置为255
    ret_img = -img + morph_close(img, kernel)
    np.putmask(ret_img, ret_img < 0, 0)
    return ret_img

if __name__ == '__main__':
    from workmap import Workmap
    map_gray = [[0] * 50 for _ in range(50)]
    import matplotlib.pyplot as plt

    map_gray[1] = [100] * 50
    map_gray[2] = [50] * 50

    plt.imshow(map_gray)
    plt.show(block=False)

    work_map = Workmap(debug=True)
    work_map.read_map_directly("../maps/1.txt")
    img = work_map.map_gray
    img = np.array(img).astype('uint8')

    kernel = np.ones((5, 5), np.uint8)
    plt.subplot(3, 5, 1)
    plt.imshow(img, cmap=plt.cm.gray)
    plt.title("img")

    erode_img = erode(img,kernel)
    plt.subplot(3, 5,2)

    plt.imshow(erode_img, cmap=plt.cm.gray)
    plt.title("erode_img_my")

    diate_img = dilate(img, kernel)
    plt.subplot(3, 5, 3)

    plt.imshow(diate_img, cmap=plt.cm.gray)
    plt.title("diate_img_my")


    erode_res = cv2.erode(img,kernel)
    plt.subplot(3, 5, 4)
    plt.imshow(erode_res, cmap=plt.cm.gray)
    plt.title("erode_cv2")
    dilate_res = cv2.dilate(img,kernel)
    plt.subplot(3, 5, 5)
    plt.imshow(dilate_res, cmap=plt.cm.gray)
    plt.title("dilate_cv2")

    opening = morph_open(img, kernel)
    closing = morph_close(img,kernel)
    plt.subplot(3, 5, 7)
    plt.imshow(closing, cmap=plt.cm.gray)
    plt.title("closing_my")
    plt.subplot(3, 5, 8)
    plt.imshow(opening, cmap=plt.cm.gray)
    plt.title("opening_my")

    opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    plt.subplot(3, 5, 9)
    plt.imshow(closing, cmap=plt.cm.gray)
    plt.title("closing_cv2")
    plt.subplot(3, 5, 10)
    plt.imshow(opening, cmap=plt.cm.gray)
    plt.title("opening_cv2")

    # top_hat = cv2.morphologyEx(img, cv2.MORPH_TOPHAT, kernel)
    # plt.subplot(3, 5, 11)
    # plt.imshow(top_hat, cmap=plt.cm.gray)
    # plt.title("image-opening")

    # black_hat = cv2.morphologyEx(img, cv2.MORPH_BLACKHAT, kernel)
    # plt.subplot(3, 5, 12)
    # plt.imshow(black_hat, cmap=plt.cm.gray)
    # plt.title("image-closing")
    # top_hat = morph_top_hat(img, kernel)
    # plt.subplot(3, 5, 13)
    # plt.imshow(top_hat, cmap=plt.cm.gray)
    # plt.title("image-opening")
    # top_hat = morph_black_hat(img, kernel)
    # plt.subplot(3, 5, 14)
    # plt.imshow(top_hat, cmap=plt.cm.gray)
    # plt.title("image-closing")
    plt.subplot(3, 5, 12)
    plt.imshow(closing-img, cmap=plt.cm.gray)
    plt.title("closing-img_cv2")
    plt.subplot(3, 5, 11)
    plt.imshow(opening-img, cmap=plt.cm.gray)
    plt.title("opening-img_cv2")
    top_hat_reverse = morph_top_hat_reverse(img,kernel)
    plt.subplot(3, 5, 13)
    plt.imshow(top_hat_reverse, cmap=plt.cm.gray)
    plt.title("opening-image_my")
    black_hat_reverse = morph_black_hat_reverse(img,kernel)
    plt.subplot(3, 5, 14)
    plt.imshow(black_hat_reverse, cmap=plt.cm.gray)
    plt.title("closing-image_my")

    plt.show()

    # opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    # closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    # plt.subplot(1, 7, 2)
    # plt.imshow(closing, cmap=plt.cm.gray)
    # plt.title("closing")
    # plt.subplot(1, 7, 3)
    # plt.imshow(opening, cmap=plt.cm.gray)
    # plt.title("opening")
    #

    # plt.show()