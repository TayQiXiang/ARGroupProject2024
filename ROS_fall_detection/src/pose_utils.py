import torch
import numpy as np
import cv2


def to_torch(ndarray):
    if type(ndarray).__module__ == 'numpy':
        return torch.from_numpy(ndarray)
    elif not torch.is_tensor(ndarray):
        raise ValueError("Cannot convert {} to torch tensor"
                         .format(type(ndarray)))
    return ndarray


def im_to_torch(img):
    img = np.transpose(img, (2, 0, 1))
    img = to_torch(img).float()
    if img.max() > 1:
        img /= 255
    return img


def color_normalize(x, mean):
    if x.size(0) == 1:
        x = x.repeat(3, 1, 1)
    normalized_mean = mean / 255
    for t, m in zip(x, normalized_mean):
        t.sub_(m)
    return x


def Cropmyimage(img, bbox, bbox_num=1):
    height, width = 256, 192
    bbox = np.array(bbox).reshape(4, ).astype(np.float32) 
    add = max(img.shape[0], img.shape[1])
    mean_value = np.array([122.7717, 115.9465, 102.9801])  # RGB
    bimg = cv2.copyMakeBorder(img, add, add, add, add, borderType=cv2.BORDER_CONSTANT, value=mean_value.tolist())
    objcenter = np.array([(bbox[0] + bbox[2]) / 2., (bbox[1] + bbox[3]) / 2.])
    bbox += add
    objcenter += add

    crop_width = bbox[2] - bbox[0]
    crop_height = bbox[3] - bbox[1]

    # crop_width = (bbox[2] - bbox[0]) * (1 + 0.1 * 2)
    # crop_height = (bbox[3] - bbox[1]) * (1 + 0.15 * 2)
    if crop_height / height > crop_width / width:
        crop_size = crop_height
        min_shape = height
    else:
        crop_size = crop_width
        min_shape = width


    crop_size = min(crop_size, objcenter[0] / width * min_shape * 2. - 1.)  
    crop_size = min(crop_size, (bimg.shape[1] - objcenter[0]) / width * min_shape * 2. - 1)
    crop_size = min(crop_size, objcenter[1] / height * min_shape * 2. - 1.)
    crop_size = min(crop_size, (bimg.shape[0] - objcenter[1]) / height * min_shape * 2. - 1)

    min_x = int(objcenter[0] - crop_size / 2. / min_shape * width)  
    max_x = int(objcenter[0] + crop_size / 2. / min_shape * width)  
    min_y = int(objcenter[1] - crop_size / 2. / min_shape * height)
    max_y = int(objcenter[1] + crop_size / 2. / min_shape * height)

    # cv2.imwrite('/home/chen/5.jpg', bimg[min_y:max_y, min_x:max_x, :])
    img = cv2.resize(bimg[min_y:max_y, min_x:max_x, :], (width, height))  
    details = np.asarray([min_x - add, min_y - add, max_x - add, max_y - add]).astype(np.float)



    img = im_to_torch(img)
    img = color_normalize(img, mean_value)  

    return img, details


def Drawkeypoints(img, result, bbox_num=1):  
    i = 0
    while i <= 49:
        x = int(result[i])
        y = int(result[i + 1])
        img[y - 3:y + 3, x - 3:x + 3, 0] = 0
        img[y - 3:y + 3, x - 3:x + 3, 1] = 0
        img[y - 3:y + 3, x - 3:x + 3, 2] = 255
        i += 3

