
import scipy.io
import numpy as np
import numpy_opencv
import cv2

from path import Path

#fname = Path('~/odroid_shopping.jpg').expand()
base_dir = Path('/mnt/mx100/places205_batches')
mean_img_fname = base_dir/('places_mean.mat')
mean_img = scipy.io.loadmat(mean_img_fname)['image_mean']/255.
mean_img = np.ascontiguousarray(mean_img)
print mean_img.shape, mean_img.dtype

#img = cv2.imread(fname)
#print img.shape, img.dtype

numpy_opencv.libnumpy_opencv.rgb_float(mean_img)

