import pyfastnoisesimd as fns
import numpy as np
import cv2
import math

def make_gradient_v2(width, height, h, k, a, b, theta):
    # Precalculate constants
    st, ct =  math.sin(theta), math.cos(theta)
    aa, bb = a**2, b**2
        
    # Generate (x,y) coordinate arrays
    y,x = np.mgrid[-k:height-k,-h:width-h]
    # Calculate the weight for each pixel
    weights = (((x * ct + y * st) ** 2) / aa) + (((x * st - y * ct) ** 2) / bb)

    return np.clip(1.0 - weights, 0, 1)


def main():

    image = cv2.imread('hmap1.png', cv2.IMREAD_GRAYSCALE)

    image = image[:750,:750]

    # print(np.shape(image))

    randlist = []

    shape_size = 120

    for i in range(0, int((np.shape(image)[0]*np.shape(image)[0])/(shape_size*shape_size))):

        start = np.random.randint(0, len(image)-shape_size-1)
        end = start + shape_size

        randlist.append(image[start:start+shape_size, start:start+shape_size])

    new_image = np.zeros((len(randlist)*shape_size, shape_size))

    for i in range(0, len(randlist)):
        j = i*shape_size
        new_image[j:j+shape_size, 0:shape_size] = randlist[i]

    new_image = np.reshape(new_image, np.shape(image))

    # hill = np.uint8(make_gradient_v2(1080,1080, np.random.randint(10, 1000), np.random.randint(10, 1000), np.random.randint(50, 300), np.random.randint(50, 300), 0) * 150)

    cv2.imwrite('hmap1.png', new_image)

if __name__ == '__main__':
    main()