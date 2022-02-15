import os
import numpy as np
import raisimpy as raisim
import math
import time
import cv2
import pyfastnoisesimd as fns

## Launching Unity OpenGL
# os.system('/home/vdorbala/IROS_2022/raisimlib/raisimUnity/linux/raisimUnity.x86_64')

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

    terrainProperties = raisim.TerrainProperties()

    terrainProperties.frequency = np.random.uniform(0.1, 0.7)
    print("Bumpiness factor is {}".format(terrainProperties.frequency))
    terrainProperties.zScale = 3.0
    terrainProperties.xSize = terrain_x
    terrainProperties.ySize = terrain_y
    terrainProperties.xSamples = 100
    terrainProperties.ySamples = 100
    terrainProperties.fractalOctaves = 3
    terrainProperties.fractalLacunarity = 2.0
    terrainProperties.fractalGain = 0.25

    raisim.World.setLicenseFile("/home/vdorbala/.raisim/activation.raisim")
    world = raisim.World()
    ground = world.addGround()

    # launch raisim server
    server = raisim.RaisimServer(world)
    server.launchServer(8080)

    # Creating random hill files
    for file in os.listdir("./hills/"):
        os.remove("./hills/" + file)

    for i in range(HILL_NUM):
        hill = np.uint8(make_gradient_v2(1080, 1080, np.random.randint(10, 1000), np.random.randint(10, 1000), np.random.randint(50, 150), np.random.randint(50, 150), math.radians(np.random.randint(0, 90)))*np.random.randint(220, 255))
        cv2.imwrite('./hills/hill_{}.png'.format(i), hill)

    # Add Objects
    actSphere = world.addSphere(1, 4, collision_group=1, collision_mask=1)
    # visSphere = server.addVisualSphere("v_sphere", 1, 1, 1, 1, 1)
    actSphere.setPosition(np.array([2, 0, 2]))

    # for file in os.listdir("./hills/"):
        # height_map = world.addHeightMap("./hills/" + file, 0, 0, terrain_x, terrain_y, 0.15, 0)

    height_map = world.addHeightMap("hmap2.png", 0, 0, terrain_x, terrain_y, -0.15, 0)
    height_map = world.addHeightMap(0.0, 0.0, terrainProperties)

    for i in range(500000):
        time.sleep(world.getTimeStep())

    server.killServer()

if __name__ == '__main__':
    
    terrain_x = 300.0
    terrain_y = 300.0

    hill_scale_factor = 0.2

    HILL_NUM = np.random.randint(1, 10)

    print("Generating {} hills".format(HILL_NUM))

    main()