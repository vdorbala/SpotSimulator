import os
import numpy as np
import raisimpy as raisim
import math
import time
import cv2
import pyfastnoisesimd as fns
from scipy.spatial import distance

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

def generate_env(friction, bumpiness, world_size, gen_hills=False, gen_obj=False):

    terrain_x, terrain_y = world_size

    pos_x = int(terrain_x/2)
    pos_y = int(terrain_y/2)

    terrainProperties = raisim.TerrainProperties()
    terrainProperties.frequency = bumpiness
    print("Bumpiness factor is {}".format(terrainProperties.frequency))
    terrainProperties.zScale = 1.0
    terrainProperties.xSize = terrain_x
    terrainProperties.ySize = terrain_y
    terrainProperties.xSamples = 200
    terrainProperties.ySamples = 200
    terrainProperties.fractalOctaves = 3
    terrainProperties.fractalLacunarity = 2.0
    terrainProperties.fractalGain = 0.5


    raisim.World.setLicenseFile("/home/vdorbala/.raisim/activation.raisim")
    world = raisim.World()
    ground = world.addGround()

    world.setDefaultMaterial(friction, 0.5, 0.5)

    # launch raisim server
    server = raisim.RaisimServer(world)
    server.launchServer(8080)


    # Add Objects
    if gen_obj == True:
        circount = 0
        poslist = []
        for i in range(OBJ_NUM):
            shape1 = shapelist[np.random.randint(0, len(shapelist))]
            shape2 = shapelist[np.random.randint(0, len(shapelist))]

            while (shape2 == shape1):
                shape2 = shapelist[np.random.randint(0, len(shapelist) - 1)]

            # Keeping track of circular shapes
            if (shape1 in ['sphere', 'capsule', 'cylinder']):
                circount += 1
            if  (shape2 in ['sphere', 'capsule', 'cylinder']):
                circount += 1

            print(shape1, shape2)

            # Setting flat shape if too many circular shapes
            if circount > (OBJ_NUM):
                shape1 = shapelist[3]
                # shape2 = shapelist[3]

            x1 = np.random.randint(-pos_x, pos_x)
            y1 = np.random.randint(-pos_y, pos_y)

            pos1 = (x1, y1)

            for pos in poslist:
                while distance.euclidean(pos1, pos) < MIN_INTEROBJ_DIST:
                    x1 = np.random.randint(-pos_x, pos_x)
                    y1 = np.random.randint(-pos_y, pos_y)
                    pos1 = (x1, y1)

            x2 = x1 + np.random.randint(-5,5)
            y2 = y1 + np.random.randint(-5,5)

            x = x1
            y = y1
            j = 0

            poslist.append((x1,y1))
            poslist.append((x2,y2))

            # for shape in {shape1, shape2}:
            #     j = j + 1
            #     if shape == 'sphere':
            #         actSphere = server.addVisualSphere(name = 'sph_{}{}'.format(i,j), radius = np.random.uniform(MIN_RADIUS, MAX_RADIUS))
            #         # server.setBodyType(STATIC')
            #         actSphere.setPosition(np.array([x, y, 5]))
            #     elif shape == 'capsule':
            #         actCapsule = server.addVisualCapsule(name = 'cap_{}{}'.format(i,j), length = np.random.uniform(MIN_HEIGHT, MAX_HEIGHT), radius = np.random.uniform(MIN_RADIUS, MAX_RADIUS))
            #         actCapsule.setPosition(np.array([x, y, 5]))
            #     elif shape == 'cylinder':
            #         actCylinder = server.addVisualCylinder(name = 'cyl_{}{}'.format(i,j), length = np.random.uniform(MIN_HEIGHT, MAX_HEIGHT), radius = np.random.uniform(MIN_RADIUS, MAX_RADIUS))
            #         actCylinder.setPosition(np.array([x, y, 5]))
            #     else:
            #         actBox = server.addVisualBox(np.random.uniform(0.5, 10), np.random.uniform(1, 10), np.random.uniform(0.5, 10))
            #         actBox.setPosition(np.array([x, y, 100]))
                
            #     x = x2 
            #     y = y2

            for shape in {shape1, shape2}:
                j = j + 1
                if shape == 'sphere':
                    radius = np.random.uniform(MIN_RADIUS, MAX_RADIUS)
                    actSphere = world.addSphere(radius = radius, mass = np.random.randint(MIN_MASS, MAX_MASS), collision_group=1, collision_mask=1)
                    actSphere.setBodyType(raisim.BodyType(3))
                    actSphere.setPosition(np.array([x, y, 7]))
                    poslist.append((x + abs(radius), y + abs(radius)))
                    poslist.append((x - abs(radius), y - abs(radius)))
                elif shape == 'capsule':
                    radius = np.random.uniform(MIN_RADIUS, MAX_RADIUS)
                    actCapsule = world.addCapsule(height = np.random.uniform(MIN_HEIGHT, MAX_HEIGHT), radius = radius, mass = np.random.randint(MIN_MASS, MAX_MASS), collision_group=1, collision_mask=1)
                    actCapsule.setPosition(np.array([x, y, 7]))
                    actCapsule.setBodyType(raisim.BodyType(3))

                    poslist.append((x + abs(radius), y + abs(radius)))
                    poslist.append((x - abs(radius), y - abs(radius)))

                elif shape == 'cylinder':
                    radius = np.random.uniform(MIN_RADIUS, MAX_RADIUS)
                    actCylinder = world.addCylinder(height = np.random.uniform(MIN_HEIGHT, MAX_HEIGHT), radius = radius, mass = np.random.randint(MIN_MASS, MAX_MASS), collision_group=1, collision_mask=1)
                    actCylinder.setPosition(np.array([x, y, 7]))
                    actCylinder.setBodyType(raisim.BodyType(3))

                    poslist.append((x + abs(radius), y + abs(radius)))
                    poslist.append((x - abs(radius), y - abs(radius)))
                else:
                    xval = np.random.uniform(10, 30)
                    yval = np.random.uniform(10, 30)
                    zval = np.random.uniform(10, 30)
                    actBox = world.addBox(x = xval, y = yval, z = zval, mass = np.random.randint(MIN_MASS, MAX_MASS), collision_group=1, collision_mask=1)
                    actBox.setPosition(np.array([x, y, 7]))
                    actBox.setBodyType(raisim.BodyType(3))

                    poslist.append((x + abs(xval)/2, y + abs(yval)/2))
                    poslist.append((x - abs(xval)/2, y - abs(yval)/2))
                x = x2 
                y = y2

            if shape1 == 'sphere':
                s1 = actSphere
            elif shape1 == 'capsule':
                s1 = actCapsule
            elif shape1 == 'cylinder':
                s1 = actCylinder
            else:
                s1 = actBox

            if shape2 == 'sphere':
                s2 = actSphere
            elif shape2 == 'capsule':
                s2 = actCapsule
            elif shape2 == 'cylinder':
                s2 = actCylinder
            else:
                s2 = actBox
            
            # raisimpy.Compound(children: List[raisimpy.Compound.CompoundObjectChild], center_of_mass: float, mass: numpy.ndarray[numpy.float64], inertia: numpy.ndarray[numpy.float64])
            # compobj = world.addCompound([s1, s2], mass = np.array(np.random.randint(MIN_MASS, MAX_MASS), np.random.randint(MIN_MASS, MAX_MASS)), inertia = np.array(np.random.randint(MIN_MASS, MAX_MASS), np.random.randint(MIN_MASS, MAX_MASS)), center_of_mass=10, collision_group=1, collision_mask=1)
            # obj1 = raisim.SingleBodyObject(s1.getObjectType())
            # obj2 = raisim.SingleBodyObject(s2.getObjectType())
            # compobj = raisim.Compound([obj1, obj2])
            # compobj = world.addCompound(children = list([raisim.Compound()]), mass = 10.0, inertia = np.array([10.0, 10.0]), center_of_mass=np.array([10.0, 10.0]), collision_group=1, collision_mask=1)

    # Add Hills
    if gen_hills == True:
        if (os.path.isfile(os.getcwd()+'/hills/') == False):
            os.mkdir(os.path.isfile(os.getcwd()+'/hills/'))
        # Creating random hill files
        for file in os.listdir("./hills/"):
            os.remove("./hills/" + file)

        for i in range(HILL_NUM):
            hill = np.uint8(make_gradient_v2(200, 200, np.random.randint(10, 1000), np.random.randint(10, 1000), np.random.randint(50, 150), np.random.randint(50, 150), math.radians(np.random.randint(0, 90)))*np.random.randint(220, 255))
            cv2.imwrite('./hills/hill_{}.png'.format(i), hill)

    # height_map = world.addHeightMap("hmap4.png", 0, 0, terrain_x, terrain_y, 0.15, 0)
    height_map = world.addHeightMap(0.0, 0.0, terrainProperties, collision_group=1, collision_mask=1)

    for i in range(500000):
        time.sleep(world.getTimeStep())

    server.killServer()

if __name__ == '__main__':
    
    terrain_x = 200
    terrain_y = 200

    shapelist = ['sphere', 'cylinder', 'capsule', 'box']

    world_size = (terrain_x, terrain_y)
    friction = 0.5

    MAX_HEIGHT = 10
    MAX_RADIUS = 10

    MIN_HEIGHT = 5
    MIN_RADIUS = 5

    MIN_MASS = 10
    MAX_MASS = 30

    MIN_INTEROBJ_DIST = 50

    HILL_NUM = np.random.randint(1, 10)
    OBJ_NUM = 15

    # print("Generating {} hills".format(HILL_NUM))
    bumpiness = np.random.uniform(0.1, 0.8)

    # world_size = (X, Y); bumpiness = 0-0.8; friction
    generate_env(friction, bumpiness, world_size, gen_obj=True)