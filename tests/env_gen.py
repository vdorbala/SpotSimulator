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

# Function to create gradient images for hills
def make_gradient_v2(width, height, h, k, a, b, theta):
    # Precalculate constants
    st, ct =  math.sin(theta), math.cos(theta)
    aa, bb = a**2, b**2
        
    # Generate (x,y) coordinate arrays
    y,x = np.mgrid[-k:height-k,-h:width-h]
    # Calculate the weight for each pixel
    weights = (((x * ct + y * st) ** 2) / aa) + (((x * st - y * ct) ** 2) / bb)

    return np.clip(1.0 - weights, 0, 1)

# Function to generate environment
def generate_env(friction, bumpiness, world_size, gen_hills=False, gen_obj=False):
    terrain_x, terrain_y = world_size

    pos_x = int(terrain_x/2)
    pos_y = int(terrain_y/2)

    terrainProperties = raisim.TerrainProperties()
    terrainProperties.frequency = bumpiness
    print("Bumpiness factor is {}".format(terrainProperties.frequency))
    terrainProperties.zScale = 3.0
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

    height_map = world.addHeightMap(0.0, 0.0, terrainProperties, collision_group=1, collision_mask=1)

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

            poslist.extend([(x1,y1), (x2,y2)])

            for shape in {shape1, shape2}:
                j = j + 1
                if shape == 'sphere':
                    radius = np.random.uniform(MIN_RADIUS, MAX_RADIUS)
                    actSphere = world.addSphere(radius = radius, mass = np.random.randint(MIN_MASS, MAX_MASS), collision_group=1, collision_mask=63)
                    actSphere.setBodyType(raisim.BodyType(3))
                    actSphere.setPosition(np.array([x, y, 7]))

                    poslist.extend(list(zip(*np.where(actSphere>0))))
                    poslist.extend([(x + abs(radius), y + abs(radius)), (x - abs(radius), y - abs(radius))])
                
                elif shape == 'capsule':
                    radius = np.random.uniform(MIN_RADIUS, MAX_RADIUS)
                    actCapsule = world.addCapsule(height = np.random.uniform(MIN_HEIGHT, MAX_HEIGHT), radius = radius, mass = np.random.randint(MIN_MASS, MAX_MASS), collision_group=1, collision_mask=63)
                    actCapsule.setPosition(np.array([x, y, 7]))
                    actCapsule.setBodyType(raisim.BodyType(3))

                    poslist.extend(list(zip(*np.where(actCapsule>0))))
                    poslist.extend([(x + abs(radius), y + abs(radius)), (x - abs(radius), y - abs(radius))])

                elif shape == 'cylinder':
                    radius = np.random.uniform(MIN_RADIUS, MAX_RADIUS)
                    actCylinder = world.addCylinder(height = np.random.uniform(MIN_HEIGHT, MAX_HEIGHT), radius = radius, mass = np.random.randint(MIN_MASS, MAX_MASS), collision_group=1, collision_mask=63)
                    actCylinder.setPosition(np.array([x, y, 7]))
                    actCylinder.setBodyType(raisim.BodyType(3))

                    poslist.extend(list(zip(*np.where(actCylinder>0))))
                    poslist.extend([(x + abs(radius), y + abs(radius)), (x - abs(radius), y - abs(radius))])
                else:
                    xval = np.random.uniform(10, 30)
                    yval = np.random.uniform(10, 30)
                    zval = np.random.uniform(10, 30)
                    actBox = world.addBox(x = xval, y = yval, z = zval, mass = np.random.randint(MIN_MASS, MAX_MASS), collision_group=1, collision_mask=63)
                    actBox.setPosition(np.array([x, y, 7]))
                    actBox.setBodyType(raisim.BodyType(3))

                    poslist.extend(list(zip(*np.where(actBox>0))))
                    poslist.extend([(x + abs(xval)/2, y + abs(yval)/2), (x - abs(xval)/2, y - abs(yval)/2)])
                x = x2 
                y = y2

            ## COMPOUND OBJECT CREATION. DOCUMENTATION IS BAD.

            # if shape1 == 'sphere':
            #     s1 = actSphere
            # elif shape1 == 'capsule':
            #     s1 = actCapsule
            # elif shape1 == 'cylinder':
            #     s1 = actCylinder
            # else:
            #     s1 = actBox

            # if shape2 == 'sphere':
            #     s2 = actSphere
            # elif shape2 == 'capsule':
            #     s2 = actCapsule
            # elif shape2 == 'cylinder':
            #     s2 = actCylinder
            # else:
            #     s2 = actBox

            # raisimpy.Compound(children: List[raisimpy.Compound.CompoundObjectChild], center_of_mass: float, mass: numpy.ndarray[numpy.float64], inertia: numpy.ndarray[numpy.float64])
            # compobj = world.addCompound([s1, s2], mass = np.array(np.random.randint(MIN_MASS, MAX_MASS), np.random.randint(MIN_MASS, MAX_MASS)), inertia = np.array(np.random.randint(MIN_MASS, MAX_MASS), np.random.randint(MIN_MASS, MAX_MASS)), center_of_mass=10, collision_group=1, collision_mask=1)
            # print(dir(raisim.Compound()))
            # obj1 = raisim.SingleBodyObject(s1.getObjectType())
            # obj2 = raisim.SingleBodyObject(s2.getObjectType())
            # compobj = raisim.Compound([obj1, obj2])
            # compobj = world.addCompound(children = list([raisim.Compound()]), mass = 10.0, inertia = np.array([10.0, 10.0]), center_of_mass=np.array([10.0, 10.0]), collision_group=1, collision_mask=1)

    # Add Hills
    if gen_hills == True:
        hillpos = []

        if gen_obj == True:
            hillpos.extend(poslist)

        # if (os.path.isdir(os.getcwd()+'/hills') == False):
        #     os.mkdir(os.getcwd()+'/hills')

        # # Creating random hills
        # for file in os.listdir("./hills/"):
        #     os.remove("./hills/" + file)

        NUM_HILL_TYPES = 1
        cnt_list = [0 for item in range(NUM_HILL_TYPES)]

        for i in range(HILL_NUM):

            # Generating Hills with images
            # hill = np.uint8(make_gradient_v2(200, 200, np.random.randint(10, 1000), np.random.randint(10, 1000), np.random.randint(50, 150), np.random.randint(50, 150), math.radians(np.random.randint(0, 90)))*np.random.randint(220, 255))
            # cv2.imwrite('./hills/hill_{}.png'.format(i), hill)

            height_list = np.zeros((terrain_x, terrain_y))
            
            width = np.random.randint(MIN_HILL_WIDTH, MAX_HILL_WIDTH)
            hill_start_x = np.random.randint(0, terrain_x - width-1)
            hill_start_y = np.random.randint(0, terrain_y - width-1)

            hill_start = (hill_start_x, hill_start_y)

            for pos in hillpos:
                while distance.euclidean(hill_start, pos) < MIN_INTERHILL_DIST:
                    hill_start_x = np.random.randint(0, terrain_x - width-1)
                    hill_start_y = np.random.randint(0, terrain_y - width-1)
                    hill_start = (hill_start_x, hill_start_y)

            htype = np.random.randint(1, NUM_HILL_TYPES+1)

            while cnt_list[htype-1] >= HILL_NUM/NUM_HILL_TYPES:
                choicelist = list(range(1, NUM_HILL_TYPES + 1))
                choicelist.remove(htype)
                htype = np.random.choice(choicelist)
                        
            hill_height = np.random.randint(MIN_HILL_HEIGHT, MAX_HILL_HEIGHT)


            # Hill type - 1
            if htype == 1:
                x_axis = np.linspace(-hill_height, hill_height, int(width))
                y_axis = np.linspace(-hill_height, hill_height, int(width))

                xx, yy = np.meshgrid(x_axis, y_axis)
                hill = -np.sqrt(xx ** 2 + yy ** 2) + MAX_HILL_HEIGHT
                cnt_list[htype-1] += 1

            # Hill type - 2
            elif htype == 2:
                x_axis = np.concatenate((np.linspace(0, hill_height, int(width/2)), np.flip(np.linspace(0, hill_height, int(width/2)))))
                y_axis = np.concatenate((np.linspace(0, hill_height, int(width/2)), np.flip(np.linspace(0, hill_height, int(width/2)))))

                xx, yy = np.meshgrid(x_axis, y_axis)
                hill = np.sqrt(xx ** 2 + yy ** 2)
                cnt_list[htype-1] += 1

            # Hill type - 3
            else:
                x_axis = np.linspace(-hill_height, hill_height, int(width))
                y_axis = np.linspace(-hill_height, hill_height, int(width))

                xx, yy = np.meshgrid(x_axis, y_axis)
                hill = np.sqrt(xx ** 2 + yy ** 2)
                cnt_list[htype-1] += 1

            # Creating a plateau on the hill
            plat_patch = int(np.ceil(PLAT_PERC*width))
            
            h1 = int((width-plat_patch)/2)
            h2 = int((width-plat_patch)/2) + plat_patch
            h3 = int((width-plat_patch)/2)
            h4 = int((width-plat_patch)/2) + plat_patch

            hill[h1:h2, h3:h4] = hill[h1,h3]*np.ones((plat_patch, plat_patch))

            # Removing -ve values
            hill[hill < 0] = 0


            # Adding random noise to the hill texture
            # Also adding it only on some hills
            if htype in [1,2] and cnt_list[0]%2 == 0:
                hill=np.random.normal(2*hill+2, HILL_NOISE)

            # Appending all points in the hill into list to check for distance
            hillpos.extend(list(zip(*np.where(hill>0))))
            hillpos.extend([(hill_start_x, hill_start_y), (hill_start_x + width, hill_start_y), (hill_start_x, hill_start_y + width), (hill_start_x + width, hill_start_y + width)])

            while True:
                try:
                    height_list[hill_start_x: hill_start_x + width, hill_start_y :hill_start_y + width] = hill
                except ValueError:
                    print("Error! Mismatch in width and plateau patch size! Changing width...")
                    if np.shape(height_list[hill_start_x: hill_start_x + width, hill_start_y :hill_start_y + width])[0] > np.shape(hill)[0]:
                        width -= 1
                    else:
                        width += 1
                    # height_list[hill_start_x: hill_start_x + width -1, hill_start_y :hill_start_y + width - 1] = hill
                else:
                    break

            height_map = world.addHeightMap(x_samples = terrain_x, y_samples = terrain_y, x_scale = terrain_x, y_scale = terrain_y, x_center = 0, y_center = 0, heights = list(height_list.ravel()))

            # height_map = world.addHeightMap("./hills/hill_{}.png".format(i), 0, 0, terrain_x, terrain_y, 0.3, 0)

    # print("Number of unique hills were {}".format(cnt_list))

    return world, server

# Function to check if object of a given position and radius can spawn in the current world 
def place_check(pos, radius, world):
    x, y = pos
    
    for obj in world.getObjList():
        if obj.getObjectType() in varlist:
            print(len(varlist))
            position = obj.getPosition()[:2]
            dist = distance.euclidean(pos, position)
            print(dist)
            if dist <= radius:
                return False

    return True

if __name__ == '__main__':
    
    # Size of Terrain
    terrain_x = 200
    terrain_y = 200
    world_size = (terrain_x, terrain_y)

    # Types of Object shapes
    shapelist = ['sphere', 'cylinder', 'capsule', 'box']
    varlist = []
    for shape in shapelist:
        varlist.append(eval("raisim.ObjectType.{}".format(shape.upper())))


    # Friction of surface
    friction = 0.5

    # Maximum and minimum heights of the objects
    MIN_HEIGHT = 5
    MAX_HEIGHT = 10

    # Max and minimum radii of the objects
    MIN_RADIUS = 5
    MAX_RADIUS = 10

    # Max and min mass of the object (DOES NOT MATTER, THEY ARE ALL STATIC = INFINITE MASS, ZERO VELOCITY)
    MIN_MASS = 10
    MAX_MASS = 30

    # Inter object and inter hill distances
    MIN_INTEROBJ_DIST = 60
    MIN_INTERHILL_DIST = 25

    # Perccentage of the hill that should be a plateau on the top (50% means that half of the hill from ground up will be a plateau)
    PLAT_PERC = np.random.uniform(0.1,0.2)

    # Random Gaussian Noise to add to the hill (Changes its appearance)
    HILL_NOISE = 0.5

    # Number of hills, and number of objects
    HILL_NUM = 6
    OBJ_NUM = 10

    # Minimum and maximum width of hills
    MAX_HILL_WIDTH = 51
    MIN_HILL_WIDTH = 50

    # Minimum and maximum height of hills
    MAX_HILL_HEIGHT = 11
    MIN_HILL_HEIGHT = 10
    
    # Bumpiness (frequency) of terrain
    bumpiness = np.random.uniform(0.1, 0.8)

    print("Generating {} hills and {} objects".format(HILL_NUM, OBJ_NUM))

    ## Parameters - world_size = (X, Y); bumpiness = 0-0.8; friction
    ## Set gen_obj for generating objects, and gen_hills for generating hills
    world, server = generate_env(friction, bumpiness, world_size, gen_obj = True, gen_hills = True)

    while True:
        # time.sleep(world.getTimeStep())
        x = np.random.randint(-terrain_x, terrain_x)
        y = np.random.randint(-terrain_y, terrain_y)

        # checkval = place_check((x,y), 60, world)
        # print(checkval)

        break

        if checkval == True:
            anymal = world.addArticulatedSystem("/home/vdorbala/IROS_2022/raisimlib/rsc/anymal/urdf/anymal.urdf")
            anymal.setName("anymal")
            anymal_nominal_joint_config = np.array([0, -1.5, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8,
                                                    -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8])
            anymal.setGeneralizedCoordinate(anymal_nominal_joint_config)
            anymal.setPdGains(200*np.ones([18]), np.ones([18]))
            anymal.setPdTarget(anymal_nominal_joint_config, np.zeros([18]))
            break

        else:
            continue

    print("outside loop")

    for i in range(500000):
        time.sleep(world.getTimeStep())

    server.killServer()
