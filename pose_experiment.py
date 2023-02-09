import blenderproc as bproc
import argparse
import numpy as np
import bpy
import time

from pathlib import Path
from tqdm import tqdm

parser = argparse.ArgumentParser()
parser.add_argument('tag_board', default="./CAD_model/tagboard_21x21x1cm.obj",help="Path to the object file containing the bin, should be examples/advanced/physics_convex_decomposition/bin.obj.")
parser.add_argument('output_dir', nargs='?', default="./output", help="Path to where the final files will be saved ")
args = parser.parse_args()
bproc.init()

def pipeline_init():
    ###############################################################
    # Set the camera pose same as world frame, located in origin
    ###############################################################
    cam_k = np.array([[21627.734375, 0, 2353.100109], 
                      [0, 21643.369141, 1917.666411],
                      [0, 0, 1]])
    # camera.camera.set_resolution(512, 512)
    W, H = int(5472), int(3648)
    bproc.camera.set_resolution(W, H)
    bproc.camera.set_intrinsics_from_K_matrix(cam_k, W, H)
    bproc.camera.add_camera_pose(np.eye(4))
    cam_pose = bproc.camera.get_camera_pose(frame=None)
    cam_R = cam_pose[0:3, 0:3]

    theta = np.random.uniform(0., 0.1*np.pi)

    random_R = np.array([
        [np.cos(theta),  np.sin(theta),     0.0],
        [0.0,            1.0,               0.0],
        [-np.sin(theta), 0.0,               np.cos(theta)],
    ])

    ###############################################################
    # Define a point light
    ###############################################################
    light = bproc.types.Light()
    light.set_type("POINT")
    light.set_location([-0.5, 0.1, -0.6])
    light.set_rotation_mat(cam_R*random_R)
    light.set_color([1, 1, 1])
    light.set_energy(100)

    ###############################################################
    # Load tag_board which is going to catch the usb objects
    ###############################################################
    tag_baord = bproc.loader.load_obj(args.tag_board)[0]
    tag_baord.set_scale([1, 1, 1])
    tag_baord.set_location(np.array([0, 0, -1.8]))
    # tag_baord.set_rotation_euler(np.array([-np.pi/2, 0, 0]))
    tag_baord.set_rotation_euler(np.array([-np.pi/2, np.pi, 0]))
    # Make the tagboard object passively participate in the physics simulation
    tag_baord.enable_rigidbody(active=False, collision_shape="CONVEX_HULL", mass=1)

###############################################################
# Load usb objects
###############################################################
# Define a function that samples the pose of a given usb object
def sample_pose(obj: bproc.types.MeshObject):
    # Sample the location above the tagboard
    obj.set_scale([1, 1, 1])
    obj.set_location(np.random.uniform([-0.04, -0.04, -1.76], [0.04, 0.04, -1.75]))
    obj.set_rotation_euler(bproc.sampler.uniformSO3())


## Save final pose for three parts seperately, 
## which means every round only simulates with one type part
parts = ['mainshell', 'topshell', 'insert_mold']

# For each part(mainshell, topshell, or insert_mold)
# The number of samples = part_num * iter
part_num = 30
iter = 500

# for obj in Path("./CAD_model/models").rglob('*.obj'):
for obj in Path("./CAD_model/UT1113").rglob('*.obj'):
    if 'background' in obj.name:
        continue

    Rot_mat = []
    Z_offset = []
    catogory = obj.name[:-4]

    ## mainshell or topshell or insert_mol
    ## each scene contains 10 parts
    for i in tqdm(range(iter)):
        pipeline_init()
        obj_queue = []
    
        for _ in range(part_num):
            obj_queue.append(bproc.loader.load_obj(str(obj)).pop())
    
        # Sample the poses of all usb objects, while making sure that no objects collide with each other.
        bproc.object.sample_poses(
            obj_queue,
            sample_pose_func=sample_pose
        )

        ###############################################################
        # Physical simulation settings
        ###############################################################
        for part in obj_queue:
            part.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", mass=0.1)

        # Simulation time
        bproc.object.simulate_physics_and_fix_final_poses(
        min_simulation_time=0.2,
        max_simulation_time=10,
        check_object_interval=1
        )

        for part in obj_queue:
            part_pose = part.get_local2world_mat()
            Rot_mat.append(part_pose[0:3, 0:3])

        bproc.clean_up(clean_up_camera=True)

    Rot_mat = np.array(Rot_mat)
    print('#'*80)
    print(f"The Rot_mat of {catogory} is \n", Rot_mat )
    np.savez_compressed('./pose_exp/'+catogory, Rotation = Rot_mat)