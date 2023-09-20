import blenderproc as bproc
import argparse
import numpy as np
import bpy
import time
import random
import os

from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('output_dir', nargs='?', default="./output/", help="Path to where the final files will be saved ")
args = parser.parse_args()
bproc.init()
Physics = True

###############################################################
# Load tag_board which is going to catch the usb objects
###############################################################
tag_board = bproc.loader.load_obj('./CAD_model/tagboard_21x21x1cm.obj')[0]
tag_board.set_scale([1, 0.35, 1])
tag_board.set_location(np.array([0, 0, 0.03]))
tag_board.set_cp("category_id", 0)
tag_board.set_name("tagboard")
tag_board.set_cp("supercategory", "background")

# tag_board2 = bproc.loader.load_obj('./CAD_model/tagboard_21x21x1cm.obj')[0]
# tag_board2.set_scale([1, 0.35, 1])
# tag_board2.set_location(np.array([0.3, 0, 0.03]))
# tag_board2.set_cp("category_id", 0)
# tag_board2.set_name("tagboard2")
# tag_board2.set_cp("supercategory", "background")

# tag_board3 = bproc.loader.load_obj('./CAD_model/tagboard_21x21x1cm.obj')[0]
# tag_board3.set_scale([1, 0.35, 1])
# tag_board3.set_location(np.array([-0.3, 0, 0.03]))
# tag_board3.set_cp("category_id", 0)
# tag_board3.set_name("tagboard2")
# tag_board3.set_cp("supercategory", "background")

bread_board = bproc.loader.load_obj('./CAD_model/breadboard.obj')[0]
bread_board.set_scale([1, 1, 1])
bread_board.set_location(np.array([0, 0, 0.01]))
# bread_board.set_rotation_euler(np.array([-np.pi/2, np.pi, np.random.uniform(0, 2*np.pi, 1).item()]))
bread_board.set_rotation_euler(np.array([-np.pi/2, np.pi, 0]))
bread_board.set_cp("category_id", 0)
bread_board.set_name("bread_board")

###############################################################
# Load other distractive objects
###############################################################
# obj_queue_other = []
obj_queue = []
for obj in Path("./CAD_model/models_other").rglob('*.obj'):
    # for _ in range(5): # duplicate each object for 5 times
    obj_queue.append(bproc.loader.load_obj(str(obj)).pop()          )
    part = obj_queue[-1]
    part.set_cp("category_id", 0)
    part.set_name("other"+str(obj))
    part.set_scale([1, 1, 1])
    part.set_cp("supercategory", "background")
    # part.set_location([random.uniform(-0.1, 0.1),random.uniform(-0.1, 0.1), 0.03])
###############################################################
# Define a random point light
###############################################################
light1 = bproc.types.Light()
light1.set_type("POINT")
light1.set_location([random.uniform(-1, 0),random.uniform(-1, 0), random.uniform(1, 4)])
light1.set_energy(np.random.uniform(20,300,1).item())

light2 = bproc.types.Light()
light2.set_type("POINT")
light2.set_location([random.uniform(0, 1),random.uniform(0, 1), random.uniform(1, 4)])
light2.set_energy(np.random.uniform(20,300,1).item())

###############################################################
# Load usb objects
###############################################################
# images = list(Path('./background').rglob('*.png')) +  list(Path('./background').rglob('*.jpg'))
images = list(Path('/data/ham/BlenderProc2/polyhaven_bgs').rglob('*.png')) +  list(Path('/data/ham/BlenderProc2/polyhaven_bgs').rglob('*.jpg'))

parts = ['mainshell', 'topshell', 'insert_mold']
for obj in Path("./CAD_model/models_single").rglob('*.obj'):
    if 'background' in obj.name:
        # print(f'Skipping loading part {obj.name}')

        background = bproc.loader.load_obj(str(obj)).pop()
        background.set_cp("category_id", 0)
        background.set_cp("supercategory", "background")
        
        pose = np.eye(4)
        pose[:, 3] = np.array([0, 0, 0, 1]).T
        background.set_scale([1, 1, 1])
        background.set_local2world_mat(pose)
        background.enable_rigidbody(active=False, collision_shape="CONVEX_HULL", collision_margin=0.0001)
        image = bpy.data.images.load(filepath=str(random.choice(images)))

        for m in background.get_materials():
            m.set_principled_shader_value('Base Color', image)
        # bproc.loader.get_random_world_background_hdr_img_path_from_haven(data_path_hdr)

        continue

    if 'gripper' in obj.name:
        continue
        # gipper = background = bproc.loader.load_obj(str(obj)).pop()
        # bproc.loader.load_obj(str(obj)).pop()
        # background.set_cp("category_id", 0)
        # background.set_cp("supercategory", "background")
        
        
    
    # TODO(yangfei): should assign each part with unique id?  e.g. mainshell_00
    for _ in range(5):
        obj_queue.append(bproc.loader.load_obj(str(obj)).pop())
        part = obj_queue[-1]

        print(obj.name)
        # remove '.obj'
        idx = parts.index(obj.name[:-4])
        part.set_cp("category_id", idx + 1)
        part.set_name(parts[idx])
        part.set_cp("supercategory", "usb")
        
        print('category_id =', part.get_cp("category_id"))
        print('Part Name =', part.get_name())


###############################################################
# Set the initial poses of objects randomly
###############################################################
# Define a function that samples the pose of a given usb object
def sample_pose(obj: bproc.types.MeshObject):
    # Sample the location above the tagboard
    obj.set_scale([1, 1, 1])
    # obj.set_location(np.random.uniform([-0.04, -0.04, 0.015], [0.04, 0.04, 0.025]))
    obj.set_location(np.random.uniform([-0.035, -0.035, 0.015+0.06], [0.035, 0.035, 0.025+0.055]))
    obj.set_rotation_euler(bproc.sampler.uniformSO3())

# Sample the poses of all usb objects, while making sure that no objects collide with each other.
bproc.object.sample_poses(
    obj_queue,
    sample_pose_func=sample_pose
)


###############################################################
# Physical simulation settings
###############################################################
if Physics:
    # Make the tagboard object passively participate in the physics simulation
    tag_board.enable_rigidbody(active=False, collision_shape="CONVEX_HULL", mass = 0.17, collision_margin=0.0006)
    # tag_board2.enable_rigidbody(active=False, collision_shape="CONVEX_HULL", mass = 0.17, collision_margin=0.0006)
    # tag_board3.enable_rigidbody(active=False, collision_shape="CONVEX_HULL", mass = 0.17, collision_margin=0.0006)
    bread_board.enable_rigidbody(active=False, collision_shape="CONVEX_HULL", mass = 50, collision_margin=0.0006)

    for part in obj_queue:
        part.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", mass=0.01, collision_margin=0.0006)

    # for part in obj_queue_other:
    #     part.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", mass=0.01, collision_margin=0.0006)

    bproc.object.simulate_physics_and_fix_final_poses(
    min_simulation_time=1,
    max_simulation_time=5,
    check_object_interval=1
    )
  


###############################################################
# Set the camera pose same as world frame, located in origin
###############################################################
################## 0 Basler  ################## chnaged camera 
W, H = int(854), int(480)
# W, H = int(2208), int(1242) 

cam_k_Basler = np.array([[10704.062350, 0, W//2], 
                    [0, 10727.438047, H//2],
                    [0, 0, 1]])

# location = [random.uniform(-0.18,0.18), random.uniform(-0.18, 0.18), random.uniform(0,0.35)]
location = [random.uniform(-0.18,0.18), random.uniform(-0.18, 0.18), random.uniform(1.3,2)]
poi = bproc.object.compute_poi(obj_queue)
rotation_matrix = bproc.camera.rotation_from_forward_vec(poi - location, inplane_rot=0)
pose_Basler = bproc.math.build_transformation_mat(location, rotation_matrix)

bproc.camera.set_resolution(W, H)
bproc.camera.set_intrinsics_from_K_matrix(cam_k_Basler, W, H)
bproc.camera.add_camera_pose(pose_Basler)

# import pdb;pdb.set_trace()

###############################################################
# render the whole pipeline and save them as COCO format
###############################################################
bproc.renderer.set_max_amount_of_samples(50)
bproc.renderer.set_noise_threshold(1)
bproc.renderer.set_cpu_threads(0)
# activate normal rendering
bproc.renderer.enable_normals_output()
bproc.renderer.enable_segmentation_output(map_by=["instance", "class", "name"])
data = bproc.renderer.render()
# seg_data = bproc.renderer.render_segmap(map_by=["instance", "class", "name"])

time_start = time.time()
# Write data to coco file
# bproc.writer.write_coco_annotations(os.path.join(args.output_dir, 'coco_data'),
bproc.writer.write_coco_annotations(os.path.join(args.output_dir),
                                    instance_segmaps=data["instance_segmaps"],
                                    instance_attribute_maps=data["instance_attribute_maps"],
                                    colors=data["colors"],
                                    mask_encoding_format='polygon',
                                    color_file_format="PNG", 
                                    append_to_existing_output=True)

print(f"Seg save time: {time.time() - time_start}")
# print(f"Generate a single image time: {time.time() - time_start}")
print()