import blenderproc as bproc
import argparse
import numpy as np
import bpy
import time
import random

from pathlib import Path
parser = argparse.ArgumentParser()
parser.add_argument('cad_path', nargs='?', default="./CAD_model/models_single", help="Path to CAD models")
parser.add_argument('camera_id', nargs='?', default="1", help="Which Camera you would like to use: 0-Beslar 1-Zed_2i 2-Zed_mini")
parser.add_argument('part_name', nargs='?', help="Which part are you generating dataset for: insert_mold, mainshell or topshell.")
parser.add_argument('output_dir', nargs='?', default="./output/dataset", help="Path to where the final files will be saved ")
args = parser.parse_args()

bproc.init()
Physics = True

###############################################################
# Load tag_board which is going to catch the usb objects
###############################################################
tag_board = bproc.loader.load_obj('./CAD_model/tagboard_21x21x1cm.obj')[0]
tag_board.set_scale([1, 0.35, 1])
tag_board.set_location(np.array([0, 0, 0.0001]))
# tag_board.set_rotation_euler(np.array([-np.pi/2, 0, 0]))
tag_board.set_rotation_euler(np.array([-np.pi/2, np.pi, np.random.uniform(0, 2*np.pi, 1).item()]))
tag_board.set_cp("category_id", 0)
tag_board.set_name("tagboard")
# Make the tagboard object passively participate in the physics simulation
tag_board.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", mass = 0.17, collision_margin=0.0006)

###############################################################
# Define a random point light
###############################################################
light = bproc.types.Light()
light.set_type("POINT")
# Sample its location in a shell around the point [0.1, 0.2, -0.6]
light.set_location([0,0, random.uniform(1, 4)])
light.set_energy(np.random.uniform(50,100,1).item())


###############################################################
# Load usb objects
###############################################################
images = list(Path('/data/ham/BlenderProc2/polyhaven_bgs').rglob('*.png')) +  list(Path('/data/ham/BlenderProc2/polyhaven_bgs').rglob('*.jpg'))

parts = ['mainshell', 'topshell', 'insert_mold']
part = args.part_name

obj_queue = []
for obj in Path(args.cad_path).rglob('*.obj'):
    if 'background' in obj.name:
        # print(f'Skipping loading part {obj.name}')

        background = bproc.loader.load_obj(str(obj)).pop()
        background.set_cp("category_id", 0)
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
    
    elif part in obj.name: 
        object = bproc.loader.load_obj(str(obj))[0]
        print(obj.name)
        # remove '.obj'
        idx = parts.index(obj.name[:-4])
        object.set_cp("category_id", idx + 1)
        object.set_name(parts[idx])
        object.set_cp("supercategory", "usb")
        object.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", mass=0.01, collision_margin=0.0006)
        obj_queue.append(object)
        print('category_id =', object.get_cp("category_id"))
        print('Part Name =', object.get_name())


###############################################################
# Set the initial poses of objects randomly
###############################################################
# Define a function that samples the pose of a given usb object
def sample_pose(obj: bproc.types.MeshObject):
    # Sample the location above the tagboard
    obj.set_scale([1, 1, 1])
    # obj.set_location(np.random.uniform([-0.05, -0.05, 0.015], [0.05, 0.05, 0.025]))
    obj.set_location(np.random.uniform([-0.005, -0.005, 0.008], [0.005, 0.005, 0.008]))
    obj.set_rotation_euler(bproc.sampler.uniformSO3())

# Sample the poses of all usb objects, while making sure that no objects collide with each other.
bproc.object.sample_poses(
    obj_queue,
    sample_pose_func=sample_pose
)


###############################################################
# Physical simulation settings
###############################################################
bproc.object.simulate_physics_and_fix_final_poses(
min_simulation_time=1,
max_simulation_time=5,
check_object_interval=1
)
# This will make the renderer render the first 20 frames of the simulation
# bproc.utility.set_keyframe_render_interval(frame_start=0, frame_end=20)


###############################################################
# Set the camera pose same as world frame, located in origin
###############################################################
################## 0 Basler  ##################
cam_k_Basler = np.array([[21971.333024, 0, 2208/2], 
                         [0, 22025.144687, 1242/2],
                         [0, 0, 1]])
# W_Basler, H_Basler = 5472, 3648
pose_Basler = [[1, 0, 0,  0],
                [0, 1, 0, 0],
                # [0, 0, 1, random.uniform(1.2, 1.4)],
                [0, 0, 1, random.uniform(0.4, 0.6)],
                [0, 0, 0, 1]]


# ################## 1 ZED 2i  ##################
# cam_k_2i = np.array([[1908.56, 0,       1113.88], 
#                   [0,       1909.06, 588.34],
#                   [0,       0,       1]])
cam_k_2i = np.array([[1908.56, 0,       256/2], 
                  [0,       1909.06, 256/2],
                  [0,       0,       1]])
# W_2i, H_2i = 2208, 1242


################## 2 ZED mini ##################
cam_k_mini = np.array([[1545.53, 0,       1110.24], 
                  [0,       1545.20, 601.27],
                  [0,       0,       1]])
# W_mini, H_mini = 2208, 1242


location = [random.uniform(-0.2,0.2), random.uniform(-0.2, 0.2), random.uniform(0.3,0.35)]
# poi = bproc.object.compute_poi(obj_queue)
poi = np.array([0, 0, 0])
rotation_matrix = bproc.camera.rotation_from_forward_vec(poi - location, inplane_rot=0)
pose_Zed = bproc.math.build_transformation_mat(location, rotation_matrix)

camera_id = int(args.camera_id)

if camera_id==0:
    cam_k = cam_k_Basler
    # cam_k = cam_k_2i
    pose_camera = pose_Basler
elif camera_id==1:
    cam_k = cam_k_2i
    pose_camera = pose_Zed
else:
    cam_k = cam_k_mini
    pose_camera = pose_Zed

# W, H = 2208, 1242
W, H = 256, 256

bproc.camera.set_resolution(W, H)
bproc.camera.set_intrinsics_from_K_matrix(cam_k, W, H)
bproc.camera.add_camera_pose(pose_camera)


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
bproc.writer.write_coco_annotations(f"{args.output_dir}",
                        # instance_segmaps=seg_data["instance_segmaps"],
                        # instance_attribute_maps=seg_data["instance_attribute_maps"],
                        instance_segmaps=data["instance_segmaps"],
                        instance_attribute_maps=data["instance_attribute_maps"],
                        colors=data["colors"],
                        mask_encoding_format='rle',
                        color_file_format="PNG", 
                        append_to_existing_output=True)


###############################################################
# Save the pose annotation of target object wrt camera
###############################################################
obj_pose_world = np.array(object.get_local2world_mat())
cam_pose_world = np.array(bproc.camera.get_camera_pose(frame=None))
# Noting that Blender uses the OpenGL coordinate frame. So, if you want to 
# use camera poses that are specified in OpenCV coordinates, you need to transform them first. 
cam_pose_world = bproc.math.change_source_coordinate_frame_of_transformation_matrix(cam_pose_world, ["X", "-Y", "-Z"])

cam_pose_inv = np.linalg.inv(cam_pose_world)
obj_pose_in_cam = np.matmul(obj_pose_world, cam_pose_inv)

print("obj pose in world", obj_pose_world)
print("cam pose in world", cam_pose_world)
print("obj pose in camera", obj_pose_in_cam)

# Get the list of files in the directory
import os
pose_path = args.output_dir + "/pose"
files = os.listdir(pose_path)
# Get the number of existing files
count_id = len(files)
np.save(pose_path+f"/pose{count_id}.npy", obj_pose_in_cam[:3][:])


print(f"Seg save time: {time.time() - time_start}")
# print(f"Generate a single image time: {time.time() - time_start}")
print()