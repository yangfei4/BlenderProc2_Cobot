import blenderproc as bproc
import argparse
import numpy as np
import bpy
import time
import random

from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('output_dir', nargs='?', default="./output/dataset", help="Path to where the final files will be saved ")
parser.add_argument('camera_num', nargs='?', help="Which Camera you would like to use: 0-Beslar 1-Zed_2i 2-Zed_mini")
args = parser.parse_args()

bproc.init()
Physics = True

###############################################################
# Load tag_board which is going to catch the usb objects
###############################################################
tag_board = bproc.loader.load_obj('./CAD_model/tagboard_21x21x1cm.obj')[0]
tag_board.set_scale([2, 0.35, 2])
tag_board.set_location(np.array([0, 0, 0.06+0.01]))
# tag_board.set_rotation_euler(np.array([-np.pi/2, 0, 0]))
tag_board.set_rotation_euler(np.array([-np.pi/2, np.pi, np.random.uniform(0, 2*np.pi, 1).item()]))
tag_board.set_cp("category_id", 99)
tag_board.set_name("tagboard")
tag_board.set_cp("supercategory", "background")

bread_board = bproc.loader.load_obj('./CAD_model/breadboard.obj')[0]
bread_board.set_scale([1, 1, 1])
bread_board.set_location(np.array([0, 0, 0.01]))
# bread_board.set_rotation_euler(np.array([-np.pi/2, np.pi, np.random.uniform(0, 2*np.pi, 1).item()]))
bread_board.set_rotation_euler(np.array([-np.pi/2, np.pi, 0]))
bread_board.set_cp("category_id", 999)
bread_board.set_name("bread_board")


###############################################################
# Define a random point light
###############################################################
light = bproc.types.Light()
light.set_type("POINT")
# Sample its location in a shell around the point [0.1, 0.2, -0.6]
light.set_location([0,0, random.uniform(1, 4)])
# light.set_location(bproc.sampler.shell(
#     center=[random.uniform(-0.1,0.1), random.uniform(-0.1, 0.1), random.uniform(2.5,4)],
#     radius_min=0.2,
#     radius_max=0.8,
#     elevation_min=15,
#     elevation_max=50
# ))
light.set_energy(np.random.uniform(50,100,1).item())


###############################################################
# Load usb objects
###############################################################
images = list(Path('/data/ham/BlenderProc2/polyhaven_bgs').rglob('*.png')) +  list(Path('/data/ham/BlenderProc2/polyhaven_bgs').rglob('*.jpg'))

parts = ['mainshell', 'topshell', 'insert_mold']
obj_queue = []
for obj in Path("./CAD_model/models").rglob('*.obj'):
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
    
    
    # TODO(yangfei): should assign each part with unique id?  e.g. mainshell_00
    for _ in range(33):
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
    obj.set_location(np.random.uniform([-0.20, -0.12, 0.015+0.06+0.01], [0.20, 0.12, 0.025+0.06+0.01]))
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
    bread_board.enable_rigidbody(active=False, collision_shape="CONVEX_HULL", mass = 50, collision_margin=0.0006)

    for part in obj_queue:
        # t_start = time.time()

        # Make the bin object actively participate in the physics simulation (they should fall into the board)
        # part.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", collision_margin = 0.01, mass=10, linear_damping = 0.2)
        # part.enable_rigidbody(active=True, collision_shape="COMPOUND", mass=5)

        part.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", mass=0.01, collision_margin=0.0006)
        # Also use convex decomposition as collision shapes
        # part.build_convex_decomposition_collision_shape('blenderproc_resources/vhacd')
        # print("#"*100+"\n"+f"decomposition time: {time.time() - t_start}")
        # print("#"*100)

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
                [0, 0, 1, random.uniform(0.4, 0.6)+0.07],
                [0, 0, 0, 1]]


################## 1 ZED 2i  ##################
cam_k_2i = np.array([[1908.56, 0,       1113.88], 
                  [0,       1909.06, 588.34],
                  [0,       0,       1]])
# W_2i, H_2i = 2208, 1242


################## 2 ZED mini ##################
cam_k_mini = np.array([[1545.53, 0,       1110.24], 
                  [0,       1545.20, 601.27],
                  [0,       0,       1]])
# W_mini, H_mini = 2208, 1242


location = [random.uniform(-0.18,0.18), random.uniform(-0.18, 0.18), random.uniform(0.35,0.4)+0.07]
poi = bproc.object.compute_poi(obj_queue)
rotation_matrix = bproc.camera.rotation_from_forward_vec(poi - location, inplane_rot=0)
pose_Zed = bproc.math.build_transformation_mat(location, rotation_matrix)

camera_num = int(args.camera_num)

if camera_num==0:
    # cam_k = cam_k_Basler
    cam_k = cam_k_2i
    pose_camera = pose_Basler
elif camera_num==1:
    cam_k = cam_k_2i
    pose_camera = pose_Zed
else:
    cam_k = cam_k_mini
    pose_camera = pose_Zed

W, H = 2208, 1242

bproc.camera.set_resolution(W, H)
bproc.camera.set_intrinsics_from_K_matrix(cam_k, W, H)
bproc.camera.add_camera_pose(pose_camera)

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
bproc.writer.write_coco_annotations(f"{args.output_dir}",
                        # instance_segmaps=seg_data["instance_segmaps"],
                        # instance_attribute_maps=seg_data["instance_attribute_maps"],
                        instance_segmaps=data["instance_segmaps"],
                        instance_attribute_maps=data["instance_attribute_maps"],
                        colors=data["colors"],
                        mask_encoding_format='polygon',
                        supercategory = "usb",
                        color_file_format="PNG", 
                        append_to_existing_output=True)

print(f"Seg save time: {time.time() - time_start}")
# print(f"Generate a single image time: {time.time() - time_start}")
print()