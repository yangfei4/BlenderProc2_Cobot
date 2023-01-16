import blenderproc as bproc
import argparse
import numpy as np
import bpy
import time

from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('tag_board', default="./CAD_model/tagboard_21x21x1cm.obj",help="Path to the object file containing the bin, should be examples/advanced/physics_convex_decomposition/bin.obj.")
parser.add_argument('output_dir', nargs='?', default="./output", help="Path to where the final files will be saved ")
parser.add_argument('vhacd_path', nargs='?', default="blenderproc_resources/vhacd", help="The directory in which vhacd should be installed or is already installed.")
args = parser.parse_args()

bproc.init()
Physics = True


###############################################################
# Set the camera pose same as world frame, located in origin
###############################################################
cam_k = np.array([[21627.734375, 0, 2353.100109], 
                  [0, 21643.369141, 1917.666411],
                  [0, 0, 1]])
# camera.camera.set_resolution(512, 512)
W, H = 5472, 3648
bproc.camera.set_resolution(W, H)
bproc.camera.set_intrinsics_from_K_matrix(cam_k, W, H)
bproc.camera.add_camera_pose(  [[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

###############################################################
# Define a random point light
###############################################################
light = bproc.types.Light()
light.set_type("POINT")
# Sample its location in a shell around the point [0.1, 0.2, -0.6]
light.set_location(bproc.sampler.shell(
    center=[0.1, 0.2, -0.3],
    radius_min=0.2,
    radius_max=0.8,
    elevation_min=15,
    elevation_max=50
))
light.set_energy(np.random.uniform(80,120,1).item())


###############################################################
# Load tag_board which is going to catch the usb objects
###############################################################
tag_baord = bproc.loader.load_obj(args.tag_board)[0]
tag_baord.set_scale([1, 1, 1])
tag_baord.set_location(np.array([0, 0, -1.6]))
# tag_baord.set_rotation_euler(np.array([-np.pi/2, 0, 0]))
tag_baord.set_rotation_euler(np.array([-np.pi/2, np.pi, np.random.uniform(0, 2*np.pi, 1).item()]))
tag_baord.set_cp("category_id", -1)

###############################################################
# Load usb objects
###############################################################
parts = ['mainshell', 'topshell', 'insert_mold']
obj_queue = []
for obj in Path("./CAD_model/models").rglob('*.obj'):
    if 'background' in obj.name:
        continue

    ## TODO(yangfei): should assign each part with unique id?  e.g. mainshell_00
    for _ in range(3):
        obj_queue.append(bproc.loader.load_obj(str(obj)).pop())
        part = obj_queue[-1]

        print(obj.name)
        # remove '.obj'
        idx = parts.index(obj.name[:-4])
        part.set_cp("category_id", idx + 1)
        part.set_name(parts[idx])
        print('category_id =', part.get_cp("category_id"))
        print('Part Name =', part.get_name())

import pdb;pdb.set_trace()

###############################################################
# Set the initial poses of objects randomly
###############################################################
# Define a function that samples the pose of a given usb object
def sample_pose(obj: bproc.types.MeshObject):
    # Sample the location above the tagboard
    obj.set_scale([1, 1, 1])
    obj.set_location(np.random.uniform([-0.03, -0.03, -1.56], [0.03, 0.03, -1.55]))
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
    tag_baord.enable_rigidbody(active=False, collision_shape="CONVEX_HULL", mass = 5)

    for part in obj_queue:
        # Make the bin object actively participate in the physics simulation (they should fall into the board)
        # part.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", collision_margin = 0.01, mass=10, linear_damping = 0.2)
        # part.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", mass=5)
        part.enable_rigidbody(active=True, collision_shape="COMPOUND", mass=5)
        # Also use convex decomposition as collision shapes
        part.build_convex_decomposition_collision_shape(args.vhacd_path)

    bproc.object.simulate_physics_and_fix_final_poses(
    min_simulation_time=0.2,
    max_simulation_time=20,
    check_object_interval=1
    )
    # This will make the renderer render the first 20 frames of the simulation
    # bproc.utility.set_keyframe_render_interval(frame_start=0, frame_end=20)

###############################################################
# Pose information
###############################################################
print('#'*80)

T_cam_to_world = bproc.camera.get_camera_pose(frame=None)
R_cam_to_world = T_cam_to_world[0:3, 0:3]
print("T_cam_to_world = \n", T_cam_to_world)

part_poses = []
for part in obj_queue:
    part_pose = part.get_local2world_mat()
    part_poses.append(part_pose)

print(obj_queue[0].get_name(), "T_part_to_world = \n", part_poses[0])

print('#'*80)

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
                        color_file_format="PNG", 
                        append_to_existing_output=True)

print(f"Seg save time: {time.time() - time_start}")
# print(f"Generate a single image time: {time.time() - time_start}")
print()