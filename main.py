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

# # Load a bin object that gonna catch the usb objects
# bin_path = "BlenderProc/examples/advanced/physics_convex_decomposition/bin.obj"
# bin_obj = bproc.loader.load_obj(bin_path)[0]
# # bin_obj.set_scale([0.15, 0.15, 0.15])
# bin_obj.set_scale([1, 1, 1])
# bin_obj.set_location(np.array([0, 0, 1.8]))
# bin_obj.set_rotation_euler(np.array([-np.pi/2, 0, 0]))

# Load tag_board that gonna catch the usb objects
tag_baord = bproc.loader.load_obj(args.tag_board)[0]
tag_baord.set_scale([1, 1, 1])
tag_baord.set_location(np.array([0, 0, -1.8]))
# tag_baord.set_rotation_euler(np.array([-np.pi/2, 0, 0]))
tag_baord.set_rotation_euler(np.array([-np.pi/2, np.pi, 0]))

parts = ['mainshell', 'topshell', 'insert_mold']
obj_queue = []
for obj in Path("./CAD_model/models").rglob('*.obj'):
    if 'background' in obj.name:
        continue

    for _ in range(1):
        offset = 0.005
        obj_queue.append(bproc.loader.load_obj(str(obj)).pop())

# Define a function that samples the pose of a given usb object
def sample_pose(obj: bproc.types.MeshObject):
    # Sample the location above the tagboard
    obj.set_scale([1, 1, 1])
    # obj.set_location(np.random.uniform([-0.04, -0.04, 1.78], [0.04, 0.04, 1.79]))
    # obj.set_location(np.random.uniform([-0.04, -0.04, 1.775], [0.04, 0.04, 1.78]))
    obj.set_location(np.random.uniform([-0.04, -0.04, -1.76], [0.04, 0.04, -1.75]))
    obj.set_rotation_euler(bproc.sampler.uniformSO3())
    # obj.set_rotation_euler(np.array([0, 0, 0]))

# Sample the poses of all usb objects, while making sure that no objects collide with each other.
bproc.object.sample_poses(
    obj_queue,
    sample_pose_func=sample_pose
)


cam_k = np.array([[21627.734375, 0, 2353.100109], 
                  [0, 21643.369141, 1917.666411],
                  [0, 0, 1]])

# camera.camera.set_resolution(512, 512)
W, H = 5472, 3648
bproc.camera.set_resolution(W, H)
bproc.camera.set_intrinsics_from_K_matrix(cam_k, W, H)

# read the camera positions file and convert into homogeneous camera-world transformation
# cam2world = bproc.math.change_source_coordinate_frame_of_transformation_matrix(np.eye(4), ["X", "-Y", "-Z"])
# bproc.camera.add_camera_pose(cam2world)
bproc.camera.add_camera_pose(np.eye(4))
cam_pose = bproc.camera.get_camera_pose(frame=None)
cam_R = cam_pose[0:3, 0:3]

theta = np.random.uniform(0., 0.1*np.pi)

random_R = np.array([
    [np.cos(theta),  np.sin(theta),     0.0],
    [0.0,            1.0,               0.0],
    [-np.sin(theta), 0.0,               np.cos(theta)],
])

# Define a sun light
light = bproc.types.Light()
light.set_type("POINT")
light.set_location([-0.5, 0.1, -0.6])
# light.set_rotation_euler([-0.063, 0.6177, -0.1985])
light.set_rotation_mat(cam_R*random_R)
light.set_color([1, 1, 1])
light.set_energy(100)



if Physics:
    # # Make the tagboard object passively participate in the physics simulation   ## COMPOUND
    tag_baord.enable_rigidbody(active=False, collision_shape="CONVEX_HULL", mass = 5)
    # Let its collision shape be a convex decomposition of its original mesh
    # This will make the simulation more stable, while still having accurate collision detection
    # tag_baord.build_convex_decomposition_collision_shape(args.vhacd_path)

    for part in obj_queue:
        # Make the bin object actively participate in the physics simulation (they should fall into the board)
        # part.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", collision_margin = 0.01, mass=10, linear_damping = 0.2)
        # part.enable_rigidbody(active=True, collision_shape="CONVEX_HULL", mass=5)
        part.enable_rigidbody(active=True, collision_shape="COMPOUND", mass=5)
        # Also use convex decomposition as collision shapes
        part.build_convex_decomposition_collision_shape(args.vhacd_path)

    # # Run the physics simulation for at most 20 seconds
    # bproc.object.simulate_physics_and_fix_final_poses(
    #     min_simulation_time=0.12,
    #     max_simulation_time=2.12,
    #     check_object_interval=1
    # )

    bproc.object.simulate_physics(
    min_simulation_time=0.2,
    max_simulation_time=20,
    check_object_interval=1
    )
    # This will make the renderer render the first 10 frames of the simulation
    bproc.utility.set_keyframe_render_interval(frame_start=0, frame_end=20)
    # bproc.utility.set_keyframe_render_interval(frame_start=9, frame_end=10)
    # bproc.utility.set_keyframe_render_interval(frame_start=450, frame_end=455)


# # render the whole pipeline
# data = bproc.renderer.render()
# # write the data to a .hdf5 container
# bproc.writer.write_hdf5(args.output_dir, data)


# render the whole pipeline
bproc.renderer.set_max_amount_of_samples(50)
bproc.renderer.set_noise_threshold(1)
bproc.renderer.set_cpu_threads(0)
data = bproc.renderer.render()
seg_data = bproc.renderer.render_segmap(map_by=["instance", "class", "name"])

# Write data to coco file
# bproc.writer.write_coco_annotations(os.path.join(args.output_dir, 'coco_data'),
time_start = time.time()
bproc.writer.write_coco_annotations(f"{args.output_dir}",
                        instance_segmaps=seg_data["instance_segmaps"],
                        instance_attribute_maps=seg_data["instance_attribute_maps"],
                        colors=data["colors"],
                        mask_encoding_format='polygon',
                        color_file_format="PNG", 
                        append_to_existing_output=True)


print(f"Seg save time: {time.time() - time_start}")
print()