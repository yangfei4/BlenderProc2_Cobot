import blenderproc as bproc
import numpy as np

from pathlib import Path

# Initialize
bproc.init()

# cad_path = Path("./CAD_model/insert_mold.obj")
cad_path = "./CAD_model/insert_mold.obj"
# load the objects into the scene
# obj = bproc.loader.load_obj(cad_path)
obj = bproc.object.create_primitive("MONKEY")

obj.set_cp("category_id", 0)
pose = np.eye(4)
pose[:, 3] = np.array([0, 0, 1.6 + 0.05, 1]).T
obj.set_scale([1, 1, 1])
obj.set_local2world_mat(pose)

# define a light and set its location and energy level
light = bproc.types.Light()
light.set_type("POINT")
light.set_location([0, 0, -1])
light.set_energy(1000)


# # Set the camera to be in front of the object
# cam_pose = bproc.math.build_transformation_mat([0, -5, 0], [np.pi / 2, 0, 0])
# bproc.camera.add_camera_pose(cam_pose)

# define the camera resolution
cam_k = np.array([[21627.734375, 0, 2353.100109], 
                  [0, 21643.369141, 1917.666411],
                  [0, 0, 1]])
# camera.camera.set_resolution(512, 512)
W, H = 5472, 3648
bproc.camera.set_resolution(W, H)
bproc.camera.set_intrinsics_from_K_matrix(cam_k, W, H)
# read the camera positions file and convert into homogeneous camera-world transformation
# Set camera pose via cam-to-world transformation matrix
cam2world = np.eye(4)
# Change coordinate frame of transformation matrix from OpenCV to Blender coordinates
cam2world = bproc.math.change_source_coordinate_frame_of_transformation_matrix(cam2world, ["X", "-Y", "-Z"])
bproc.camera.add_camera_pose(cam2world)


# Render the scene
data = bproc.renderer.render()

# Write the rendering into an hdf5 file
# bproc.writer.write_hdf5("output/", data)

# COCO
seg_data = bproc.renderer.render_segmap(map_by=["instance", "class", "name"])
# Write data to coco file
output_path = "output/"
bproc.writer.write_coco_annotations(output_path,
                                    instance_segmaps=seg_data["instance_segmaps"],
                                    instance_attribute_maps=seg_data["instance_attribute_maps"],
                                    colors=data["colors"],
                                    color_file_format="PNG")