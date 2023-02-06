import cv2
import glob


img_array = []
path_list = glob.glob('./output/images/*.png')
path_list.sort()

for filename in path_list:
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)

out = cv2.VideoWriter('physical.avi',cv2.VideoWriter_fourcc(*'DIVX'), 5, size)

for i in range(len(img_array)):
    out.write(img_array[i])
out.release()