# Blenderproc2_Cobot
This repo is a demo script of blenderProc2, particularly the physics collision and simulation tutorial and examples.
* Sythetic datasets generation pipeline for object detection and pose estimation
* Augmented synthetic dataset with annotation
<p align="center">
  <img src="https://github.com/yangfei4/Sim2real/blob/main/figures/synthetic.jpg" width="300">
  <img src="https://github.com/yangfei4/Sim2real/blob/main/figures/synthetic_data_with_annotation.jpg?raw=true" width="400">
</p>

### Install blenderproc lib

```
git clone https://github.com/DLR-RM/BlenderProc
cd BlenderProc
pip install -e .
```
To test if blenderproc library was installed correctly
```
blenderproc quickstart
blenderproc vis hdf5 output/0.hdf5
```

### Physical collision experiment
Before running this, please download USB CAD models, and put them in `./CAD_model/models`
```
bash scripts/physical_sim
```

### Generate video from `./output/images` folder
```
python generate_video.py
```

![](https://github.com/yangfei4/BlenderProc2_Cobot/blob/main/output/simulation_demo.gif)

![plot](https://github.com/D-YF/BlenderProc2_Cobot/blob/main/output/demo.png)
