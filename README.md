# Blenderproc2_Cobot
This repo is a demo script of blenderProc2, particularly the physics collision and simulation tutorial and examples. Bonus points if modifying it to use some of Cobot's USB Type-C cad models.

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
```
 blenderproc run ./main.py ./CAD_model/tagboard_21x21x1cm.obj
 blenderproc vis hdf5 output/0.hdf5
```

![plot](https://github.com/D-YF/BlenderProc2_Cobot/blob/main/output/demo.png)

When I turn on the `bin_obj.enable_rigidbody(active=False, collision_shape="COMPOUND")`
![plot2](https://github.com/D-YF/BlenderProc2_Cobot/blob/main/output/error.png)
