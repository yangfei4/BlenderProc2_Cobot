# Blenderproc2_Cobot
This repo is a demo script of blenderProc2, particularly the physics collision and simulation tutorial and examples. Bonus points if modifying it to use some of Cobot's USB Type-C cad models.

```
pip install blenderproc
blenderproc quickstart
blenderproc vis hdf5 output/0.hdf5
```

### Physical collision experiment
```
 blenderproc run ./main.py ./BlenderProc/examples/advanced/physics_convex_decomposition/bin.obj
 blenderproc vis hdf5 output/0.hdf5
```

![plot](https://github.com/D-YF/BlenderProc2_Cobot/blob/main/output/demo.png)
