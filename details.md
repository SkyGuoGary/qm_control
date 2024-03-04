### The property of the hinged door now: 
- a spring-loaded door
- the handle doesn't affect opening and can't revolute, just for grasp

### A possible method for handle-opening 
- attach the handle with a tiny invisible collision box, which can rotate together 
- make it be in front of / behind the door collision 

### Open collision of Kinova or not:
- modify the switch of collision in 
```
qm_description/urdf/manipulator/common/kinova_common.xacro
```

### Modify the initial pose of rviz ball:
- modify line 29-35:
```
qm_controllers/src/QmTargetTrajectoriesPublisher.cpp
```

- then build it in terminal:
```
catkin build qm_controllers
```
