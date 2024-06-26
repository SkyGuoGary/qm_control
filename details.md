### The property of the hinged door now:

- a spring-loaded door
- the handle doesn't affect opening and can't revolute, just for grasp

### A possible method for handle-opening

- attach the handle with a tiny invisible collision box, which can rotate together 
- make it be in front of / behind the door collision 

### Modify configuration of Kinova:

- open/close collision in line 24 of:
  
  ```
  qm_description/urdf/manipulator/common/kinova_common.xacro
  ```

- open grasp_fix_plugin in line 155 of:
  
  ```
  qm_control/qm_description/urdf/manipulator/j2n6s300fix.xacro
  ```

### Modify the initial pose of rviz ball:

- modify line 29-35 of:
  
  ```
  qm_controllers/src/QmTargetTrajectoriesPublisher.cpp
  ```

- then build it in terminal:
  
  ```
  catkin build qm_controllers --no-deps
  ```

### Self-collision:

- not detected now

### /qm_mpc_observation/state

- dim=30, state=
  - base_linear_v(3), 
  - base_angular_v(3), 
  - base_position(3), 
  - base_twist(3), the first one is yaw
  - leg_joint_position(12),
  - arm_joint_position(6)

### /cmd_vel & /ee_cmd_vel for velocity controlling

- when need to keep the base static in trot mode:
  - set **/cmd_vel/angular/z** = 0.0047 and others = 0
- **/ee_cmd_vel** is **NOT** good for ee target moving

### /marker_cmd_vel

- marker linear velocity controller 

- angular vel & pos control around any axis of any frame

- TODO:
  
  - thoroughly resolve gimbal lock problem by modifying rotate method (now: RPY)

### /gait_mode_cmd

- use simple_gait.info with only **stance** , **trot** , **standing_trot** gait


