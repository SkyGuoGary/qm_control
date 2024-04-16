## 运行demo步骤

### MPC-WBC

- 打开gazebo并载入控制器：
  
  ```
  roslaunch qm_gazebo empty_world.launch
  roslaunch qm_controllers load_controller.launch
  ```

- 按下gazebo左下角的三角按钮，开始仿真

- 载入gui界面：
  
  ```
  rosrun rqt_controller_manager rqt_controller_manager
  ```

- 右键两个节点，依次打开，等待机械臂末端回正

- 打开步态控制器和rviz：
  
  ```
  roslaunch qm_controllers load_qm_target.launch 
  roslaunch qm_controllers rviz.launch
  ```

- 在rviz里根据需要调整初始位置，并右键send position，此时机器人应当站起

- 在load_qm_target窗口里输入trot等步态，待机器人稳定行走后修改marker位置

### MPC-only

- 将MPC-WBC的前两步改为：
  
  ```
  roslaunch qm_gazebo empty_world_mpc.launch
  roslaunch qm_controllers load_controller_mpc.launch
  ```

- 注意此时需按下三角按钮，开始仿真，控制器才能成功载入

- 其他步骤完全一致

### door-opening

- 将MPC-WBC的前两步改为：
  
  ```
  roslaunch qm_gazebo door_world.launch
  roslaunch qm_controllers load_controller_mpc.launch
  ```

- 注意此时需按下三角按钮，开始仿真，控制器才能成功载入

- 待其他步骤完成后，启用**步态切换器**，**marker控制**，**动作控制**：
  
  ```
  rosrun qm_controllers gait_switch 
  rosrun qm_controllers marker_pose_publisher 
  rosrun qm_controllers door_opening_planner 2
  ```