## 运行demo步骤

### door-opening

- 打开gazebo并载入控制器：
  
  ```
  roslaunch qm_gazebo door_world.launch
  roslaunch qm_controllers load_controller_mpc.launch
  rosrun rqt_controller_manager rqt_controller_manager
  ```

- 按下gazebo左下角的三角按钮，开始仿真

- 载入gui界面，右键两个节点，依次打开，等待机械臂末端回正

- 打开步态控制器和rviz：
  
  ```
  roslaunch qm_controllers load_qm_target.launch 
  roslaunch qm_controllers rviz.launch
  ```

- 在rviz里右键send position，发送初始位姿，此时机器人应当站起

- 在load_qm_target窗口里输入trot等步态，待机器人稳定行走后修改marker位置

- 待其他步骤完成后，启用**推门规划**或**拉门规划**：
  
  ```
  rosrun qm_controllers door_push 3
  rosrun qm_controllers door_pull 3
  ```