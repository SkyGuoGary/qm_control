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

- 在rviz里选一个较高的点位，并右键send position

- 在load_qm_target窗口里输入trot等步态
