
#### Gazebo 展示
![Gazebo](as_arm/as_arm_description/img/gazebo.png)  

#### Real环境展示
![Real Environment](as_arm/as_arm_description/img/real.png)  

## URDF描述文件
* 机械臂相关描述文件位于 _as_arm_description/urdf_ 目录中
    * as_arm.xacro 为机械臂描述文件
    * camera.xacro 摄像机和机架描述文件
    * sink.xacro 物品槽描述文件

## Launch启动相关命令：
* 启动gazebo仿真环境：
    * roslaunch as_arm_gazebo as_arm_bringup.launch
* 启动moveit Demo：
    * roslaunch as_arm_moveit_config demo.launch
* 启动grasp 生成器
    * roslaunch as_arm_gazebo grasp_generator_server.launch
* 查看摄像头图像：
    * rosrun image_view image_view image:=/camera/image_raw
* 控制某个joint移动角度：
    * rostopic pub -1 /rrbot/joint1_position_controller/command std_msgs/Float64 "data: 1.5"
* 获取cube位置
    * rostopic echo -n 1 /gazebo/cubes

* 获取joint位置：
    * rostopic echo -n 1 /as_arm/joint_states
* 获取某个link(如end effector)的世界坐标
    * rosrun tf tf_echo /world /grasp_frame_link
    
## 碰撞检测相关
* 碰撞检测包含self-collision和environment-collision两种，相关文件如下：
    * Service描述文件： as_arm_description/srv/CheckCollisionValid.srv
    * Service服务文件（需要在catkin环境编译）： as_arm_control/src/check_collision.cpp
    
## 仿真环境运行：
* 相关控制脚本位于 _as_arm_control/test/_目录中
    * 抓取脚本: pick_and_place.py
    * moveit运动规划： test_planner.py

## 真实环境运行：
* arduino 文件在 as_arm_real/data/servo_v4.0.ino
* 启动real节点，控制真实机械臂运动：
    * roslaunch as_arm_real servo_bringup_real.launch
* 启动gazebo仿真环境后，运行控制脚本 _as_arm_real/scripts/_
    * 真实环境与仿真环境**机械臂**同步随机运动： random_run_drive.py
    * 真实环境与仿真环境**机械手**同步运动： run_gripper_driver.py
    * 抓取脚本（调用ompl IK算法）： pick_and_place.py

## 深度增强学习训练：
* 训练脚本位于 _as_arm_control/scripts/_ 目录中
    * gazebo仿真环境控制和状态获取脚本: simulate_state.py
    * DDPG算法脚本(TF实现): ddpg.py
    * 仿真环境Ａgent接口:  asm_env.py
    * 训练脚本：learning.py

