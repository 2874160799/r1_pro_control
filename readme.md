#### work in supcon



### 网页控制
源码地址： /home/nvidia/flask(1)/flask-template

命令：（vscode）：

`cd /home/nvidia/flask(1)/flask-template`

`python3 app.py`后本机打开浏览器链接即可

### 底盘旋转&二维码定位&磁力计驱动
源码地址：/home/nvidia/lyl_ws/src

命令：Apriltag

`ros2 run v4l2_camera v4l2_camera_node`

默认摄像头0，012345分别代表左前，右前，左，右，后

`ros2 launch apriltag_ros tag_realsense.launch.py`

底盘旋转目前的代码是基于磁力计的，旋转代码还没有测试，先看再测

`ros2 launch chassis_control chassis_control.launch.py`

磁力计驱动这里的代码是修改后的，增加了输出yaw角的话题

### 老代码
源码地址： /home/nvidia/galaxea/install/share/mobiman/scripts

改为ros2后的代码在atc_standard-V2.1.1/mobiman目录下,仅供参考

### 雷达驱动&FastLIO&定位服务
源码位置：/home/nvidia/ros2_humble_main/src

启动代码：

雷达驱动：

`ros2 launch livox_ros_driver2 msg_MID360_launch.py`

Fast-LIO:
`ros2 launch fast_lio mapping.launch.py`

需要建图时运行./save_pcd.sh有PCD文件后再执行接下来的导航等服务，如果不想事先建图可以运行：

`ros2 launch octomap_server2 octomap_server_launch.py`转八叉树地图可以实时跑NAV2

定位与重定位：

转2d激光雷达：

`ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py`

转栅格地图：

`ros2 launch pcd2pgm pcd2pgm.launch.py`

重定位：

`ros2 launch icp_registration icp.launch.py`

NAV2导航：

`ros2 launch robot_navigation2 navigation2.launch.py`




