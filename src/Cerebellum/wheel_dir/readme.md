在原有wheeltec_noetic_src x86_64 成功编译版的基础上移除了一些包，具体说明如下：
1. 移除aruco_ros-noetic-devel   该包用于ArUco标记识别
2. 移除bodyreader               该包用于人体姿态识别
3. 移除cob_navigation           该包为cob导航功能包，疑似没用到
4. 移除gnss_convert             该包用于GNSS数据转换，无室外定位场景，暂时移除
5. 移除ipa_exploration          该包用于全图探索建图，暂时移除
6. 移除ira_laser_tools-ros1-master 该包用于多路激光雷达数据处理，暂时移除
7. 移除kcf_tracker              该包用于目标跟踪，暂时移除
8. 移除laser_double             该包用于双路激光雷达，暂时移除
9. 移除laserscan_merge-master   该包用于多路激光雷达数据融合，暂时移除
10. 移除ldlidar_14              该包为乐动LDLIDAR 14 sdk，设备不包含此款雷达
11. 移除orb_slam_2_ros-master    该包为ORB-SLAM2，移除
12. 移除qt_ros_test              该包为wheeltec所写的qt测试包，无用
13. 移除ros_tensorflow
14. 移除rplidar_ros              其他雷达sdk，设备不包含
15. 移除simple_follower          该包为wheeltec的简单跟随功能包
16. 移除slam_karto               该包为karto建图包，未使用到
17. 移除web_video_server         该包用于将本地视频流发布到http服务
18. 移除wheeltec_multi           多车协同功能包
19. 移除wheeltec_yolo_action     wheeltec yolo 目标识别接口，暂时移除
20. 移除wheeltec_gps_driver
21. 由于未使用外置imu，故fdilink_imu,yesense_imu使用不到，只是用下位机自带的IMU，估计是MPU6050，移除yesense_imu、fdilink_ahrs
22. 移除了除S100底盘以外，其他wheeltec底盘的urdf、meshes、params_costmap