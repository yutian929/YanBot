#CP2102 串口号0002 设置别名为wheeltec_controller
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller.rules
#CH9102，同时系统安装了对应驱动 串口号0002 设置别名为wheeltec_controller
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller2.rules
#CH9102，同时系统没有安装对应驱动 串口号0002 设置别名为wheeltec_controller
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller3.rules

echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_gnss"' >/etc/udev/rules.d/wheeltec_gnss.rules

#CP2102 串口号0001 设置别名为wheeltec_lidar
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar.rules
#CH9102，同时系统安装了对应驱动 串口号0001 设置别名为wheeltec_lidar
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar2.rules
#CH9102，同时系统没有安装对应驱动 串口号0001 设置别名为wheeltec_lidar
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar3.rules

#CP2102 串口号0003 设置别名为wheeltec_IMU
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_IMU"' >/etc/udev/rules.d/wheeltec_imu.rules
#CH9102 ，如果已经安装了驱动使用CH343	
echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_IMU"' >/etc/udev/rules.d/wheeltec_imu_343.rules
#CH9102  如果没有安装驱动
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_IMU"' >/etc/udev/rules.d/wheeltec_imu_ACM.rules

echo  'SUBSYSTEM=="video4linux",ATTR{name}=="GENERAL WEBCAM: GENERAL WEBCAM",ATTR{index}=="0",MODE:="0777",SYMLINK+="RgbCam"' >>/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="Integrated Webcam: Integrated W",ATTR{index}=="0",MODE:="0777",SYMLINK+="RgbCam"' >>/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="Astra Pro HD Camera: Astra Pro ",ATTR{index}=="0",MODE:="0777",SYMLINK+="Astra_Pro"' >>/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="USB 2.0 Camera: USB Camera",ATTR{index}=="0",MODE:="0777",SYMLINK+="Astra_Dabai"' >>/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTRS{idVendor}=="2bc5",ATTRS{idProduct}=="0511",ATTR{index}=="0",MODE:="0777",SYMLINK+="Astra_Gemini"' >>/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="Intel(R) RealSense(TM) Depth Ca",ATTR{index}=="0",MODE:="0777",SYMLINK+="realsense"' >>/etc/udev/rules.d/camera.rules

service udev reload
sleep 2
service udev restart
