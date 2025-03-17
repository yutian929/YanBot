#CH9102，同时系统安装了对应驱动 串口号0004 设置别名为wheeltec_mic
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0004", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_mic"' >>/etc/udev/rules.d/wheeltec_mic.rules
#CH9102，同时系统没有安装对应驱动 串口号0004 设置别名为wheeltec_mic
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0004", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_mic"' >>/etc/udev/rules.d/wheeltec_mic.rules

service udev reload
sleep 2
service udev restart
