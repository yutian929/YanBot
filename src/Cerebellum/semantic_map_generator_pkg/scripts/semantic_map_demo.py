#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np


def create_semantic_cloud():
    # 创建点云消息
    cloud = PointCloud2()
    cloud.header.stamp = rospy.Time.now()
    cloud.header.frame_id = "map"  # 设置坐标系

    # 定义点云字段（坐标x,y,z + 颜色rgb）
    cloud.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("rgb", 12, PointField.UINT32, 1),
        PointField("label_hash", 16, PointField.UINT32, 1),
    ]

    # 点云参数设置
    cloud.height = 1  # 非结构化点云
    cloud.width = 4  # 4个点
    cloud.is_bigendian = False
    cloud.point_step = 20  # 每个点20字节（3*float + 2*uint32）
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True

    # 创建带有颜色的虚拟点云数据（x, y, z, RGB, label_hash)
    points = [
        (1.0, 0.0, 0.0, (255, 0, 0), 111),  # 红色点
        (0.0, 1.0, 0.0, (0, 255, 0), 222),  # 绿色点
        (0.0, 0.0, 1.0, (0, 0, 255), 333),  # 蓝色点
        (-1.0, 0.0, 0.0, (255, 255, 0), 444),  # 黄色点
    ]

    # 将数据打包为二进制格式
    packed_data = []
    for x, y, z, (r, g, b), label_hash in points:
        # 将RGB颜色打包为4字节（BGR格式）
        rgba = struct.pack("BBBB", b, g, r, 0)
        rgb_value = struct.unpack("<I", rgba)[0]
        # 打包坐标和颜色
        packed_data.append(struct.pack("<fffII", x, y, z, rgb_value, label_hash))

    cloud.data = b"".join(packed_data)
    return cloud


if __name__ == "__main__":
    rospy.init_node("semantic_cloud_publisher")
    pub = rospy.Publisher("semantic_clouds", PointCloud2, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz发布频率

    while not rospy.is_shutdown():
        cloud = create_semantic_cloud()
        pub.publish(cloud)
        rate.sleep()
