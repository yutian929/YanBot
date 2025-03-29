#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf2_ros import Buffer, TransformListener
import tf
import struct
import threading
# from sklearn.neighbors import KDTree
# from sklearn.cluster import DBSCAN
from semantic_map_pkg.msg import SemanticObject
from datetime import datetime
from grounding_sam_ros.srv import (
    VitDetection,
    VitDetectionResponse,
    UpdatePrompt,
    UpdatePromptResponse,
)
from yolo_evsam_ros.msg import AnnotationInfo, MaskInfo
import supervision as sv

import concurrent.futures
import time 
from scipy.optimize import curve_fit

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
matplotlib.use('Agg')  # 设定非GUI后端，避免 Tkinter 问题

import os


class SemanticMapGenerator:
    def __init__(self):
        rospy.init_node("semantic_map_generator")

        self.camera_info = None
        self.color_image_cache = {}
        self.depth_raw_cache = {}
        self.tf_cache = {}

        self.processing_timestamp = None
        self.processing_image = None
        self.processing_depth_raw = None

        # 创建一个 Event 对象，用来确保图像标注完成后才进行最终的点云对象生成（由于标注图像是在最后一步才用到，所以一般也不会发生时间问题）
        self.annotate_finished_event = threading.Event()

        self.bridge = CvBridge()

        # 从annotate传递给点云生成
        self.annotated_image = None

        # 下采样步长
        self.downsample_step = rospy.get_param("~downsample_step", 2)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # 多线程运行
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)  # 创建 4 个线程

        ### 由于只需要处理回调一次，不用放进线程池 ###
        # 订阅相机内外参（读取一次即可）
        topic_camera_info_sub = rospy.get_param(
            "~topic_camera_info_sub", "/camera/color/camera_info"
        )
        self.camera_info_sub = rospy.Subscriber(topic_camera_info_sub, CameraInfo, self.read_camera_info,queue_size=2)

        ### 线程1 ###
        # 订阅rgb图像和原始深度图像、tf变换（在定时器发布时就存取）
        color_image_sub = Subscriber("color_to_process", Image, queue_size=1)
        depth_raw_sub = Subscriber("depth_to_process", Image, queue_size=1)
        tf_sub = Subscriber('tf_transform', TransformStamped, queue_size=5)
        # 同步回调
        ts1 = ApproximateTimeSynchronizer(
            [color_image_sub, depth_raw_sub, tf_sub], queue_size=3, slop=0.005
        )
        # ts1.registerCallback(self.store_image)
        ts1.registerCallback(lambda color_image_msg, depth_raw_msg, tf_msg: self.executor.submit(self.store_image, color_image_msg, depth_raw_msg, tf_msg))

        # 订阅图像标注信息
        # self.annotation_info_sub = rospy.Subscriber("annotation_info", AnnotationInfo, self.annotate, queue_size=2)
        self.annotation_info_sub = rospy.Subscriber("annotation_info", AnnotationInfo, lambda msg: self.executor.submit(self.annotate, msg), queue_size=2)

        ## 点云生成的回调
        # 订阅修复后的深度图像
        depth_repaired_sub = Subscriber("depth_repaired", Image, queue_size=5)
        # 订阅掩码信息
        mask_info_sub = Subscriber("mask_info", MaskInfo, queue_size=5)
        # 同步回调
        ts = ApproximateTimeSynchronizer(
            [depth_repaired_sub, mask_info_sub], queue_size=5, slop=0.005
        )
        ts.registerCallback(self.sync_sub_callback)
        # ts.registerCallback(lambda depth_msg, mask_msg: self.executor.submit(self.sync_sub_callback, depth_msg, mask_msg))

        # 语义地图发布
        topic_semantic_object_pub = rospy.get_param(
            "~topic_semantic_object_pub", "/semantic_object"
        )
        self.semantic_object_pub = rospy.Publisher(
            topic_semantic_object_pub, SemanticObject, queue_size=10
        )

        rospy.loginfo("Semantic map generator distributed node initialization complete.")

    def sync_sub_callback(self, depth_repaired_msg, mask_info_msg):
        """接收到修复后的深度图像和掩码信息后进行点云生成"""
        rospy.loginfo("reveice depth_repaired and mask_info")
        # 获取当前处理图像的时间戳
        time_stamp = depth_repaired_msg.header.stamp

        # try:
        #     # 查询tf变换
        #     transform = self.tf_buffer.lookup_transform(
        #                 "map",
        #                 "camera_color_optical_frame",  # 使用光学坐标系
        #                 time_stamp,  # 使用图像时间戳
        #                 rospy.Duration(0.05),
        #             )
        # except Exception as e:
        #     rospy.logerr(f"Sync callback error: {str(e)}")
        #     return 

        # 获取tf变换
        transform = self.tf_cache.get(time_stamp.to_sec(), None)
        if transform is None:
            rospy.logwarn(f"tf transform for timestamp {time_stamp.to_sec()} not found.")
            return   # tf变换不存在，直接返回

        # 构造变换矩阵
        transform_matrix = self.transform_to_matrix(transform)

        # 获取原始深度
        depth_raw = self.depth_raw_cache.get(time_stamp.to_sec(), None)
        if depth_raw is None:
            rospy.logwarn(f"Depth data for timestamp {time_stamp.to_sec()} not found.")
            return   # 原始深度不存在，直接返回

        # 获取修复后的深度
        depth_repaired = self.bridge.imgmsg_to_cv2(depth_repaired_msg, desired_encoding='passthrough')
        # 获取掩码
        masks_stacked = self.bridge.imgmsg_to_cv2(mask_info_msg.segmasks, desired_encoding="passthrough")  # 保持原始格式
        # 掩码格式变换————根据掩码的维度数确认如何转换格式
        if masks_stacked.ndim == 3:  # 正常情况形状为 (H, W, N) 即有多张掩码  
            masks = np.moveaxis(masks_stacked, -1, 0)  # 从 (H, W, N) 变回 (N, H, W)
        else:  # 形状为 (H, W) 只有一张掩码
            masks = np.expand_dims(masks_stacked, axis=0)  # 变成 (1, H, W)

        # 获取每个物体的类别标签和置信度
        labels = mask_info_msg.labels
        scores = mask_info_msg.scores

        start_time = rospy.Time.now()

        # 初始化最终深度图，先填充为修复后的深度
        final_depth = np.copy(depth_repaired)
        
        # 对每张掩码进行处理
        for mask, label, score in zip(masks, labels, scores):
            # valid_mask = mask > 0  # 将掩码数据类型转化为bool数组

            # 对掩码进行腐蚀操作，减少物体边缘点云离群的概率
            kernel = np.ones((3, 3), np.uint8)  # 可以调整 kernel 大小来控制去噪的强度
            mask = cv2.erode(mask, kernel, iterations=4)  # 执行腐蚀操作

            # 对掩码部分进行二次深度修复，此时返回的是一维数组，仅包含mask部分的深度
            # second_depth_repaired = self.second_depth_repair(depth_raw.copy(), depth_repaired.copy(), valid_mask)
            second_depth_repaired_masked = self.second_depth_repair(depth_raw.copy(), depth_repaired.copy(), mask)

            # 生成单张掩码所对应的语义对象
            semantic_obj = self.create_semantic_object(
                mask,
                label,
                score,
                self.processing_image,
                second_depth_repaired_masked,
                self.camera_info,
                transform_matrix,
            )
            if semantic_obj is not None:
                self.semantic_object_pub.publish(semantic_obj)

        self.annotate_finished_event.clear()  # 设置事件为 False，供下一帧语义对象的生成使用

        end_time = rospy.Time.now()
        depth_repair_time = (end_time - start_time).to_sec()*1000
        rospy.loginfo(f"create semantic map time: {depth_repair_time:.1f} ms")
        print(" ")

    def create_semantic_object(
        self,
        mask,
        label,
        score,
        color_image,
        depth_masked,
        camera_info,
        transform_matrix,
    ):
        # 针对一张语义掩码生成语义点云
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        y, x = np.nonzero(mask)  # 获取掩码中每个点的x,y坐标
        z = depth_masked / 1000.0   # 获取掩码中每个点对应的深度，单位（mm -> m）

        # 进行坐标变换，得到世界坐标
        world_coordinates = self.pixel_to_world_batch(x, y, z, camera_info, transform_matrix)

        # 获取掩码中有效像素的坐标
        valid_mask = mask > 0  # 掩码部分，确保有效区域
        # 提取有效的 BGR 值
        bgr_values = color_image[valid_mask]  # 提取掩码区域的 BGR 值

        # 进行下采样
        if self.downsample_step > 1:  # 只有步长大于1才进行下采样
            sampled_indices = np.arange(0, world_coordinates.shape[1], self.downsample_step)  # 统一索引
            world_coordinates = world_coordinates[:, sampled_indices]  # 只对第二维下采样
            bgr_values = bgr_values[sampled_indices]  # 直接按索引下采样

        x_list = world_coordinates[0].tolist()
        y_list = world_coordinates[1].tolist()
        z_list = world_coordinates[2].tolist()
        points_cnt = len(x_list)

        # 使用 struct.pack 和 struct.unpack 转换为 RGB 格式
        rgb_list = []
        for b, g, r in bgr_values:
            rgb = struct.pack("BBBB", b, g, r, 0)  # 将 BGR 打包
            rgb_value = struct.unpack("<I", rgb)[0]  # 解包为 32 位整数
            rgb_list.append(rgb_value)

        # 等待图像标注完成才进行最终的语义对象生成
        self.annotate_finished_event.wait()

        # 创建SemanticObject消息
        semantic_obj = SemanticObject(
            category=label,
            count=points_cnt,
            x=x_list,
            y=y_list,
            z=z_list,
            rgb=rgb_list,
            confidence=score,
            image=self.bridge.cv2_to_imgmsg(self.annotated_image, "bgr8"),
        )

        rospy.loginfo(f"Generated {points_cnt} points for {label}")
        return semantic_obj


    def pixel_to_world_batch(self, u_coords, v_coords, z_coords, camera_info, transform_matrix):
        """
        批量将像素坐标 (u, v) 和深度值 z 转换到世界坐标系。
        """
        # 从CameraInfo获取内参
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]
        
        # 计算相机坐标系中的 x, y, z
        x_cam = (u_coords - cx) * z_coords / fx
        y_cam = (v_coords - cy) * z_coords / fy
        z_cam = z_coords

        # 将 x, y, z 组成齐次坐标 (x, y, z, 1) 对应每个像素
        point_cam = np.vstack([x_cam, y_cam, z_cam, np.ones_like(x_cam)])

        # 执行坐标系转换
        point_world = transform_matrix.dot(point_cam)

        # 返回世界坐标系中的 (x, y, z)
        return point_world[:3]

    def second_depth_repair(self, depth_raw, depth_repaired, mask, threshold=500, min_valid_points=50):
        # 1. 使用掩码选择有效区域的深度
        valid_mask = mask > 0  # 掩码部分，确保有效区域

        mask_depth_raw = depth_raw[valid_mask].astype(np.float32)  # 选择所有掩码为非零部分的原始深度
        mask_depth_repaired = depth_repaired[valid_mask].copy().astype(np.float32)  # 选择所有掩码为非零部分的修复深度
        
        # 2. 去掉原始深度为0的部分
        valid_depth_mask = mask_depth_raw > 0  # 筛选原始深度非零的部分
        valid_depth_raw = mask_depth_raw[valid_depth_mask]
        valid_depth_repaired = mask_depth_repaired[valid_depth_mask]  # 同步选择修复后的深度
        
        # 3. 去掉修复深度与原始深度差值绝对值超过500的部分
        diff = np.abs(valid_depth_raw - valid_depth_repaired)
        valid_depth_raw = valid_depth_raw[diff <= threshold]
        valid_depth_repaired = valid_depth_repaired[diff <= threshold]

        def inverse_depth_model(x, a, b):
            return a * x + b

        # 4. 如果剩下的有效点数量大于阈值，进行拟合
        if len(valid_depth_raw) >= min_valid_points:
            # # 使用线性拟合得到最贴合的修复深度
            # (a_opt, b_opt), _ = curve_fit(inverse_depth_model, valid_depth_repaired, valid_depth_raw, p0=[1, 0])
            # fitted_depth = a_opt * depth_repaired + b_opt

            # 使用 np.linalg.lstsq 进行线性拟合
            X_valid = valid_depth_repaired
            D_valid = valid_depth_raw
            # 构造设计矩阵 X_stack
            X_stack = np.vstack([X_valid, np.ones_like(X_valid)]).T  # X_valid 和常数项（1）构成设计矩阵
            # 求解最小二乘法
            params, residuals, rank, s = np.linalg.lstsq(X_stack, D_valid, rcond=None)
            # 从返回的参数中获取拟合参数 A 和 b
            A, b = params
            fitted_depth = A * (mask_depth_repaired) + b # 仅掩码部分的深度，拼接（x,y,z）数组，用于后续点云计算
        else:
            # 5. 如果有效点过少，直接使用首次修复的深度
            fitted_depth = mask_depth_repaired

        return fitted_depth


    def read_camera_info(self, camera_info_msg):
        # 只读取一次相机内参
        self.camera_info = camera_info_msg
        # 取消订阅
        self.camera_info_sub.unregister() 
        rospy.loginfo("camera_info saved, Unsubscribed from /camera/color/camera_info")

    def store_image(self, color_image_msg, depth_raw_msg, tf_msg):
        ## 在定时器发布时存取rgb图像和原始深度、tf变换
        time_stamp = color_image_msg.header.stamp.to_sec()
        color_image = self.bridge.imgmsg_to_cv2(color_image_msg, "bgr8")
        depth_raw = self.bridge.imgmsg_to_cv2(depth_raw_msg, "passthrough")
        self.color_image_cache[time_stamp] = color_image
        self.depth_raw_cache[time_stamp] = depth_raw
        self.tf_cache[time_stamp] = tf_msg
        # rospy.loginfo("receive color image and raw depth ")

    def annotate(self, annotation_info_msg):
        ## 进行图像标注
        # 获取消息中记录的原图像的时间戳，从而根据时间戳找到对应的原rgb图像
        start_time = rospy.Time.now()

        # self.processing_timestamp = annotation_info_msg.header.stamp
        time_stamp = annotation_info_msg.header.stamp
        self.processing_image = self.color_image_cache[time_stamp.to_sec()]

        class_id = np.array(annotation_info_msg.class_id)
        labels = annotation_info_msg.labels
        boxes = annotation_info_msg.boxes
        boxes = np.array(annotation_info_msg.boxes.data).reshape(
            (annotation_info_msg.boxes.layout.dim[0].size, annotation_info_msg.boxes.layout.dim[0].stride)
        )

        # annotate image with detections
        box_annotator = sv.BoxAnnotator()
        label_annotator = sv.LabelAnnotator()
        detections = sv.Detections(boxes, class_id=class_id)
        annotated_image = box_annotator.annotate(scene=self.processing_image.copy(), detections=detections)
        annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections, labels=labels)

        # 作为类的成员变量，供点云生成使用
        self.annotated_image = annotated_image
        self.annotate_finished_event.set()  # 设置标注已完成的事件，通知 点云对象生成 继续执行

        end_time = rospy.Time.now()
        time = (end_time - start_time).to_sec()*1000
        rospy.loginfo(f"annotate image time: {time:.1f} ms")

    @staticmethod
    def transform_to_matrix(transform):
        """将geometry_msgs/TransformStamped转换为4x4矩阵"""
        t = transform.transform.translation
        q = transform.transform.rotation

        # 构造旋转矩阵
        R = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])

        # 构造平移矩阵
        T = np.eye(4)
        T[:3, 3] = [t.x, t.y, t.z]

        # 组合变换矩阵
        return np.dot(T, R)
    
   
if __name__ == "__main__":
    try:
        node = SemanticMapGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
