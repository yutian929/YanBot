#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
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



class SemanticMapGenerator:
    def __init__(self):
        rospy.init_node("semantic_map_generator")

        self.camera_info = None
        self.image_cache = {}

        self.processing_timestamp = None
        self.processing_image = None

        self.bridge = CvBridge()

        # 从annotate传递给点云生成
        self.annotated_image = None

        # 下采样步长
        self.downsample_step = rospy.get_param("~downsample_step", 2)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # 多线程运行
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)  # 创建 4 个线程

        # 订阅相机内外参（读取一次即可）
        topic_camera_info_sub = rospy.get_param(
            "~topic_camera_info_sub", "/camera/color/camera_info"
        )
        self.camera_info_sub = rospy.Subscriber(topic_camera_info_sub, CameraInfo, self.read_camera_info,queue_size=2)

        ## 订阅rgb图像（在定时器发布时就存取）
        # self.image_sub = rospy.Subscriber("color_to_process", Image, self.store_image, queue_size=1)
        self.image_sub = rospy.Subscriber("color_to_process", Image, lambda msg: self.executor.submit(self.store_image, msg), queue_size=1)

        # 订阅图像标注信息
        # self.mask_info_sub = rospy.Subscriber("annotation_info", AnnotationInfo, self.annotate, queue_size=2)
        self.mask_info_sub = rospy.Subscriber("annotation_info", AnnotationInfo, lambda msg: self.executor.submit(self.annotate, msg), queue_size=2)

        ## 点云生成的回调
        # 订阅修复后的深度图像
        # depth_repaired_sub = rospy.Subscriber("depth_repaired", Image, self.depth_repaired_show, queue_size=2)
        depth_repaired_sub = Subscriber("depth_repaired", Image, queue_size=2)
        
        # 订阅掩码信息
        # self.mask_info_sub = rospy.Subscriber("mask_info", MaskInfo, self.mask_show, queue_size=2)
        mask_info_sub = Subscriber("mask_info", MaskInfo, queue_size=2)
        
        # 同步回调
        ts = ApproximateTimeSynchronizer(
            [depth_repaired_sub, mask_info_sub], queue_size=2, slop=0.005
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


        rospy.loginfo("semantic_map_generator_distributed_node initialization complete.")



    def depth_repaired_show(self, depth_repaired_msg):
        depth = self.bridge.imgmsg_to_cv2(depth_repaired_msg, "passthrough")
        # 可视化深度图像
        vis_depth = cv2.convertScaleAbs(depth, alpha=0.1)
        cv2.imshow("vis_depth",vis_depth)
        cv2.waitKey(1)


    def sync_sub_callback(self, depth_repaired_msg, mask_info_msg):
        """接收到修复后的深度图像和掩码信息后进行点云生成"""
        rospy.loginfo("reveice depth_repaired and mask_info")

        # 先查询tf变换
        transform = self.tf_buffer.lookup_transform(
                    "map",
                    "camera_color_optical_frame",  # 使用光学坐标系
                    self.processing_timestamp,  # 使用图像时间戳
                    rospy.Duration(0.005),
                )
        
        # 获取修复后的深度
        depth_repaired = self.bridge.imgmsg_to_cv2(depth_repaired_msg, desired_encoding='passthrough')

        # 获取掩码
        masks_stacked = self.bridge.imgmsg_to_cv2(mask_info_msg.segmasks, desired_encoding="passthrough")  # 保持原始格式
        print(masks_stacked.shape)
        # 掩码格式变换
        # 根据掩码的维度数确认如何转换格式
        if masks_stacked.ndim == 3:  # 正常情况形状为 (H, W, N) 即有多张掩码  
            masks = np.moveaxis(masks_stacked, -1, 0)  # 从 (H, W, N) 变回 (N, H, W)
        else:  # 形状为 (H, W) 只有一张掩码
            masks = np.expand_dims(masks_stacked, axis=0)  # 变成 (1, H, W)

        # masks_stacked = self.bridge.imgmsg_to_cv2(mask_info_msg.segmasks, desired_encoding="passthrough")  # 保持原始格式
        # print(masks_stacked.shape)
        # masks = np.moveaxis(masks_stacked, -1, 0)  # 从 (H, W, N) 变回 (N, H, W)
        print(masks.shape)

        labels = mask_info_msg.labels
        scores = mask_info_msg.scores

        # 生成当前帧语义对象
        for mask, label, score in zip(masks, labels, scores):
            semantic_obj = self.create_semantic_object(
                mask,
                label,
                score,
                self.annotated_image,
                self.processing_image,
                depth_repaired,
                self.camera_info,
                transform,
            )
            if semantic_obj is not None:
                self.semantic_object_pub.publish(semantic_obj)

        if self.processing_timestamp.to_sec() in self.image_cache:
            rospy.loginfo("del processed image")
            del self.image_cache[self.processing_timestamp.to_sec()]


    def read_camera_info(self, camera_info_msg):
        # 只读取一次相机内参
        self.camera_info = camera_info_msg
        # 取消订阅
        self.camera_info_sub.unregister() 
        rospy.loginfo("Unsubscribed from /camera/color/camera_info")

    def store_image(self, image_msg):
        time_stamp = image_msg.header.stamp.to_sec()
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        self.image_cache[time_stamp] = image
        rospy.loginfo("receive rgb image")

    def annotate(self, annotation_info_msg):
        # rospy.loginfo("annotate image start")

        # # 测试消息发布到接收的时间
        # stored_time = rospy.get_param("/current_time")
        # now_time = rospy.Time.now().to_sec()
        # message_time = (now_time - stored_time)*1000
        # print("message transmission time (ms):", message_time)

        # 获取消息中记录的原图像的时间戳，从而根据时间戳找到对应的原rgb图像
        self.processing_timestamp = annotation_info_msg.header.stamp
        self.processing_image = self.image_cache[self.processing_timestamp.to_sec()]

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

        rospy.loginfo("annotate image")

        # # 测试多线程
        # time.sleep(10)
        # rospy.loginfo("complete sleep 10s")

        # # 显示标注结果
        # cv2.imshow("Annotated Image", annotated_image)  
        # cv2.waitKey(1)  # 等待按键，按任意键关闭

        # if self.processing_timestamp in self.image_cache:
        #     rospy.loginfo("del processed image")
        #     del self.image_cache[self.processing_timestamp]



    def mask_show(self, mask_info_msg):
        masks_stacked = self.bridge.imgmsg_to_cv2(mask_info_msg.segmasks, desired_encoding="passthrough")  # 保持原始格式
        # 测试消息发布到接收的时间
        stored_time = rospy.get_param("/current_time")
        now_time = rospy.Time.now().to_sec()
        message_time = (now_time - stored_time)*1000
        print("message transmission time (ms):", message_time)
        masks = np.moveaxis(masks_stacked, -1, 0)  # 从 (H, W, N) 变回 (N, H, W)
        print(masks_stacked.shape)

        # 显示掩码
        cv2.imshow("Mask", masks[0])
        cv2.waitKey(1)
        
    def create_semantic_object(
        self,
        mask,
        label,
        score,
        anonated,
        latest_image,
        latest_depth,
        camera_info,
        latest_transform,
    ):
        # 针对一张语义掩码生成语义点云
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        x_list = []
        y_list = []
        z_list = []
        rgb_list = []
        print(mask.shape)
        height, width = mask.shape

        if latest_transform is None:
            rospy.logwarn("No valid TF transform available")
            return None

        h, w = latest_depth.shape
        for v in range(0, height, self.downsample_step):  # 针对掩码中的每个像素
            for u in range(0, width, self.downsample_step):
                if mask[v, u] > 0:
                    z = latest_depth[v, u]  # mm
                    point = self.pixel_to_world(
                        u, v, z, camera_info, latest_transform
                    )  # m, 针对每个世界点
                    if point is not None:
                        if 0 <= v < h and 0 <= u < w:
                            b, g, r = latest_image[v, u]  # 提取BGR
                        else:
                            r, g, b = 255, 255, 255  # 默认白色
                        # 将RGB打包成UINT32（格式：0x00RRGGBB）
                        rgb = struct.pack("BBBB", b, g, r, 0)
                        rgb_value = struct.unpack("<I", rgb)[0]
                        x_list.append(point[0])
                        y_list.append(point[1])
                        z_list.append(point[2])
                        rgb_list.append(rgb_value)

        # # 点云聚类与剔除优化
        # x_list, y_list, z_list, rgb_list = self.point_clouds_filter(
        #     x_list, y_list, z_list, rgb_list
        # )
        points_cnt = len(x_list)

        # 创建SemanticObject消息
        semantic_obj = SemanticObject(
            category=label,
            count=points_cnt,
            x=x_list,
            y=y_list,
            z=z_list,
            rgb=rgb_list,
            confidence=score,
            image=self.bridge.cv2_to_imgmsg(anonated, "bgr8"),
        )

        rospy.loginfo(f"Generated {points_cnt} points for {label}")
        return semantic_obj
    
    def pixel_to_world(self, u, v, z, camera_info, latest_transform):
        """将像素坐标转换为世界坐标"""
        # 相机坐标系

        if z <= 0:  # 无效深度
            return None
        else:
            z = z / 1000.0  # mm -> m

        # 从CameraInfo获取内参
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]

        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        z_cam = z

        # 构造齐次坐标
        point_cam = np.array([x_cam, y_cam, z_cam, 1.0])

        # 从TF变换获取转换矩阵
        T = self.transform_to_matrix(latest_transform)

        # 坐标系转换
        point_world = T.dot(point_cam)
        return point_world[:3]
    

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
    

    # def sync_sub_callback(self, img_msg, depth_msg):
    #     """接收到最终的rgb图像和修复后的深度图像后进行点云生成"""
    #     # 先查询tf变换
    #     transform = self.tf_buffer.lookup_transform(
    #                 "map",
    #                 "camera_color_optical_frame",  # 使用光学坐标系
    #                 img_msg.header.stamp,  # 使用图像时间戳
    #                 rospy.Duration(0.05),
    #             )
        
    #     # 生成当前帧语义对象
    #     for mask, label, score in zip(masks, labels, scores):
    #         semantic_obj = self.create_semantic_object(
    #             mask,
    #             label,
    #             score,
    #             annotated,
    #             image_snapshot,
    #             depth_snapshot,
    #             camera_info_snapshot,
    #             tf_snapshot,
    #         )
    #         if semantic_obj is not None:
    #             self.semantic_object_pub.publish(semantic_obj)
        
    
   
if __name__ == "__main__":
    try:
        node = SemanticMapGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


