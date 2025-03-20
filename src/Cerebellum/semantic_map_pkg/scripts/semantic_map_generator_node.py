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
from sklearn.neighbors import KDTree
from sklearn.cluster import DBSCAN
from semantic_map_pkg.msg import SemanticObject
from grounding_sam_ros.srv import (
    VitDetection,
    VitDetectionResponse,
    UpdatePrompt,
    UpdatePromptResponse,
)


class SemanticMapGenerator:
    def __init__(self):
        rospy.init_node("semantic_map_generator")

        # 初始化参数
        self.current_prompt = rospy.get_param(
            "~default_prompt",
            "keyboard. mouse. cellphone. earphone. laptop. computer. water bottle. plant. keys. door. chair. ",
        )
        self.downsample_step = rospy.get_param("~downsample_step", 2)
        self.bridge = CvBridge()
        self.data_lock = threading.Lock()
        self.latest_image = None
        self.latest_depth = None
        self.camera_info = None
        self.latest_transform = None

        # 图像订阅
        topic_image_sub = rospy.get_param("~topic_image_sub", "/camera/color/image_raw")
        image_sub = Subscriber(topic_image_sub, Image)
        topic_depth_sub = rospy.get_param(
            "~topic_depth_sub", "/camera/aligned_depth_to_color/image_raw"
        )
        depth_sub = Subscriber(topic_depth_sub, Image)

        # 相机内外参订阅, 深度图对齐
        topic_camera_info_sub = rospy.get_param(
            "~topic_camera_info_sub", "/camera/color/camera_info"
        )
        camera_info_sub = Subscriber(topic_camera_info_sub, CameraInfo)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # 同步订阅
        ts = ApproximateTimeSynchronizer(
            [image_sub, depth_sub, camera_info_sub], queue_size=5, slop=0.05
        )
        ts.registerCallback(self.sync_sub_callback)

        # 标注图像发布
        topic_annotated_pub = rospy.get_param(
            "~topic_annotated_pub", "/semantic_annotated"
        )
        self.annotated_pub = rospy.Publisher(topic_annotated_pub, Image, queue_size=1)
        topic_masks_pub = rospy.get_param("~topic_masks_pub", "/semantic_masks")
        self.masks_pub = rospy.Publisher(topic_masks_pub, Image, queue_size=1)

        # 初始化语义对象检测器, 客户端
        service_semantic_object_detector = rospy.get_param(
            "~service_semantic_object_detector", "/vit_detection"
        )
        rospy.wait_for_service(service_semantic_object_detector)
        self.detector = rospy.ServiceProxy(
            service_semantic_object_detector, VitDetection
        )
        rospy.loginfo("Semantic object detector service is ready")

        # 设置是否进行点云滤波，默认为True
        self.enable_filtering = rospy.get_param("~enable_filtering", True)

        # 语义地图发布
        topic_semantic_object_pub = rospy.get_param(
            "~topic_semantic_object_pub", "/semantic_object"
        )
        self.semantic_object_pub = rospy.Publisher(
            topic_semantic_object_pub, SemanticObject, queue_size=10
        )

        # Prompt更新服务
        service_update_prompt = rospy.get_param(
            "~service_update_prompt", "/update_prompt"
        )
        rospy.Service(service_update_prompt, UpdatePrompt, self.prompt_callback)

        # 定时器
        rospy.Timer(rospy.Duration(1), self.timer_callback)

        rospy.loginfo("semantic_map_generator_node initialization complete.")

    def sync_sub_callback(self, img_msg, depth_msg, camera_info_msg):
        """同步订阅回调"""
        try:
            with self.data_lock:

                # 查询时间同步的TF变换
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    "camera_color_optical_frame",  # 使用光学坐标系
                    img_msg.header.stamp,  # 使用图像时间戳
                    rospy.Duration(0.1),
                )
                self.latest_transform = transform
                self.latest_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                self.camera_info = camera_info_msg
        except Exception as e:
            rospy.logerr(f"Sync callback error: {str(e)}")
            self.latest_image = None
            self.latest_depth = None
            self.camera_info = None
            self.latest_transform = None

    def prompt_callback(self, req):
        """Prompt更新服务"""
        original_prompt = self.current_prompt  # 保留原值用于异常恢复

        try:
            # CASE 1: 直接赋值操作 =
            if req.data.startswith("="):
                new_prompt = req.data[1:].strip()  # 移除=号和首尾空格
                self.current_prompt = new_prompt
                rospy.loginfo(f"[Prompt SET] => {self.current_prompt}")
                return UpdatePromptResponse(
                    success=True, message=f"Prompt SET: {self.current_prompt}"
                )

            # CASE 2: 追加操作 +
            elif req.data.startswith("+"):
                append_str = req.data[1:].strip()  # 移除+号和首尾空格

                # 空内容检查
                if not append_str:
                    rospy.logwarn("Empty append content")
                    return UpdatePromptResponse(
                        success=False, message="Append content cannot be empty"
                    )

                # 智能空格拼接
                if self.current_prompt:
                    if not self.current_prompt.endswith(" "):
                        append_str = " " + append_str
                    self.current_prompt += append_str
                else:
                    self.current_prompt = append_str

                rospy.loginfo(f"[Prompt APPEND] => {self.current_prompt}")
                return UpdatePromptResponse(
                    success=True,
                    message=f"Appended: {append_str}, current prompt: {self.current_prompt}",
                )

            # CASE 3: 非法操作
            else:
                rospy.logwarn(f"Illegal syntax: {req.data}")
                return UpdatePromptResponse(
                    success=False,
                    message="Invalid syntax. Start with '=' to SET or '+' to APPEND.\n"
                    "Example:\n"
                    "  = chair. table.\n"
                    "  + lamp. book.",
                )

        except Exception as e:
            # 异常恢复机制
            self.current_prompt = original_prompt
            rospy.logerr(f"Prompt update failed: {str(e)}")
            return UpdatePromptResponse(
                success=False, message=f"Critical error: {str(e)}"
            )

    def maskmsg_to_array(self, mask_msg, mask_shape):
        masks = np.array(mask_msg.data).reshape(
            (mask_msg.layout.dim[0].size, mask_msg.layout.dim[0].stride)
        )
        masks = masks.reshape((masks.shape[0], mask_shape[0], mask_shape[1])).astype(
            np.uint8
        )
        return masks

    def semantic_object_detect(self, rgb, prompt):
        rgb_msg = self.bridge.cv2_to_imgmsg(np.array(rgb))
        results = self.detector(rgb_msg, prompt)
        labels = results.labels
        scores = results.scores
        boxes = results.boxes
        masks = self.maskmsg_to_array(results.segmasks, rgb.shape[:2])
        boxes = np.array(results.boxes.data).reshape(
            (results.boxes.layout.dim[0].size, results.boxes.layout.dim[0].stride)
        )
        annotated_frame = self.bridge.imgmsg_to_cv2(results.annotated_frame)
        return annotated_frame, boxes, masks, labels, scores

    @staticmethod
    def apply_mask_overlay(image, masks):
        """将掩码以半透明固定颜色叠加到图像上"""
        overlay = image.copy()

        # 定义固定的颜色列表（RGB格式）
        fixed_colors = [
            [255, 0, 0],  # 红色
            [0, 255, 0],  # 绿色
            [0, 0, 255],  # 蓝色
            [255, 255, 0],  # 黄色
            [255, 0, 255],  # 紫色
            [0, 255, 255],  # 青色
            [128, 0, 0],  # 深红
            [0, 128, 0],  # 深绿
            [0, 0, 128],  # 深蓝
            [128, 128, 0],  # 橄榄色
        ]

        for i, mask in enumerate(masks):
            # 使用模运算循环选择颜色
            color = fixed_colors[i % len(fixed_colors)]

            # 将二值掩码转换为bool类型
            binary_mask = mask.astype(bool)

            # 创建颜色掩码
            color_mask = np.zeros_like(image)
            color_mask[binary_mask] = color

            # 使用cv2.addWeighted进行叠加
            alpha = 0.15  # 透明度
            cv2.addWeighted(color_mask, alpha, overlay, 1 - alpha, 0, overlay)

            # 绘制轮廓加强显示
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(overlay, contours, -1, color, 2)

        return overlay

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

    def point_clouds_filter(self, x: list, y: list, z: list, rgb: list):
        # 根据rosparam确定要不要进行滤波
        if not self.enable_filtering:
            return x, y, z, rgb  # 直接返回不做处理

        # 1. 组织点云数据
        points = np.array([x, y, z]).T

        # 2. KDTree 最近邻距离计算
        kdtree = KDTree(points)
        distances, _ = kdtree.query(points, k=5)  # 计算每个点的最近邻距离
        mean_distances = np.mean(distances, axis=1)
        threshold = np.percentile(mean_distances, 95)  # 设定阈值（如95%分位数）
        filtered_indices = mean_distances < threshold
        points = points[filtered_indices]
        rgb = np.array(rgb)[filtered_indices].tolist()

        # 3. DBSCAN 聚类
        dbscan = DBSCAN(eps=0.05, min_samples=10)  # 可调整超参数
        labels = dbscan.fit_predict(points)

        unique_labels = set(labels) - {-1}  # 去除噪声点
        if len(unique_labels) == 0:
            rospy.logwarn("No valid clusters found.")
            return [], [], [], []

        # 4. 计算每个聚类的中心坐标和点数
        cluster_sizes = {}
        cluster_centers = {}
        for label in unique_labels:
            cluster_points = points[labels == label]
            cluster_sizes[label] = len(cluster_points)
            cluster_centers[label] = np.mean(cluster_points, axis=0)  # 计算质心

        # 5. 计算聚类之间的最小距离
        cluster_labels = list(cluster_centers.keys())
        cluster_distances = np.zeros((len(cluster_labels), len(cluster_labels)))

        for i in range(len(cluster_labels)):
            for j in range(i + 1, len(cluster_labels)):
                dist = np.linalg.norm(cluster_centers[cluster_labels[i]] - cluster_centers[cluster_labels[j]])
                cluster_distances[i, j] = dist
                cluster_distances[j, i] = dist

        # 6. 检查是否有多个聚类中心且最近距离 > 0.3m
        if len(cluster_labels) > 1 and np.min(cluster_distances[np.nonzero(cluster_distances)]) > 0.3:
            rospy.logwarn("Multiple clusters detected with large separation. Keeping the largest one.")

            # 选择最大的聚类
            largest_cluster_label = max(cluster_sizes, key=cluster_sizes.get)
            valid_indices = labels == largest_cluster_label
        else:
            valid_indices = labels != -1  # 保留所有有效点（非噪声）

        # 7. 过滤点云数据
        filtered_x = points[valid_indices, 0].tolist()
        filtered_y = points[valid_indices, 1].tolist()
        filtered_z = points[valid_indices, 2].tolist()
        filtered_rgb = np.array(rgb)[valid_indices].tolist()

        rospy.loginfo(f"Point cloud filtered: {len(filtered_x)} points remain.")
        return filtered_x, filtered_y, filtered_z, filtered_rgb

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

        # 点云聚类与剔除优化
        x_list, y_list, z_list, rgb_list = self.point_clouds_filter(
            x_list, y_list, z_list, rgb_list
        )
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

    def timer_callback(self, event):
        """定时检测回调"""
        with self.data_lock:
            if self.latest_image is None or self.latest_depth is None:
                return
            # 深拷贝当前数据
            image_snapshot = self.latest_image.copy()
            depth_snapshot = self.latest_depth.copy()
            tf_snapshot = self.latest_transform
            camera_info_snapshot = self.camera_info

        try:
            # 执行检测
            annotated, boxes, masks, labels, scores = self.semantic_object_detect(
                image_snapshot, self.current_prompt
            )

            # 发布标注结果
            self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))

            # 生成掩码叠加图像
            if len(masks) > 0:
                mask_overlay = self.apply_mask_overlay(annotated, masks)
                self.masks_pub.publish(self.bridge.cv2_to_imgmsg(mask_overlay, "bgr8"))

                # 生成当前帧语义对象
                for mask, label, score in zip(masks, labels, scores):
                    semantic_obj = self.create_semantic_object(
                        mask,
                        label,
                        score,
                        annotated,
                        image_snapshot,
                        depth_snapshot,
                        camera_info_snapshot,
                        tf_snapshot,
                    )
                    if semantic_obj is not None:
                        self.semantic_object_pub.publish(semantic_obj)

        except rospy.ServiceException as e:
            rospy.logerr(f"Detection failed: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Processing error: {str(e)}")

        self.latest_image = None
        self.latest_depth = None


if __name__ == "__main__":
    try:
        node = SemanticMapGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
