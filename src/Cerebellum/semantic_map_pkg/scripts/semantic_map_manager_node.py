#!/usr/bin/env python
import rospy
import ast
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from semantic_map_database import SemanticMapDatabase
from semantic_map_pkg.msg import SemanticObject
from semantic_map_pkg.srv import Show, ShowResponse


class SemanticMapManager:
    def __init__(self):
        rospy.init_node("semantic_map_manager_node")

        # 数据库配置
        self.db_path = rospy.get_param("~db_path", "semantic_map.db")
        self.renew_db = rospy.get_param("~renew_db", False)
        self.last_seen_imgs_dir = rospy.get_param(
            "~last_seen_imgs_dir", "last_seen_imgs"
        )
        rospy.loginfo(f"setting semantic map dataset:")
        rospy.loginfo(f"* db path = {self.db_path}")
        rospy.loginfo(f"* last_seen_imgs_dir = {self.last_seen_imgs_dir}")
        if self.renew_db:
            rospy.logwarn(f"* renew_db = {self.renew_db}")
        else:
            rospy.loginfo(f"* renew_db = {self.renew_db}")
        self.database = SemanticMapDatabase(
            self.db_path, self.last_seen_imgs_dir, 3, self.renew_db
        )

        # ROS配置
        ## 语义对象订阅
        topic_semantic_object_sub = rospy.get_param(
            "~topic_semantic_object_sub", "/semantic_object"
        )
        self.sub = rospy.Subscriber(
            topic_semantic_object_sub, SemanticObject, self.semantic_object_callback
        )
        self.bridge = CvBridge()
        ## 语义地图发布器
        topic_semantic_map_pub = rospy.get_param(
            "~topic_semantic_map_pub", "/semantic_map"
        )
        self.pub = rospy.Publisher(topic_semantic_map_pub, PointCloud2, queue_size=10)
        ## 语义地图显示服务
        self.semantic_map_frame_id = rospy.get_param("~semantic_map_frame_id", "map")
        service_show_server = rospy.get_param("~service_show", "/semantic_map_show")
        self.service_show = rospy.Service(service_show_server, Show, self.handle_show)
        topic_bbox_markers = rospy.get_param(
            "~topic_bbox_markers", "/semantic_map_bbox"
        )
        self.bbox_pub = rospy.Publisher(topic_bbox_markers, MarkerArray, queue_size=10)
        ## BBox可视化参数
        bbox_colora_str = rospy.get_param("~bbox_colora", "[0.0, 1.0, 0.0, 0.5]")
        try:
            bbox_colora = ast.literal_eval(bbox_colora_str)
            # 验证数据类型和长度
            if not (
                isinstance(bbox_colora, list)
                and len(bbox_colora) == 4
                and all(isinstance(x, float) for x in bbox_colora)
            ):
                raise ValueError
        except (SyntaxError, ValueError) as e:
            rospy.logwarn(
                "Invalid bbox_color %s, using default [0.0, 1.0, 0.0, 0.5]. Error: %s",
                bbox_colora_str,
                e,
            )
            bbox_colora = [0.0, 1.0, 0.0, 0.5]  # 默认绿色， Alpha=0.5
        self.bbox_color = bbox_colora[:3]
        self.bbox_alpha = bbox_colora[3]
        self.bbox_line_width = rospy.get_param("~bbox_line_width", 0.01)
        self.show_bbox = rospy.get_param("~show_bbox", True)

        rospy.loginfo("semantic_map_manager_node initialized complete.")

    def update_db(
        self, category, bbox, count, x_list, y_list, z_list, rgb_list, cv_image
    ):
        """
        更新数据库:检测是否有冲突，再选择是合并还是新增
        params:
            category: 标签, eg: chair, 不是label_id, eg: chair@1
            bbox: 包围盒
            count: 语义对象的点云数量
            x_bin, y_bin, z_bin, rgb_bin: 二进制数据
            cv_image: OpenCV图像, last_seen_img
        """
        # 首先提取同类别的所有条目
        # entries = [{"label":str, "bbox": list(float), "time_stamp": str, "x": list(float), "y": list(float), "z": list(float), "rgb": list(int)}, ...]
        entries = self.database._get_entries_by_category(category)
        # 如果没有同类别的条目，直接插入
        if not entries:
            label = f"{category}@1"
            self.database._update_entry(label, bbox, x_list, y_list, z_list, rgb_list)
            self.database._save_last_seen_img(label, cv_image)
            return
        else:  # 如果有同类别的条目，用bbox去匹配每一个条目，看是否是同一个语义对象
            for entry in entries:
                label_entry = entry["label"]
                bbox_entry = entry["bbox"]
                if self.bbox_match(bbox, bbox_entry):
                    x_merged = x_list + entry["x"]
                    y_merged = y_list + entry["y"]
                    z_merged = z_list + entry["z"]
                    rgb_merged = rgb_list + entry["rgb"]
                    bbox_merged = self.bbox_merge(bbox, bbox_entry)
                    self.database._update_entry(
                        label_entry,
                        bbox_merged,
                        x_merged,
                        y_merged,
                        z_merged,
                        rgb_merged,
                    )
                    self.database._save_last_seen_img(label_entry, cv_image)
                    return
                else:
                    # 如果不匹配，检查下一个条目
                    continue
            # 如果所有条目都不匹配，插入新的语义对象
            existed_label_ids = [entry["label"].split("@")[1] for entry in entries]
            new_label_id = int(max(existed_label_ids)) + 1
            label = f"{category}@{new_label_id}"
            self.database._update_entry(label, bbox, x_list, y_list, z_list, rgb_list)
            self.database._save_last_seen_img(label, cv_image)
            return

    def bbox_merge(self, bbox1, bbox2):
        """合并两个bbox"""
        x_min1, y_min1, z_min1, x_max1, y_max1, z_max1 = bbox1
        x_min2, y_min2, z_min2, x_max2, y_max2, z_max2 = bbox2
        x_min = min(x_min1, x_min2)
        y_min = min(y_min1, y_min2)
        z_min = min(z_min1, z_min2)
        x_max = max(x_max1, x_max2)
        y_max = max(y_max1, y_max2)
        z_max = max(z_max1, z_max2)
        return [x_min, y_min, z_min, x_max, y_max, z_max]

    def bbox_match(self, bbox1, bbox2):
        """检查两个bbox是否匹配, 是否有交集"""
        x_min1, y_min1, z_min1, x_max1, y_max1, z_max1 = bbox1
        x_min2, y_min2, z_min2, x_max2, y_max2, z_max2 = bbox2
        if (
            x_min1 > x_max2
            or x_max1 < x_min2
            or y_min1 > y_max2
            or y_max1 < y_min2
            or z_min1 > z_max2
            or z_max1 < z_min2
        ):
            return False
        return True

    def bbox_calc(self, x_list, y_list, z_list):
        """计算AABB"""
        x_min = min(x_list)
        y_min = min(y_list)
        z_min = min(z_list)
        x_max = max(x_list)
        y_max = max(y_list)
        z_max = max(z_list)
        return [x_min, y_min, z_min, x_max, y_max, z_max]

    def semantic_object_callback(self, msg):
        """增加数据有效性检查的写入方法"""
        try:
            # 验证数据长度是否有效, 以及一致性
            if msg.count == 0:
                rospy.logwarn(f"Received Empty data for {msg.category}")
                return
            if not (
                len(msg.x) == len(msg.y) == len(msg.z) == len(msg.rgb) == msg.count
            ):
                rospy.logerr(
                    f"Inconsistent data length, len(x): {len(msg.x)}, len(y): {len(msg.y)}, len(z): {len(msg.z)}, len(rgb): {len(msg.rgb)}, count: {msg.count}"
                )
                return
            # 获取msg
            try:
                category = msg.category
                count = msg.count
                confidence = msg.confidence
                x_list = list(msg.x)
                y_list = list(msg.y)
                z_list = list(msg.z)
                rgb_list = list(msg.rgb)
                cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding="bgr8")
            except Exception as e:
                rospy.logerr(f"Message unpack error: {str(e)}")
                return
            # 计算bbox， AABB
            bbox = self.bbox_calc(x_list, y_list, z_list)
            # 更新语义对象到数据库
            self.update_db(
                category, bbox, count, x_list, y_list, z_list, rgb_list, cv_image
            )

        except Exception as e:
            rospy.logerr(f"Cloud callback error: {str(e)}")

    def handle_show(self, req):
        data = req.data
        # 对data进行区分,是要show全部，还是只要show特定label，还是show特定category
        show_type = None
        if data == "all":
            show_type = "all"
            self.pub_all()
        elif "@" in data:
            show_type = "label"
            self.pub_label(data)
        else:
            show_type = "category"
            self.pub_category(data)
        res = ShowResponse()
        res.success = True
        res.message = (
            f"Show {show_type} success " + "with bbox"
            if self.show_bbox
            else "without bbox"
        )
        return res

    def entry_to_points(self, entry: dict) -> list:
        # entry = {"label":str, "bbox": list(float), "time_stamp": str, "x": list(float), "y": list(float), "z": list(float), "rgb": list(int)}
        x_list = entry["x"]
        y_list = entry["y"]
        z_list = entry["z"]
        rgb_list = entry["rgb"]
        points = []
        for x, y, z, rgb in zip(x_list, y_list, z_list, rgb_list):
            points.append([x, y, z, rgb])
        return points

    def pub_semantic_map(self, points: list):
        try:
            # 转换为ROS消息
            header = Header(stamp=rospy.Time.now(), frame_id=self.semantic_map_frame_id)
            fields = [
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
                PointField("rgb", 12, PointField.UINT32, 1),
            ]

            # 定义结构化数据类型
            point_dtype = [
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("rgb", np.uint32),
            ]

            # 转换为结构化数组
            structured_array = np.array(
                [(p[0], p[1], p[2], p[3]) for p in points], dtype=point_dtype
            )

            # 直接使用数组的tobytes()方法
            packed_data = structured_array.tobytes()

            cloud = PointCloud2(
                header=header,
                height=1,
                width=len(structured_array),
                is_dense=True,
                is_bigendian=False,
                fields=fields,
                point_step=16,
                row_step=16 * len(structured_array),
                data=packed_data,
            )

            self.pub.publish(cloud)

        except Exception as e:
            rospy.logerr(f"Publishing failed: {str(e)}")

    def pub_all(self):
        try:
            # 获取所有entry
            # all_entries = [{"label":str, "bbox": list(float), "time_stamp": str, "x": list(float), "y": list(float), "z": list(float), "rgb": list(int)}, ...]
            all_entries = self.database._get_all_entries()
            if not all_entries:
                rospy.logwarn("No entries to publish")
                return
            # 从entry中提取x, y, z, rgb, 并合并所有点云
            all_points = []
            for entry in all_entries:
                points = self.entry_to_points(entry)
                all_points.extend(points)
            if not all_points:
                rospy.logwarn("No points to publish")
                return
            # 发布点云
            self.pub_semantic_map(all_points)
        except Exception as e:
            rospy.logerr(f"Publish all error: {str(e)}")

        if self.show_bbox:
            self.pub_bbox_markers(all_entries)

    def pub_label(self, label):
        try:
            # 获取指定label的entry
            # entry = {"label":str, "bbox": list(float), "time_stamp": str, "x": list(float), "y": list(float), "z": list(float), "rgb": list(int)}
            entry = self.database._get_entry_by_label(label)
            if not entry:
                rospy.logwarn(f"No entry with label {label}")
                return
            # 从entry中提取x, y, z, rgb
            points = self.entry_to_points(entry)
            if not points:
                rospy.logwarn(f"No points to publish for label {label}")
                return
            # 发布点云
            self.pub_semantic_map(points)
        except Exception as e:
            rospy.logerr(f"Publish label error: {str(e)}")

        if self.show_bbox:
            self.pub_bbox_markers(entry)

    def pub_category(self, category):
        try:
            # 获取指定category的所有entry
            # category_entries = [{"label":str, "bbox": list(float), "time_stamp": str, "x": list(float), "y": list(float), "z": list(float), "rgb": list(int)}, ...]
            category_entries = self.database._get_entries_by_category(category)
            if not category_entries:
                rospy.logwarn(f"No entries with category {category}")
                return
            # 从entry中提取x, y, z, rgb, 并合并所有点云
            all_points = []
            for entry in category_entries:
                points = self.entry_to_points(entry)
                all_points.extend(points)
            if not all_points:
                rospy.logwarn(f"No points to publish for category {category}")
                return
            # 发布点云
            self.pub_semantic_map(all_points)
        except Exception as e:
            rospy.logerr(f"Publish category error: {str(e)}")

        if self.show_bbox:
            self.pub_bbox_markers(category_entries)

    def create_bbox_marker(self, bbox, label):
        """
        根据AABB创建线框Marker
        :param bbox: [x_min, y_min, z_min, x_max, y_max, z_max]
        :param label: 对象标签（用于命名空间）
        :return: Marker对象
        """
        marker = Marker()
        marker.header.frame_id = self.semantic_map_frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = label
        marker.id = 0  # 同一个ns下保持唯一即可
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0)  # 永久显示

        # 设置尺寸和颜色
        marker.scale.x = self.bbox_line_width
        marker.color.r = self.bbox_color[0]
        marker.color.g = self.bbox_color[1]
        marker.color.b = self.bbox_color[2]
        marker.color.a = self.bbox_alpha

        # 计算立方体的8个顶点
        x_min, y_min, z_min, x_max, y_max, z_max = bbox
        points = [
            [x_min, y_min, z_min],
            [x_max, y_min, z_min],
            [x_max, y_max, z_min],
            [x_min, y_max, z_min],
            [x_min, y_min, z_max],
            [x_max, y_min, z_max],
            [x_max, y_max, z_max],
            [x_min, y_max, z_max],
        ]

        # 定义连接线段的顶点索引对
        lines = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),  # 底面
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),  # 顶面
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),  # 侧面
        ]

        # 添加线段到Marker
        for i, j in lines:
            marker.points.append(Point(*points[i]))
            marker.points.append(Point(*points[j]))

        return marker

    def pub_bbox_markers(self, entries):
        """
        删除旧的并发布新的包围盒Marker
        :param entries: 语义对象条目列表
        """
        # 删除旧的包围盒Marker
        self.del_all_bbox_markers()
        # 发布新的包围盒Marker
        markers = MarkerArray()
        if type(entries) is not list:
            entries = [entries]
        for entry in entries:
            label = entry["label"]
            bbox = entry["bbox"]
            marker = self.create_bbox_marker(bbox, label)
            markers.markers.append(marker)
        self.bbox_pub.publish(markers)
        rospy.loginfo(f"Published {len(markers.markers)} bounding box markers")

    def del_all_bbox_markers(self):
        """
        删除所有包围盒Marker
        """
        # 创建删除指令
        delete_marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = self.semantic_map_frame_id
        delete_marker.action = Marker.DELETEALL
        delete_marker_array.markers.append(delete_marker)

        # 发布删除指令
        self.bbox_pub.publish(delete_marker_array)


if __name__ == "__main__":
    manager = SemanticMapManager()
    rospy.spin()
