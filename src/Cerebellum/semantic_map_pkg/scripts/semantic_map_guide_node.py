#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from semantic_map_database import SemanticMapDatabase
from tf2_ros import Buffer, TransformListener
import time
from semantic_map_CLIPMatcher import CLIPMatcher


class SemanticMapGuide:
    def __init__(self):
        rospy.init_node("semantic_map_guide_node")

        # 数据库配置
        self.db_path = rospy.get_param("~db_path", "semantic_map.db")
        self.last_seen_imgs_dir = rospy.get_param(
            "~last_seen_imgs_dir", "last_seen_imgs"
        )
        self.database = SemanticMapDatabase(
            self.db_path, self.last_seen_imgs_dir, renew_db=False
        )
        # CLIP匹配器
        self.clip_matcher = CLIPMatcher("ViT-B/16")
        # ROS配置
        ## opencv
        self.bridge = CvBridge()
        ## 订阅机器人基座位姿
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.base_link_father = rospy.get_param("~base_link_father", "odom")
        self.base_link_child = rospy.get_param("~base_link_child", "camera_link")

        rospy.loginfo("Semantic map guide node initialized complete.")
        self.debug()

    def debug(self):
        # 测试
        while not rospy.is_shutdown():
            time.sleep(1)
            nearest_object = self.find_semantic_object_by_category("mouse")
            if nearest_object:
                rospy.loginfo(f"Nearest object: {nearest_object}")

    def get_base_pose(self):
        """
        监听tf，获取机器人基座位
        返回(x, y, z, qx, qy, qz, qw)
        """
        try:
            # 获取机器人基座位
            base_pose = self.tf_buffer.lookup_transform(
                self.base_link_father, self.base_link_child, rospy.Time(0)
            )
            x = base_pose.transform.translation.x
            y = base_pose.transform.translation.y
            z = base_pose.transform.translation.z
            qx = base_pose.transform.rotation.x
            qy = base_pose.transform.rotation.y
            qz = base_pose.transform.rotation.z
            qw = base_pose.transform.rotation.w
            return [x, y, z, qx, qy, qz, qw]
        except Exception as e:
            rospy.logerr(f"Get base pose error: {str(e)}")
            return None

    def get_distance(self, base_pose, semantic_obj):
        # base_pose: [x, y, z, qx, qy, qz, qw]
        base_x, base_y, base_z, _, _, _, _ = base_pose
        # [x_min, y_min, z_min, x_max, y_max, z_max]
        semantic_obj_bbox = semantic_obj["bbox"]
        obj_x = (semantic_obj_bbox[0] + semantic_obj_bbox[3]) / 2
        obj_y = (semantic_obj_bbox[1] + semantic_obj_bbox[4]) / 2
        obj_z = (semantic_obj_bbox[2] + semantic_obj_bbox[5]) / 2
        distance = (
            (base_x - obj_x) ** 2 + (base_y - obj_y) ** 2 + (base_z - obj_z) ** 2
        ) ** 0.5
        return distance

    def find_semantic_object_by_category(self, category):
        """
        通过类别查找语义对象
        返回距离最近的那个语义对象
        """
        # # semantic_object = {"label":str, "bbox": list(float), "time_stamp": str, "x": list(float), "y": list(float), "z": list(float), "rgb": list(int)}
        semantic_objects = self.database._get_entries_by_category(category)
        if not semantic_objects:
            rospy.logwarn(f"No semantic object found by category: {category}")
            return None
        base_pose = self.get_base_pose()
        if not base_pose:
            rospy.logwarn("No base pose found.")
            return None
        min_distance = float("inf")
        nearest_object = None
        for obj in semantic_objects:
            distance = self.get_distance(base_pose, obj)
            if distance < min_distance:
                min_distance = distance
                nearest_object = obj
        return nearest_object["label"]

    def lang_imgs_match(self, lang, img_paths):
        """
        语言描述和图片匹配
        返回匹配度
        """
        if type(img_paths) is not list:
            img_paths = [img_paths]
        match = self.clip_matcher.match_batch(lang, img_paths)
        return match

    def find_semantic_object_by_language(self, language):
        """
        通过语言描述查找语义对象
        返回匹配度最高的的那个语义对象
        """
        pass


if __name__ == "__main__":
    manager = SemanticMapGuide()
    rospy.spin()
