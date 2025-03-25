#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from semantic_map_database import SemanticMapDatabase
from tf2_ros import Buffer, TransformListener
import time
from semantic_map_CLIPMatcher import CLIPMatcher
from llm_pkg.srv import LLMChat, LLMChatRequest, LLMChatResponse
from semantic_map_pkg.srv import Guide, GuideResponse
import json
import os
import re


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
        self.semantic_categories_json_path = os.path.join(
            os.path.dirname(__file__), "semantic_categories.json"
        )
        # ROS配置
        ## opencv
        self.bridge = CvBridge()
        ## 订阅机器人基座位姿
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.base_link_father = rospy.get_param("~base_link_father", "odom")
        self.base_link_child = rospy.get_param("~base_link_child", "camera_link")
        ## LLM服务
        rospy.loginfo("Waiting for LLM service...")
        rospy.wait_for_service("llm_chat")
        self.llm_chat_client = rospy.ServiceProxy("llm_chat", LLMChat)
        rospy.wait_for_service("llm_reason")
        self.llm_reason_client = rospy.ServiceProxy("llm_reason", LLMChat)
        rospy.loginfo("LLM service initialized complete.")
        ## Guide服务
        self.guide_server = rospy.Service(
            "semantic_map_guide", Guide, self.guide_callback
        )

        rospy.loginfo("Semantic map guide node initialized complete.")
        # self.debug()

    def debug(self):
        # 测试
        while not rospy.is_shutdown():
            time.sleep(1)
            # nearest_object = self.find_semantic_object_by_category("mouse")
            # if nearest_object:
            #     rospy.loginfo(f"Nearest object: {nearest_object}")
            best_object = self.find_semantic_object_by_language(
                "cellphone", "一个插着充电线的iPhone手机"
            )
            if best_object:
                rospy.loginfo(f"Best object: {best_object}")

    def get_base_pose(self):
        """
        监听tf，获取机器人基座位姿
        返回(x, y, z, qx, qy, qz, qw)
        """
        try:
            # 获取机器人基座位姿
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
            rospy.logerr(
                f"Get base pose error: {str(e)}, use default pose [0, 0, 0, 0, 0, 0, 0]"
            )
            return [0, 0, 0, 0, 0, 0, 0]

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

    def lang_imgs_match(self, lang, img_paths):
        """
        语言描述和图片匹配
        返回匹配度
        """
        if type(img_paths) is not list:
            img_paths = [img_paths]
        match = self.clip_matcher.match_batch(lang, img_paths)
        return match

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

    def find_semantic_object_by_language(self, category, language):
        """
        通过语言描述查找语义对象
        返回匹配度最高的的那个语义对象
        """
        semantic_categories = json.load(open(self.semantic_categories_json_path, "r"))
        semantic_categories = semantic_categories["categories"]

        if category in semantic_categories:
            label_img_paths = self.database.get_img_paths_by_category(category)
            if label_img_paths:
                best_label = None
                best_score = 0
                for label, img_paths in label_img_paths.items():
                    # match_pairs: [(img_path, match_score), ...]
                    match_pairs = self.lang_imgs_match(language, img_paths)
                    label_socre = sum(
                        [match_pair[1] for match_pair in match_pairs]
                    ) / len(match_pairs)
                    if label_socre > best_score:
                        best_score = label_socre
                        best_label = label
                return best_label
        return None

    def guide_callback(self, req):
        try:
            cmd = req.cmd  # 1.找一个在桌子上的苹果 2.找一个苹果
            semantic_categories = json.load(
                open(self.semantic_categories_json_path, "r")
            )
            semantic_categories = semantic_categories["categories"]
            content = f"以下是语义对象列表：{semantic_categories}。用户的指令是：{cmd}"
            llm_chat_req = LLMChatRequest(
                type="find_semantic_object_chat", content=content
            )
            llm_chat_res = self.llm_chat_client(llm_chat_req)
            print(f"ask: {content}")
            print(f"res: {llm_chat_res.response}")
            if llm_chat_res.success:
                rospy.loginfo(
                    f"LLM chat response: {llm_chat_res.response}"
                )  # 函数1：language_object(category, language) 函数2：category_object(category) 函数3：not_in_list()
                if "not_in_list" in llm_chat_res.response:
                    rospy.logerr("No semantic object found.")
                    return GuideResponse(success=False, label="")
                if "language_object" in llm_chat_res.response:
                    category, language = re.findall(
                        r"language_object\((.*), (.*)\)", llm_chat_res.response
                    )[0]
                    label = self.find_semantic_object_by_language(category, language)
                    if label:
                        rospy.loginfo(f"Semantic object found: {label}")
                        return GuideResponse(success=True, label=label)
                    else:
                        rospy.logerr("No semantic object found by language.")
                        return GuideResponse(success=False, label="")
                elif "category_object" in llm_chat_res.response:
                    category = re.findall(
                        r"category_object\((.*)\)", llm_chat_res.response
                    )[0]
                    label = self.find_semantic_object_by_category(category)
                    if label:
                        rospy.loginfo(f"Semantic object found: {label}")
                        return GuideResponse(success=True, label=label)
                    else:
                        rospy.logerr("No semantic object found by category.")
                        return GuideResponse(success=False, label="")
                else:
                    rospy.logerr("llm chat response format error.")
                    return GuideResponse(success=False, label="")
            else:
                rospy.logerr(f"LLM chat error: {llm_chat_res.response}")
                return GuideResponse(success=False, label="")
        except Exception as e:
            rospy.logerr(f"Guide callback error: {str(e)}")
            return GuideResponse(success=False, label="")


if __name__ == "__main__":
    try:
        manager = SemanticMapGuide()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt exception.")
