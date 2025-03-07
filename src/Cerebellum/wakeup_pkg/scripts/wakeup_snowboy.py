#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import Bool
from snowboy.examples.Python3.lib_wakeup_snowboy import HotwordDetector


class SnowboyWakeupNode:
    def __init__(self):
        rospy.init_node("wakeup_snowboy", anonymous=True)

        # 初始化发布器
        self.wakeup_pub = rospy.Publisher("topic_wakeup", Bool, queue_size=10)

        # 获取模型路径
        model_path = self.get_normalized_path(
            rospy.get_param(
                "~model_path", "./snowboy/resources/models/nihaoyanyan.pmdl"
            )
        )

        # 初始化唤醒检测器
        self.detector = HotwordDetector(
            model_path=model_path,
            detected_callback=self.wakeup_callback,
            sensitivity=rospy.get_param("~sensitivity", 0.5),
        )

        rospy.loginfo("Wakeup node initialized, waiting for wake word...")

    def get_normalized_path(self, relative_path):
        """处理路径标准化"""
        current_script_path = os.path.abspath(__file__)
        current_dir = os.path.dirname(current_script_path)
        target_path = os.path.normpath(os.path.join(current_dir, relative_path))

        if not os.path.exists(target_path):
            rospy.logerr(f"Model file not found: {target_path}")
            raise FileNotFoundError(f"Path not exist: {target_path}")

        return target_path

    def wakeup_callback(self):
        """唤醒回调函数"""
        rospy.loginfo("Wake word detected!")
        self.wakeup_pub.publish(Bool(True))

    def run(self):
        try:
            self.detector.start()
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down wakeup node...")
        finally:
            self.detector.cleanup()


if __name__ == "__main__":
    try:
        node = SnowboyWakeupNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"Error in wakeup node: {str(e)}")
