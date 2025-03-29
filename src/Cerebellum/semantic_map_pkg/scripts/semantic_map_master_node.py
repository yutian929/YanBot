#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from message_filters import ApproximateTimeSynchronizer, Subscriber
import threading
import time

class SemanticMapMaster:
    def __init__(self):
        rospy.init_node("semantic_map_master")

        # 设置参数服务器的全局参数——两个任务是否正在进行
        rospy.set_param("det_seg_processing", False)
        rospy.set_param("depth_repair_processing", False)

        # 设置检测目标（通过参数服务器给整个系统共用）
        class_list = ['keyboard', 'mouse', 'cell phone', 'earphone', 'laptop', 'water bottle', 'keys', 'potted plant']
        rospy.set_param("detection_prompt", class_list)

        self.latest_image = None
        self.latest_depth = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)    

        # 轮询参数，等待目标节点设置 "init" 为 True，即模型加载完成
        while not (rospy.has_param("/depth_anything_ros_init") and rospy.has_param("/yolo_evsam_ros_init")):
            rospy.loginfo("Waiting for model to load...")
            time.sleep(2)

        rospy.loginfo("All model loaded! Now starting master node...")    

        self.data_lock = threading.Lock()    

        # 定时器
        rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        # 订阅rgb图像
        topic_image_sub = rospy.get_param(
            "~topic_image_sub", "/camera/color/image_raw"
        )
        image_sub = Subscriber(topic_image_sub, Image, queue_size=2)
        # 订阅深度图像
        topic_depth_sub = rospy.get_param(
            "~topic_depth_sub", "/camera/aligned_depth_to_color/image_raw"
        )
        depth_sub = Subscriber(topic_depth_sub, Image, queue_size=2)
        
        # 同步订阅
        ts = ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=2, slop=0.005
        )
        ts.registerCallback(self.update_latest_image_and_depth)

        # 待处理的rgb图像和原始深度图像发布（发布给 检测分割节点 和 深度修复节点 ，作为开始处理的触发信号）
        self.color_to_process_pub = rospy.Publisher("color_to_process", Image, queue_size=1)
        self.depth_to_process_pub = rospy.Publisher("depth_to_process", Image, queue_size=1)

        rospy.loginfo("Semantic map master initialization complete.")


    def timer_callback(self, event):
        """定时器回调"""
        try:
            with self.data_lock:
                if self.latest_image is None or self.latest_depth is None:
                    return
                # 先确认当前最新图像有没有对应时间的tf变换
                # 相机坐标系变换约在图像数据到达的0.03s之后才会生成，map->odom的tf的时间戳反而是当前时间的未来0.1s
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    "camera_color_optical_frame",  # 使用光学坐标系
                    self.latest_image.header.stamp,  # 使用图像时间戳
                    rospy.Duration(0.05),
                )
                # 主要用于控制点云生成整体频率（通过控制 待处理的rgb图像和原始深度图像 的发布来实现）
                # 先确认两个图像处理节点是否正在执行处理
                det_seg_processing = rospy.get_param("det_seg_processing")
                depth_repair_processing = rospy.get_param("depth_repair_processing")

                # 检测分割和深度修复均不在进行中
                if (not det_seg_processing) and (not depth_repair_processing):
                    self.color_to_process_pub.publish(self.latest_image)
                    self.depth_to_process_pub.publish(self.latest_depth)
                    print(" ")
                    rospy.loginfo("publish color_to_process and depth_to_process.")
                    self.latest_image = None
                    self.latest_depth = None

        except Exception as e:
            rospy.logerr(f"Sync callback error: {str(e)}")


    def update_latest_image_and_depth(self, img_msg, depth_msg):
        """持续记录当前最新的图像消息"""
        with self.data_lock:
            self.latest_image = img_msg
            self.latest_depth = depth_msg
            # rospy.loginfo("latest_image and latest_depth update.")



if __name__ == "__main__":
    try:
        node = SemanticMapMaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
