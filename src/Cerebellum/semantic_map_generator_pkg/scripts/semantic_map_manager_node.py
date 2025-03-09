#!/usr/bin/env python
import rospy
import sqlite3
import numpy as np
import struct
import threading
from semantic_map_generator_pkg.msg import SemanticObject
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2


class SemanticMapManager:
    def __init__(self):
        rospy.init_node("semantic_map_manager_node")

        # 数据库配置
        self.db_path = rospy.get_param("~db_path", "semantic_map.db")
        self.lock = threading.Lock()
        self._init_db()

        # ROS配置
        topic_semantic_object_sub = rospy.get_param(
            "~topic_semantic_object_sub", "/semantic_object"
        )
        self.sub = rospy.Subscriber(
            topic_semantic_object_sub, SemanticObject, self.semantic_object_callback
        )
        topic_semantic_map_pub = rospy.get_param(
            "~topic_semantic_map_pub", "/semantic_map"
        )
        self.pub = rospy.Publisher(topic_semantic_map_pub, PointCloud2, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(3), self.publish_map)
        # HACK
        self.debug_image_pub = rospy.Publisher("/debug_image", Image, queue_size=10)
        rospy.loginfo("Semantic map manager node initialization complete.")

    def _get_conn(self):
        """获取线程安全连接并配置二进制支持"""
        conn = sqlite3.connect(
            self.db_path,
            check_same_thread=False,
            detect_types=sqlite3.PARSE_DECLTYPES,  # 添加类型解析支持
        )
        # 注册二进制数据适配器
        sqlite3.register_converter("BLOB", lambda x: x)
        conn.execute("PRAGMA journal_mode = WAL")
        return conn

    def _init_db(self):
        """初始化数据库表结构"""
        with self.lock:
            conn = self._get_conn()
            try:
                conn.execute(
                    """
                    CREATE TABLE IF NOT EXISTS label_clouds (
                        label TEXT PRIMARY KEY,
                        x_data BLOB NOT NULL,
                        y_data BLOB NOT NULL,
                        z_data BLOB NOT NULL,
                        rgb_data BLOB NOT NULL
                    )
                """
                )
                conn.commit()
            finally:
                conn.close()

    def _pack_coordinates(self, data):
        """将坐标数据打包为二进制格式"""
        return np.array(data, dtype=np.float32).tobytes()

    def _pack_colors(self, data):
        """将颜色数据打包为二进制格式"""
        return np.array(data, dtype=np.uint32).tobytes()

    def semantic_object_callback(self, msg):
        """增加数据有效性检查的写入方法"""
        try:
            # 验证数据长度一致性
            if not (len(msg.x) == len(msg.y) == len(msg.z) == len(msg.rgb)):
                rospy.logerr("Inconsistent data length")
                return

            # 转换为二进制时捕获异常
            try:
                x_bin = np.array(msg.x, dtype=np.float32).tobytes()
                y_bin = np.array(msg.y, dtype=np.float32).tobytes()
                z_bin = np.array(msg.z, dtype=np.float32).tobytes()
                rgb_bin = np.array(msg.rgb, dtype=np.uint32).tobytes()
            except Exception as e:
                rospy.logerr(f"Data conversion failed: {str(e)}")
                return

            # 获取图像 msg.image， debug pub
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg.image, desired_encoding="bgr8")
            self.debug_image_pub.publish(
                bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            )

            with self.lock:
                conn = self._get_conn()
                try:
                    # 使用参数化查询确保数据安全
                    conn.execute(
                        """
                        INSERT INTO label_clouds (label, x_data, y_data, z_data, rgb_data)
                        VALUES (?, ?, ?, ?, ?)
                        ON CONFLICT(label) DO UPDATE SET
                            x_data = x_data || excluded.x_data,
                            y_data = y_data || excluded.y_data,
                            z_data = z_data || excluded.z_data,
                            rgb_data = rgb_data || excluded.rgb_data
                    """,
                        (
                            msg.label,
                            sqlite3.Binary(x_bin),  # 显式声明二进制类型
                            sqlite3.Binary(y_bin),
                            sqlite3.Binary(z_bin),
                            sqlite3.Binary(rgb_bin),
                        ),
                    )
                    conn.commit()
                    rospy.loginfo(f"Updated {msg.label} with {len(msg.x)} points")
                except sqlite3.IntegrityError as e:
                    rospy.logerr(f"Database integrity error: {str(e)}")
                finally:
                    conn.close()

        except Exception as e:
            rospy.logerr(f"Cloud callback error: {str(e)}")

    def _unpack_cloud(self, label):
        """修复后的解包方法"""
        with self.lock:
            conn = self._get_conn()
            try:
                # 使用CAST确保二进制类型识别
                cur = conn.execute(
                    """
                    SELECT
                        CAST(x_data AS BLOB),
                        CAST(y_data AS BLOB),
                        CAST(z_data AS BLOB),
                        CAST(rgb_data AS BLOB)
                    FROM label_clouds WHERE label = ?
                """,
                    (label,),
                )
                row = cur.fetchone()

                if row:
                    # 空数据检查
                    if not any(row):
                        return None

                    # 解包时增加错误处理
                    try:
                        x = np.frombuffer(row[0], dtype=np.float32)
                        y = np.frombuffer(row[1], dtype=np.float32)
                        z = np.frombuffer(row[2], dtype=np.float32)
                        rgb = np.frombuffer(row[3], dtype=np.uint32)
                    except ValueError as e:
                        rospy.logwarn(f"Data unpack error: {str(e)}")
                        return None

                    # 数据长度校验
                    min_len = min(x.size, y.size, z.size, rgb.size)
                    if min_len == 0:
                        return None

                    return np.vstack(
                        [x[:min_len], y[:min_len], z[:min_len], rgb[:min_len]]
                    ).T
                return None
            finally:
                conn.close()

    def publish_map(self, event=None):
        """修复后的发布方法"""
        try:
            # 获取所有标签
            with self.lock:
                conn = self._get_conn()
                try:
                    labels = [
                        row[0] for row in conn.execute("SELECT label FROM label_clouds")
                    ]
                finally:
                    conn.close()

            # 合并所有点云
            all_points = []
            for label in labels:
                cloud = self._unpack_cloud(label)
                # 修改判断逻辑
                if cloud is not None and len(cloud) > 0:
                    all_points.extend(cloud.tolist())

            if not all_points:
                rospy.logwarn("No points to publish")
                return
            # breakpoint()
            # 转换为ROS消息
            header = Header(stamp=rospy.Time.now(), frame_id="map")
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
                [(p[0], p[1], p[2], p[3]) for p in all_points], dtype=point_dtype
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
            rospy.loginfo(f"Published {len(structured_array)} points")

        except Exception as e:
            rospy.logerr(f"Publishing failed: {str(e)}")


if __name__ == "__main__":
    manager = SemanticMapManager()
    rospy.spin()
