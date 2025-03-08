#!/usr/bin/env python
import rospy
import sqlite3
from semantic_map_generator_pkg.msg import SemanticPointCloud
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import threading


class SemanticMapManager:
    def __init__(self):
        rospy.init_node("semantic_map_manager_node")

        # 初始化线程安全组件
        self.db_lock = threading.Lock()
        self._init_database()

        # 订阅语义点云
        self.sub = rospy.Subscriber(
            "/semantic_cloud", SemanticPointCloud, self.cloud_callback
        )

        # 创建地图发布器
        self.pub = rospy.Publisher("/semantic_map", PointCloud2, queue_size=10)

        # 定时发布地图（3秒间隔）
        self.timer = rospy.Timer(rospy.Duration(3), self.publish_map_callback)

    def _get_connection(self):
        """创建新的线程安全数据库连接"""
        conn = sqlite3.connect("semantic_map.db", check_same_thread=False)
        conn.execute("PRAGMA journal_mode = WAL")
        rospy.loginfo("Connected to database")
        return conn

    def _init_database(self):
        """初始化数据库结构"""
        with self.db_lock:
            conn = self._get_connection()
            try:
                conn.execute(
                    """
                    CREATE TABLE IF NOT EXISTS points (
                        x REAL, y REAL, z REAL,
                        rgb INTEGER,
                        label TEXT,
                        confidence REAL
                    )
                """
                )
                conn.execute("CREATE INDEX IF NOT EXISTS label_idx ON points(label)")
                conn.commit()
            finally:
                conn.close()

    def cloud_callback(self, msg):
        """处理点云订阅回调"""
        try:
            point_cnt = msg.count
            x_list = msg.x
            y_list = msg.y
            z_list = msg.z
            rgb_list = msg.rgb
            label = msg.label
            confidence = msg.confidence

            points = [
                (x_list[i], y_list[i], z_list[i], rgb_list[i], label, confidence)
                for i in range(point_cnt)
            ]

            # breakpoint()

            with self.db_lock:
                conn = self._get_connection()
                try:
                    conn.executemany("INSERT INTO points VALUES (?,?,?,?,?,?)", points)
                    conn.commit()
                    rospy.loginfo(f"Inserted {len(points)} points into database")
                finally:
                    conn.close()

        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {str(e)}")

    def publish_map_callback(self, event=None):
        """定时发布完整语义地图（线程安全版本）"""
        try:
            points = []
            with self.db_lock:
                conn = self._get_connection()
                try:
                    cursor = conn.cursor()
                    cursor.execute("SELECT x, y, z, rgb FROM points")
                    points = cursor.fetchall()
                finally:
                    conn.close()

            if not points:
                return

            # 准备点云数据
            header = Header(stamp=rospy.Time.now(), frame_id="map")
            fields = [
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
                PointField("rgb", 12, PointField.UINT32, 1),
            ]

            # 使用更高效的方式打包数据
            fmt = "<3fI"  # 小端字节序：3个float + 1个uint32
            packed_data = [struct.pack(fmt, p[0], p[1], p[2], p[3]) for p in points]

            # breakpoint()

            cloud = PointCloud2(
                header=header,
                height=1,
                width=len(points),
                is_dense=True,
                is_bigendian=False,
                fields=fields,
                point_step=16,  # 4 * 3 + 4 = 16 bytes
                row_step=16 * len(points),
                data=b"".join(packed_data),
            )

            self.pub.publish(cloud)
            rospy.loginfo(f"Published {len(points)} points to semantic_map")

        except Exception as e:
            rospy.logerr(f"Error publishing map: {str(e)}")

    def __del__(self):
        """确保关闭所有数据库连接"""
        pass  # 不需要显式关闭，因为每次操作后都立即关闭了连接


if __name__ == "__main__":
    manager = SemanticMapManager()
    rospy.spin()
