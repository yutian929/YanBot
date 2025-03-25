import sqlite3
import threading
from datetime import datetime
import numpy as np
import cv2
import os


class SemanticMapDatabase:
    def __init__(
        self,
        db_path: str,
        last_seen_imgs_dir: str,
        last_seen_imgs_max: int = 3,
        renew_db: bool = False,
    ):
        """
        初始化语义地图数据库

        :param db_path: SQLite 数据库文件路径
        :param last_seen_imgs_dir: 最近观测图像的存储目录
        """
        self.db_path = db_path
        self.last_seen_imgs_dir = last_seen_imgs_dir
        self.last_seen_imgs_max = last_seen_imgs_max
        if not os.path.exists(self.last_seen_imgs_dir):
            os.makedirs(self.last_seen_imgs_dir)
        if renew_db:
            self._renew_db()
        self.lock = threading.Lock()
        self._init_db()

    def _get_conn(self) -> sqlite3.Connection:
        """获取线程安全的数据库连接并配置二进制支持"""
        conn = sqlite3.connect(
            self.db_path, check_same_thread=False, detect_types=sqlite3.PARSE_DECLTYPES
        )
        # 注册 BLOB 类型适配器
        sqlite3.register_converter("BLOB", lambda x: x)
        conn.execute("PRAGMA journal_mode = WAL")
        return conn

    def _renew_db(self):
        """删除数据库文件和最近观测图像目录"""
        if os.path.exists(self.db_path):
            os.remove(self.db_path)
        if os.path.exists(self.last_seen_imgs_dir):
            os.system(f"rm -rf {self.last_seen_imgs_dir}")

    def _init_db(self):
        """
        初始化数据库表结构和图像库: semantic_objects
        label: category@id
        bbox: AABB, [x_min, y_min, z_min, x_max, y_max, z_max]
        time_stamp: 时间戳
        x_data, y_data, z_data: 二进制数据(float32)
        rgb_data: 二进制数据(uint32)
        """
        with self.lock:
            conn = self._get_conn()
            try:
                conn.execute(
                    """
                    CREATE TABLE IF NOT EXISTS semantic_objects (
                        label TEXT PRIMARY KEY,
                        bbox BLOB,
                        time_stamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
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
        if not os.path.exists(self.last_seen_imgs_dir):
            os.makedirs(self.last_seen_imgs_dir)

    def _float32_to_blob(self, data: list) -> bytes:
        """将 float32 数据转换为二进制数据"""
        return np.array(data, dtype=np.float32).tobytes()

    def _uint32_to_blob(self, data: list) -> bytes:
        """将 uint32 数据转换为二进制数据"""
        return np.array(data, dtype=np.uint32).tobytes()

    def _blob_to_float32(self, blob: bytes) -> list:
        """将二进制反序列化为float32列表"""
        return np.frombuffer(blob, dtype=np.float32).tolist()

    def _blob_to_uint32(self, blob: bytes) -> list:
        """将二进制反序列化为uint32列表"""
        return np.frombuffer(blob, dtype=np.uint32).tolist()

    def _update_entry(
        self,
        label: str,
        bbox: list,
        x: list,
        y: list,
        z: list,
        rgb: list,
        time_stamp: str = None,
    ):
        """
        更新/插入语义对象条目
        :param label: 格式为 category@id 的标识符 (如 chair@1)
        :param bbox: 边界框 [x_min, y_min, z_min, x_max, y_max, z_max]
        :param x: x坐标列表
        :param y: y坐标列表
        :param z: z坐标列表
        :param rgb: RGB值列表 (uint32 packed)
        :param time_stamp: 可选时间戳 (格式: YYYY-MM-DD HH:MM:SS)
        """
        bbox_data = self._float32_to_blob(bbox)
        time_stamp = time_stamp or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        x_data = self._float32_to_blob(x)
        y_data = self._float32_to_blob(y)
        z_data = self._float32_to_blob(z)
        rgb_data = self._uint32_to_blob(rgb)
        with self.lock:
            conn = self._get_conn()
            try:
                conn.execute(
                    """
                    INSERT OR REPLACE INTO semantic_objects
                    (label, bbox, time_stamp, x_data, y_data, z_data, rgb_data)
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                    """,
                    (label, bbox_data, time_stamp, x_data, y_data, z_data, rgb_data),
                )
                conn.commit()
            finally:
                conn.close()

    def _get_entry_by_label(self, label: str) -> dict:
        """
        通过标签获取语义对象条目
        :param label: 格式为 category@id 的标识符 (如 chair@1)
        :return: 语义对象条目字典
        """
        with self.lock:
            conn = self._get_conn()
            try:
                cursor = conn.execute(
                    """
                    SELECT * FROM semantic_objects WHERE label=?
                    """,
                    (label,),
                )
                row = cursor.fetchone()
                if row:
                    return {
                        "label": row[0],
                        "bbox": self._blob_to_float32(row[1]),
                        "time_stamp": row[2],
                        "x": self._blob_to_float32(row[3]),
                        "y": self._blob_to_float32(row[4]),
                        "z": self._blob_to_float32(row[5]),
                        "rgb": self._blob_to_uint32(row[6]),
                    }
                return None
            finally:
                conn.close()

    def _get_entries_by_category(self, category: str) -> list:
        """
        通过类别, 获取同一类语义对象条目列表
        :param category: 类别标识符 (如 chair)
        :return: 语义对象条目列表
        """
        with self.lock:
            conn = self._get_conn()
            try:
                cursor = conn.execute(
                    """
                    SELECT * FROM semantic_objects WHERE label LIKE ?
                    """,
                    (f"{category}@%",),
                )
                rows = cursor.fetchall()
                return [
                    {
                        "label": row[0],
                        "bbox": self._blob_to_float32(row[1]),
                        "time_stamp": row[2],
                        "x": self._blob_to_float32(row[3]),
                        "y": self._blob_to_float32(row[4]),
                        "z": self._blob_to_float32(row[5]),
                        "rgb": self._blob_to_uint32(row[6]),
                    }
                    for row in rows
                ]
            finally:
                conn.close()

    def _get_all_entries(self) -> list:
        """
        获取所有语义对象条目
        :return: 语义对象条目列表
        """
        with self.lock:
            conn = self._get_conn()
            try:
                cursor = conn.execute("SELECT * FROM semantic_objects")
                rows = cursor.fetchall()
                return [
                    {
                        "label": row[0],
                        "bbox": self._blob_to_float32(row[1]),
                        "time_stamp": row[2],
                        "x": self._blob_to_float32(row[3]),
                        "y": self._blob_to_float32(row[4]),
                        "z": self._blob_to_float32(row[5]),
                        "rgb": self._blob_to_uint32(row[6]),
                    }
                    for row in rows
                ]
            finally:
                conn.close()

    def get_img_paths_by_category(self, category: str) -> dict:
        """
        获取指定类别的所有观测图像路径
        :param category: 类别标识符 (如 chair)
        :return: 观测图像路径字典{label: [img_path1, img_path2, ...]}
        """
        label_dir_names = os.listdir(self.last_seen_imgs_dir)
        label_dir_names.sort()
        img_paths_dict = {}
        for label_dir_name in label_dir_names:
            if label_dir_name.startswith(category):
                img_paths = os.listdir(
                    os.path.join(self.last_seen_imgs_dir, label_dir_name)
                )
                img_paths.sort()
                img_paths = [
                    os.path.join(self.last_seen_imgs_dir, label_dir_name, img_path)
                    for img_path in img_paths
                ]
                img_paths_dict[label_dir_name] = img_paths
        return img_paths_dict

    def _save_last_seen_img(self, label: str, cv_image: np.ndarray):
        """
        保存最近观测图像
        :param label: 格式为 category@id 的标识符 (如 chair@1)
        :param cv_image: OpenCV 图像对象
        """
        if os.path.exists(os.path.join(self.last_seen_imgs_dir, label)):
            self._clear_last_seen_imgs(label)
        else:
            os.makedirs(os.path.join(self.last_seen_imgs_dir, label))
        time = datetime.now().strftime("%Y%m%d%H%M%S")
        img_path = os.path.join(self.last_seen_imgs_dir, label, f"{label}@{time}.png")
        cv2.imwrite(img_path, cv_image)

    def _clear_last_seen_imgs(self, label: str):
        """
        清除指定label的最久远的观测图像，当超过self.last_seen_imgs_max时
        :param label: 格式为 category@id 的标识符 (如 chair@1)
        """
        img_dir = os.path.join(self.last_seen_imgs_dir, label)
        img_files = os.listdir(img_dir)
        if len(img_files) >= self.last_seen_imgs_max:
            img_files.sort()
            os.remove(os.path.join(img_dir, img_files[0]))
