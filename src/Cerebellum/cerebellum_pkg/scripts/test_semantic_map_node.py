#!/usr/bin/env python3

import rospy
import numpy as np
import soundfile as sf
import sounddevice as sd
import json
from wakeup_pkg.msg import WakeUp
from std_srvs.srv import SetBool
from stt_pkg.srv import STT, STTResponse
from semantic_map_pkg.srv import Guide, GuideRequest


class TestSemanticMapNode:
    """语音交互测试节点"""

    def __init__(self):
        rospy.init_node("test_semantic_map_node")

        # 获取参数
        self.save_path = rospy.get_param("~default_stt_wav_path", "stt.wav")

        # 初始化服务代理
        rospy.wait_for_service("wakeup_control")
        self.wakeup_ctrl = rospy.ServiceProxy("wakeup_control", SetBool)

        # 初始化STT服务客户端
        rospy.wait_for_service("srv_stt")
        self.stt_client = rospy.ServiceProxy("srv_stt", STT)

        # 初始化Guide服务客户端
        rospy.wait_for_service("semantic_map_guide")
        self.guide_client = rospy.ServiceProxy("semantic_map_guide", Guide)

        # 订阅唤醒话题
        rospy.Subscriber("wakeup", WakeUp, self.wakeup_callback)

        # 录音参数
        self.sample_rate = 16000
        self.recording = []
        self.is_recording = False

        rospy.loginfo("test_semantic_map_node initialized complete.")

    def wakeup_callback(self, msg):
        """唤醒消息回调处理"""
        if msg.wakeup:
            try:
                # 关闭语音唤醒功能
                resp = self.wakeup_ctrl(False)
                if resp.success:
                    rospy.loginfo("检测到唤醒，开始录制指令...")
                    self.start_recording()
            except rospy.ServiceException as e:
                rospy.logerr(f"服务调用失败: {e}")

    def start_recording(self):
        """启动语音指令录制"""
        self.is_recording = True
        self.recording = []

        # 设置录音设备
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype="int16",
            blocksize=512,
            callback=self.audio_callback,
        )
        self.stream.start()

        # 设置超时定时器
        rospy.Timer(rospy.Duration(5), self.stop_recording, oneshot=True)

    def audio_callback(self, indata, frames, time, status):
        """音频数据回调"""
        if status:
            rospy.logwarn(f"音频输入异常: {status}")
        if self.is_recording:
            self.recording.append(indata.copy())

    def stop_recording(self, event=None):
        """停止并处理录音"""
        if self.is_recording:
            # 停止录音设备
            self.stream.stop()
            self.stream.close()
            self.is_recording = False

            try:
                # 保存录音文件
                audio_data = np.concatenate(self.recording, axis=0)
                sf.write(self.save_path, audio_data, self.sample_rate)
                rospy.loginfo(f"录音保存成功: {self.save_path}")

                # 调用STT服务
                stt_result = self.handle_stt()
                if stt_result["status"] != "success":
                    raise Exception(f"STT失败: {stt_result.get('message', '未知错误')}")

                # 调用Guide服务
                cmd = stt_result["asr_result"]
                if not self.handle_guide(cmd):
                    raise Exception("语义地图导航失败")

            except Exception as e:
                rospy.logerr(f"处理流程异常: {str(e)}")
            finally:
                # 重新启用唤醒功能
                try:
                    self.wakeup_ctrl(True)
                except rospy.ServiceException as e:
                    rospy.logerr(f"无法重新启用唤醒: {e}")

    def handle_stt(self):
        """处理STT语音识别"""
        try:
            # 构建STT请求
            stt_req = {"file_path": self.save_path}
            response = self.stt_client(json.dumps(stt_req))

            # 解析响应
            result = json.loads(response.output_json)
            if result["status"] == "success":
                rospy.loginfo(f"语音识别成功: {result['asr_result']}")
            return result
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def handle_guide(self, cmd: str):
        """处理语义地图导航服务"""
        try:
            # 构建服务请求
            req = GuideRequest()
            req.cmd = cmd

            # 调用服务
            response = self.guide_client(req)

            # 处理响应
            if response.success:
                rospy.loginfo(f"导航信息获取成功！{response.message}")
                rospy.loginfo(f"目标物体: {response.label}")
                rospy.loginfo(f"物体位置: {response.label_position}")
                rospy.loginfo(f"导航目标朝向: {response.nav_orientation}")
                return True
            else:
                rospy.logwarn(f"语义地图服务返回失败, {response.message}")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"处理导航数据异常: {str(e)}")
            return False


if __name__ == "__main__":
    try:
        node = TestSemanticMapNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
