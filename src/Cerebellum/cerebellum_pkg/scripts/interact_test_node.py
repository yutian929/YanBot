#!/usr/bin/env python3

import rospy
import sounddevice as sd
import numpy as np
import threading
import queue
import json
import io
import os
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from wakeup_pkg.msg import WakeUp
from stt_pkg.srv import STT
from tts_pkg.srv import TTS
from funasr import AutoModel


def get_normalized_path(relative_path):
    current_script_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_script_path)
    target_path = os.path.normpath(os.path.join(current_dir, relative_path))
    if not os.path.exists(target_path):
        os.makedirs(target_path)
    return target_path


class InteractTestNode:
    """
    交互测试节点，实现语音唤醒、语音识别和语音合成的流程测试
    """

    def __init__(self):
        """初始化InteractTestNode类，设置订阅、发布和服务"""
        rospy.init_node("interaction_test_node")

        # 初始化音频处理参数
        self.sample_rate = 16000
        self.window = 5 * 1600 * 2  # 500ms 窗口
        self.is_recording = False
        self.is_running = True
        self.audio_buffer = b""

        # 初始化队列
        self.audio_queue = queue.Queue()
        self.vad_queue = queue.Queue()

        # 设置音频保存路径
        self.audio_relative_dir = "../../../../last_heard_audios"
        self.file_name = "stt.wav"
        self.file_path = os.path.join(
            get_normalized_path(self.audio_relative_dir), self.file_name
        )

        # 订阅唤醒词检测结果
        self.wakeup_sub = rospy.Subscriber("wakeup", WakeUp, self.wakeup_callback)

        # 初始化服务客户端
        rospy.loginfo("等待STT服务...")
        rospy.wait_for_service("srv_stt")
        self.stt_client = rospy.ServiceProxy("srv_stt", STT)

        rospy.loginfo("等待TTS服务...")
        rospy.wait_for_service("srv_tts")
        self.tts_client = rospy.ServiceProxy("srv_tts", TTS)

        rospy.loginfo("等待唤醒控制服务...")
        rospy.wait_for_service("wakeup_control")
        self.wakeup_control_client = rospy.ServiceProxy("wakeup_control", SetBool)

        # 加载VAD模型
        rospy.loginfo("正在加载VAD语音活动检测模型...")
        self.vad_model = AutoModel(
            model="fsmn-vad", model_revision="v2.0.4", disable_pbar=True
        )
        rospy.loginfo("VAD模型加载完成")

        # 初始化线程
        self.record_thread = None
        self.vad_thread = None

        rospy.loginfo("交互测试节点初始化完成")

    def create_wav_header(
        self, dataflow, sample_rate=16000, num_channels=1, bits_per_sample=16
    ):
        """
        创建WAV文件头的字节串。

        Args:
            dataflow: 音频bytes数据（以字节为单位）
            sample_rate: 采样率，默认16000
            num_channels: 声道数，默认1（单声道）
            bits_per_sample: 每个样本的位数，默认16

        Returns:
            WAV文件头的字节串和音频bytes数据
        """
        total_data_len = len(dataflow)
        byte_rate = sample_rate * num_channels * bits_per_sample // 8
        block_align = num_channels * bits_per_sample // 8
        data_chunk_size = total_data_len
        fmt_chunk_size = 16
        riff_chunk_size = 4 + (8 + fmt_chunk_size) + (8 + data_chunk_size)

        # 使用 bytearray 构建字节串
        header = bytearray()

        # RIFF/WAVE header
        header.extend(b"RIFF")
        header.extend(riff_chunk_size.to_bytes(4, byteorder="little"))
        header.extend(b"WAVE")

        # fmt subchunk
        header.extend(b"fmt ")
        header.extend(fmt_chunk_size.to_bytes(4, byteorder="little"))
        header.extend((1).to_bytes(2, byteorder="little"))  # Audio format (1 is PCM)
        header.extend(num_channels.to_bytes(2, byteorder="little"))
        header.extend(sample_rate.to_bytes(4, byteorder="little"))
        header.extend(byte_rate.to_bytes(4, byteorder="little"))
        header.extend(block_align.to_bytes(2, byteorder="little"))
        header.extend(bits_per_sample.to_bytes(2, byteorder="little"))

        # data subchunk
        header.extend(b"data")
        header.extend(data_chunk_size.to_bytes(4, byteorder="little"))

        return bytes(header) + dataflow

    def save_wav_file(self, audio_data):
        """将音频数据保存为WAV文件"""
        wav_data = self.create_wav_header(
            audio_data,
            sample_rate=self.sample_rate,
            num_channels=1,
            bits_per_sample=16,
        )

        with open(self.file_path, "wb") as f:
            f.write(wav_data)

        rospy.loginfo(f"音频已保存到: {self.file_path}")
        return self.file_path

    def wakeup_callback(self, msg):
        """处理唤醒词检测回调"""
        if msg.wakeup and not self.is_recording:
            rospy.loginfo("检测到唤醒词，开始录音...")

            # 关闭唤醒词检测
            try:
                resp = self.wakeup_control_client(False)
                if resp.success:
                    rospy.loginfo(resp.message)
                else:
                    rospy.logwarn(f"关闭唤醒词检测失败: {resp.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"调用唤醒控制服务失败: {str(e)}")

            # 开始录音
            self.start_recording()

    def start_recording(self):
        """开始录音和VAD处理"""
        self.is_recording = True
        self.audio_buffer = b""

        # 启动VAD处理线程
        self.vad_thread = threading.Thread(target=self.vad_process)
        self.vad_thread.daemon = True
        self.vad_thread.start()

        # 启动录音线程
        self.record_thread = threading.Thread(target=self.record_audio)
        self.record_thread.daemon = True
        self.record_thread.start()

    def record_audio(self):
        """录制音频线程"""
        rospy.loginfo("开始录制音频...")

        try:
            with sd.InputStream(
                samplerate=self.sample_rate, channels=1, dtype="int16"
            ) as stream:
                silence_counter = 0
                initial_silence_counter = 0
                max_initial_silence = 10  # 5秒(10个500ms)的初始静音后停止录音
                max_silence_after_speech = 6  # 3秒(6个500ms)的静音后停止录音
                speech_detected = False

                while not rospy.is_shutdown() and self.is_recording and self.is_running:
                    # 读取音频数据
                    dataflow = stream.read(self.window)[0].tobytes()
                    if len(dataflow) == 0:
                        break

                    # 将音频数据放入队列
                    self.audio_queue.put(dataflow)

                    # 判断是否应该结束录音
                    try:
                        has_speech = self.vad_queue.get(block=False)

                        # 情况1: 检测到语音活动
                        if has_speech:
                            speech_detected = True
                            silence_counter = 0
                            initial_silence_counter = 0
                        # 情况2: 未检测到语音活动
                        else:
                            # 2.1: 已经检测到过语音，现在是语音之后的静音
                            if speech_detected:
                                silence_counter += 1
                                if silence_counter >= max_silence_after_speech:
                                    rospy.loginfo("检测到语音后的静音，停止录音")
                                    break
                            # 2.2: 从未检测到过语音，仍在初始静音状态
                            else:
                                initial_silence_counter += 1
                                if initial_silence_counter >= max_initial_silence:
                                    rospy.loginfo("持续未检测到语音，停止录音")
                                    break
                    except queue.Empty:
                        # 队列为空，继续录音
                        pass

        except Exception as e:
            rospy.logerr(f"录音出错: {str(e)}")

        finally:
            self.is_recording = False
            rospy.loginfo("录音结束")

            # 处理录制的音频
            if len(self.audio_buffer) > 0 and speech_detected:
                self.process_recorded_audio()
            else:
                rospy.logwarn("未录制到有效音频")
                # 重新启用唤醒词检测
                self.activate_wakeup()

    def vad_process(self):
        """VAD处理线程，检测语音活动"""
        rospy.loginfo("VAD处理线程启动")

        while not rospy.is_shutdown() and self.is_recording and self.is_running:
            try:
                audio_chunk = self.audio_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            # VAD检测
            vad_data = self.create_wav_header(
                audio_chunk,
                sample_rate=self.sample_rate,
                num_channels=1,
                bits_per_sample=16,
            )

            # 进行VAD检测
            res = self.vad_model.generate(input=vad_data)
            has_speech = res[0]["value"]

            # 将VAD结果放入队列
            self.vad_queue.put(has_speech)

            # 如果检测到语音，将音频数据添加到缓冲区
            if has_speech:
                self.audio_buffer += audio_chunk

    def process_recorded_audio(self):
        """处理录制的音频，执行STT和TTS"""
        rospy.loginfo("处理录制的音频...")

        # 保存录制的音频
        audio_path = self.save_wav_file(self.audio_buffer)

        # 调用STT服务
        try:
            stt_request = json.dumps({"file_path": audio_path})
            stt_response = self.stt_client(stt_request)
            stt_result = json.loads(stt_response.output_json)

            if stt_result["status"] == "success":
                asr_text = stt_result["asr_result"]
                rospy.loginfo(f"语音识别结果: {asr_text}")

                # 调用TTS服务
                try:
                    tts_request = json.dumps({"text": asr_text})
                    tts_response = self.tts_client(tts_request)
                    tts_result = json.loads(tts_response.output_json)

                    if tts_result["status"] == "success":
                        rospy.loginfo(f"语音合成成功，文件保存在: {tts_result['file_path']}")

                        # 使用aplay播放音频
                        os.system(f"aplay {tts_result['file_path']}")

                    else:
                        rospy.logerr(f"语音合成失败: {tts_result.get('message', '未知错误')}")

                except rospy.ServiceException as e:
                    rospy.logerr(f"调用TTS服务失败: {str(e)}")
            else:
                rospy.logerr(f"语音识别失败: {stt_result.get('message', '未知错误')}")

        except rospy.ServiceException as e:
            rospy.logerr(f"调用STT服务失败: {str(e)}")

        # 重新启用唤醒词检测
        self.activate_wakeup()

    def activate_wakeup(self):
        """重新启用唤醒词检测"""
        rospy.loginfo("重新启用唤醒词检测...")
        try:
            resp = self.wakeup_control_client(True)
            if resp.success:
                rospy.loginfo(resp.message)
            else:
                rospy.logwarn(f"启用唤醒词检测失败: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"调用唤醒控制服务失败: {str(e)}")

    def run(self):
        """主函数，启动节点并等待关闭"""
        rospy.loginfo("交互测试节点已启动，等待唤醒...")
        rospy.spin()

    def shutdown(self):
        """关闭节点时的清理工作"""
        self.is_running = False
        self.is_recording = False

        # 等待线程结束
        if self.record_thread and self.record_thread.is_alive():
            self.record_thread.join(timeout=2.0)
        if self.vad_thread and self.vad_thread.is_alive():
            self.vad_thread.join(timeout=2.0)

        rospy.loginfo("交互测试节点已关闭")


if __name__ == "__main__":
    try:
        node = InteractTestNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if "node" in locals():
            node.shutdown()
