#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from modelscope.pipelines import pipeline
from modelscope.utils.constant import Tasks
import sounddevice as sd
import queue
import threading
import numpy as np
import io
import pypinyin
import re
import difflib
from funasr import AutoModel
from wakeup_pkg.msg import WakeUp
from std_srvs.srv import SetBool, SetBoolResponse


class WakeUpNode:
    """
    语音唤醒节点，包含语音降噪、VAD检测和ASR语音识别功能
    """

    def __init__(self):
        """初始化WakeUpNode类，加载模型和设置参数"""
        rospy.init_node("wakeup_node")

        # 发布者
        self.wakeup_pub = rospy.Publisher("wakeup", WakeUp, queue_size=1)

        # 订阅控制命令
        self.control_sub = rospy.Subscriber(
            "wakeup_control", Bool, self.control_callback
        )

        # 麦克风状态标志
        self.mic_active = True

        # 初始化音频处理参数
        self.sample_rate = 16000
        self.window = 5 * 1600 * 2  # 500ms 窗口

        # 初始化队列
        self.denoise_queue = queue.Queue()
        self.vad_queue = queue.Queue()

        # 加载模型
        rospy.loginfo("正在加载ANS降噪模型...")
        self.ans_pipeline = pipeline(
            Tasks.acoustic_noise_suppression,
            model="iic/speech_zipenhancer_ans_multiloss_16k_base",
        )

        rospy.loginfo("正在加载VAD语音活动检测模型...")
        self.vad_model = AutoModel(
            model="fsmn-vad", model_revision="v2.0.4", disable_pbar=True
        )

        rospy.loginfo("正在加载ASR语音识别模型...")
        self.asr_pipeline = pipeline(
            task=Tasks.auto_speech_recognition,
            model="iic/SenseVoiceSmall",
            model_revision="master",
            device="cuda:0",
            disable_pbar=True,
        )

        rospy.loginfo("所有模型加载完成")

        # 初始化线程
        self.asr_thread = None
        self.is_running = True

        # 添加服务
        self.wakeup_service = rospy.Service(
            "wakeup_control", SetBool, self.service_callback
        )

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

    def find_similar_substrings(self, target, long_string, threshold=0.7):
        """
        在一个长字符串中查找与目标字符串相似度大于阈值的子字符串

        Args:
            target: 目标字符串
            long_string: 长字符串
            threshold: 相似度阈值，默认为 0.7

        Returns:
            相似度大于阈值的子字符串列表
        """
        similar_substrings = []
        target_length = len(target)
        long_string_length = len(long_string)

        # 遍历长字符串的所有可能子字符串
        for start in range(long_string_length - target_length + 1):
            substring = long_string[start : start + target_length]
            similarity = difflib.SequenceMatcher(None, target, substring).ratio()
            if similarity >= threshold:
                similar_substrings.append((substring, similarity))

        return similar_substrings

    def asr_process(self):
        """
        ASR 模块，分线程使用

        从语音流vad_queue获取数据，进行语音识别
        """
        rospy.loginfo("ASR线程启动")

        while self.is_running:
            try:
                speech = self.vad_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            # 处理ASR识别
            ret = self.asr_pipeline(speech)
            cleaned_text = re.sub("<\|[^|]+\|>", "", ret[0]["text"])

            # pinyin_str带声调，pinyin_str2不带声调
            pinyin = pypinyin.lazy_pinyin(
                ret[0]["text"], style=pypinyin.TONE3, errors=lambda x: " "
            )
            pinyin_str = "".join(pinyin)
            pinyin2 = pypinyin.lazy_pinyin(
                ret[0]["text"], style=pypinyin.NORMAL, errors=lambda x: ""
            )
            pinyin_str2 = "".join(pinyin2)

            # 唤醒词检测
            ret1 = self.find_similar_substrings("ni3hao3yan4yan4", pinyin_str, 0.5)
            ret2 = self.find_similar_substrings("nihaoyanyan", pinyin_str2, 0.5)
            ret3 = self.find_similar_substrings("ni3hao3yan4", pinyin_str2, 0.5)
            ret4 = self.find_similar_substrings("nihaoyan", pinyin_str, 0.5)
            ret5 = self.find_similar_substrings("xiao3yan4xiao3yan4", pinyin_str, 0.5)
            ret6 = self.find_similar_substrings("xiaoyanxiaoyan", pinyin_str2, 0.5)

            wakeup_msg = WakeUp()
            wakeup_msg.wakeup = False
            asr_result = ""

            # 发布ASR结果
            if len(cleaned_text) > 0:
                asr_result = "ASR结果: " + cleaned_text

            # 检测唤醒词并发布唤醒信号
            if any([ret1, ret2, ret3, ret4, ret5, ret6]):
                asr_result += "\t唤醒词: "
                if ret1:
                    asr_result += ret1[0][0]
                if ret2:
                    asr_result += ret2[0][0]
                if ret3:
                    asr_result += ret3[0][0]
                if ret4:
                    asr_result += ret4[0][0]
                if ret5:
                    asr_result += ret5[0][0]
                if ret6:
                    asr_result += ret6[0][0]
                wakeup_msg.wakeup = True

            wakeup_msg.asr_result = asr_result

            self.wakeup_pub.publish(wakeup_msg)
            rospy.loginfo(asr_result)

    def control_callback(self, msg):
        """处理开关控制命令"""
        if msg.data and not self.mic_active:
            # 开启麦克风
            rospy.loginfo("开启语音唤醒功能")
            self.mic_active = True
            # 如果麦克风线程不存在或已结束，则重新启动
            if not hasattr(self, "mic_thread") or not self.mic_thread.is_alive():
                self.mic_thread = threading.Thread(target=self.run_mic)
                self.mic_thread.daemon = True
                self.mic_thread.start()
        elif not msg.data and self.mic_active:
            # 关闭麦克风
            rospy.loginfo("关闭语音唤醒功能")
            self.mic_active = False

    def service_callback(self, req):
        response = SetBoolResponse()

        if req.data and not self.mic_active:
            # 开启麦克风
            rospy.loginfo("开启语音唤醒功能")
            self.mic_active = True
            # 如果麦克风线程不存在或已结束，则重新启动
            if not hasattr(self, "mic_thread") or not self.mic_thread.is_alive():
                self.mic_thread = threading.Thread(target=self.run_mic)
                self.mic_thread.daemon = True
                self.mic_thread.start()
            response.success = True
            response.message = "语音唤醒功能已开启"
        elif not req.data and self.mic_active:
            # 关闭麦克风
            rospy.loginfo("关闭语音唤醒功能")
            self.mic_active = False
            response.success = True
            response.message = "语音唤醒功能已关闭"
        else:
            # 已经处于请求的状态
            response.success = True
            response.message = f"语音唤醒功能已经{'开启' if self.mic_active else '关闭'}"

        return response

    def run_mic(self):
        """麦克风处理线程"""
        rospy.loginfo("开始录音并等待语音输入...")

        try:
            with sd.InputStream(
                samplerate=self.sample_rate, channels=1, dtype="int16"
            ) as stream:
                vad_buffer = b""
                buffer = b""

                while not rospy.is_shutdown() and self.is_running and self.mic_active:
                    # 读取音频数据
                    dataflow = stream.read(self.window)[0].tobytes()
                    if len(dataflow) == 0:
                        break

                    # 降噪处理
                    result = self.ans_pipeline(
                        self.create_wav_header(
                            dataflow,
                            sample_rate=self.sample_rate,
                            num_channels=1,
                            bits_per_sample=16,
                        )
                    )
                    output = result["output_pcm"]

                    # VAD检测
                    vad_buffer = self.create_wav_header(
                        output,
                        sample_rate=self.sample_rate,
                        num_channels=1,
                        bits_per_sample=16,
                    )
                    res = self.vad_model.generate(input=vad_buffer)

                    # 将降噪后的数据放入队列
                    self.denoise_queue.put(output)

                    # VAD处理结果
                    if res[0]["value"]:
                        rospy.logdebug("检测到语音活动")
                        buffer += output
                    else:
                        # 如果有累积的语音数据且当前没有语音活动，将缓冲区数据送入ASR处理
                        if len(buffer) > 0:
                            wav_data = self.create_wav_header(
                                buffer,
                                sample_rate=self.sample_rate,
                                num_channels=1,
                                bits_per_sample=16,
                            )
                            self.vad_queue.put(wav_data)
                            buffer = b""

                    # # 在检测到唤醒词后自动关闭麦克风（可选）
                    # if any([ret1, ret2, ret3, ret4, ret5, ret6]):
                    #     wakeup_msg = WakeUp()
                    #     wakeup_msg.wakeup = True
                    #     wakeup_msg.asr_result = "ASR结果: 唤醒词检测到"
                    #     self.wakeup_pub.publish(wakeup_msg)
                    #     rospy.loginfo("检测到唤醒词，暂停语音唤醒功能")
                    #     self.mic_active = False
                    #     break

        except Exception as e:
            rospy.logerr(f"录音出错: {str(e)}")

        finally:
            rospy.loginfo("麦克风录音暂停")

    def run(self):
        """主函数，启动ASR线程和麦克风线程"""
        # 启动ASR线程
        self.asr_thread = threading.Thread(target=self.asr_process)
        self.asr_thread.daemon = True
        self.asr_thread.start()

        # 启动麦克风线程
        self.mic_thread = threading.Thread(target=self.run_mic)
        self.mic_thread.daemon = True
        self.mic_thread.start()

        # 等待ROS关闭
        rospy.spin()

    def shutdown(self):
        """关闭节点时的清理工作"""
        self.is_running = False
        if self.asr_thread and self.asr_thread.is_alive():
            self.asr_thread.join(timeout=2.0)
        rospy.loginfo("唤醒节点已关闭")


if __name__ == "__main__":
    try:
        node = WakeUpNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if "node" in locals():
            node.shutdown()
