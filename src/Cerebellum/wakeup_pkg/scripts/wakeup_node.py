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


class WakeUpNode:
    """
    语音唤醒节点，包含语音降噪、VAD检测和ASR语音识别功能
    """

    def __init__(self):
        """初始化WakeUpNode类，加载模型和设置参数"""
        rospy.init_node("wakeup_node")

        # 发布者
        self.asr_pub = rospy.Publisher("ASR/msg", String, queue_size=1)
        self.wakeup_pub = rospy.Publisher("ASR/wakeup", String, queue_size=1)
        self.topic_wakeup_pub = rospy.Publisher("topic_wakeup", Bool, queue_size=1)

        # 初始化音频处理参数
        self.sample_rate = 16000
        self.window = 8 * 1600 * 2  # 800ms 窗口

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
        识别结果发布到话题:
        - /ASR/msg (String): 识别的文本
        - /ASR/wakeup (String): 唤醒词检测结果
        - /topic_wakeup (Bool): 唤醒状态
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
            ret3 = self.find_similar_substrings("nihaoyan", pinyin_str2, 0.5)
            ret4 = self.find_similar_substrings("nihaoyayan", pinyin_str2, 0.5)
            ret5 = self.find_similar_substrings("xiao3yan4xiao3yan4", pinyin_str, 0.5)
            ret6 = self.find_similar_substrings("xiaoyanxiaoyan", pinyin_str2, 0.5)

            # 发布ASR结果
            if len(cleaned_text) > 0:
                self.asr_pub.publish(cleaned_text)
                rospy.loginfo("ASR结果: %s", cleaned_text)

            # 检测唤醒词并发布唤醒信号
            if ret1 or ret2 or ret3 or ret4:
                msg = "ASR唤醒"
                if ret1:
                    msg += ret1[0][0]
                if ret2:
                    msg += ret2[0][0]
                self.wakeup_pub.publish(msg)
                self.topic_wakeup_pub.publish(True)
                rospy.loginfo("ASR唤醒: %s", msg)

            if ret5 or ret6:
                msg = "ASR唤醒"
                if ret5:
                    msg += ret5[0][0]
                if ret6:
                    msg += ret6[0][0]
                self.wakeup_pub.publish(msg)
                self.topic_wakeup_pub.publish(True)
                rospy.loginfo("ASR唤醒: %s", msg)

    def run(self):
        """
        主循环，处理音频流

        从麦克风读取音频，经过降噪、VAD检测，将有语音活动的片段送入ASR处理
        """
        # 启动ASR线程
        self.asr_thread = threading.Thread(target=self.asr_process)
        self.asr_thread.daemon = True
        self.asr_thread.start()

        rospy.loginfo("开始录音并等待语音输入...")

        try:
            with sd.InputStream(
                samplerate=self.sample_rate, channels=1, dtype="int16"
            ) as stream:
                vad_buffer = b""
                buffer = b""

                while not rospy.is_shutdown() and self.is_running:
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

        except Exception as e:
            rospy.logerr(f"录音出错: {str(e)}")

        finally:
            rospy.loginfo("录音结束")

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
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if "node" in locals():
            node.shutdown()
