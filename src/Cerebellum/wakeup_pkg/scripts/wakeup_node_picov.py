import queue
import time
import sounddevice as sd
import pvporcupine
import numpy as np
from funasr import AutoModel
from scipy.io.wavfile import write
import os


class VoiceAssistant:
    def __init__(self):
        # 初始化唤醒引擎
        self.porcupine = pvporcupine.create(
            access_key="", keyword_paths=["./Hi-Robot_en_linux_v3_0_0.ppn"]
        )

        # 获取引擎参数
        self.sample_rate = self.porcupine.sample_rate  # 16000
        self.frame_length = self.porcupine.frame_length  # 512

        # 音频输入配置
        self.channels = 1
        self.dtype = "int16"  # Porcupine要求的输入格式

        # 缓冲区配置
        self.buffer = np.array([], dtype=self.dtype)
        self.silence_threshold = 300  # 适用于int16的静音阈值

        # 录音存储
        self.recording_data = []

        # 语音端点检测配置
        self.vad_model = AutoModel(model="fsmn-vad", model_revision="v2.0.4")
        self.chunk_size = 200  # ms
        self.chunk_stride = int(self.chunk_size * self.sample_rate / 1000)  # 样本数
        self.vad_cache = {}

        # 端点检测参数
        self.no_speech_frames = 0
        self.max_no_speech_frames = 10  # 连续10次没有检测到语音则停止录音
        self.is_recording = False
        self.has_speech = False

    def _audio_callback(self, indata, frames, time, status):
        """音频流回调函数（每次处理512样本）"""
        # 转换并缓存音频数据
        pcm_data = indata.flatten().astype(self.dtype)
        self.buffer = np.concatenate((self.buffer, pcm_data))

        # 正在录音时，保存数据
        if self.is_recording:
            self.recording_data.append(pcm_data)

            # 当缓存足够进行VAD检测时
            if len(pcm_data) >= self.chunk_stride:
                speech_chunk = (
                    pcm_data[: self.chunk_stride].astype(np.float32) / 32768.0
                )  # 转换为float32
                is_final = False
                res = self.vad_model.generate(
                    input=speech_chunk,
                    cache=self.vad_cache,
                    is_final=is_final,
                    chunk_size=self.chunk_size,
                )

                # 检查VAD结果
                if len(res[0]["value"]):
                    self.has_speech = True
                    self.no_speech_frames = 0
                    print("检测到语音活动")
                else:
                    self.no_speech_frames += 1

                # 如果已经检测到语音，且之后连续多帧没有语音，结束录音
                if (
                    self.has_speech
                    and self.no_speech_frames >= self.max_no_speech_frames
                ):
                    self._save_recording()

        # 当缓存足够处理时进行唤醒检测
        while len(self.buffer) >= self.frame_length and not self.is_recording:
            # 提取单帧数据
            frame = self.buffer[: self.frame_length]
            self.buffer = self.buffer[self.frame_length :]

            # 执行唤醒检测
            keyword_index = self.porcupine.process(frame)
            if keyword_index >= 0:
                print("唤醒词检测成功!")
                self._start_recording()

    def _start_recording(self):
        """开始录音"""
        print("开始录音...")
        # 清空录音数据和VAD缓存
        self.recording_data = []
        self.vad_cache = {}
        self.no_speech_frames = 0
        self.has_speech = False
        self.is_recording = True
        self.recording_start = time.time()

    def _save_recording(self):
        """保存录音"""
        print("停止录音")
        self.is_recording = False

        # 确保有录音数据
        if not self.recording_data:
            print("没有检测到有效语音数据")
            return

        # 合并所有录音数据
        audio_data = np.concatenate(self.recording_data)

        # 使用VAD模型最终确认有效语音段
        is_final = True
        final_res = self.vad_model.generate(
            input=audio_data.astype(np.float32) / 32768.0,
            cache={},
            is_final=is_final,
            chunk_size=self.chunk_size,
        )

        # 根据时间创建唯一文件名
        timestamp = int(time.time())
        filename = f"recording_{timestamp}.wav"

        # 保存为WAV文件
        write(filename, self.sample_rate, audio_data)
        print(f"录音已保存为: {filename}")

        # 清理状态
        self.recording_data = []
        self.vad_cache = {}

    def run(self):
        """主运行循环"""
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype=self.dtype,  # 直接获取int16格式
            blocksize=self.frame_length,  # 关键参数：对齐帧长度
            latency="low",
        ) as stream:
            print("等待唤醒...")
            while True:
                # 读取音频数据（阻塞模式）
                data, overflowed = stream.read(self.frame_length)
                if overflowed:
                    print("音频溢出!")

                # 处理音频帧
                self._audio_callback(data, None, None, None)

                # 设置最大录音时间限制
                if self.is_recording and time.time() - self.recording_start > 30:
                    print("达到最大录音时间限制")
                    self._save_recording()


if __name__ == "__main__":
    assistant = VoiceAssistant()
    try:
        assistant.run()
    except KeyboardInterrupt:
        print("程序已终止")
