import queue
import time
import sounddevice as sd
import pvporcupine
import numpy as np


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

    def _audio_callback(self, indata, frames, time, status):
        """音频流回调函数（每次处理512样本）"""
        # 转换并缓存音频数据
        pcm_data = indata.flatten().astype(self.dtype)
        self.buffer = np.concatenate((self.buffer, pcm_data))

        # 当缓存足够处理时进行唤醒检测
        while len(self.buffer) >= self.frame_length:
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
        # 清空缓冲区，准备记录后续的10秒音频
        self.buffer = np.array([], dtype=self.dtype)
        self.recording_start = time.time()

    def _check_audio(self):
        """检查录音状态"""
        if hasattr(self, "recording_start"):
            # 持续10秒录音
            if time.time() - self.recording_start > 10:
                self._save_recording()
            # 静音检测（示例：简单阈值法）
            elif (
                len(self.buffer) > 0
                and np.abs(self.buffer[-1000:]).mean() < self.silence_threshold
            ):
                self._save_recording()

    def _save_recording(self):
        """保存录音"""
        print("停止录音")
        # 保存前处理缓冲区数据
        valid_data = self.buffer[: 16000 * 10]  # 最多10秒
        # 调用FunASR处理
        # asr_result = self.process_with_funasr(valid_data)
        # 保存为WAV文件
        from scipy.io.wavfile import write

        write("recording.wav", self.sample_rate, valid_data)
        del self.recording_start

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
                self._check_audio()


if __name__ == "__main__":
    assistant = VoiceAssistant()
    try:
        assistant.run()
    except KeyboardInterrupt:
        print("程序已终止")
