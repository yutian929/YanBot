#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from stt_msgs.srv import STT, STTRequest
from tts_msgs.srv import TTS, TTSRequest  # 确保TTS服务类型存在
import sounddevice as sd
import numpy as np
from scipy.io.wavfile import write
import json
import os
import subprocess  # 用于播放音频


def get_normalized_path(relative_path, verbose=False):
    # 获取当前脚本的绝对路径
    current_script_path = os.path.abspath(__file__)

    # 获取当前脚本所在目录
    current_dir = os.path.dirname(current_script_path)

    # 拼接路径并规范化
    target_path = os.path.normpath(os.path.join(current_dir, relative_path))
    # if not os.path.exists(target_path):
    #     raise FileNotFoundError(f"路径不存在：{target_path}")

    if verbose:
        print(f"current_script_path={current_script_path}")
        print(f"current_dir={current_dir}")
        print(f"target_path={target_path}")

    return target_path


class InteractNode:
    def __init__(self):
        rospy.init_node("interact_node")
        self.wakeup_sub = rospy.Subscriber("topic_wakeup", Bool, self.wakeup_callback)

        # 初始化服务客户端
        rospy.wait_for_service("srv_stt")
        rospy.wait_for_service("srv_tts")
        self.stt_client = rospy.ServiceProxy("srv_stt", STT)
        self.tts_client = rospy.ServiceProxy("srv_tts", TTS)

        # 音频参数
        self.fs = 16000  # 采样率
        self.silence_threshold = 0.02
        self.max_silence = 1.0
        self.min_voice = 0.5
        self.chunk_size = 1024

        self.is_recording = False
        self.recording = []
        self.last_voice_time = 0

    def wakeup_callback(self, msg):
        if msg.data and not self.is_recording:
            self.start_recording()

    def audio_callback(self, indata, frames, time, status):
        if status:
            rospy.logwarn(f"Audio error: {status}")

        rms = np.sqrt(np.mean(indata**2))
        if rms > self.silence_threshold:
            self.last_voice_time = rospy.get_time()
        self.recording.append(indata.copy())

    def start_recording(self):
        self.is_recording = True
        self.recording = []
        self.last_voice_time = rospy.get_time()

        rospy.loginfo("Starting recording...")
        stream = sd.InputStream(
            samplerate=self.fs,
            channels=1,
            dtype="int16",
            blocksize=self.chunk_size,
            callback=self.audio_callback,
        )

        with stream:
            try:
                while self.is_recording:
                    elapsed_silence = rospy.get_time() - self.last_voice_time
                    total_duration = len(self.recording) * self.chunk_size / self.fs

                    if (
                        elapsed_silence > self.max_silence
                        and total_duration > self.min_voice
                    ):
                        break
                    if total_duration > 10.0:
                        break
                    rospy.sleep(0.1)

            except Exception as e:
                rospy.logerr(f"Recording error: {str(e)}")
            finally:
                self.save_and_process()

    def save_and_process(self):
        try:
            if len(self.recording) == 0:
                rospy.logwarn("No audio data recorded")
                return

            # 生成音频数据
            audio_data = np.concatenate(self.recording, axis=0)

            # 分步骤处理流程
            self.save_audio(audio_data, get_normalized_path("../../../../stt.wav"))
            stt_result = self.call_srv_stt("stt.wav")
            tts_file = self.call_srv_tts(stt_result)
            tts_file = get_normalized_path(f"../../../../{tts_file}")
            self.play_audio(tts_file)

        except Exception as e:
            rospy.logerr(f"Processing error: {str(e)}")
        finally:
            self.is_recording = False

    def save_audio(self, audio_data, filename):
        """保存音频到本地文件"""
        try:
            write(filename, self.fs, audio_data)
            rospy.loginfo(f"Audio saved to {filename}")
            return True
        except Exception as e:
            rospy.logerr(f"Save audio failed: {str(e)}")
            return False

    def call_srv_stt(self, filename):
        """调用STT语音识别服务"""
        try:
            req = STTRequest()
            req.input_json = json.dumps({"query": filename})
            resp = self.stt_client(req)
            result = json.loads(resp.output_json)["ans"]
            rospy.loginfo(f"STT Result: {result}")
            return result
        except rospy.ServiceException as e:
            rospy.logerr(f"STT Service call failed: {str(e)}")
            raise
        except Exception as e:
            rospy.logerr(f"STT Processing error: {str(e)}")
            raise

    def call_srv_tts(self, text):
        """调用TTS语音合成服务"""
        try:
            req = TTSRequest()
            req.input_json = json.dumps({"query": text})
            resp = self.tts_client(req)
            tts_path = json.loads(resp.output_json)["ans"]
            rospy.loginfo(f"TTS generated: {tts_path}")
            return tts_path
        except rospy.ServiceException as e:
            rospy.logerr(f"TTS Service call failed: {str(e)}")
            raise
        except Exception as e:
            rospy.logerr(f"TTS Processing error: {str(e)}")
            raise

    def play_audio(self, filename):
        """播放生成的语音文件"""
        try:
            # Linux系统使用aplay
            subprocess.call(["aplay", "-q", filename])
            # MacOS系统使用：subprocess.call(["afplay", filename])
        except FileNotFoundError:
            rospy.logerr(f"Audio file not found: {filename}")
        except Exception as e:
            rospy.logerr(f"Play audio failed: {str(e)}")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = InteractNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
