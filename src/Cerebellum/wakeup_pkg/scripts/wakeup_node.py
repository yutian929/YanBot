import rospy
import sounddevice as sd
import numpy as np
import re
import pypinyin
import difflib
from funasr import AutoModel
from wakeup_pkg.msg import WakeUp
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from modelscope.pipelines import pipeline
from modelscope.utils.constant import Tasks
import time


class WakeUpNode:
    """
    语音唤醒节点，使用VAD检测和ASR语音识别功能
    """

    def __init__(self):
        """初始化WakeUpNode类，加载模型和设置参数"""
        rospy.init_node("wakeup_node")

        # 发布者
        self.wakeup_pub = rospy.Publisher("wakeup", WakeUp, queue_size=1)

        # 麦克风状态标志
        self.mic_active = True

        # 初始化音频处理参数
        self.sample_rate = 16000
        self.chunk_size = 300  # ms
        self.chunk_samples = int(self.sample_rate * self.chunk_size / 1000)  # 样本数
        self.frame_length = 512  # 每次处理的样本数

        # 初始化VAD模型
        vad_model_dir = rospy.get_param("~vad_model_dir", "fsmn-vad")
        rospy.loginfo(f"正在加载VAD语音活动检测模型: {vad_model_dir}...")
        self.vad_model = AutoModel(model=vad_model_dir, disable_pbar=True)

        # 初始化ASR模型
        asr_model_dir = rospy.get_param("~asr_model_dir", "iic/SenseVoiceSmall")
        rospy.loginfo(f"正在加载ASR语音识别模型: {asr_model_dir}...")
        self.asr_pipeline = pipeline(
            task=Tasks.auto_speech_recognition,
            model=asr_model_dir,
            model_revision="master",
            device="cuda:0",
            disable_pbar=True,
        )

        # 初始化唤醒词检测阈值
        self.similar_threshold = rospy.get_param("~similar_threshold", 0.8)

        # 录音相关参数
        self.recording_data = []
        self.vad_cache = {}
        self.no_speech_frames = 0
        self.max_no_speech_frames = rospy.get_param(
            "~max_no_speech_frames", 3
        )  # 连续max_no_speech_frames次没有检测到语音则停止录音
        self.is_recording = False
        self.has_speech = False
        self.audio_buffer = np.array([], dtype=np.int16)  # 添加音频缓冲区
        self.max_recording_frames = rospy.get_param(
            "~max_recording_frames", 100
        )  # 添加最大录音帧数限制

        # 添加服务
        self.wakeup_control_service = rospy.Service(
            "wakeup_control", SetBool, self.service_callback
        )

        self.stream = None  # 添加stream属性

        rospy.loginfo("wakeup_node Started")

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

    def process_recording(self):
        """处理录音数据，进行ASR识别检测唤醒词"""
        if not self.recording_data or len(self.recording_data) == 0:
            rospy.loginfo("没有检测到有效语音数据")
            return False

        try:
            # 合并所有录音数据
            audio_data = np.concatenate(self.recording_data)

            # 确保数据长度足够
            if len(audio_data) < 1600:  # 至少100ms的音频
                rospy.logwarn(f"录音数据太短: {len(audio_data)} 样本")
                return False

            # 进行ASR识别
            audio_data_float = audio_data.astype(np.float32) / 32768.0
            ret = self.asr_pipeline(audio_data_float)
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
            ret1 = self.find_similar_substrings(
                "ni3hao3yan4yan4", pinyin_str, self.similar_threshold
            )
            ret2 = self.find_similar_substrings(
                "nihaoyanyan", pinyin_str2, self.similar_threshold
            )
            ret3 = self.find_similar_substrings(
                "ni3hao3yan4", pinyin_str2, self.similar_threshold
            )
            ret4 = self.find_similar_substrings(
                "nihaoyan", pinyin_str, self.similar_threshold
            )
            ret5 = self.find_similar_substrings(
                "xiao3yan4xiao3yan4", pinyin_str, self.similar_threshold
            )
            ret6 = self.find_similar_substrings(
                "xiaoyanxiaoyan", pinyin_str2, self.similar_threshold
            )

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

            return wakeup_msg.wakeup

        except Exception as e:
            rospy.logerr(f"ASR处理出错: {str(e)}")
            return False

    def start_recording(self):
        """开始录音"""
        # 清空录音数据和VAD缓存
        self.recording_data = []
        self.vad_cache = {}
        self.no_speech_frames = 0
        self.has_speech = False
        self.is_recording = True

    def stop_recording(self):
        """停止录音并处理"""
        self.is_recording = False

        # 处理录音数据
        is_wakeup = self.process_recording()

        # 清理状态
        self.recording_data = []
        self.vad_cache = {}

        return is_wakeup

    def service_callback(self, req):
        response = SetBoolResponse()

        if req.data and not self.mic_active:
            # 开启麦克风
            rospy.loginfo("开启语音唤醒功能")
            self.mic_active = True
            # 重新初始化音频相关变量
            self.recording_data = []
            self.vad_cache = {}
            self.no_speech_frames = 0
            self.has_speech = False
            self.is_recording = False
            self.audio_buffer = np.array([], dtype=np.int16)
            response.success = True
            response.message = "语音唤醒功能已开启"
        elif not req.data and self.mic_active:
            # 关闭麦克风
            rospy.loginfo("关闭语音唤醒功能")
            # 先将状态设为False，确保run循环不再读取数据
            self.mic_active = False
            # 等待一小段时间确保run循环退出当前读取
            rospy.sleep(0.1)
            # 清空所有音频相关变量
            self.recording_data = []
            self.vad_cache = {}
            self.no_speech_frames = 0
            self.has_speech = False
            self.is_recording = False
            self.audio_buffer = np.array([], dtype=np.int16)
            # 关闭音频流
            try:
                if self.stream is not None:
                    self.stream.stop()
                    rospy.sleep(0.1)  # 给一点时间让stream完全停止
                    self.stream.close()
                    self.stream = None
            except Exception as e:
                rospy.logwarn(f"关闭音频流时出现警告: {str(e)}")
            response.success = True
            response.message = "语音唤醒功能已关闭"
        else:
            # 已经处于请求的状态
            response.success = True
            response.message = f"语音唤醒功能已经{'开启' if self.mic_active else '关闭'}"

        return response

    def run(self):
        """主循环，进行录音、VAD检测和ASR识别"""
        while not rospy.is_shutdown():
            try:
                if not self.mic_active:
                    if self.stream is not None:
                        try:
                            self.stream.stop()
                            self.stream.close()
                        except Exception as e:
                            rospy.logwarn(f"关闭音频流时出现警告: {str(e)}")
                        finally:
                            self.stream = None
                    rospy.sleep(0.1)
                    continue

                # 如果stream不存在或已关闭，重新打开
                if self.stream is None:
                    self.stream = sd.InputStream(
                        samplerate=self.sample_rate,
                        channels=1,
                        dtype="int16",
                        blocksize=self.frame_length,
                        latency="low",
                    )
                    self.stream.start()

                # 读取音频数据
                indata, overflowed = self.stream.read(self.frame_length)
                if overflowed:
                    rospy.logwarn("音频溢出！")

                # 转换数据格式
                pcm_data = indata.flatten().astype(np.int16)

                # 将新数据添加到缓冲区
                self.audio_buffer = np.concatenate((self.audio_buffer, pcm_data))

                # 如果正在录音，保存数据
                if self.is_recording:
                    self.recording_data.append(pcm_data)

                    # 添加安全检查，防止录音无限进行
                    if len(self.recording_data) > self.max_recording_frames:
                        rospy.logwarn(f"<<<已达最大帧数限制({self.max_recording_frames})，停止录音")
                        self.stop_recording()
                        continue

                    # 检查是否有足够的数据进行VAD检测
                    if len(self.audio_buffer) >= self.chunk_samples:
                        # 获取一个chunk的数据
                        chunk = self.audio_buffer[: self.chunk_samples]
                        self.audio_buffer = self.audio_buffer[self.chunk_samples :]

                        # 进行VAD检测
                        try:
                            chunk_float = chunk.astype(np.float32) / 32768.0
                            is_final = False
                            res = self.vad_model.generate(
                                input=chunk_float,
                                cache=self.vad_cache,
                                is_final=is_final,
                                chunk_size=self.chunk_size,
                            )

                            # 检查VAD结果
                            if (
                                res
                                and len(res) > 0
                                and "value" in res[0]
                                and res[0]["value"]
                            ):
                                self.has_speech = True  # 确保设置has_speech为True
                                self.no_speech_frames = 0
                                rospy.logdebug("语音活动持续")
                            else:
                                self.no_speech_frames += 1

                            # 如果已经检测到语音，且之后连续多帧没有语音，结束录音
                            if (
                                self.has_speech
                                and self.no_speech_frames >= self.max_no_speech_frames
                            ):
                                rospy.loginfo(f"<<<")
                                self.stop_recording()
                        except Exception as e:
                            rospy.logerr(f"VAD处理出错: {str(e)}")
                else:
                    # 检查是否有足够的数据进行VAD检测
                    if len(self.audio_buffer) >= self.chunk_samples:
                        # 获取一个chunk的数据
                        chunk = self.audio_buffer[: self.chunk_samples]
                        self.audio_buffer = self.audio_buffer[self.chunk_samples :]

                        # 进行VAD检测
                        try:
                            chunk_float = chunk.astype(np.float32) / 32768.0
                            res = self.vad_model.generate(input=chunk_float)

                            # 如果检测到语音，开始录音
                            if (
                                res
                                and len(res) > 0
                                and "value" in res[0]
                                and res[0]["value"]
                            ):
                                rospy.loginfo(">>>")
                                self.start_recording()
                                self.recording_data.append(chunk)  # 保存当前块
                        except Exception as e:
                            rospy.logerr(f"VAD处理出错: {str(e)}")

            except Exception as e:
                rospy.logerr(f"录音出错: {str(e)}")
                try:
                    if self.stream is not None:
                        self.stream.stop()
                        self.stream.close()
                except Exception as close_error:
                    rospy.logwarn(f"关闭音频流时出现警告: {str(close_error)}")
                finally:
                    self.stream = None

    def shutdown(self):
        """关闭节点时的清理工作"""
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()


if __name__ == "__main__":
    try:
        node = WakeUpNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if "node" in locals():
            node.shutdown()
