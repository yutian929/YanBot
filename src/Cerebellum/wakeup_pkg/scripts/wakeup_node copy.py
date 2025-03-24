#!/usr/bin/env python3

from modelscope.pipelines import pipeline
from modelscope.utils.constant import Tasks
from modelscope.fileio import File
import sounddevice as sd
import queue
import threading
import soundfile
from funasr import AutoModel
import numpy as np
import io
import pypinyin
import re
import difflib
import rospy
from std_msgs.msg import String

"""
    pip install modelscope sounddevice soundfile funasr pypinyin simplejson datasets==2.18.0 addict sortedcontainers
    sudo apt-get install libsndfile1

    有可能出现的问题：
        SSL报错，解决办法：先remove掉pip，再通过get-pip.py安装pip，最后pip install pyopenssl --upgrade


"""


def create_wav_header(dataflow, sample_rate=16000, num_channels=1, bits_per_sample=16):
    """
    创建WAV文件头的字节串。

    :param dataflow: 音频bytes数据（以字节为单位）。
    :param sample_rate: 采样率，默认16000。
    :param num_channels: 声道数，默认1（单声道）。
    :param bits_per_sample: 每个样本的位数，默认16。
    :return: WAV文件头的字节串和音频bytes数据。
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


def find_similar_substrings(target, long_string, threshold=0.7):
    """
    在一个长字符串中查找与目标字符串相似度大于阈值的子字符串

    :param target: 目标字符串
    :param long_string: 长字符串
    :param threshold: 相似度阈值，默认为 0.7
    :return: 相似度大于阈值的子字符串列表
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


ANS_pipeline = pipeline(
    Tasks.acoustic_noise_suppression,
    model="iic/speech_zipenhancer_ans_multiloss_16k_base",
)

VAD_model = AutoModel(
    model="fsmn-vad",
    model_revision="v2.0.4",
    disable_pbar=True,
)

ASR_pipeline = pipeline(
    task=Tasks.auto_speech_recognition,
    model="iic/SenseVoiceSmall",
    model_revision="master",
    device="cuda:0",
    disable_pbar=True,
)

sample_rate = 16000
window = 8 * 1600 * 2  # 800ms 窗口
denoise_queue = queue.Queue()
vad_queue = queue.Queue()


def ASR():
    """
    ASR 模块，分线程使用

    :IN: 语音流 vad_queue

    :TOPIC: /ASR/msg String
    :TOPIC: /ASR/wakeup String
    话题按需调整，目前直接把utf8扔进去了

    ASR拿到以后识别，文字转拼音，判断与唤醒词拼音的相似程度，大于阈值则发布话题，否则不发布话题
    """
    global vad_active, asr_buffer
    print("ASR start")
    ASR_pub = rospy.Publisher("ASR/msg", String, queue_size=1)
    wakeup_pub = rospy.Publisher("ASR/wakeup", String, queue_size=1)
    while True:
        try:
            speech = vad_queue.get()
        except queue.Empty:
            continue
        ret = ASR_pipeline(speech)
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

        # 唤醒词
        ret1 = find_similar_substrings("ni3hao3yan4yan4", pinyin_str, 0.5)
        ret2 = find_similar_substrings("nihaoyanyan", pinyin_str2, 0.5)
        ret3 = find_similar_substrings("nihaoyan", pinyin_str2, 0.5)
        ret4 = find_similar_substrings("nihaoyayan", pinyin_str2, 0.5)
        ret5 = find_similar_substrings("xiao3yan4xiao3yan4", pinyin_str, 0.5)
        ret6 = find_similar_substrings("xiaoyanxiaoyan", pinyin_str2, 0.5)

        if len(cleaned_text) > 0:
            ASR_pub.publish(cleaned_text)
            rospy.loginfo("ASR结果:%s", cleaned_text)

        if ret1 or ret2 or ret3 or ret4:
            msg = "ASR唤醒" + ret1[0][0] + ret2[0][0]
            wakeup_pub.publish(msg)
            rospy.loginfo("ASR唤醒:%s", msg)

        if ret5 or ret6:
            msg = "ASR唤醒" + ret5[0][0] + ret6[0][0]
            wakeup_pub.publish(msg)
            rospy.loginfo("ASR唤醒:%s", msg)


if __name__ == "__main__":
    rospy.init_node("ASR_node")

    ASR_thread = threading.Thread(target=ASR)
    ASR_thread.start()

    # 从mic每800ms读入数据，先过降噪ANS，再过VAD，VAD检测每个800ms片段有没有说话，有就放入buffer或拼接到最后，当没有语音活动时，将buffer中的数据放入vad_queue
    # ASR线程从vad_queue中取出数据，进行ASR

    try:
        with sd.InputStream(
            samplerate=sample_rate, channels=1, dtype="int16"
        ) as stream:
            vad_buffer = b""
            buffer = b""
            while True:

                dataflow = stream.read(window)[0].tobytes()
                print(len(dataflow))
                if len(dataflow) == 0:
                    break
                result = ANS_pipeline(
                    create_wav_header(
                        dataflow,
                        sample_rate=sample_rate,
                        num_channels=1,
                        bits_per_sample=16,
                    )
                )
                output = result["output_pcm"]
                vad_buffer = create_wav_header(
                    output, sample_rate=sample_rate, num_channels=1, bits_per_sample=16
                )
                res = VAD_model.generate(input=vad_buffer)
                denoise_queue.put(output)
                if res[0]["value"]:
                    print(res)
                    buffer += output
                else:
                    if len(buffer) > 0:
                        vad_queue.put(
                            create_wav_header(
                                buffer,
                                sample_rate=sample_rate,
                                num_channels=1,
                                bits_per_sample=16,
                            )
                        )
                        buffer = b""
    except KeyboardInterrupt:
        print("Recording stopped.")

    rospy.spin()
    ASR_thread.join()
